// Added StartupMode + boot/wakeup detection to prep for deep sleep.
// TODO: Hook actual deep sleep entry where determineStartupMode() decides AutoStart.
#include "app/App.h"

#include <WiFi.h>
#include <esp_bt.h>
#include <RadioLib.h>

#include "config/AppConfig.h"
#include "drivers/BootButton.h"
#include "drivers/DisplaySh1106.h"
#include "drivers/GpsL76k.h"
#include "drivers/PowerAxp2101.h"
#include "services/LoraWanService.h"
#include "services/TelemetryService.h"
#include "services/UplinkScheduler.h"

// Bundelt hardwaredrivers en services voor gedeeld gebruik.
class AppContext {
public:
	BootButton bootButton;
	PowerAxp2101 power;
	DisplaySh1106 display;
	GpsL76k gps;
	LoraWanService lorawan;
	TelemetryService telemetry;
	UplinkScheduler scheduler;
};

// Globale contextinstantie voor de applicatie.
static AppContext ctx;

namespace {
uint8_t readBatteryPercentFromPmu() {
	return ctx.power.readBatteryPercent();
}

void printPayloadHex(const std::vector<uint8_t>& payload) {
	Serial.print("[uplink] payload=");
	for (size_t i = 0; i < payload.size(); ++i) {
		if (payload[i] < 16) Serial.print('0');
		Serial.print(payload[i], HEX);
		if (i + 1 < payload.size()) Serial.print(' ');
	}
	Serial.println();
}

const char* wakeCauseLabel(esp_sleep_wakeup_cause_t cause) {
	switch (cause) {
		case ESP_SLEEP_WAKEUP_UNDEFINED: return "COLD/RESET";
		case ESP_SLEEP_WAKEUP_TIMER: return "TIMER";
		case ESP_SLEEP_WAKEUP_EXT0: return "EXT0";
		case ESP_SLEEP_WAKEUP_EXT1: return "EXT1";
		case ESP_SLEEP_WAKEUP_ULP: return "ULP";
		case ESP_SLEEP_WAKEUP_GPIO: return "GPIO";
		case ESP_SLEEP_WAKEUP_UART: return "UART";
		default: return "OTHER";
	}
}

const char* resetReasonLabel(esp_reset_reason_t reason) {
	switch (reason) {
		case ESP_RST_POWERON: return "POWERON";
		case ESP_RST_EXT: return "EXT";
		case ESP_RST_SW: return "SW";
		case ESP_RST_PANIC: return "PANIC";
		case ESP_RST_INT_WDT: return "INT_WDT";
		case ESP_RST_TASK_WDT: return "TASK_WDT";
		case ESP_RST_WDT: return "WDT";
		case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
		case ESP_RST_BROWNOUT: return "BROWNOUT";
		case ESP_RST_SDIO: return "SDIO";
		default: return "UNKNOWN";
	}
}
} // namespace

void App::begin() {
	Serial.begin(115200);
	delay(200);

	ctx.bootButton.begin();
	state_.inputReady = true;

	ctx.power.begin();
	ctx.display.begin();
	disableRadios();
	ctx.gps.begin();

	ctx.telemetry.begin(&fixSnapshot_);
	ctx.telemetry.bindBatteryReader(readBatteryPercentFromPmu);
	ctx.scheduler.begin(300000);

	state_.initDone = true;
	detectBootInfo();

	if (state_.startupMode == StartupMode::AutoStart) {
		startNetwork();
	} else {
		resetJoinState();
		ctx.lorawan.resetJoinTracking();
	}
}

void App::disableRadios() {
	WiFi.mode(WIFI_OFF);
	btStop();
}

void App::resetJoinState() {
	state_.joinState = JoinState::Radio;
	state_.uplinkResult = UplinkResult::Idle;
	state_.lastLoraErr = RADIOLIB_ERR_NONE;
	state_.nextActionMs = millis();
	state_.nextUplinkMs = 0;
	state_.lastUplinkOkMs = 0;
	state_.joinAttempts = 0;
	state_.radioInitAttempts = 0;
	state_.radioReady = false;
	state_.joined = false;
	state_.sessionRestored = false;
	state_.testUplinkBackoff = 0;
	state_.forceOtaaThisBoot = false;
	ctx.scheduler.begin(300000);
}

void App::forceOtaaReset() {
	ctx.lorawan.clearSession();

	LoraWanService::SessionBuffer tmp{};
	bool stillThere = ctx.lorawan.loadSession(tmp);
	ctx.display.showModeMessage(stillThere ? "CLEAR FAIL?!" : "SESSION CLEARED");
	delay(900);

	state_.forceOtaaThisBoot = true;
	state_.joined = false;
	state_.sessionRestored = false;
	state_.joinState = JoinState::Join;
	state_.nextActionMs = millis();
	state_.nextUplinkMs = 0;
	state_.uplinkResult = UplinkResult::Idle;
	state_.lastLoraErr = RADIOLIB_ERR_NONE;
	state_.joinAttempts = 0;
	state_.testUplinkBackoff = 0;
	ctx.lorawan.resetJoinTracking();
	ctx.scheduler.begin(300000);
}

void App::startNetwork() {
	state_.gate = GateState::Running;
	resetJoinState();
	ctx.lorawan.resetJoinTracking();
}

void App::detectBootInfo() {
	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
	state_.wakeCause = (cause == ESP_SLEEP_WAKEUP_UNDEFINED)
							   ? BootWakeCause::ColdBoot
							   : BootWakeCause::DeepSleepWake;
	state_.startupMode = determineStartupMode(state_.wakeCause);

	esp_reset_reason_t reason = esp_reset_reason();
	state_.resetReason = static_cast<uint8_t>(reason);

	Serial.printf("[boot] wake=%s reset=%s startup=%s\n",
				  wakeCauseLabel(cause),
				  resetReasonLabel(reason),
				  (state_.startupMode == StartupMode::AutoStart) ? "AUTO" : "MANUAL");
}

StartupMode App::determineStartupMode(BootWakeCause cause) {
	return (cause == BootWakeCause::DeepSleepWake) ? StartupMode::AutoStart : StartupMode::ManualStart;
}

void App::fixSnapshotUpdate(const CurrentFix& fix, uint32_t nowMs) {
	fixSnapshot_.hasFix = fix.hasFix;
	fixSnapshot_.latE7 = fix.hasFix ? static_cast<int32_t>(lroundf(fix.lat * 10000000.0f)) : 0;
	fixSnapshot_.lonE7 = fix.hasFix ? static_cast<int32_t>(lroundf(fix.lon * 10000000.0f)) : 0;

	if (fix.hasFix && fix.hasSpeed) {
		int32_t kmh = static_cast<int32_t>(lroundf(fix.speedMps * 3.6f));
		if (kmh < 0) kmh = 0;
		if (kmh > 255) kmh = 255;
		fixSnapshot_.speedKmh = static_cast<uint8_t>(kmh);
	} else {
		fixSnapshot_.speedKmh = 0;
	}

	if (fix.lastFixMs > 0 && nowMs >= fix.lastFixMs) {
		fixSnapshot_.ageMs = nowMs - fix.lastFixMs;
	} else {
		fixSnapshot_.ageMs = 0;
	}
}

void App::tick() {
	uint32_t now = millis();

	ctx.gps.pump();
	ctx.bootButton.update();
	ctx.power.updateChargeLed();

	if (ctx.bootButton.consumeLongPressTrigger()) {
		if (state_.initDone) {
			state_.gate = GateState::Running;
			forceOtaaReset();
		} else {
			ctx.display.showModeMessage("INIT...");
			delay(400);
		}
	} else if (ctx.bootButton.consumeShortPress()) {
		if (state_.initDone) {
			startNetwork();
		} else {
			ctx.display.showModeMessage("INIT...");
			delay(400);
		}
	}

	JoinState prevJoinState = state_.joinState;
	ctx.lorawan.updateJoinFlow(state_);
	if (prevJoinState != JoinState::Ok && state_.joinState == JoinState::Ok) {
		ctx.gps.anchorToCurrent();
		ctx.scheduler.onLinkReady();
		Serial.printf("[lorawan] link ready (%s)\n", state_.sessionRestored ? "restore" : "otaa");
	}

	if (ctx.scheduler.loop(now)) {
		Telemetry telemetry = ctx.telemetry.collect();
		std::vector<uint8_t> payload = ctx.telemetry.encode(telemetry);
		printPayloadHex(payload);

		int16_t st = ctx.lorawan.sendUplink(payload.data(), payload.size(), AppConfig::LORAWAN_FPORT);
		state_.lastLoraErr = st;
		if (st == RADIOLIB_ERR_NONE) {
			state_.uplinkResult = UplinkResult::Ok;
			state_.lastUplinkOkMs = now;
			ctx.scheduler.markSent(now);
			Serial.println("[uplink] send ok (unconfirmed)");
		} else {
			state_.uplinkResult = UplinkResult::Fail;
			Serial.printf("[uplink] send fail err=%d\n", st);
		}
	}

	static uint32_t lastStatusLogMs = 0;
	if (state_.joined && now - lastStatusLogMs >= 10000) {
		lastStatusLogMs = now;
		Serial.printf("[uplink] next send in %lus\n", static_cast<unsigned long>(ctx.scheduler.secondsUntilNext(now)));
	}

	static uint32_t lastDrawMs = 0;
	if (now - lastDrawMs >= 250) {
		lastDrawMs = now;
		CurrentFix fix = ctx.gps.getFix();
		fixSnapshotUpdate(fix, now);
		if (ctx.bootButton.inHoldUi()) {
			ctx.display.showBootHold(ctx.bootButton.getHeldMs());
		} else {
			float battV = ctx.power.readBatteryVoltageV();
			uint8_t battPct = ctx.power.readBatteryPercent();
			char gpsLine[64];
			ctx.gps.renderGpsLine(gpsLine, sizeof(gpsLine));
			ctx.display.renderMain(state_, battV, battPct, gpsLine);
		}
	}
}
