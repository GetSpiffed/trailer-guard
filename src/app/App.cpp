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
#include "services/TrackerService.h"

// Bundelt hardwaredrivers en services voor gedeeld gebruik.
class AppContext {
public:
	BootButton bootButton;
	PowerAxp2101 power;
	DisplaySh1106 display;
	GpsL76k gps;
	TrackerService tracker;
	LoraWanService lorawan;
};

// Globale contextinstantie voor de applicatie.
static AppContext ctx;

void App::begin() {
	Serial.begin(115200);
	delay(200);

	ctx.bootButton.begin();
	state_.inputReady = true;

	ctx.power.begin();
	ctx.display.begin();
	disableRadios();
	ctx.gps.begin();

	state_.initDone = true;
	resetJoinState();
	ctx.lorawan.resetJoinTracking();
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
}

void App::forceOtaaReset() {
	ctx.lorawan.clearSession();

	LoRaWANSchemeSession_t tmp;
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
			state_.gate = GateState::Running;
			resetJoinState();
			ctx.lorawan.resetJoinTracking();
		} else {
			ctx.display.showModeMessage("INIT...");
			delay(400);
		}
	}

	CurrentFix fix = ctx.gps.getFix();
	uint16_t battMv = ctx.power.readBatteryMv();

	JoinState prevJoinState = state_.joinState;
	ctx.lorawan.updateJoinFlow(state_, fix, ctx.tracker, battMv);
	if (prevJoinState != JoinState::Ok && state_.joinState == JoinState::Ok) {
		ctx.gps.anchorToCurrent();
	}

	uint32_t prevLastUplinkOkMs = state_.lastUplinkOkMs;
	ctx.lorawan.updateUplinkFlow(state_, fix, ctx.tracker, battMv);
	if (state_.lastUplinkOkMs != prevLastUplinkOkMs) {
		ctx.gps.anchorToCurrent();
	}

	static uint32_t lastDrawMs = 0;
	if (now - lastDrawMs >= 250) {
		lastDrawMs = now;
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
