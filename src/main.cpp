// Core app: LoRaWAN + power + OLED
#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_bt.h>

#include <SPI.h>
#include <RadioLib.h>
#include "Preferences.h"

#include "secrets.h"

// ---------------- PMU bus ----------------
TwoWire Wire1_PMU(0);
static constexpr uint8_t PMU_SDA = 42;
static constexpr uint8_t PMU_SCL = 41;

// ---------------- OLED + sensors bus ----------------
static constexpr uint8_t DEV_SDA = 17;
static constexpr uint8_t DEV_SCL = 18;

// ---------------- BOOT button ----------------
static constexpr uint8_t BOOT_PIN = 0; // Button1 (BOOT)
static constexpr uint32_t FORCE_OTAA_MIN_MS = 2000;
static constexpr uint32_t FORCE_OTAA_MAX_MS = 6000;

// ---------------- T-Beam S3 Supreme SX1262 pinmap ----------------
static constexpr int LORA_SCK = 12;
static constexpr int LORA_MISO = 13;
static constexpr int LORA_MOSI = 11;
static constexpr int LORA_CS = 10;
static constexpr int LORA_RST = 5;
static constexpr int LORA_DIO1 = 1;
static constexpr int LORA_BUSY = 4;

// ---------------- App settings (tunable) ----------------
static constexpr uint8_t LORAWAN_FPORT = 1;
static constexpr uint32_t UPLINK_INTERVAL_MS = 60000;
static constexpr uint32_t JOIN_RETRY_MS = 10000;
static constexpr uint32_t RESTORE_TEST_DELAY_MS = 2000;
static constexpr uint8_t JOIN_MAX_ATTEMPTS = 8;
static constexpr uint8_t RADIO_INIT_MAX_ATTEMPTS = 5;
static constexpr float RADIO_TCXO_VOLTAGE = 1.6f;

// ---------------- Globals ----------------
XPowersAXP2101 axp;

// OLED: SH1106 128x64, SW I2C met expliciete pins
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, DEV_SCL, DEV_SDA, U8X8_PIN_NONE);

// ---------------- LoRaWAN (RadioLib) ----------------
SX1262 radio(new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY));

LoRaWANNode node(&radio, &EU868);
using LoRaWANSession = LoRaWANSchemeSession_t;
#ifndef RADIOLIB_ERR_UNSUPPORTED
#define RADIOLIB_ERR_UNSUPPORTED RADIOLIB_ERR_UNKNOWN
#endif

enum class GateState : uint8_t {
	WaitingForBoot,
	Running
};

enum class JoinState : uint8_t {
	Radio,
	Restore,
	TestUplink,
	Join,
	Ok,
	Fail
};

enum class UplinkResult : uint8_t {
	Idle,
	Ok,
	Fail
};

enum class BootAction : uint8_t {
	Normal,
	ForceOTAA
};

struct AppState {
	GateState gate = GateState::WaitingForBoot;
	JoinState joinState = JoinState::Radio;
	UplinkResult uplinkResult = UplinkResult::Idle;
	int16_t lastLoraErr = 0;
	uint32_t nextActionMs = 0;
	uint32_t nextUplinkMs = 0;
	uint32_t lastUplinkOkMs = 0;
	uint8_t joinAttempts = 0;
	uint8_t radioInitAttempts = 0;
	bool radioReady = false;
	bool joined = false;
	bool sessionRestored = false;
	uint8_t testUplinkBackoff = 0;
};

static AppState app;

// ---------- PMU helpers ----------
static uint16_t readBatteryMv() {
	if (!axp.isBatteryConnect()) return 0;
	float v = axp.getBattVoltage();
	if (v > 0 && v < 10.0f) return (uint16_t)lroundf(v * 1000.0f);
	if (v >= 10.0f && v < 1000.0f) return (uint16_t)lroundf(v * 10.0f);
	return (uint16_t)lroundf(v);
}

static float readBatteryVoltageV() {
	uint16_t mv = readBatteryMv();
	if (mv == 0) return 0.0f;
	return (float)mv / 1000.0f;
}

static uint8_t readBatteryPercent() {
	if (!axp.isBatteryConnect()) return 0;
	return (uint8_t)axp.getBatteryPercent();
}

static void updateChargeLed() {
	// LED aan als er USB/VBUS binnenkomt, anders uit.
	// (simpel en stabiel voor debug)
	bool vbus = axp.isVbusIn();
	axp.setChargingLedMode(vbus ? XPOWERS_CHG_LED_ON : XPOWERS_CHG_LED_OFF);
}

// ---- PowerManager (AXP2101) ----
struct PowerManager {
	bool begin() {
		Wire1_PMU.begin(PMU_SDA, PMU_SCL);
		bool ok = axp.begin(Wire1_PMU, AXP2101_SLAVE_ADDRESS, PMU_SDA, PMU_SCL);
		if (!ok) return false;

		axp.enableSystemVoltageMeasure();
		axp.enableVbusVoltageMeasure();
		axp.enableBattVoltageMeasure();
		axp.enableBattDetection();
		axp.enableTemperatureMeasure();

		axp.setALDO1Voltage(3300);
		axp.enableALDO1();

		axp.setALDO3Voltage(3300);
		axp.enableALDO3();

		return true;
	}
};

// ---- DisplayManager (OLED) ----
struct DisplayManager {
	void begin() {
		u8g2.begin();
		u8g2.setFont(u8g2_font_6x10_tf);
	}

	static const char* joinStateText(JoinState state) {
		switch (state) {
			case JoinState::Radio: return "RADIO";
			case JoinState::Restore: return "RESTORE";
			case JoinState::TestUplink: return "TESTUP";
			case JoinState::Join: return "JOIN";
			case JoinState::Ok: return "OK";
			case JoinState::Fail: return "FAIL";
			default: return "?";
		}
	}

	static const char* loraErrLabel(int16_t err, bool uplinkOk) {
		if (uplinkOk) return "OK";
		switch (err) {
			case RADIOLIB_ERR_NONE:
				return "NONE";
#ifdef RADIOLIB_ERR_JOIN_FAILED
			case RADIOLIB_ERR_JOIN_FAILED:
				return "JOIN_FAILED";
#endif
#ifdef RADIOLIB_ERR_NO_JOIN_ACCEPT
			case RADIOLIB_ERR_NO_JOIN_ACCEPT:
				return "NO_ACCEPT";
#endif
#ifdef RADIOLIB_ERR_RX_TIMEOUT
			case RADIOLIB_ERR_RX_TIMEOUT:
				return "TIMEOUT";
#endif
#ifdef RADIOLIB_ERR_TIMEOUT
			case RADIOLIB_ERR_TIMEOUT:
				return "TIMEOUT";
#endif
#ifdef RADIOLIB_ERR_MIC_MISMATCH
			case RADIOLIB_ERR_MIC_MISMATCH:
				return "MIC_MISMATCH";
#endif
#ifdef RADIOLIB_ERR_INVALID_FCNT
			case RADIOLIB_ERR_INVALID_FCNT:
				return "INVALID_FCNT";
#endif
#ifdef RADIOLIB_ERR_DOWNLINK_MALFORMED
			case RADIOLIB_ERR_DOWNLINK_MALFORMED:
				return "DOWNLINK_BAD";
#endif
			default:
				return "OTHER";
		}
	}

	void render(const AppState& state, uint16_t battMv, uint8_t battPct) {
		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_6x10_tf);

		char line[32];

		// 1) Battery
		float battV = readBatteryVoltageV();
		if (battV > 0.0f) {
			snprintf(line, sizeof(line), "BAT %.2fV %u%%", (double)battV,
						 (unsigned)battPct);
		} else {
			snprintf(line, sizeof(line), "BAT --.-V %u%%", (unsigned)battPct);
		}
		u8g2.drawStr(0, 12, line);

		if (state.gate != GateState::Running) {
			u8g2.drawStr(0, 30, "LORA: press BOOT");
			// gewoon doorrenderen, geen return
		}

		uint32_t now = millis();
		int32_t countdown = 0;
		if (state.nextActionMs > now) {
			countdown = (int32_t)((state.nextActionMs - now) / 1000);
		}

		// 2) Join state + countdown
		if (countdown > 0) {
			uint8_t attempt = (state.joinState == JoinState::Radio) ? state.radioInitAttempts
																											 : state.joinAttempts;
			uint8_t maxAttempt = (state.joinState == JoinState::Radio) ? RADIO_INIT_MAX_ATTEMPTS
																											 : JOIN_MAX_ATTEMPTS;
			snprintf(line, sizeof(line), "JOIN %s %u/%u T-%lds", joinStateText(state.joinState),
						 (unsigned)attempt, (unsigned)maxAttempt, (long)countdown);
		} else {
			uint8_t attempt = (state.joinState == JoinState::Radio) ? state.radioInitAttempts
																											 : state.joinAttempts;
			uint8_t maxAttempt = (state.joinState == JoinState::Radio) ? RADIO_INIT_MAX_ATTEMPTS
																											 : JOIN_MAX_ATTEMPTS;
			snprintf(line, sizeof(line), "JOIN %s %u/%u", joinStateText(state.joinState),
						 (unsigned)attempt, (unsigned)maxAttempt);
		}

		if (state.gate == GateState::Running) {
			u8g2.drawStr(0, 30, line);
		}

		if (state.gate != GateState::Running) {
			u8g2.drawStr(0, 42, "LORA: idle");
		} else {
			bool uplinkOk = (state.uplinkResult == UplinkResult::Ok);
			snprintf(line, sizeof(line), "LORA %d %s", (int)state.lastLoraErr,
						 loraErrLabel(state.lastLoraErr, uplinkOk));
			u8g2.drawStr(0, 42, line);
		}

		u8g2.sendBuffer();
	}

	void showModeMessage(const char* line) {
		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_6x10_tf);
		u8g2.drawStr(0, 30, line);
		u8g2.sendBuffer();
	}
};

static PowerManager powerManager;
static DisplayManager displayManager;

static bool beginPower_AXP2101_Wire1() {
	if (!powerManager.begin()) {
		return false;
	}
	return true;
}

// ---- LoRaWAN session storage ----
static constexpr uint32_t SESSION_MAGIC = 0x4C57534Eu; // "LWSN"
static constexpr uint16_t SESSION_VERSION = 1;

struct StoredSession {
	uint32_t magic;
	uint16_t version;
	uint16_t size;
	LoRaWANSession session;
};

// ---- LoRaWAN Manager ----
// Session-only persistence (RadioLib v7.5.x)
struct LoRaWanManager {
	// ---------- Session storage ----------
	static bool loadSession(LoRaWANSession& session) {
		Preferences prefs;
		if (!prefs.begin("lorawan", true)) return false;
		size_t len = prefs.getBytesLength("session");
		if (len != sizeof(StoredSession)) {
			prefs.end();
			return false;
		}
		StoredSession stored{};
		size_t read = prefs.getBytes("session", &stored, sizeof(stored));
		prefs.end();
		if (read != sizeof(stored)) return false;
		if (stored.magic != SESSION_MAGIC) return false;
		if (stored.version != SESSION_VERSION) return false;
		if (stored.size != sizeof(LoRaWANSession)) return false;
		session = stored.session;
		return true;
	}

	static bool saveSession(const LoRaWANSession& session) {
		Preferences prefs;
		if (!prefs.begin("lorawan", false)) return false;
		StoredSession stored{};
		stored.magic = SESSION_MAGIC;
		stored.version = SESSION_VERSION;
		stored.size = sizeof(LoRaWANSession);
		stored.session = session;
		size_t written = prefs.putBytes("session", &stored, sizeof(stored));
		prefs.end();
		return written == sizeof(stored);
	}

	static void clearSession() {
		Preferences prefs;
		if (!prefs.begin("lorawan", false)) return;
		prefs.remove("session");
		prefs.end();
	}

	// ---------- Session ↔ node glue (compat) ----------
	template <typename NodeT, typename SessionT>
	static auto setNodeSession(NodeT& target, const SessionT& session, int)
			-> decltype(target.setSession(&session), int16_t()) {
		return target.setSession(&session);
	}

	template <typename NodeT, typename SessionT>
	static auto setNodeSession(NodeT& target, const SessionT& session, long)
			-> decltype(target.setSession(session), int16_t()) {
		return target.setSession(session);
	}

	template <typename NodeT, typename SessionT>
	static int16_t setNodeSession(NodeT&, const SessionT&, ...) {
		return RADIOLIB_ERR_UNSUPPORTED;
	}

	template <typename NodeT, typename SessionT>
	static auto getNodeSession(NodeT& target, SessionT& session, int)
			-> decltype(target.getSession(&session), int16_t()) {
		return target.getSession(&session);
	}

	template <typename NodeT, typename SessionT>
	static auto getNodeSession(NodeT& target, SessionT& session, long)
			-> decltype(target.getSession(session), int16_t()) {
		return target.getSession(session);
	}

	template <typename NodeT, typename SessionT>
	static int16_t getNodeSession(NodeT&, SessionT&, ...) {
		return RADIOLIB_ERR_UNSUPPORTED;
	}

	// ---------- Radio init ----------
	bool initRadio(AppState& state) {
		SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

		pinMode(LORA_RST, OUTPUT);
		digitalWrite(LORA_RST, LOW);
		delay(50);
		digitalWrite(LORA_RST, HIGH);
		delay(50);

		int16_t st = radio.begin();
		if (st != RADIOLIB_ERR_NONE) {
			state.lastLoraErr = st;
			return false;
		}

		st = radio.setTCXO(RADIO_TCXO_VOLTAGE);
		if (st != RADIOLIB_ERR_NONE) {
			state.lastLoraErr = st;
			return false;
		}

		st = radio.setRegulatorDCDC();
		if (st != RADIOLIB_ERR_NONE) {
			state.lastLoraErr = st;
			return false;
		}

		st = radio.setDio2AsRfSwitch(true);
		if (st != RADIOLIB_ERR_NONE) {
			state.lastLoraErr = st;
			return false;
		}

		radio.setOutputPower(14);
		radio.setCurrentLimit(140);

		state.lastLoraErr = RADIOLIB_ERR_NONE;
		return true;
	}

	bool restoreSession(AppState& state, const LoRaWANSession& session) {
		int16_t st = setNodeSession(node, session, 0);
		if (st != RADIOLIB_ERR_NONE) {
			state.lastLoraErr = st;
			return false;
		}
		state.lastLoraErr = RADIOLIB_ERR_NONE;
		return true;
	}

	// ---------- OTAA join ----------
	bool join(AppState& state) {
		// 1) init OTAA
		int16_t st = node.beginOTAA(JOIN_EUI, DEV_EUI, NWK_KEY, APP_KEY);
		if (st != RADIOLIB_ERR_NONE) {
			state.lastLoraErr = st;
			return false;
		}

		// 2) join
		st = node.activateOTAA();
		if (st != RADIOLIB_ERR_NONE && st != RADIOLIB_LORAWAN_NEW_SESSION) {
			state.lastLoraErr = st;
			return false;
		}

		LoRaWANSession session;
		if (fetchSession(state, session)) {
			(void)saveSession(session);
		}

		state.lastLoraErr = RADIOLIB_ERR_NONE;
		return true;
	}

	bool fetchSession(AppState& state, LoRaWANSession& session) {
		int16_t st = getNodeSession(node, session, 0);
		if (st != RADIOLIB_ERR_NONE) {
			state.lastLoraErr = st;
			return false;
		}
		state.lastLoraErr = RADIOLIB_ERR_NONE;
		return true;
	}

	int16_t sendUplink(const uint8_t* payload, size_t len, uint8_t fport) {
		return node.sendReceive(payload, len, fport);
	}
};

static LoRaWanManager lorawanManager;

static void disableRadios() {
	WiFi.mode(WIFI_OFF);
	btStop();
}

static bool isTimeoutErr(int16_t err) {
#ifdef RADIOLIB_ERR_RX_TIMEOUT
	if (err == RADIOLIB_ERR_RX_TIMEOUT) return true;
#endif
#ifdef RADIOLIB_ERR_TIMEOUT
	if (err == RADIOLIB_ERR_TIMEOUT) return true;
#endif
	return false;
}

static bool isSessionInvalidError(int16_t err) {
	switch (err) {
#ifdef RADIOLIB_ERR_MIC_MISMATCH
		case RADIOLIB_ERR_MIC_MISMATCH:
#endif
#ifdef RADIOLIB_ERR_INVALID_FCNT
		case RADIOLIB_ERR_INVALID_FCNT:
#endif
#ifdef RADIOLIB_ERR_DOWNLINK_MALFORMED
		case RADIOLIB_ERR_DOWNLINK_MALFORMED:
#endif
			return true;
		default:
			return false;
	}
}

static bool isUplinkOkError(int16_t err) {
	if (err == RADIOLIB_ERR_NONE) return true;
#ifdef RADIOLIB_ERR_RX_TIMEOUT
	if (err == RADIOLIB_ERR_RX_TIMEOUT) return true;
#endif
#ifdef RADIOLIB_ERR_TIMEOUT
	if (err == RADIOLIB_ERR_TIMEOUT) return true;
#endif
	return false;
}

static size_t buildPayload(uint8_t* out, size_t maxLen) {
	// payload: 1 + 2 = 3 bytes (msg type + battery mv)
	if (maxLen < 3) return 0;

	uint8_t msgType = 0x00;
	uint16_t battMv = readBatteryMv();

	out[0] = msgType;
	out[1] = (uint8_t)(battMv & 0xFF);
	out[2] = (uint8_t)((battMv >> 8) & 0xFF);

	return 3;
}

// ---- Boot short-press gate ----
struct BootGate {
	static constexpr uint32_t DEBOUNCE_MS = 25;
	static constexpr uint32_t MIN_PRESS_MS = 20;
	static constexpr uint32_t MAX_PRESS_MS = 1200;

	enum class State {
		Idle,
		Pressing,
		AwaitReleaseStable,
		Open
	};

	bool gateOpen = false;
	bool lastRawDown = false;
	uint32_t rawChangeMs = 0;
	uint32_t pressStartMs = 0;
	uint32_t releaseStartMs = 0;
	State state = State::Idle;
	bool initialized = false;

	void begin() {
		pinMode(BOOT_PIN, INPUT_PULLUP);
		lastRawDown = (digitalRead(BOOT_PIN) == LOW);
		rawChangeMs = millis();
		if (lastRawDown) {
			state = State::Pressing;
			pressStartMs = rawChangeMs;
		}
		initialized = true;
	}

	void forceOpen() {
		gateOpen = true;
		state = State::Open;
		initialized = true;
	}

	void update() {
		if (gateOpen || state == State::Open) return;
		if (!initialized) begin();

		bool rawDown = (digitalRead(BOOT_PIN) == LOW);
		uint32_t now = millis();

		if (rawDown != lastRawDown) {
			lastRawDown = rawDown;
			rawChangeMs = now;
		}

		switch (state) {
			case State::Idle:
				if (rawDown) {
					state = State::Pressing;
					pressStartMs = now;
				}
				break;
			case State::Pressing: {
				if (rawDown) {
					if (now - pressStartMs > MAX_PRESS_MS) {
						state = State::Idle;
					}
				} else {
					uint32_t pressMs = now - pressStartMs;
					if (pressMs >= MIN_PRESS_MS && pressMs <= MAX_PRESS_MS) {
						state = State::AwaitReleaseStable;
						releaseStartMs = now;
					} else {
						state = State::Idle;
					}
				}
				break;
			}
			case State::AwaitReleaseStable:
				if (rawDown) {
					state = State::Pressing;
					pressStartMs = now;
				} else if (now - releaseStartMs >= DEBOUNCE_MS &&
									 now - rawChangeMs >= DEBOUNCE_MS) {
					gateOpen = true;
					state = State::Open;
				}
				break;
			case State::Open:
			default:
				break;
		}
	}
};

static BootGate bootGate;

static BootAction detectBootAction() {
	pinMode(BOOT_PIN, INPUT_PULLUP);
	if (digitalRead(BOOT_PIN) != LOW) return BootAction::Normal;

	uint32_t startMs = millis();
	while (digitalRead(BOOT_PIN) == LOW) {
		uint32_t elapsed = millis() - startMs;
		if (elapsed > FORCE_OTAA_MAX_MS) {
			return BootAction::Normal;
		}
		delay(10);
	}

	uint32_t heldMs = millis() - startMs;
	if (heldMs >= FORCE_OTAA_MIN_MS && heldMs <= FORCE_OTAA_MAX_MS) {
		return BootAction::ForceOTAA;
	}
	return BootAction::Normal;
}

static void resetJoinState() {
	app.joinState = JoinState::Radio;
	app.uplinkResult = UplinkResult::Idle;
	app.lastLoraErr = RADIOLIB_ERR_NONE;
	app.nextActionMs = millis();
	app.nextUplinkMs = 0;
	app.lastUplinkOkMs = 0;
	app.joinAttempts = 0;
	app.radioInitAttempts = 0;
	app.radioReady = false;
	app.joined = false;
	app.sessionRestored = false;
	app.testUplinkBackoff = 0;
}

static void updateJoinFlow() {
	if (app.gate != GateState::Running) return;

	uint32_t now = millis();

	switch (app.joinState) {
		case JoinState::Radio: {
			if (now < app.nextActionMs) return;
			app.radioReady = lorawanManager.initRadio(app);
			if (app.radioReady) {
				app.joinState = JoinState::Restore;
				app.nextActionMs = now;
			} else {
				app.radioInitAttempts++;
				if (app.radioInitAttempts >= RADIO_INIT_MAX_ATTEMPTS) {
					app.joinState = JoinState::Fail;
				} else {
					app.joinState = JoinState::Radio;
					app.nextActionMs = now + JOIN_RETRY_MS;
				}
			}
			break;
		}
		case JoinState::Restore: {
			LoRaWANSession session;
			if (lorawanManager.loadSession(session) &&
					lorawanManager.restoreSession(app, session)) {
				app.sessionRestored = true;
				app.testUplinkBackoff = 0;
				app.joinState = JoinState::TestUplink;
				app.nextActionMs = now + RESTORE_TEST_DELAY_MS;
			} else {
				app.joinState = JoinState::Join;
				app.nextActionMs = now;
			}
			break;
		}
		case JoinState::TestUplink: {
			if (now < app.nextActionMs) return;

			uint8_t payload[3];
			size_t n = buildPayload(payload, sizeof(payload));

			int16_t st = lorawanManager.sendUplink(payload, n, LORAWAN_FPORT);
			app.lastLoraErr = st;
			app.uplinkResult = isUplinkOkError(st) ? UplinkResult::Ok : UplinkResult::Fail;

			if (app.uplinkResult == UplinkResult::Ok) {
				app.joined = true;
				app.joinState = JoinState::Ok;
				app.testUplinkBackoff = 0;

				LoRaWANSession saved;
				if (lorawanManager.fetchSession(app, saved)) {
					lorawanManager.saveSession(saved);
				}

				app.nextUplinkMs = now + UPLINK_INTERVAL_MS;
			} else if (isSessionInvalidError(st)) {
				// echte sessie-fout: session wissen en opnieuw joinen
				lorawanManager.clearSession();
				app.sessionRestored = false;

				app.joinState = JoinState::Join;
				app.nextActionMs = now;
			} else {
				if (app.testUplinkBackoff < 5) {
					app.testUplinkBackoff++;
				}
				uint32_t backoffMs = JOIN_RETRY_MS + (uint32_t)app.testUplinkBackoff * 10000;
				app.joinState = JoinState::TestUplink;
				app.nextActionMs = now + backoffMs;
			}
			break;
		}

		case JoinState::Join: {
			if (now < app.nextActionMs) return;
			if (app.joinAttempts >= JOIN_MAX_ATTEMPTS) {
				app.joinState = JoinState::Fail;
				return;
			}
			app.joinAttempts++;
			bool ok = lorawanManager.join(app);
			if (ok) {
				app.joined = true;
				app.joinState = JoinState::Ok;
				app.testUplinkBackoff = 0;
				app.nextUplinkMs = now + UPLINK_INTERVAL_MS;
			} else {
				app.joinState = JoinState::Join;
				app.nextActionMs = now + JOIN_RETRY_MS;
			}
			break;
		}
		case JoinState::Ok:
		case JoinState::Fail:
		default:
			break;
	}
}

static void updateUplinkFlow() {
	if (!app.joined) return;

	uint32_t now = millis();
	if (app.nextUplinkMs == 0 || now < app.nextUplinkMs) return;

	uint8_t payload[3];
	size_t n = buildPayload(payload, sizeof(payload));

	int16_t st = lorawanManager.sendUplink(payload, n, LORAWAN_FPORT);
	app.lastLoraErr = st;
	app.uplinkResult = isUplinkOkError(st) ? UplinkResult::Ok : UplinkResult::Fail;

	if (app.uplinkResult == UplinkResult::Ok) {
		app.lastUplinkOkMs = now;
		LoRaWANSession session;
		if (lorawanManager.fetchSession(app, session)) {
			lorawanManager.saveSession(session);
		}
	} else if (isSessionInvalidError(st)) {
		lorawanManager.clearSession();
		app.joined = false;
		app.sessionRestored = false;
		app.joinState = JoinState::Join;
		app.nextActionMs = now + JOIN_RETRY_MS;
	}

	app.nextUplinkMs = now + UPLINK_INTERVAL_MS;
}

void setup() {
	Serial.begin(115200);
	delay(200);
	Serial.println("Boot");

	if (!beginPower_AXP2101_Wire1()) {
		Serial.println("PMU init failed");
	}

	Serial.printf("PMU SYS=%.2f VBUS=%.2f BATT=%.2f VBUS_IN=%d BAT_CONN=%d\n",
						 (double)axp.getSystemVoltage(), (double)axp.getVbusVoltage(),
						 (double)axp.getBattVoltage(), axp.isVbusIn(),
						 axp.isBatteryConnect());

	displayManager.begin();
	disableRadios();

	BootAction bootAction = detectBootAction();
	if (bootAction == BootAction::ForceOTAA) {
		lorawanManager.clearSession();
		displayManager.showModeMessage("MODE: FORCE OTAA");
		delay(1500);
	}

	bootGate.begin();
	if (bootAction == BootAction::ForceOTAA) {
		bootGate.forceOpen();
		app.gate = GateState::Running;
	}
	resetJoinState();
}

void loop() {
	uint32_t now = millis();

	bootGate.update();
	updateChargeLed();

	if (bootGate.gateOpen && app.gate != GateState::Running) {
		app.gate = GateState::Running;
		resetJoinState();
	}

	updateJoinFlow();
	updateUplinkFlow();

	static uint32_t lastDrawMs = 0;
	if (now - lastDrawMs >= 250) {
		lastDrawMs = now;
		uint16_t battMv = readBatteryMv();
		uint8_t battPct = readBatteryPercent();
		displayManager.render(app, battMv, battPct);
	}
}
