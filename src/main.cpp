#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <esp_bt.h>

#include <SPI.h>
#include <RadioLib.h>
#include "Preferences.h"

#include "secrets.h"



// ---------------- PMU bus ----------------
TwoWire I2C_PMU(0);
static constexpr uint8_t PMU_SDA = 42;
static constexpr uint8_t PMU_SCL = 41;

// ---------------- OLED + sensors bus ----------------
static constexpr uint8_t DEV_SDA = 17;
static constexpr uint8_t DEV_SCL = 18;

// ---------------- GNSS UART ----------------
// Board docs: GNSS TX = GPIO8, GNSS RX = GPIO9
// HardwareSerial.begin(baud, config, RX, TX): RX must be GPIO8, TX must be GPIO9
static constexpr uint8_t GNSS_RX_PIN = 8;	// ESP32 RX <- GNSS TX
static constexpr uint8_t GNSS_TX_PIN = 9;	// ESP32 TX -> GNSS RX
static constexpr uint8_t GNSS_WAKE_PIN = 7; // L76K wake-up
static constexpr uint32_t GNSS_BAUD = 9600;

static constexpr uint8_t GNSS_PPS_PIN = 6; // per LilyGO wiki
static volatile uint32_t ppsCount = 0;

static void IRAM_ATTR onPpsRise() { ppsCount++; }


// ---------------- BOOT button ----------------
static constexpr uint8_t BOOT_PIN = 0; // Button1 (BOOT)
static constexpr uint32_t FORCE_OTAA_MIN_MS = 2000;
static constexpr uint32_t FORCE_OTAA_MAX_MS = 6000;

// ---------------- T-Beam S3 Supreme SX1262 pinmap ----------------
static constexpr int LORA_SCK	= 12;
static constexpr int LORA_MISO = 13;
static constexpr int LORA_MOSI = 11;
static constexpr int LORA_CS	 = 10;
static constexpr int LORA_RST	= 5;
static constexpr int LORA_DIO1 = 1;
static constexpr int LORA_BUSY = 4;

// ---------------- App settings (tunable) ----------------
static constexpr uint8_t	LORAWAN_FPORT = 1;
static constexpr uint32_t UPLINK_INTERVAL_MS = 60000;
static constexpr uint32_t JOIN_RETRY_MS = 10000;
static constexpr uint32_t RESTORE_TEST_DELAY_MS = 2000;
static constexpr uint8_t	JOIN_MAX_ATTEMPTS = 8;
static constexpr uint8_t	RADIO_INIT_MAX_ATTEMPTS = 5;
static constexpr float		RADIO_TCXO_VOLTAGE = 1.6f;

// ---------------- Globals ----------------
XPowersAXP2101 axp;
HardwareSerial GNSS(2);
TinyGPSPlus gps;

// GPS stats (voor OLED diagnose)
static uint32_t gpsChars = 0;
static uint32_t gpsLastStatsMs = 0;
static uint16_t gpsCharsPerSec = 0;

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

// ---------- GPS snapshot (cache TinyGPSPlus reads) ----------
struct GpsSnapshot {
	uint16_t charsPerSec = 0;
	uint8_t sats = 0;
	uint16_t hdop = 9999; // in hundredths
	uint32_t ageMs = 999999;
	bool locationValid = false;
	double lat = 0.0;
	double lon = 0.0;
	float speedKmph = 0.0f;
	bool fix = false;
};

static inline bool gpsFixPolicy(uint32_t ageMs, uint16_t hdop, uint8_t sats) {
	if (ageMs >= 5000) return false;
	if (hdop > 500) return false; // > 5.0
	if (sats < 4) return false;
	return true;
}

static GpsSnapshot readGpsSnapshot() {
	GpsSnapshot snap;
	snap.charsPerSec = gpsCharsPerSec;
	snap.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
	snap.hdop = gps.hdop.isValid() ? gps.hdop.value() : 9999;
	bool locValid = gps.location.isValid();
	snap.ageMs = locValid ? gps.location.age() : 999999;
	snap.locationValid = locValid;
	if (snap.locationValid) {
		snap.lat = gps.location.lat();
		snap.lon = gps.location.lng();
	}
	snap.speedKmph = gps.speed.isValid() ? gps.speed.kmph() : 0.0f;
	snap.fix = gpsFixPolicy(snap.ageMs, snap.hdop, snap.sats);
	return snap;
}

// ---------- PMU helpers ----------
static uint16_t readBatteryMv() {
	if (!axp.isBatteryConnect()) return 0;
	float v = axp.getBattVoltage();
	if (v > 0 && v < 10.0f) return (uint16_t)lroundf(v * 1000.0f);
	if (v >= 10.0f && v < 1000.0f) return (uint16_t)lroundf(v * 10.0f);
	return (uint16_t)lroundf(v);
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
		I2C_PMU.begin(PMU_SDA, PMU_SCL);
		bool ok = axp.begin(I2C_PMU, AXP2101_SLAVE_ADDRESS, PMU_SDA, PMU_SCL);
		if (!ok) return false;

		axp.setALDO1Voltage(3300);
		axp.enableALDO1();

		axp.setALDO3Voltage(3300);
		axp.enableALDO3();

		return true;
	}

	void enableGnssRail() {
		axp.setALDO4Voltage(3300);
		axp.enableALDO4();

		pinMode(GNSS_WAKE_PIN, OUTPUT);
		digitalWrite(GNSS_WAKE_PIN, LOW);      // <— fix: LOW vasthouden
		delay(50);

		Serial.printf("WAKE raw=%d (expected 0)\n", digitalRead(GNSS_WAKE_PIN));
		Serial.printf("AXP: VBUS=%d BattConn=%d BattV=%.2f\n",
									axp.isVbusIn(),
									axp.isBatteryConnect(),
									axp.getBattVoltage());

		delay(500);
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
			case JoinState::Radio:			return "RADIO";
			case JoinState::Restore:		return "RESTORE";
			case JoinState::TestUplink: return "TESTUP";
			case JoinState::Join:			 return "JOIN";
			case JoinState::Ok:				 return "OK";
			case JoinState::Fail:			 return "FAIL";
			default:										return "?";
		}
	}

	static const char* loraErrLabel(int16_t err, bool uplinkOk) {
		if (uplinkOk) return "OK";
		switch (err) {
			case RADIOLIB_ERR_NONE: return "NONE";
#ifdef RADIOLIB_ERR_JOIN_FAILED
			case RADIOLIB_ERR_JOIN_FAILED: return "JOIN_FAILED";
#endif
#ifdef RADIOLIB_ERR_NO_JOIN_ACCEPT
			case RADIOLIB_ERR_NO_JOIN_ACCEPT: return "NO_ACCEPT";
#endif
#ifdef RADIOLIB_ERR_RX_TIMEOUT
			case RADIOLIB_ERR_RX_TIMEOUT: return "TIMEOUT";
#endif
#ifdef RADIOLIB_ERR_TIMEOUT
			case RADIOLIB_ERR_TIMEOUT: return "TIMEOUT";
#endif
#ifdef RADIOLIB_ERR_MIC_MISMATCH
			case RADIOLIB_ERR_MIC_MISMATCH: return "MIC_MISMATCH";
#endif
#ifdef RADIOLIB_ERR_INVALID_FCNT
			case RADIOLIB_ERR_INVALID_FCNT: return "INVALID_FCNT";
#endif
#ifdef RADIOLIB_ERR_DOWNLINK_MALFORMED
			case RADIOLIB_ERR_DOWNLINK_MALFORMED: return "DOWNLINK_BAD";
#endif
			default: return "OTHER";
		}
	}

	void render(const AppState& state, const GpsSnapshot& gpsSnap,
							uint16_t battMv, uint8_t battPct) {
		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_6x10_tf);

		char line[32];

		// 1) Battery
		snprintf(line, sizeof(line), "BAT %umV %u%%",
						 (unsigned)battMv,
						 (unsigned)battPct);
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
			uint8_t attempt = (state.joinState == JoinState::Radio)
													? state.radioInitAttempts
													: state.joinAttempts;
			uint8_t maxAttempt = (state.joinState == JoinState::Radio)
														 ? RADIO_INIT_MAX_ATTEMPTS
														 : JOIN_MAX_ATTEMPTS;
			snprintf(line, sizeof(line), "JOIN %s %u/%u T-%lds",
							 joinStateText(state.joinState),
							 (unsigned)attempt,
							 (unsigned)maxAttempt,
							 (long)countdown);
		} else {
			uint8_t attempt = (state.joinState == JoinState::Radio)
													? state.radioInitAttempts
													: state.joinAttempts;
			uint8_t maxAttempt = (state.joinState == JoinState::Radio)
														 ? RADIO_INIT_MAX_ATTEMPTS
														 : JOIN_MAX_ATTEMPTS;
			snprintf(line, sizeof(line), "JOIN %s %u/%u",
							 joinStateText(state.joinState),
							 (unsigned)attempt,
							 (unsigned)maxAttempt);
		}

		if (state.gate == GateState::Running) {
			u8g2.drawStr(0, 30, line);
		}

		if (state.gate != GateState::Running) {
			u8g2.drawStr(0, 42, "LORA: idle");
		} else {
			bool uplinkOk = (state.uplinkResult == UplinkResult::Ok);
			snprintf(line, sizeof(line), "LORA %d %s",
							(int)state.lastLoraErr,
							loraErrLabel(state.lastLoraErr, uplinkOk));
			u8g2.drawStr(0, 42, line);
		}


		// 3) GPS diagnostics
		if (gpsSnap.charsPerSec == 0) {
			snprintf(line, sizeof(line), "GPS NO UART");
		} else {
				const char* v = gpsSnap.locationValid ? "VAL" : "NOVAL";
				const char* f = gpsSnap.fix ? "FIX" : "NOFIX";
				snprintf(line, sizeof(line), "GPS %uc/s S%u H%.1f A%lus %s %s",
					(unsigned)gpsSnap.charsPerSec,
					(unsigned)gpsSnap.sats,
					(double)(gpsSnap.hdop / 100.0f),
					(unsigned)(gpsSnap.ageMs / 1000),
					v, f);
		}
		u8g2.drawStr(0, 54, line);

		u8g2.sendBuffer();
	}

	void showModeMessage(const char* line) {
		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_6x10_tf);
		u8g2.drawStr(0, 30, line);
		u8g2.sendBuffer();
	}
};

// ---- GNSS Manager ----
// ---- GNSS Manager ----
struct GnssManager {
  void begin() {
    GNSS.setRxBufferSize(2048);          // helpt tegen overflow
    GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
    while (GNSS.available()) GNSS.read(); // flush rommel
  }

  void update() {
    while (GNSS.available()) {
      char c = (char)GNSS.read();
      gpsChars++;

      // NMEA start-char detectie (veelzeggender dan random 1 byte)
      if (c == '$') {
        Serial.println("GNSS: saw '$' (NMEA start)");
      }

      // zet raw dump UIT (kan timing slopen)
      // Serial.write(c);

      gps.encode(c);
    }

    uint32_t now = millis();
    if (now - gpsLastStatsMs >= 1000) {
      gpsCharsPerSec = (uint16_t)gpsChars;
      gpsChars = 0;
      gpsLastStatsMs = now;
    }
  }
};


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
    if (st != RADIOLIB_ERR_NONE) { state.lastLoraErr = st; return false; }

    st = radio.setTCXO(RADIO_TCXO_VOLTAGE);
    if (st != RADIOLIB_ERR_NONE) { state.lastLoraErr = st; return false; }

    st = radio.setRegulatorDCDC();
    if (st != RADIOLIB_ERR_NONE) { state.lastLoraErr = st; return false; }

    st = radio.setDio2AsRfSwitch(true);
    if (st != RADIOLIB_ERR_NONE) { state.lastLoraErr = st; return false; }

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


static PowerManager powerManager;
static DisplayManager displayManager;
static GnssManager gnssManager;
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

static size_t buildPayload(uint8_t* out, size_t maxLen, const GpsSnapshot& gpsSnap) {
  // payload: 1 + 1 + 2 + 2 + 4 + 4 + 2 + 2 = 18 bytes
  if (maxLen < 18) return 0;

  uint8_t msgType = gpsSnap.fix ? 0x01 : 0x02;

  // diagnose altijd
  uint8_t sats = gpsSnap.sats;
  uint16_t hdop = gpsSnap.hdop; // already in hundredths
  uint16_t ageSec = (gpsSnap.ageMs >= 60000) ? 60 : (uint16_t)(gpsSnap.ageMs / 1000);

  int32_t latE5 = 0;
  int32_t lonE5 = 0;
  uint16_t spd10 = 0;

  if (gpsSnap.locationValid) {
    // let op: dit is "heeft een lat/lon", niet per se jouw fix-policy
    // Wil je ALTIJD lat/lon meesturen zodra TinyGPS het valid vindt? Zet dit aan:
    // (dan verstuur je ook "rommelige" posities, maar je ziet wél data)
    latE5 = (int32_t)llround(gpsSnap.lat * 1e5);
    lonE5 = (int32_t)llround(gpsSnap.lon * 1e5);
    spd10 = gps.speed.isValid() ? (uint16_t)lroundf(gpsSnap.speedKmph * 10.0f) : 0;
  }

  uint16_t battMv = readBatteryMv();

  out[0] = msgType;
  out[1] = sats;
  out[2] = (uint8_t)(hdop & 0xFF);
  out[3] = (uint8_t)((hdop >> 8) & 0xFF);
  out[4] = (uint8_t)(ageSec & 0xFF);
  out[5] = (uint8_t)((ageSec >> 8) & 0xFF);

  out[6]  = (uint8_t)(latE5 & 0xFF);
  out[7]  = (uint8_t)((latE5 >> 8) & 0xFF);
  out[8]  = (uint8_t)((latE5 >> 16) & 0xFF);
  out[9]  = (uint8_t)((latE5 >> 24) & 0xFF);

  out[10] = (uint8_t)(lonE5 & 0xFF);
  out[11] = (uint8_t)((lonE5 >> 8) & 0xFF);
  out[12] = (uint8_t)((lonE5 >> 16) & 0xFF);
  out[13] = (uint8_t)((lonE5 >> 24) & 0xFF);

  out[14] = (uint8_t)(spd10 & 0xFF);
  out[15] = (uint8_t)((spd10 >> 8) & 0xFF);

  out[16] = (uint8_t)(battMv & 0xFF);
  out[17] = (uint8_t)((battMv >> 8) & 0xFF);

  return 18;
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

			uint8_t payload[18];
			GpsSnapshot snap = readGpsSnapshot();
			size_t n = buildPayload(payload, sizeof(payload), snap);

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

	uint8_t payload[18];
	GpsSnapshot snap = readGpsSnapshot();
	size_t n = buildPayload(payload, sizeof(payload), snap);

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

  powerManager.begin();

  // ---------- GNSS hard power-cycle + WAKE (stabiel) ----------
  // 1) hard uit
  axp.disableALDO4();
  delay(1000);

  // 2) hard aan
  axp.setALDO4Voltage(3300);
  axp.enableALDO4();
  delay(1000);

	// WAKE pin testen: eerst HIGH hold, daarna LOW hold (met pauze)
	pinMode(GNSS_WAKE_PIN, OUTPUT);

	Serial.println("WAKE test: HIGH hold 3s");
	digitalWrite(GNSS_WAKE_PIN, HIGH);
	delay(3000);

	Serial.println("WAKE test: LOW hold 3s");
	digitalWrite(GNSS_WAKE_PIN, LOW);
	delay(3000);

	// kies daarna de variant die UART leven geeft (laatste gekozen state blijft staan)
	Serial.println("WAKE test done (leave LOW for now)");
	digitalWrite(GNSS_WAKE_PIN, LOW);
	delay(50);

  // -----------------------------------------------------------

  // jouw helper mag blijven (zet ALDO4 ook aan + WAKE LOW + prints)
  powerManager.enableGnssRail();

  // PPS: gebruik pullup voor stabiel signaal
	pinMode(GNSS_PPS_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), onPpsRise, RISING);
  Serial.println("PPS interrupt attached");

  displayManager.begin();

  // GNSS UART pas starten na power-cycle + wake
  gnssManager.begin();

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
	gnssManager.update();
	static uint32_t lastMs = 0;
	static uint32_t lastPps = 0;
	static uint32_t lastUart = 0;

	uint32_t now = millis();
	if (now - lastMs >= 1000) {
		uint32_t p = ppsCount;           // volatile read
		uint32_t ppsPerSec = p - lastPps;
		lastPps = p;

		uint32_t uartPerSec = gpsCharsPerSec; // jij berekent dit al elke sec
		// let op: gpsCharsPerSec reset je al in GnssManager.update()

		Serial.printf("GNSS pps/s=%lu uart/s=%lu | wake=%d\n",
									(unsigned long)ppsPerSec,
									(unsigned long)uartPerSec,
									(int)digitalRead(GNSS_WAKE_PIN));

		if (ppsPerSec > 0 && uartPerSec == 0) {
			Serial.println("=> GNSS leeft (PPS), maar geen UART: baud/pins/protocol");
		} else if (ppsPerSec == 0 && uartPerSec == 0) {
			Serial.println("=> GNSS lijkt uit/sleep: power/wake/hardware");
		} else if (uartPerSec > 0) {
			Serial.println("=> UART data binnen (goed).");
		}

		lastMs = now;
	}


	bootGate.update();
	updateChargeLed();
	
	if (bootGate.gateOpen && app.gate != GateState::Running) {
		app.gate = GateState::Running;
		resetJoinState();
	}

	updateJoinFlow();
	updateUplinkFlow();

	static uint32_t lastDrawMs = 0;
	uint32_t now2 = millis();
	if (now2 - lastDrawMs >= 250) {
		lastDrawMs = now2;
		uint16_t battMv = readBatteryMv();
		uint8_t battPct = readBatteryPercent();
		GpsSnapshot snap = readGpsSnapshot();
		displayManager.render(app, snap, battMv, battPct);
	}
}
