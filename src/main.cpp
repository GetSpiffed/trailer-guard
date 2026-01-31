// Core app: LoRaWAN + power + OLED + GPS
#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_bt.h>

#include <SPI.h>
#include <RadioLib.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>

#include "secrets.h"

// ---------------- Pinmap ----------------
// PMU bus
static constexpr uint8_t PMU_SDA = 42;
static constexpr uint8_t PMU_SCL = 41;

// OLED + sensors bus
static constexpr uint8_t DEV_SDA = 17;
static constexpr uint8_t DEV_SCL = 18;

// BOOT button
static constexpr uint8_t BOOT_PIN = 0; // Button1 (BOOT)
static constexpr uint32_t FORCE_OTAA_MIN_MS = 2000;
static constexpr uint32_t FORCE_OTAA_MAX_MS = 6000;

// T-Beam S3 Supreme SX1262 pinmap
static constexpr int LORA_SCK = 12;
static constexpr int LORA_MISO = 13;
static constexpr int LORA_MOSI = 11;
static constexpr int LORA_CS = 10;
static constexpr int LORA_RST = 5;
static constexpr int LORA_DIO1 = 1;
static constexpr int LORA_BUSY = 4;

// GPS (L76K)
static constexpr uint8_t GPS_RX_PIN = 9;	 // ESP RX <- GNSS TX
static constexpr uint8_t GPS_TX_PIN = 8;	 // ESP TX -> GNSS RX
static constexpr uint8_t GPS_EN_PIN = 7;	 // GPS_EN
static constexpr uint8_t GPS_PPS_PIN = 6;	// PPS (optional)
static constexpr uint32_t GPS_BAUD = 9600;

// ---------------- App settings (tunable) ----------------
static constexpr uint8_t LORAWAN_FPORT = 1;
static constexpr uint32_t UPLINK_INTERVAL_MS = 60000;
static constexpr uint32_t JOIN_RETRY_MS = 10000;
static constexpr uint32_t RESTORE_TEST_DELAY_MS = 2000;
static constexpr uint8_t JOIN_MAX_ATTEMPTS = 8;
static constexpr uint8_t RADIO_INIT_MAX_ATTEMPTS = 5;
static constexpr float RADIO_TCXO_VOLTAGE = 1.6f;

// Tracker placeholders
static constexpr float MOVE_SPEED_MPS = 1.5f;
static constexpr float MOVE_DISTANCE_M = 30.0f;
static constexpr uint32_t KEEPALIVE_INTERVAL_MS = 86400000;

// ---------------- Globals ----------------
TwoWire Wire1_PMU(0);
XPowersAXP2101 axp;

// OLED: SH1106 128x64, SW I2C met expliciete pins
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, DEV_SCL, DEV_SDA, U8X8_PIN_NONE);

// GPS
HardwareSerial SerialGPS(2);
TinyGPSPlus gps;

// ---------------- LoRaWAN (RadioLib) ----------------
SX1262 radio(new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY));

LoRaWANNode node(&radio, &EU868);
using LoRaWANSession = LoRaWANSchemeSession_t;
#ifndef RADIOLIB_ERR_UNSUPPORTED
#define RADIOLIB_ERR_UNSUPPORTED RADIOLIB_ERR_UNKNOWN
#endif

// ---------------- Debug ----------------
#define GPS_DEBUG 0

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
	BootAction bootAction = BootAction::Normal;
	bool forceOtaaThisBoot = false;
};

static AppState app;

struct CurrentFix {
	bool hasFix = false;
	bool hasSpeed = false;
	bool hasCourse = false;
	float lat = 0.0f;
	float lon = 0.0f;
	float speedMps = 0.0f;
	float courseDeg = 0.0f;
	int32_t sats = -1;
	float hdop = -1.0f;
	uint32_t lastFixMs = 0;
	uint32_t lastGpsByteMs = 0;
	char lastNmeaLine[220] = {0};
	bool hadNmeaLine = false;
	float anchorLat = 0.0f;
	float anchorLon = 0.0f;
	bool anchorValid = false;
};

static CurrentFix currentFix;

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

		axp.setALDO4Voltage(3300);
		axp.enableALDO4();

		axp.setALDO1Voltage(3300);
		axp.enableALDO1();

		axp.setALDO3Voltage(3300);
		axp.enableALDO3();

		return true;
	}
};

// ---- GPS init ----
static void drainGpsUart(uint32_t ms = 150) {
	uint32_t t0 = millis();
	while (millis() - t0 < ms) {
		while (SerialGPS.available()) (void)SerialGPS.read();
		delay(1);
	}
}

static void gpsEnableHigh() {
	pinMode(GPS_EN_PIN, OUTPUT);
	digitalWrite(GPS_EN_PIN, HIGH);
	delay(350);
}

static void gpsBegin() {
	SerialGPS.end();
	delay(30);
	SerialGPS.setRxBufferSize(4096);
	SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
	delay(80);
	drainGpsUart(200);
}

static void gpsStopNmea() {
	SerialGPS.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
	delay(60);
	drainGpsUart(120);
}

static void gpsInitSequence() {
	gpsStopNmea();
	SerialGPS.write("$PCAS06,0*1B\r\n");
	delay(80);
	SerialGPS.write("$PCAS04,5*1C\r\n");
	delay(120);
	SerialGPS.write("$PCAS11,3*1E\r\n");
	delay(120);
	SerialGPS.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
	delay(120);
}

static void gpsStart() {
	pinMode(GPS_PPS_PIN, INPUT);
	gpsEnableHigh();
	gpsBegin();
	gpsInitSequence();
}

// ---- GPS pump + parse ----
static void updateGpsFix() {
	currentFix.hasFix = gps.location.isValid();
	currentFix.lat = currentFix.hasFix ? gps.location.lat() : currentFix.lat;
	currentFix.lon = currentFix.hasFix ? gps.location.lng() : currentFix.lon;
	currentFix.sats = gps.satellites.isValid() ? gps.satellites.value() : -1;
	currentFix.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : -1.0f;
	currentFix.hasSpeed = gps.speed.isValid();
	currentFix.speedMps = currentFix.hasSpeed ? gps.speed.mps() : 0.0f;
	currentFix.hasCourse = gps.course.isValid();
	currentFix.courseDeg = currentFix.hasCourse ? gps.course.deg() : 0.0f;
	if (currentFix.hasFix && gps.location.isUpdated()) {
		currentFix.lastFixMs = millis();
		if (!currentFix.anchorValid) {
			currentFix.anchorLat = currentFix.lat;
			currentFix.anchorLon = currentFix.lon;
			currentFix.anchorValid = true;
		}
	}
}

static void pumpGpsUartAndParse() {
	static char line[220];
	static uint16_t idx = 0;

	while (SerialGPS.available()) {
		char c = (char)SerialGPS.read();
		gps.encode(c);
		currentFix.lastGpsByteMs = millis();

		if (c == '\n') {
			line[idx] = 0;
			if (idx > 6) {
				strncpy(currentFix.lastNmeaLine, line, sizeof(currentFix.lastNmeaLine) - 1);
				currentFix.lastNmeaLine[sizeof(currentFix.lastNmeaLine) - 1] = 0;
				currentFix.hadNmeaLine = true;
			}
			idx = 0;
		} else if (c != '\r') {
			if (idx < sizeof(line) - 1) line[idx++] = c;
		}
	}

	updateGpsFix();

	uint32_t now = millis();
	if (currentFix.lastGpsByteMs != 0 && (now - currentFix.lastGpsByteMs) > 5000) {
#if GPS_DEBUG
		Serial.println("GPS: 5s stil, re-init");
#endif
		gpsBegin();
		gpsInitSequence();
		currentFix.lastGpsByteMs = now;
	}
}

// ---- OLED 1-line GPS renderer ----
static void renderGpsLine(char* out, size_t outLen) {
	int sats = currentFix.sats;
	float hdop = currentFix.hdop;
	uint32_t now = millis();
	bool noNmea = currentFix.lastGpsByteMs == 0 || (now - currentFix.lastGpsByteMs) > 5000;

	if (currentFix.hasFix) {
		snprintf(out, outLen, "FIX lat=%.6f lon=%.6f sats=%d hdop=%.1f",
				 (double)currentFix.lat, (double)currentFix.lon, sats, (double)hdop);
		return;
	}

	if (noNmea) {
		snprintf(out, outLen, "NO FIX sats=%d hdop=%.1f NO NMEA", sats, (double)hdop);
		return;
	}

	snprintf(out, outLen, "NO FIX sats=%d hdop=%.1f", sats, (double)hdop);
}

// ---- DisplayManager (OLED) ----
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

	struct ScrollText {
		int16_t x = 0;
		uint32_t lastStepMs = 0;

		// stabiele buffer
		char buf[96] = {0};
		uint32_t lastAcceptMs = 0;

		// widths in pixels
		int16_t textW = 0;
		int16_t sepW = 0;
		bool scrolling = false;
	};

	static ScrollText gpsScroll;

	static void acceptScrollText(ScrollText& st, const char* newText, U8G2& u8g2,
															 const char* sep = " - ",
															 uint32_t minUpdateMs = 1000) {
		if (!newText) newText = "";

		// gelijk? niks doen
		if (strncmp(st.buf, newText, sizeof(st.buf)) == 0) return;

		uint32_t now = millis();

		// throttle
		if (st.lastAcceptMs != 0 && (now - st.lastAcceptMs) < minUpdateMs) {
			return;
		}

		strncpy(st.buf, newText, sizeof(st.buf) - 1);
		st.buf[sizeof(st.buf) - 1] = 0;
		st.lastAcceptMs = now;

		st.textW = u8g2.getStrWidth(st.buf);
		st.sepW	= u8g2.getStrWidth(sep);
		int16_t screenW = u8g2.getDisplayWidth();
		bool scrollingNow = (st.textW > screenW);

		// als hij net gaat scrollen: begin vanaf start
		if (!st.scrolling && scrollingNow) {
			st.x = 0;
			st.lastStepMs = now;
		}

		st.scrolling = scrollingNow;
	}

	void showBootHold(uint32_t heldMs, bool inWindow) {
		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_6x10_tf);

		char line1[32];
		char line2[32];

		snprintf(line1, sizeof(line1), "BOOT hold: %lums", (unsigned long)heldMs);
		if (inWindow) {
			snprintf(line2, sizeof(line2), "Release: FORCE OTAA");
		} else {
			snprintf(line2, sizeof(line2), "Hold 2-6s for OTAA");
		}

		u8g2.drawStr(0, 24, line1);
		u8g2.drawStr(0, 40, line2);

		// mini “progress” balkje 0..6s
		const uint16_t cap = FORCE_OTAA_MAX_MS;
		uint16_t w = (heldMs >= cap) ? 128 : (uint16_t)((heldMs * 128UL) / cap);
		u8g2.drawFrame(0, 52, 128, 10);
		u8g2.drawBox(1, 53, (w > 2 ? w - 2 : 0), 8);

		// extra statusregel "READY"
		if (heldMs >= FORCE_OTAA_MIN_MS && heldMs <= FORCE_OTAA_MAX_MS) {
			u8g2.drawStr(0, 12, "FORCE OTAA: READY");
		} else if (heldMs > FORCE_OTAA_MAX_MS) {
			u8g2.drawStr(0, 12, "FORCE OTAA: TOO LONG");
		} else {
			u8g2.drawStr(0, 12, "FORCE OTAA: NOT READY");
		}

		u8g2.sendBuffer();
	}


	static void drawMarqueeLoop(U8G2& u8g2, int y, ScrollText& st,
															const char* sep = " - ",
															uint8_t speedPx = 1,
															uint16_t stepMs = 40) {
		const char* text = st.buf;
		if (!text || !text[0]) return;

		const int16_t screenW = u8g2.getDisplayWidth();
		const int16_t textW = st.textW > 0 ? st.textW : u8g2.getStrWidth(text);

		// past -> statisch
		if (textW <= screenW) {
			st.x = 0;
			u8g2.drawStr(0, y, text);
			return;
		}

		const int16_t sepW = st.sepW > 0 ? st.sepW : u8g2.getStrWidth(sep);
		const int16_t periodW = textW + sepW;	 // 1 cyclus: TEXT + SEP
		uint32_t now = millis();

		if (now - st.lastStepMs >= stepMs) {
			st.lastStepMs = now;
			st.x -= (int16_t)speedPx;

			// modulo loop (aaneengesloten)
			if (st.x <= -periodW) {
				st.x += periodW;
			}
		}

		// Teken twee keer zodat het scherm altijd gevuld blijft:
		// [TEXT][SEP][TEXT][SEP]...
		int16_t x0 = st.x;

		// 1) eerste tekst
		u8g2.drawStr(x0, y, text);

		// 2) separator direct erachter
		u8g2.drawStr(x0 + textW, y, sep);

		// 3) tweede tekst (begin) voor de loop
		u8g2.drawStr(x0 + periodW, y, text);

		// 4) (optioneel) nog een sep, helpt bij heel brede fonts
		u8g2.drawStr(x0 + periodW + textW, y, sep);
	}

	void render(const AppState& state, uint16_t battMv, uint8_t battPct, const char* gpsLine) {
		(void)battMv;
		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_6x10_tf);

		char line[32];

		// 1) Battery
		float battV = readBatteryVoltageV();
		if (battV > 0.0f) {
			snprintf(line, sizeof(line), "BAT %.2fV %u%%", (double)battV, (unsigned)battPct);
		} else {
			snprintf(line, sizeof(line), "BAT --.-V %u%%", (unsigned)battPct);
		}
		u8g2.drawStr(0, 12, line);

		if (state.gate != GateState::Running) {
			u8g2.drawStr(0, 30, "LORA: press BOOT");
		}

		uint32_t now = millis();
		int32_t countdown = 0;
		if (state.nextActionMs > now) {
			countdown = (int32_t)((state.nextActionMs - now) / 1000);
		}

		// 2) Join state + countdown
		uint8_t attempt = (state.joinState == JoinState::Radio) ? state.radioInitAttempts : state.joinAttempts;
		uint8_t maxAttempt = (state.joinState == JoinState::Radio) ? RADIO_INIT_MAX_ATTEMPTS : JOIN_MAX_ATTEMPTS;

		if (countdown > 0) {
			snprintf(line, sizeof(line), "JOIN %s %u/%u T-%lds",
							 joinStateText(state.joinState),
							 (unsigned)attempt, (unsigned)maxAttempt, (long)countdown);
		} else {
			snprintf(line, sizeof(line), "JOIN %s %u/%u",
							 joinStateText(state.joinState),
							 (unsigned)attempt, (unsigned)maxAttempt);
		}


		if (state.forceOtaaThisBoot) {
			size_t L = strlen(line);
			if (L < sizeof(line) - 7) strcat(line, " FORCE");
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

		// 3) GPS line: aaneengesloten marquee loop met " <<< " ertussen
		if (gpsLine) {
			acceptScrollText(gpsScroll, gpsLine, u8g2, " <<< ", 200);
		}
		drawMarqueeLoop(u8g2, 54, gpsScroll, " <<< ", 3, 40);

		u8g2.sendBuffer();
	}

	void showModeMessage(const char* line) {
		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_6x10_tf);
		u8g2.drawStr(0, 30, line);
		u8g2.sendBuffer();
	}
};



DisplayManager::ScrollText DisplayManager::gpsScroll;
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

static bool isMoving(const CurrentFix& fix) {
	if (!fix.hasFix) return false;
	if (fix.hasSpeed && fix.speedMps > MOVE_SPEED_MPS) return true;
	if (fix.anchorValid) {
		double distance = TinyGPSPlus::distanceBetween(
			fix.anchorLat,
			fix.anchorLon,
			fix.lat,
			fix.lon);
		if (distance > MOVE_DISTANCE_M) return true;
	}
	return false;
}

static uint32_t nextUplinkIntervalMs(const CurrentFix& fix) {
	return isMoving(fix) ? UPLINK_INTERVAL_MS : KEEPALIVE_INTERVAL_MS;
}

static size_t buildPayload(uint8_t* out, size_t maxLen, const CurrentFix& fix) {
	// Compact payload mapping (little endian):
	// Byte0: flags (bit0=fix, bit1=moving, bit2=speed/course)
	// Byte1-2: battery mV (uint16)
	// FIX:
	// Byte3-6: lat * 1e6 (int32)
	// Byte7-10: lon * 1e6 (int32)
	// Byte11: sats (uint8)
	// Byte12-13: hdop * 10 (uint16)
	// Byte14-15: speed * 10 (uint16, m/s) [optional]
	// Byte16-17: course * 10 (uint16, deg) [optional]
	// NO FIX:
	// Byte3: sats (uint8)
	// Byte4-5: hdop * 10 (uint16)
	// Byte6-7: last fix age (uint16, seconds, 0xFFFF unknown)
	if (maxLen < 3) return 0;

	uint8_t flags = 0;
	if (fix.hasFix) flags |= 0x01;
	if (isMoving(fix)) flags |= 0x02;
	if (fix.hasSpeed && fix.hasCourse) flags |= 0x04;

	uint16_t battMv = readBatteryMv();
	out[0] = flags;
	out[1] = (uint8_t)(battMv & 0xFF);
	out[2] = (uint8_t)((battMv >> 8) & 0xFF);

	if (fix.hasFix) {
		if (maxLen < 14) return 3;
		int32_t lat = (int32_t)lroundf(fix.lat * 1000000.0f);
		int32_t lon = (int32_t)lroundf(fix.lon * 1000000.0f);
		uint8_t sats = (fix.sats >= 0 && fix.sats <= 255) ? (uint8_t)fix.sats : 0;
		uint16_t hdop10 = (fix.hdop >= 0.0f) ? (uint16_t)lroundf(fix.hdop * 10.0f) : 0;

		memcpy(out + 3, &lat, sizeof(lat));
		memcpy(out + 7, &lon, sizeof(lon));
		out[11] = sats;
		out[12] = (uint8_t)(hdop10 & 0xFF);
		out[13] = (uint8_t)((hdop10 >> 8) & 0xFF);

		size_t len = 14;
		if (flags & 0x04) {
			if (maxLen < 18) return len;
			uint16_t speed10 = (uint16_t)lroundf(fix.speedMps * 10.0f);
			uint16_t course10 = (uint16_t)lroundf(fix.courseDeg * 10.0f);
			out[14] = (uint8_t)(speed10 & 0xFF);
			out[15] = (uint8_t)((speed10 >> 8) & 0xFF);
			out[16] = (uint8_t)(course10 & 0xFF);
			out[17] = (uint8_t)((course10 >> 8) & 0xFF);
			len = 18;
		}
		return len;
	}

	if (maxLen < 8) return 3;
	uint8_t sats = (fix.sats >= 0 && fix.sats <= 255) ? (uint8_t)fix.sats : 0;
	uint16_t hdop10 = (fix.hdop >= 0.0f) ? (uint16_t)lroundf(fix.hdop * 10.0f) : 0;
	uint16_t ageSec = 0xFFFF;
	if (fix.lastFixMs > 0) {
		uint32_t ageMs = millis() - fix.lastFixMs;
		ageSec = (ageMs > 0xFFFFu * 1000u) ? 0xFFFF : (uint16_t)(ageMs / 1000u);
	}
	out[3] = sats;
	out[4] = (uint8_t)(hdop10 & 0xFF);
	out[5] = (uint8_t)((hdop10 >> 8) & 0xFF);
	out[6] = (uint8_t)(ageSec & 0xFF);
	out[7] = (uint8_t)((ageSec >> 8) & 0xFF);
	return 8;
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

static BootAction detectBootActionWithOled() {
	pinMode(BOOT_PIN, INPUT_PULLUP);

	// Niet ingedrukt: normaal
	if (digitalRead(BOOT_PIN) != LOW) return BootAction::Normal;

	uint32_t startMs = millis();
	uint32_t lastDraw = 0;

	while (digitalRead(BOOT_PIN) == LOW) {
		uint32_t now = millis();
		uint32_t held = now - startMs;

		bool inWindow = (held >= FORCE_OTAA_MIN_MS && held <= FORCE_OTAA_MAX_MS);

		if (now - lastDraw >= 80) { // vlot maar niet te druk
			lastDraw = now;
			displayManager.showBootHold(held, inWindow);
		}

		if (held > FORCE_OTAA_MAX_MS) {
			// te lang -> normaal
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

			uint8_t payload[24];
			size_t n = buildPayload(payload, sizeof(payload), currentFix);

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

				app.nextUplinkMs = now + nextUplinkIntervalMs(currentFix);
				currentFix.anchorLat = currentFix.lat;
				currentFix.anchorLon = currentFix.lon;
				currentFix.anchorValid = currentFix.hasFix;
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
				app.nextUplinkMs = now + nextUplinkIntervalMs(currentFix);
				currentFix.anchorLat = currentFix.lat;
				currentFix.anchorLon = currentFix.lon;
				currentFix.anchorValid = currentFix.hasFix;
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

	uint8_t payload[24];
	size_t n = buildPayload(payload, sizeof(payload), currentFix);

	int16_t st = lorawanManager.sendUplink(payload, n, LORAWAN_FPORT);
	app.lastLoraErr = st;
	app.uplinkResult = isUplinkOkError(st) ? UplinkResult::Ok : UplinkResult::Fail;

	if (app.uplinkResult == UplinkResult::Ok) {
		app.lastUplinkOkMs = now;
		LoRaWANSession session;
		if (lorawanManager.fetchSession(app, session)) {
			lorawanManager.saveSession(session);
		}
		currentFix.anchorLat = currentFix.lat;
		currentFix.anchorLon = currentFix.lon;
		currentFix.anchorValid = currentFix.hasFix;
	} else if (isSessionInvalidError(st)) {
		lorawanManager.clearSession();
		app.joined = false;
		app.sessionRestored = false;
		app.joinState = JoinState::Join;
		app.nextActionMs = now + JOIN_RETRY_MS;
	}

	app.nextUplinkMs = now + nextUplinkIntervalMs(currentFix);
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
	gpsStart();

	BootAction bootAction = detectBootActionWithOled();
	app.bootAction = bootAction;
	app.forceOtaaThisBoot = (bootAction == BootAction::ForceOTAA);

	if (bootAction == BootAction::ForceOTAA) {
		lorawanManager.clearSession();

		LoRaWANSession tmp;
		bool stillThere = lorawanManager.loadSession(tmp);
		displayManager.showModeMessage(
				stillThere ? "CLEAR FAIL?!" : "SESSION CLEARED"
		);

		delay(1200);
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

	pumpGpsUartAndParse();
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
		char gpsLine[64];
		renderGpsLine(gpsLine, sizeof(gpsLine));
		displayManager.render(app, battMv, battPct, gpsLine);
	}
}
