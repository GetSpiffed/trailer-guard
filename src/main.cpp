#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <esp_bt.h>

#include <SPI.h>
#include <RadioLib.h>
#include <Preferences.h>

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
static constexpr uint8_t GNSS_RX_PIN = 8;  // ESP32 RX <- GNSS TX
static constexpr uint8_t GNSS_TX_PIN = 9;  // ESP32 TX -> GNSS RX
static constexpr uint32_t GNSS_BAUD = 9600;

// ---------------- BOOT button ----------------
static constexpr uint8_t BOOT_PIN = 0; // Button1 (BOOT)

// ---------------- T-Beam S3 Supreme SX1262 pinmap ----------------
static constexpr int LORA_SCK  = 12;
static constexpr int LORA_MISO = 13;
static constexpr int LORA_MOSI = 11;
static constexpr int LORA_CS   = 10;
static constexpr int LORA_RST  = 5;
static constexpr int LORA_DIO1 = 1;
static constexpr int LORA_BUSY = 4;

// ---------------- App settings (tunable) ----------------
static constexpr uint8_t  LORAWAN_FPORT = 1;
static constexpr uint32_t UPLINK_INTERVAL_MS = 60000;
static constexpr uint32_t JOIN_RETRY_MS = 10000;
static constexpr uint32_t RESTORE_TEST_DELAY_MS = 2000;
static constexpr uint8_t  JOIN_MAX_ATTEMPTS = 8;

// ---------------- Globals ----------------
XPowersAXP2101 axp;
HardwareSerial GNSS(1);
TinyGPSPlus gps;

// GPS stats (voor OLED diagnose)
static uint32_t gpsChars = 0;
static uint32_t gpsLastStatsMs = 0;
static uint16_t gpsCharsPerSec = 0;

// OLED: SH1106 128x64, SW I2C met expliciete pins
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, DEV_SCL, DEV_SDA, U8X8_PIN_NONE);

// ---------------- LoRaWAN (RadioLib) ----------------
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
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

struct AppState {
  GateState gate = GateState::WaitingForBoot;
  JoinState joinState = JoinState::Radio;
  UplinkResult uplinkResult = UplinkResult::Idle;
  int16_t lastLoraErr = 0;
  uint32_t nextActionMs = 0;
  uint32_t nextUplinkMs = 0;
  uint32_t lastUplinkOkMs = 0;
  uint8_t joinAttempts = 0;
  bool radioReady = false;
  bool joined = false;
  bool sessionRestored = false;
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
    // ALDO4 powers the GPS; enable before GNSS.begin() and wait briefly.
    axp.setALDO4Voltage(3300);
    axp.enableALDO4();
    delay(150);
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
      case JoinState::Radio:      return "RADIO";
      case JoinState::Restore:    return "RESTORE";
      case JoinState::TestUplink: return "TESTUP";
      case JoinState::Join:       return "JOIN";
      case JoinState::Ok:         return "OK";
      case JoinState::Fail:       return "FAIL";
      default:                    return "?";
    }
  }

  static const char* loraErrLabel(int16_t err, bool uplinkOk) {
    if (uplinkOk) return "OK";
    switch (err) {
      case RADIOLIB_ERR_NONE: return "NONE";
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
      u8g2.drawStr(0, 30, "Press BOOT");
      u8g2.drawStr(0, 42, "short press");
      u8g2.drawStr(0, 60, "to start");
      u8g2.sendBuffer();
      return;
    }

    uint32_t now = millis();
    int32_t countdown = 0;
    if (state.nextActionMs > now) {
      countdown = (int32_t)((state.nextActionMs - now) / 1000);
    }

    // 2) Join state + countdown
    if (countdown > 0) {
      snprintf(line, sizeof(line), "JOIN %s T-%lds",
               joinStateText(state.joinState),
               (long)countdown);
    } else {
      snprintf(line, sizeof(line), "JOIN %s", joinStateText(state.joinState));
    }
    u8g2.drawStr(0, 30, line);

    // 3) LoRa error line
    bool uplinkOk = (state.uplinkResult == UplinkResult::Ok);
    snprintf(line, sizeof(line), "LORA %d %s",
             (int)state.lastLoraErr,
             loraErrLabel(state.lastLoraErr, uplinkOk));
    u8g2.drawStr(0, 42, line);

    // 4) GPS diagnostics
    if (gpsSnap.charsPerSec == 0) {
      snprintf(line, sizeof(line), "GPS NO UART");
    } else {
      const char* fixText = gpsSnap.fix ? "FIX" : "NOFIX";
      float hdop = gpsSnap.hdop / 100.0f;
      snprintf(line, sizeof(line), "GPS %uc/s S%u H%.1f A%lus %s",
               (unsigned)gpsSnap.charsPerSec,
               (unsigned)gpsSnap.sats,
               (double)hdop,
               (unsigned)(gpsSnap.ageMs / 1000),
               fixText);
    }
    u8g2.drawStr(0, 60, line);

    u8g2.sendBuffer();
  }
};

// ---- GNSS Manager ----
struct GnssManager {
  void begin() {
    GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  }

  void update() {
    while (GNSS.available()) {
      char c = (char)GNSS.read();
      gpsChars++;
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
struct LoRaWanManager {
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

  template <typename Node, typename Session>
  static auto setNodeSession(Node& target, const Session& session, int)
      -> decltype(target.setSession(&session), int16_t()) {
    return target.setSession(&session);
  }

  template <typename Node, typename Session>
  static auto setNodeSession(Node& target, const Session& session, long)
      -> decltype(target.setSession(session), int16_t()) {
    return target.setSession(session);
  }

  template <typename Node, typename Session>
  static int16_t setNodeSession(Node&, const Session&, ...) {
    return RADIOLIB_ERR_UNSUPPORTED;
  }

  template <typename Node, typename Session>
  static auto getNodeSession(Node& target, Session& session, int)
      -> decltype(target.getSession(&session), int16_t()) {
    return target.getSession(&session);
  }

  template <typename Node, typename Session>
  static auto getNodeSession(Node& target, Session& session, long)
      -> decltype(target.getSession(session), int16_t()) {
    return target.getSession(session);
  }

  template <typename Node, typename Session>
  static int16_t getNodeSession(Node&, Session&, ...) {
    return RADIOLIB_ERR_UNSUPPORTED;
  }

  bool initRadio(AppState& state) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);

    int16_t st = radio.begin();
    if (st != RADIOLIB_ERR_NONE) {
      state.lastLoraErr = st;
      return false;
    }

    radio.setDio2AsRfSwitch(true);
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

  bool join(AppState& state) {
    int16_t st = node.beginOTAA(JOIN_EUI, DEV_EUI, NWK_KEY, APP_KEY);
    if (st != RADIOLIB_ERR_NONE) {
      state.lastLoraErr = st;
      return false;
    }

    st = node.activateOTAA();
    if (st != RADIOLIB_ERR_NONE && st != RADIOLIB_LORAWAN_NEW_SESSION) {
      state.lastLoraErr = st;
      return false;
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
  return err == RADIOLIB_ERR_NONE || isTimeoutErr(err);
}

static size_t buildPayload(uint8_t* out, size_t maxLen, const GpsSnapshot& gpsSnap) {
  if (maxLen < 13) return 0;

  uint8_t msgType = gpsSnap.fix ? 0x01 : 0x02;
  int32_t latE5 = 0;
  int32_t lonE5 = 0;
  uint16_t spd10 = 0;

  if (gpsSnap.fix && gpsSnap.locationValid) {
    latE5 = (int32_t)llround(gpsSnap.lat * 1e5);
    lonE5 = (int32_t)llround(gpsSnap.lon * 1e5);
    spd10 = (uint16_t)lroundf(gpsSnap.speedKmph * 10.0f);
  }

  uint16_t battMv = readBatteryMv();

  out[0] = msgType;
  out[1] = (uint8_t)(latE5 & 0xFF);
  out[2] = (uint8_t)((latE5 >> 8) & 0xFF);
  out[3] = (uint8_t)((latE5 >> 16) & 0xFF);
  out[4] = (uint8_t)((latE5 >> 24) & 0xFF);

  out[5] = (uint8_t)(lonE5 & 0xFF);
  out[6] = (uint8_t)((lonE5 >> 8) & 0xFF);
  out[7] = (uint8_t)((lonE5 >> 16) & 0xFF);
  out[8] = (uint8_t)((lonE5 >> 24) & 0xFF);

  out[9]  = (uint8_t)(spd10 & 0xFF);
  out[10] = (uint8_t)((spd10 >> 8) & 0xFF);

  out[11] = (uint8_t)(battMv & 0xFF);
  out[12] = (uint8_t)((battMv >> 8) & 0xFF);

  return 13;
}

// ---- Boot short-press gate ----
struct BootGate {
  static constexpr uint32_t DEBOUNCE_MS = 25;
  static constexpr uint32_t MIN_PRESS_MS = 20;
  static constexpr uint32_t MAX_PRESS_MS = 1200;

  bool gateOpen = false;
  bool lastReadDown = false;
  bool stableDown = false;
  uint32_t lastChangeMs = 0;
  uint32_t downMs = 0;
  bool initialized = false;

  void begin() {
    pinMode(BOOT_PIN, INPUT_PULLUP);
    lastReadDown = (digitalRead(BOOT_PIN) == LOW);
    stableDown = lastReadDown;
    lastChangeMs = millis();
    initialized = true;
  }

  void update() {
    if (gateOpen) return;
    if (!initialized) begin();

    bool down = (digitalRead(BOOT_PIN) == LOW);
    uint32_t now = millis();

    if (down != lastReadDown) {
      lastChangeMs = now;
      lastReadDown = down;
    }

    if (now - lastChangeMs >= DEBOUNCE_MS && down != stableDown) {
      stableDown = down;
      if (stableDown) {
        downMs = now;
      } else {
        uint32_t pressMs = now - downMs;
        if (pressMs >= MIN_PRESS_MS && pressMs <= MAX_PRESS_MS) {
          gateOpen = true;
        }
      }
    }
  }
};

static BootGate bootGate;

static void resetJoinState() {
  app.joinState = JoinState::Radio;
  app.uplinkResult = UplinkResult::Idle;
  app.lastLoraErr = RADIOLIB_ERR_NONE;
  app.nextActionMs = millis();
  app.nextUplinkMs = 0;
  app.lastUplinkOkMs = 0;
  app.joinAttempts = 0;
  app.radioReady = false;
  app.joined = false;
  app.sessionRestored = false;
}

static void updateJoinFlow() {
  if (app.gate != GateState::Running) return;

  uint32_t now = millis();

  switch (app.joinState) {
    case JoinState::Radio: {
      app.radioReady = lorawanManager.initRadio(app);
      if (app.radioReady) {
        app.joinState = JoinState::Restore;
        app.nextActionMs = now;
      } else {
        app.joinState = JoinState::Fail;
      }
      break;
    }
    case JoinState::Restore: {
      LoRaWANSession session;
      if (lorawanManager.loadSession(session) &&
          lorawanManager.restoreSession(app, session)) {
        app.sessionRestored = true;
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
      uint8_t payload[16];
      GpsSnapshot snap = readGpsSnapshot();
      size_t n = buildPayload(payload, sizeof(payload), snap);
      int16_t st = lorawanManager.sendUplink(payload, n, LORAWAN_FPORT);
      app.lastLoraErr = st;
      app.uplinkResult = isUplinkOkError(st) ? UplinkResult::Ok : UplinkResult::Fail;
      if (app.uplinkResult == UplinkResult::Ok) {
        app.joined = true;
        app.joinState = JoinState::Ok;
        LoRaWANSession saved;
        if (lorawanManager.fetchSession(app, saved)) {
          lorawanManager.saveSession(saved);
        }
        app.nextUplinkMs = now + UPLINK_INTERVAL_MS;
      } else if (isSessionInvalidError(st)) {
        lorawanManager.clearSession();
        app.sessionRestored = false;
        app.joinState = JoinState::Join;
        app.nextActionMs = now;
      } else {
        app.joinState = JoinState::Join;
        app.nextActionMs = now + JOIN_RETRY_MS;
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
        LoRaWANSession session;
        if (lorawanManager.fetchSession(app, session)) {
          lorawanManager.saveSession(session);
        }
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

  uint8_t payload[16];
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
  powerManager.begin();
  powerManager.enableGnssRail();
  displayManager.begin();
  gnssManager.begin();
  disableRadios();

  bootGate.begin();
  resetJoinState();
}

void loop() {
  gnssManager.update();
  bootGate.update();

  if (bootGate.gateOpen && app.gate != GateState::Running) {
    app.gate = GateState::Running;
    resetJoinState();
  }

  updateJoinFlow();
  updateUplinkFlow();

  static uint32_t lastDrawMs = 0;
  uint32_t now = millis();
  if (now - lastDrawMs >= 250) {
    lastDrawMs = now;
    uint16_t battMv = readBatteryMv();
    uint8_t battPct = readBatteryPercent();
    GpsSnapshot snap = readGpsSnapshot();
    displayManager.render(app, snap, battMv, battPct);
  }
}
