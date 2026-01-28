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

// ---------------- PMU bus ----------------
TwoWire I2C_PMU(0);
static constexpr uint8_t PMU_SDA = 42;
static constexpr uint8_t PMU_SCL = 41;

// ---------------- OLED + sensors bus ----------------
static constexpr uint8_t DEV_SDA = 17;
static constexpr uint8_t DEV_SCL = 18;

// ---------------- GNSS UART ----------------
static constexpr uint8_t GNSS_RX_PIN = 8;
static constexpr uint8_t GNSS_TX_PIN = 9;
static constexpr uint32_t GNSS_BAUD = 9600;

// ---------------- BOOT button ----------------
static constexpr uint8_t BOOT_PIN = 0; // Button1 (BOOT)

// ---------------- LoRaWAN keys (TTN) ----------------
static const uint64_t JOIN_EUI = 0x0000000000000000ULL;
static const uint64_t DEV_EUI  = 0x70B3D57ED0075678ULL;

static const uint8_t APP_KEY[16] = {
  0x74, 0x51, 0xDC, 0xC4,
  0xD4, 0xCC, 0xA8, 0xA8,
  0xC4, 0xF3, 0x11, 0xEB,
  0x09, 0xCD, 0x73, 0x2D
};

static const uint8_t NWK_KEY[16] = {
  0x94, 0x60, 0xA0, 0x60,
  0xE8, 0x29, 0x60, 0x3D,
  0x75, 0x2F, 0xC0, 0xFF,
  0xDC, 0x6A, 0xA5, 0xF0
};

// ---------------- T-Beam S3 Supreme SX1262 pinmap ----------------
static constexpr int LORA_SCK  = 12;
static constexpr int LORA_MISO = 13;
static constexpr int LORA_MOSI = 11;
static constexpr int LORA_CS   = 10;
static constexpr int LORA_RST  = 5;
static constexpr int LORA_DIO1 = 1;
static constexpr int LORA_BUSY = 4;

// ---------------- App settings ----------------
static constexpr uint8_t  LORAWAN_FPORT   = 1;
static constexpr uint32_t CHG_LED_INTERVAL_MS = 30000;
static constexpr uint32_t CHG_LED_PULSE_MS = 120;
static constexpr uint8_t  RESTORE_UPLINK_MAX_TRIES = 2;
static constexpr uint32_t RESTORE_UPLINK_RETRY_MS = 8000;

// ---------------- Globals ----------------
XPowersAXP2101 axp;
HardwareSerial GNSS(1);
TinyGPSPlus gps;

// OLED: SH1106 128x64, SW I2C met expliciete pins
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, DEV_SCL, DEV_SDA, U8X8_PIN_NONE);

uint32_t lastDrawMs = 0;

// ---------------- LoRaWAN (RadioLib) ----------------
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
LoRaWANNode node(&radio, &EU868);

enum class JoinStatus : uint8_t {
  Boot,
  RadioInit,
  Restoring,
  RestoreOk,
  RestoreFail,
  Joining,
  JoinOk,
  JoinFail
};

enum class UplinkStatus : uint8_t {
  Idle,
  Ok,
  Fail
};

// app gate state
enum class RunGate : uint8_t {
  WaitingForBoot,
  Starting,
  Running
};

struct AppState {
  JoinStatus joinStatus = JoinStatus::Boot;
  UplinkStatus uplinkStatus = UplinkStatus::Idle;
  int16_t lastLoraErr = 0;
  uint32_t lastUplinkOkMs = 0;
  uint32_t nextJoinAttemptMs = 0;
  bool radioReady = false;
  bool joined = false;
  bool uplinkSent = false;
  bool sessionRestored = false;
  bool sessionChecked = false;
  bool joinAttempted = false;
  uint8_t restoreUplinkAttempts = 0;
  uint32_t nextRestoreUplinkMs = 0;

  RunGate gate = RunGate::WaitingForBoot;
};

static AppState app;
static uint32_t nextChgLedBlinkMs = 0;
static uint32_t chgLedPulseEndMs = 0;

// ---------- PMU helpers ----------
static void setChgLed(bool on) {
  axp.setChargingLedMode(on ? XPOWERS_CHG_LED_ON : XPOWERS_CHG_LED_OFF);
}

static int chargerStatusRaw() {
  // meestal: 0=not charging, 1=charging, 2=done
  return axp.getChargerStatus();
}

static const char* chargerStatusText(int st) {
  switch (st) {
    case 0: return "NO";
    case 1: return "CHG";
    case 2: return "DONE";
    default: return "?";
  }
}

// LED constant aan bij CHG of DONE (USB eraan + klaar) om te zien dat USB "leeft"
static bool isChargingOrDone() {
  int st = chargerStatusRaw();
  return (st == 1) || (st == 2);
}

static uint16_t readBatteryMv() {
  if (!axp.isBatteryConnect()) return 0;

  // XPowersLib builds verschillen: soms V, soms mV, soms 10mV units
  float v = axp.getBattVoltage();

  if (v > 0 && v < 10.0f) {
    return (uint16_t)lroundf(v * 1000.0f);   // V -> mV
  } else if (v >= 10.0f && v < 1000.0f) {
    return (uint16_t)lroundf(v * 10.0f);     // 10mV units -> mV
  } else {
    return (uint16_t)lroundf(v);             // al mV
  }
}

// VBUS / USB detect + VBUS spanning
static bool hasVbus() {
  return axp.isVbusIn();
}

static uint16_t readVbusMv() {
  float v = axp.getVbusVoltage();
  if (v > 0 && v < 10.0f) return (uint16_t)lroundf(v * 1000.0f);
  return (uint16_t)lroundf(v);
}

// Battery current: probeer charge current. Als dit bij jou niet compilet,
// vervang deze ene regel door een methode die wél bestaat in jouw XPowersLib build.
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
    delay(50);

    axp.setALDO4Voltage(3300);
    axp.enableALDO4();

    return true;
  }
};

// ---- DisplayManager (OLED) ----
struct DisplayManager {
  void begin() {
    u8g2.begin();
    u8g2.setFont(u8g2_font_6x10_tf);
  }

  static const char* joinStatusText(JoinStatus status) {
    switch (status) {
      case JoinStatus::Boot:      return "BOOT";
      case JoinStatus::RadioInit: return "RADIO INIT";
      case JoinStatus::Restoring: return "RESTORING";
      case JoinStatus::RestoreOk: return "RESTORE OK";
      case JoinStatus::RestoreFail: return "RESTORE FAIL";
      case JoinStatus::Joining:   return "JOINING...";
      case JoinStatus::JoinOk:    return "JOIN OK";
      case JoinStatus::JoinFail:  return "JOIN FAIL";
      default:                    return "";
    }
  }

  // Titelregel opgeofferd; we gebruiken 2 regels voor power debug
  void render(const AppState& state, TinyGPSPlus& gps,
            uint16_t battMv, uint8_t battPct,
            int chgRaw, bool vbus, uint16_t vbusMv) {

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);

    // 1 regel: "BAT 3875mV 82%"
    char line[32];
    snprintf(line, sizeof(line), "BAT %umV %u%%",
             (unsigned)battMv,
             (unsigned)battPct);
    u8g2.drawStr(0, 12, line);


    // gate screen
    if (state.gate != RunGate::Running) {
      u8g2.drawStr(0, 40, "Press BOOT to start");
      u8g2.drawStr(0, 58, "(short press)");
      u8g2.sendBuffer();
      return;
    }

    snprintf(line, sizeof(line), "JOIN:%s", joinStatusText(state.joinStatus));
    u8g2.drawStr(0, 34, line);

    snprintf(line, sizeof(line), "ERR:%d", (int)state.lastLoraErr);
    u8g2.drawStr(0, 46, line);

    int sat = gps.satellites.isValid() ? gps.satellites.value() : 0;
    if (gps.location.isValid()) {
      snprintf(line, sizeof(line), "GPS:FIX(%d)", sat);
    } else {
      snprintf(line, sizeof(line), "GPS:NO(%d)", sat);
    }
    u8g2.drawStr(0, 58, line);

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
      gps.encode((char)GNSS.read());
    }
  }
};

static size_t buildPayload(uint8_t* out, size_t maxLen);

// ---- LoRaWAN Manager ----
struct LoRaWanManager {
  static bool loadSession(LoRaWANSession_t& session) {
    Preferences prefs;
    if (!prefs.begin("lorawan", true)) return false;
    bool valid = prefs.getBool("valid", false);
    size_t len = prefs.getBytesLength("session");
    if (!valid || len != sizeof(session)) {
      prefs.end();
      return false;
    }
    size_t read = prefs.getBytes("session", &session, sizeof(session));
    prefs.end();
    return read == sizeof(session);
  }

  static bool saveSession(const LoRaWANSession_t& session) {
    Preferences prefs;
    if (!prefs.begin("lorawan", false)) return false;
    prefs.putBool("valid", true);
    size_t written = prefs.putBytes("session", &session, sizeof(session));
    prefs.end();
    return written == sizeof(session);
  }

  static void clearSession() {
    Preferences prefs;
    if (!prefs.begin("lorawan", false)) return;
    prefs.remove("valid");
    prefs.remove("session");
    prefs.end();
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
    state.lastLoraErr = 0;
    return true;
  }

  bool restoreSession(AppState& state, const LoRaWANSession_t& session) {
    int16_t st = node.setSession(&session);
    if (st != RADIOLIB_ERR_NONE) {
      state.lastLoraErr = st;
      return false;
    }
    state.lastLoraErr = 0;
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

    state.lastLoraErr = 0;
    return true;
  }

  bool fetchSession(AppState& state, LoRaWANSession_t& session) {
    int16_t st = node.getSession(&session);
    if (st != RADIOLIB_ERR_NONE) {
      state.lastLoraErr = st;
      return false;
    }
    state.lastLoraErr = 0;
    return true;
  }

  void sendOnce(AppState& state) {
    uint8_t payload[16];
    size_t n = buildPayload(payload, sizeof(payload));
    if (n == 0) return;

    int16_t st = node.sendReceive(payload, n, LORAWAN_FPORT);
    state.lastLoraErr = st;
    if (st == RADIOLIB_ERR_NONE) {
      state.lastLoraErr = 0;
      state.lastUplinkOkMs = millis();
      state.uplinkStatus = UplinkStatus::Ok;
    } else {
      state.uplinkStatus = UplinkStatus::Fail;
    }
  }
};

static PowerManager powerManager;
static DisplayManager displayManager;
static GnssManager gnssManager;
static LoRaWanManager lorawanManager;

// payload: 13 bytes
static size_t buildPayload(uint8_t* out, size_t maxLen) {
  if (maxLen < 13) return 0;

  bool hasFix = gps.location.isValid();
  uint8_t msgType = hasFix ? 0x01 : 0x02;

  int32_t latE5 = 0;
  int32_t lonE5 = 0;
  uint16_t spd10 = 0;

  if (hasFix) {
    latE5 = (int32_t)llround(gps.location.lat() * 1e5);
    lonE5 = (int32_t)llround(gps.location.lng() * 1e5);

    if (gps.speed.isValid()) {
      float kmh = gps.speed.kmph();
      spd10 = (uint16_t)lroundf(kmh * 10.0f);
    }
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

// ---- App State / Flow ----
static void disableRadios() {
  WiFi.mode(WIFI_OFF);
  btStop();
}

// gate state machine (short press)
static void updateStartGate() {
  static bool inited = false;
  static bool wasDown = false;
  static uint32_t downMs = 0;

  if (!inited) {
    pinMode(BOOT_PIN, INPUT_PULLUP);
    inited = true;
  }

  if (app.gate == RunGate::Running) return;

  bool down = (digitalRead(BOOT_PIN) == LOW); // actief-low

  // detecteer "net ingedrukt"
  if (down && !wasDown) {
    downMs = millis();
  }

  // detecteer "net losgelaten" + debounce/short press
  if (!down && wasDown) {
    if (millis() - downMs >= 30) {
      app.gate = RunGate::Running;

      app.joinStatus = JoinStatus::Boot;
      app.nextJoinAttemptMs = millis();
      app.joined = false;
      app.uplinkSent = false;
      app.uplinkStatus = UplinkStatus::Idle;
      app.lastLoraErr = 0;
      app.lastUplinkOkMs = 0;
      app.sessionRestored = false;
      app.sessionChecked = false;
      app.joinAttempted = false;
      app.restoreUplinkAttempts = 0;
      app.nextRestoreUplinkMs = 0;
    }
  }

  wasDown = down;
}

static void updateJoinFlow() {
  uint32_t now = millis();

  if (app.joinStatus == JoinStatus::Boot) {
    app.joinStatus = JoinStatus::RadioInit;
  }

  if (app.joinStatus == JoinStatus::RadioInit) {
    app.radioReady = lorawanManager.initRadio(app);
    if (app.radioReady) {
      app.joinStatus = JoinStatus::Restoring;
      app.nextJoinAttemptMs = now;
    } else {
      app.joinStatus = JoinStatus::JoinFail;
    }
  }

  if (app.joinStatus == JoinStatus::Restoring && !app.sessionChecked) {
    app.sessionChecked = true;
    LoRaWANSession_t session;
    bool haveSession = lorawanManager.loadSession(session);
    if (haveSession && lorawanManager.restoreSession(app, session)) {
      app.joined = true;
      app.sessionRestored = true;
      app.joinStatus = JoinStatus::RestoreOk;
      app.restoreUplinkAttempts = 0;
      app.nextRestoreUplinkMs = now;
      return;
    }

    app.joinStatus = JoinStatus::RestoreFail;
  }

  if ((app.joinStatus == JoinStatus::RestoreFail || app.joinStatus == JoinStatus::Joining) && !app.joined) {
    if (app.joinAttempted) {
      app.joinStatus = JoinStatus::JoinFail;
      return;
    }

    if (now < app.nextJoinAttemptMs) return;

    app.joinAttempted = true;
    app.joinStatus = JoinStatus::Joining;
    bool ok = lorawanManager.join(app);
    if (ok) {
      app.joined = true;
      app.joinStatus = JoinStatus::JoinOk;
      LoRaWANSession_t session;
      if (lorawanManager.fetchSession(app, session)) {
        lorawanManager.saveSession(session);
      }
    } else {
      app.lastUplinkOkMs = 0;
      app.joinStatus = JoinStatus::JoinFail;
    }
  }
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

static void updateUplinkFlow() {
  if (!app.joined) return;

  uint32_t now = millis();

  if (app.sessionRestored) {
    if (app.restoreUplinkAttempts >= RESTORE_UPLINK_MAX_TRIES) return;
    if (now < app.nextRestoreUplinkMs) return;

    app.restoreUplinkAttempts++;
    lorawanManager.sendOnce(app);

    if (app.uplinkStatus == UplinkStatus::Ok) {
      app.uplinkSent = true;
      LoRaWANSession_t session;
      if (lorawanManager.fetchSession(app, session)) {
        lorawanManager.saveSession(session);
      }
      return;
    }

    if (isSessionInvalidError(app.lastLoraErr) ||
        app.restoreUplinkAttempts >= RESTORE_UPLINK_MAX_TRIES) {
      lorawanManager.clearSession();
      app.joined = false;
      app.sessionRestored = false;
      app.joinStatus = JoinStatus::RestoreFail;
      app.nextJoinAttemptMs = now;
      return;
    }

    app.nextRestoreUplinkMs = now + RESTORE_UPLINK_RETRY_MS;
    return;
  }

  if (app.uplinkSent) return;

  lorawanManager.sendOnce(app);
  app.uplinkSent = true;
  if (app.uplinkStatus == UplinkStatus::Ok) {
    LoRaWANSession_t session;
    if (lorawanManager.fetchSession(app, session)) {
      lorawanManager.saveSession(session);
    }
  }
}

static void updateChgLed() {
  // constant aan als PMU CHG of DONE ziet (USB eraan)
  if (isChargingOrDone()) {
    setChgLed(true);
    nextChgLedBlinkMs = 0;
    chgLedPulseEndMs = 0;
    return;
  }

  // niet laden/geen USB (of status NO)
  if (!app.joined) {
    setChgLed(false);
    nextChgLedBlinkMs = 0;
    chgLedPulseEndMs = 0;
    return;
  }

  // na join: pulse
  uint32_t now = millis();

  if (nextChgLedBlinkMs == 0) {
    nextChgLedBlinkMs = now + CHG_LED_INTERVAL_MS;
  }

  if (chgLedPulseEndMs != 0 && now >= chgLedPulseEndMs) {
    setChgLed(false);
    chgLedPulseEndMs = 0;
  }

  if (now >= nextChgLedBlinkMs) {
    setChgLed(true);
    chgLedPulseEndMs = now + CHG_LED_PULSE_MS;
    nextChgLedBlinkMs = now + CHG_LED_INTERVAL_MS;
  }
}

void setup() {
  powerManager.begin();
  displayManager.begin();
  gnssManager.begin();
  disableRadios();

  app.gate = RunGate::WaitingForBoot;
  pinMode(BOOT_PIN, INPUT_PULLUP);

  app.joinStatus = JoinStatus::Boot;
  app.nextJoinAttemptMs = millis();
}

void loop() {
  gnssManager.update();

  updateStartGate();

  if (app.gate == RunGate::Running) {
    updateJoinFlow();
    updateUplinkFlow();
  }

  updateChgLed();

  uint32_t now = millis();
  if (now - lastDrawMs >= 250) {
    lastDrawMs = now;

    uint16_t battMv = readBatteryMv();
    int chgRaw = chargerStatusRaw();

    bool vbus = hasVbus();
    uint16_t vbusMv = readVbusMv();

    uint8_t battPct = readBatteryPercent();
    displayManager.render(app, gps, battMv, battPct, chgRaw, vbus, vbusMv);
  }
}
