#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <esp_bt.h>



#include <SPI.h>
#include <RadioLib.h>

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
static constexpr uint32_t UPLINK_EVERY_MS = 60000;   // 60s voor testen
static constexpr uint8_t  LORAWAN_FPORT   = 1;

// ---------------- Globals ----------------
XPowersAXP2101 axp;
HardwareSerial GNSS(1);
TinyGPSPlus gps;

// OLED: SH1106 128x64, SW I2C met expliciete pins
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, DEV_SCL, DEV_SDA, U8X8_PIN_NONE);

uint32_t lastDrawMs = 0;
uint32_t lastUplinkMs = 0;

// ---------------- LoRaWAN (RadioLib) ----------------
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
LoRaWANNode node(&radio, &EU868);

// --- TTN OLED STATUS ---
bool lorawanJoined = false;
int16_t lastLoraErr = 0;            // 0 = OK
uint32_t lastUplinkOkMs = 0;        // millis() laatste OK uplink
// --- end TTN OLED STATUS ---

static void logRadioErr(const char* where, int16_t err) {
  Serial.printf("[LoRaWAN] %s failed, code=%d\n", where, err);
}

static uint16_t readBatteryMv() {
  if (axp.isBatteryConnect()) {
    return (uint16_t)axp.getBattVoltage(); // mV bij veel XPowersLib builds
  }
  return 0;
}

static bool radioInited = false;

static bool initRadioOnce() {
  if (radioInited) return true;

  Serial.println("[LoRaWAN] SPI begin...");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  Serial.println("[LoRaWAN] radio reset...");
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);  delay(10);
  digitalWrite(LORA_RST, HIGH); delay(10);

  Serial.println("[LoRaWAN] radio begin...");
  int16_t st = radio.begin();
  if (st != RADIOLIB_ERR_NONE) {
    lastLoraErr = st;
    logRadioErr("radio.begin", st);
    return false;
  }

  radio.setDio2AsRfSwitch(true);
  radio.setOutputPower(14);
  radio.setCurrentLimit(140);

  radioInited = true;
  return true;
}

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

static bool lorawanJoin() {
  if (!initRadioOnce()) return false;

  Serial.println("[LoRaWAN] begin OTAA...");
  int16_t st = node.beginOTAA(JOIN_EUI, DEV_EUI, NWK_KEY, APP_KEY);
  if (st != RADIOLIB_ERR_NONE) {
    lastLoraErr = st;
    logRadioErr("node.beginOTAA", st);
    return false;
  }

  Serial.println("[LoRaWAN] activate OTAA (join)...");
  st = node.activateOTAA();
  if (st != RADIOLIB_ERR_NONE && st != RADIOLIB_LORAWAN_NEW_SESSION) {
    lastLoraErr = st;
    logRadioErr("node.activateOTAA", st);
    return false;
  }

  lastLoraErr = 0;
  Serial.println("[LoRaWAN] joined OK");
  return true;
}

static bool lorawanJoinWithRetry(uint8_t tries = 3, uint32_t pauseMs = 10000) {
  for (uint8_t i = 0; i < tries; i++) {
    Serial.printf("[LoRaWAN] join attempt %u/%u\n", (unsigned)(i + 1), (unsigned)tries);

    if (lorawanJoin()) return true;

    // OLED: join is niet gelukt, dus "OK-age" resetten
    lastUplinkOkMs = 0;

    delay(pauseMs);
  }
  return false;
}

static void lorawanSendOnce() {
  uint8_t payload[16];
  size_t n = buildPayload(payload, sizeof(payload));
  if (n == 0) return;

  Serial.printf("[LoRaWAN] uplink (%u bytes) ...\n", (unsigned)n);
  int16_t st = node.sendReceive(payload, n, LORAWAN_FPORT);

  lastLoraErr = st;
  if (st == RADIOLIB_ERR_NONE) {
    lastLoraErr = 0;
    lastUplinkOkMs = millis();
    Serial.println("[LoRaWAN] uplink OK");
  } else {
    logRadioErr("node.sendReceive", st);
  }
}

// --- TTN OLED STATUS ---
static void buildTtnStatusLine(char* out, size_t outLen) {
  if (!lorawanJoined) {
    snprintf(out, outLen, "TTN:NOJOIN");
    return;
  }
  if (lastLoraErr != 0) {
    snprintf(out, outLen, "TTN:ERR%d", (int)lastLoraErr);
    return;
  }
  if (lastUplinkOkMs == 0) {
    snprintf(out, outLen, "TTN:JOIN");
    return;
  }
  uint32_t ageS = (millis() - lastUplinkOkMs) / 1000;
  if (ageS > 9999) ageS = 9999;
  snprintf(out, outLen, "TTN:OK %lus", (unsigned long)ageS);
}
// --- end TTN OLED STATUS ---

void drawScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.drawStr(0, 10, "lora-trailer-guard-001");
  u8g2.drawHLine(0, 12, 128);

  char line[32];
  int sat = gps.satellites.isValid() ? gps.satellites.value() : 0;

  snprintf(line, sizeof(line), "SAT: %d", sat);
  u8g2.drawStr(0, 26, line);

  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();

    snprintf(line, sizeof(line), "LAT: %.6f", lat);
    u8g2.drawStr(0, 40, line);

    snprintf(line, sizeof(line), "LON: %.6f", lon);
    u8g2.drawStr(0, 54, line);
  } else {
    u8g2.drawStr(0, 44, "GPS: NO FIX (indoor ok)");
  }

  char ttnLine[24];
  buildTtnStatusLine(ttnLine, sizeof(ttnLine));
  u8g2.drawStr(0, 62, ttnLine);

  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("boot ok");

  // Init PMU
  I2C_PMU.begin(PMU_SDA, PMU_SCL);
  bool pmuOk = axp.begin(I2C_PMU, AXP2101_SLAVE_ADDRESS, PMU_SDA, PMU_SCL);

  if (pmuOk) {
    Serial.println("AXP ok, enabling ALDO1 (OLED) + ALDO3 (LoRa) + ALDO4 (GNSS)");

    axp.setALDO1Voltage(3300);
    axp.enableALDO1();

    axp.setALDO3Voltage(3300);   // SX1262 radio
    axp.enableALDO3();
    delay(50);                   // radio power settle

    axp.setALDO4Voltage(3300);
    axp.enableALDO4();
  } else {
    Serial.println("AXP init failed");
  }

  delay(200);

  // OLED init
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 20, "OLED: init ok");
  u8g2.drawStr(0, 36, "Starting GNSS...");
  u8g2.sendBuffer();

  // GNSS serial init
  GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  Serial.println("GNSS UART started");

  WiFi.mode(WIFI_OFF);
  btStop();
  delay(100);


  // LoRaWAN join (retry + backoff)
  lorawanJoined = lorawanJoinWithRetry(3, 20000);

  if (!lorawanJoined) {
    Serial.println("[LoRaWAN] join failed after retries (check TTN events / coverage)");
    // lastLoraErr bevat al de echte foutcode (meestal -1116)
  } else {
    lastLoraErr = 0;
    Serial.println("[LoRaWAN] joined OK (after retry loop)");
  }

  lastUplinkMs = millis();
}

void loop() {
  while (GNSS.available()) {
    gps.encode((char)GNSS.read());
  }

  uint32_t now = millis();

  if (now - lastDrawMs >= 1000) {
    lastDrawMs = now;

    int sat = gps.satellites.isValid() ? gps.satellites.value() : 0;
    if (gps.location.isValid()) {
      Serial.printf("FIX lat=%.6f lon=%.6f sat=%d\n",
                    gps.location.lat(), gps.location.lng(), sat);
    } else {
      Serial.printf("no fix, sat=%d\n", sat);
    }

    drawScreen();
  }

  if (lorawanJoined && (now - lastUplinkMs >= UPLINK_EVERY_MS)) {
    lastUplinkMs = now;
    lorawanSendOnce();
  }
}
