#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>

// PMU bus
TwoWire I2C_PMU(0);
static constexpr uint8_t PMU_SDA = 42;
static constexpr uint8_t PMU_SCL = 41;

// OLED + sensors bus
static constexpr uint8_t DEV_SDA = 17;
static constexpr uint8_t DEV_SCL = 18;

// GNSS UART (zoals eerder)
static constexpr uint8_t GNSS_RX_PIN = 8;
static constexpr uint8_t GNSS_TX_PIN = 9;
static constexpr uint32_t GNSS_BAUD = 9600;

XPowersAXP2101 axp;
HardwareSerial GNSS(1);
TinyGPSPlus gps;

// OLED: SH1106 128x64, SW I2C met expliciete pins (stabiel)
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, DEV_SCL, DEV_SDA, U8X8_PIN_NONE);

uint32_t lastDrawMs = 0;

void drawScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.drawStr(0, 10, "wake-my-horse");
  u8g2.drawHLine(0, 12, 128);

  // Satellites + fix state
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
    Serial.println("AXP ok, enabling ALDO1 (OLED) + ALDO4 (GNSS)");
    axp.setALDO1Voltage(3300);
    axp.enableALDO1();
    axp.setALDO4Voltage(3300);
    axp.enableALDO4();
  } else {
    Serial.println("AXP init failed (maar we proberen toch OLED/GNSS)");
  }

  delay(200);

  // OLED init (teken meteen iets, dan weet je dat 'ie leeft)
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 20, "OLED: init ok");
  u8g2.drawStr(0, 36, "Starting GNSS...");
  u8g2.sendBuffer();

  // GNSS serial init
  GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  Serial.println("GNSS UART started");
}

void loop() {
  // GNSS bytes -> TinyGPS
  while (GNSS.available()) {
    gps.encode((char)GNSS.read());
  }

  // 1 Hz update
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
}
