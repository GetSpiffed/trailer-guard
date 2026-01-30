#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <TinyGPSPlus.h>

// T-Beam S3 Supreme (utilities.h)
static constexpr uint8_t PMU_SDA = 42;
static constexpr uint8_t PMU_SCL = 41;

static constexpr uint8_t GPS_RX_PIN  = 9;   // ESP RX <- GNSS TX
static constexpr uint8_t GPS_TX_PIN  = 8;   // ESP TX -> GNSS RX
static constexpr uint8_t GPS_EN_PIN  = 7;   // GPS_EN
static constexpr uint8_t GPS_PPS_PIN = 6;   // PPS
static constexpr uint32_t GPS_BAUD   = 9600;

TwoWire Wire1_PMU(0);
XPowersLibInterface *PMU = nullptr;
HardwareSerial SerialGPS(2);

TinyGPSPlus gps;

static volatile uint32_t ppsCount = 0;
static void IRAM_ATTR onPpsRise() { ppsCount++; }

// --- helpers ---
static void drainGpsUart(uint32_t ms = 150) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (SerialGPS.available()) (void)SerialGPS.read();
    delay(1);
  }
}

static void printPowerSanity() {
  if (!PMU) return;

  // Sommige XPowersLib versies geven voltage in mV (int), andere als float (V).
  // We doen het simpel: print als float (werkt meestal goed met printf).
  float sysV = PMU->getSystemVoltage();
  float vbusV = PMU->getVbusVoltage();
  float battV = PMU->getBattVoltage();

  Serial.printf("PWR: SYS=%.2f V  VBUS=%.2f V  BATT=%.2f V  VBUS_IN=%d BAT_CONN=%d\n",
                sysV, vbusV, battV, PMU->isVbusIn(), PMU->isBatteryConnect());
}

static bool beginPower_AXP2101_Wire1() {
  Wire1_PMU.begin(PMU_SDA, PMU_SCL);

  PMU = new XPowersAXP2101(Wire1_PMU);
  if (!PMU->init()) {
    Serial.println("PMU: AXP2101 init FAIL");
    delete PMU; PMU = nullptr;
    return false;
  }
  Serial.println("PMU: AXP2101 init OK");

  // GNSS op ALDO4 3.3V (boards.cpp T_BEAM_S3_SUPREME)
  PMU->setPowerChannelVoltage(XPOWERS_ALDO4, 3300);
  PMU->enablePowerOutput(XPOWERS_ALDO4);
  delay(200);

  // Metingen aan
  PMU->enableSystemVoltageMeasure();
  PMU->enableVbusVoltageMeasure();
  PMU->enableBattVoltageMeasure();

  // Extra voor BattV=0.00 / 0.24V issues op AXP2101
  PMU->enableBattDetection();
  PMU->enableTemperatureMeasure(); // optioneel maar handig

  printPowerSanity();
  return true;
}

static void gpsEnableHigh() {
  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  delay(350);
  Serial.printf("GPS_EN raw=%d\n", digitalRead(GPS_EN_PIN));
}

static void gpsBegin() {
  SerialGPS.end();
  delay(30);
  SerialGPS.setRxBufferSize(4096);
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(80);
  drainGpsUart(200);
}

// Probe zoals LilyGO: stop output, vraag versie, configureer, zet RMC+GGA aan
static void l76kInitAndEnableNmea() {
  // 1) Stop alle NMEA (zodat responses niet verdrinken)
  SerialGPS.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
  delay(60);
  drainGpsUart(120);

  // 2) Vraag versie (GPTXT...)
  SerialGPS.write("$PCAS06,0*1B\r\n");
  delay(80);

  // 3) Constellation: GPS+GLONASS
  SerialGPS.write("$PCAS04,5*1C\r\n");
  delay(120);

  // 4) Vehicle mode
  SerialGPS.write("$PCAS11,3*1E\r\n");
  delay(120);

  // 5) NMEA weer aan: alleen GGA + RMC (zoals voorbeeld)
  SerialGPS.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
  delay(120);

  Serial.println("L76K init done, NMEA enabled (RMC+GGA).");
}

static void printGpsSummaryOncePerSec() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint < 1000) return;
  lastPrint = now;

  // power sanity af en toe (handig als BattV raar blijft)
  printPowerSanity();

  if (gps.location.isValid()) {
    Serial.printf("FIX lat=%.6f lon=%.6f sats=%d hdop=%.1f\n",
                  gps.location.lat(), gps.location.lng(),
                  gps.satellites.isValid() ? gps.satellites.value() : -1,
                  gps.hdop.isValid() ? gps.hdop.hdop() : -1.0);
  } else {
    Serial.printf("NO FIX sats=%d hdop=%.1f\n",
                  gps.satellites.isValid() ? gps.satellites.value() : -1,
                  gps.hdop.isValid() ? gps.hdop.hdop() : -1.0);
  }
}

static void pumpGpsUartAndParse() {
  static uint32_t bytesInSec = 0;
  static uint32_t dollarsInSec = 0;
  static uint32_t lastMs = 0;
  static uint32_t lastAnyByteMs = 0;

  // NMEA line buffer (alleen printen naar Serial)
  static char line[220];
  static uint16_t idx = 0;

  while (SerialGPS.available()) {
    char c = (char)SerialGPS.read();

    // TinyGPSPlus
    gps.encode(c);

    // stats
    bytesInSec++;
    if (c == '$') dollarsInSec++;
    lastAnyByteMs = millis();

    // print complete NMEA lines (debug)
    if (c == '\n') {
      line[idx] = 0;
      if (idx > 6) {
        Serial.print("NMEA: ");
        Serial.println(line);
      }
      idx = 0;
    } else if (c != '\r') {
      if (idx < sizeof(line) - 1) line[idx++] = c;
    }
  }

  uint32_t now = millis();
  if (now - lastMs >= 1000) {
    static uint32_t lastPps = 0;
    uint32_t p = ppsCount;
    uint32_t ppsPerSec = p - lastPps;
    lastPps = p;

    Serial.printf("STAT pps/s=%lu bytes/s=%lu $/s=%lu en=%d rx=%u tx=%u baud=%lu\n",
                  (unsigned long)ppsPerSec,
                  (unsigned long)bytesInSec,
                  (unsigned long)dollarsInSec,
                  (int)digitalRead(GPS_EN_PIN),
                  (unsigned)GPS_RX_PIN,
                  (unsigned)GPS_TX_PIN,
                  (unsigned long)GPS_BAUD);

    // Als het stilvalt: re-init + NMEA aan
    if (lastAnyByteMs != 0 && (now - lastAnyByteMs) > 5000) {
      Serial.println("=> 5s stil: re-init + NMEA aan...");
      gpsBegin();
      l76kInitAndEnableNmea();
      lastAnyByteMs = now;
    }

    bytesInSec = 0;
    dollarsInSec = 0;
    lastMs = now;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n--- L76K GNSS TEST (serial only + TinyGPSPlus + PMU sanity) ---");

  pinMode(GPS_PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), onPpsRise, RISING);
  Serial.println("PPS interrupt attached");

  if (!beginPower_AXP2101_Wire1()) {
    Serial.println("Stop: PMU init failed");
    return;
  }

  gpsEnableHigh();
  gpsBegin();
  l76kInitAndEnableNmea();
}

void loop() {
  pumpGpsUartAndParse();
  printGpsSummaryOncePerSec();
}
