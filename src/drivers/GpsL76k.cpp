#include "drivers/GpsL76k.h"

#include "config/AppConfig.h"
#include "config/BoardPins.h"

GpsL76k::GpsL76k()
		: serial_(2) {
}

void GpsL76k::drainUart(uint32_t ms) {
	uint32_t t0 = millis();
	while (millis() - t0 < ms) {
		while (serial_.available()) (void)serial_.read();
		delay(1);
	}
}

void GpsL76k::enableHigh() {
	pinMode(BoardPins::GPS_EN_PIN, OUTPUT);
	digitalWrite(BoardPins::GPS_EN_PIN, HIGH);
	delay(350);
}

void GpsL76k::gpsBegin() {
	serial_.end();
	delay(30);
	serial_.setRxBufferSize(4096);
	serial_.begin(AppConfig::GPS_BAUD, SERIAL_8N1, BoardPins::GPS_RX_PIN, BoardPins::GPS_TX_PIN);
	delay(80);
	drainUart(200);
}

void GpsL76k::gpsStopNmea() {
	serial_.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
	delay(60);
	drainUart(120);
}

void GpsL76k::gpsInitSequence() {
	gpsStopNmea();
	serial_.write("$PCAS06,0*1B\r\n");
	delay(80);
	serial_.write("$PCAS04,5*1C\r\n");
	delay(120);
	serial_.write("$PCAS11,3*1E\r\n");
	delay(120);
	serial_.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
	delay(120);
}

void GpsL76k::begin() {
	pinMode(BoardPins::GPS_PPS_PIN, INPUT);
	enableHigh();
	gpsBegin();
	gpsInitSequence();
}

void GpsL76k::updateFix() {
	fix_.hasFix = gps_.location.isValid();
	fix_.lat = fix_.hasFix ? gps_.location.lat() : fix_.lat;
	fix_.lon = fix_.hasFix ? gps_.location.lng() : fix_.lon;
	fix_.sats = gps_.satellites.isValid() ? gps_.satellites.value() : -1;
	fix_.hdop = gps_.hdop.isValid() ? gps_.hdop.hdop() : -1.0f;
	fix_.hasSpeed = gps_.speed.isValid();
	fix_.speedMps = fix_.hasSpeed ? gps_.speed.mps() : 0.0f;
	fix_.hasCourse = gps_.course.isValid();
	fix_.courseDeg = fix_.hasCourse ? gps_.course.deg() : 0.0f;
	if (fix_.hasFix && gps_.location.isUpdated()) {
		fix_.lastFixMs = millis();
		if (!fix_.anchorValid) {
			fix_.anchorLat = fix_.lat;
			fix_.anchorLon = fix_.lon;
			fix_.anchorValid = true;
		}
	}
}

void GpsL76k::pump() {
	static char line[220];
	static uint16_t idx = 0;

	while (serial_.available()) {
		char c = static_cast<char>(serial_.read());
		gps_.encode(c);
		fix_.lastGpsByteMs = millis();

		if (c == '\n') {
			line[idx] = 0;
			if (idx > 6) {
				strncpy(fix_.lastNmeaLine, line, sizeof(fix_.lastNmeaLine) - 1);
				fix_.lastNmeaLine[sizeof(fix_.lastNmeaLine) - 1] = 0;
				fix_.hadNmeaLine = true;
			}
			idx = 0;
		} else if (c != '\r') {
			if (idx < sizeof(line) - 1) line[idx++] = c;
		}
	}

	updateFix();

	uint32_t now = millis();
	if (fix_.lastGpsByteMs != 0 && (now - fix_.lastGpsByteMs) > 5000) {
		gpsBegin();
		gpsInitSequence();
		fix_.lastGpsByteMs = now;
	}
}

void GpsL76k::renderGpsLine(char* out, size_t outLen) const {
	int sats = fix_.sats;
	float hdop = fix_.hdop;
	uint32_t now = millis();
	bool noNmea = fix_.lastGpsByteMs == 0 || (now - fix_.lastGpsByteMs) > 5000;

	if (fix_.hasFix) {
		snprintf(out, outLen, "FIX lat=%.6f lon=%.6f sats=%d hdop=%.1f",
					 static_cast<double>(fix_.lat), static_cast<double>(fix_.lon), sats, static_cast<double>(hdop));
		return;
	}

	if (noNmea) {
		snprintf(out, outLen, "NO FIX sats=%d hdop=%.1f NO NMEA", sats, static_cast<double>(hdop));
		return;
	}

	snprintf(out, outLen, "NO FIX sats=%d hdop=%.1f", sats, static_cast<double>(hdop));
}

CurrentFix GpsL76k::getFix() const {
	return fix_;
}

void GpsL76k::anchorToCurrent() {
	fix_.anchorLat = fix_.lat;
	fix_.anchorLon = fix_.lon;
	fix_.anchorValid = fix_.hasFix;
}
