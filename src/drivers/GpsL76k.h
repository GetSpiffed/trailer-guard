#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

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

class GpsL76k {
public:
	GpsL76k();

	void begin();
	void pump();
	void renderGpsLine(char* out, size_t outLen) const;
	CurrentFix getFix() const;
	void anchorToCurrent();

private:
	void drainUart(uint32_t ms = 150);
	void enableHigh();
	void gpsBegin();
	void gpsStopNmea();
	void gpsInitSequence();
	void updateFix();

	HardwareSerial serial_;
	TinyGPSPlus gps_;
	CurrentFix fix_;
};
