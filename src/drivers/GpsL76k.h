#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// Snapshot van de laatst bekende GPS-fix en metadata.
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

// Driver voor de L76K GPS-module en fixbeheer.
class GpsL76k {
public:
	// Maak een GPS-driver aan met standaardinstellingen.
	GpsL76k();

	// Initialiseer de GPS-hardware en seriële poort.
	void begin();
	// Lees binnenkomende NMEA-data en update de fix.
	void pump();
	// Schrijf een compacte GPS-statusregel naar outputbuffer.
	void renderGpsLine(char* out, size_t outLen) const;
	// Geef de laatst bekende fix terug.
	CurrentFix getFix() const;
	// Zet het ankerpunt op de huidige fix.
	void anchorToCurrent();

private:
	// Leeg de UART-buffer gedurende een gegeven tijd.
	void drainUart(uint32_t ms = 150);
	// Zet de GPS in high-power modus.
	void enableHigh();
	// Start de GPS-seriële communicatie.
	void gpsBegin();
	// Stop NMEA-output om bufferdruk te verminderen.
	void gpsStopNmea();
	// Voer de initiële configuratiestappen uit.
	void gpsInitSequence();
	// Update de interne fixstructuur met nieuwe data.
	void updateFix();

	HardwareSerial serial_;
	TinyGPSPlus gps_;
	CurrentFix fix_;
};
