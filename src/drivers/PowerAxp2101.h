#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>

// Driver voor de AXP2101 power management unit.
class PowerAxp2101 {
public:
	// Maak de PMU-driver aan.
	PowerAxp2101();

	// Initialiseer de PMU en configureer voedingen.
	bool begin();
	// Lees de batterijspanning in millivolt.
	uint16_t readBatteryMv();
	// Lees de batterijspanning in volt.
	float readBatteryVoltageV();
	// Lees de batterijlading in procenten.
	uint8_t readBatteryPercent();
	// Update de laad-LED op basis van VBUS-status.
	void updateChargeLed();

private:
	TwoWire wire_;
	XPowersAXP2101 axp_;
};
