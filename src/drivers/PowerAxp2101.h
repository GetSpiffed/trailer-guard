#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>

class PowerAxp2101 {
public:
	PowerAxp2101();

	bool begin();
	uint16_t readBatteryMv();
	float readBatteryVoltageV();
	uint8_t readBatteryPercent();
	void updateChargeLed();

private:
	TwoWire wire_;
	XPowersAXP2101 axp_;
};
