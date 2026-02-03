#include "drivers/PowerAxp2101.h"

#include "config/BoardPins.h"

PowerAxp2101::PowerAxp2101()
		: wire_(0) {
}

bool PowerAxp2101::begin() {
	wire_.begin(BoardPins::PMU_SDA, BoardPins::PMU_SCL);
	bool ok = axp_.begin(wire_, AXP2101_SLAVE_ADDRESS, BoardPins::PMU_SDA, BoardPins::PMU_SCL);
	if (!ok) return false;

	axp_.enableSystemVoltageMeasure();
	axp_.enableVbusVoltageMeasure();
	axp_.enableBattVoltageMeasure();
	axp_.enableBattDetection();
	axp_.enableTemperatureMeasure();

	axp_.setALDO4Voltage(3300);
	axp_.enableALDO4();

	axp_.setALDO1Voltage(3300);
	axp_.enableALDO1();

	axp_.setALDO3Voltage(3300);
	axp_.enableALDO3();

	return true;
}

uint16_t PowerAxp2101::readBatteryMv() {
	if (!axp_.isBatteryConnect()) return 0;
	float v = axp_.getBattVoltage();
	if (v > 0 && v < 10.0f) return static_cast<uint16_t>(lroundf(v * 1000.0f));
	if (v >= 10.0f && v < 1000.0f) return static_cast<uint16_t>(lroundf(v * 10.0f));
	return static_cast<uint16_t>(lroundf(v));
}

float PowerAxp2101::readBatteryVoltageV() {
	uint16_t mv = readBatteryMv();
	if (mv == 0) return 0.0f;
	return static_cast<float>(mv) / 1000.0f;
}

uint8_t PowerAxp2101::readBatteryPercent() {
	if (!axp_.isBatteryConnect()) return 0;
	return static_cast<uint8_t>(axp_.getBatteryPercent());
}

void PowerAxp2101::updateChargeLed() {
	bool vbus = axp_.isVbusIn();
	axp_.setChargingLedMode(vbus ? XPOWERS_CHG_LED_ON : XPOWERS_CHG_LED_OFF);
}
