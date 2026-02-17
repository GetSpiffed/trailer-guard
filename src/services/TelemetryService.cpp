#include "services/TelemetryService.h"

void TelemetryService::begin(FixSnapshot* fix) {
	fix_ = fix;
}

void TelemetryService::bindBatteryReader(uint8_t (*reader)()) {
	batteryReader_ = reader;
}

Telemetry TelemetryService::collect() const {
	Telemetry telemetry{};
	if (fix_) {
		telemetry.fix = *fix_;
	}
	telemetry.battPct = batteryReader_ ? batteryReader_() : 0;
	return telemetry;
}

std::vector<uint8_t> TelemetryService::encode(const Telemetry& telemetry) const {
	std::vector<uint8_t> payload(11, 0);
	payload[0] = telemetry.fix.hasFix ? 0x01 : 0x00;

	int32_t lat = telemetry.fix.hasFix ? telemetry.fix.latE7 : 0;
	int32_t lon = telemetry.fix.hasFix ? telemetry.fix.lonE7 : 0;
	uint8_t speed = telemetry.fix.hasFix ? telemetry.fix.speedKmh : 0;

	payload[1] = static_cast<uint8_t>(lat & 0xFF);
	payload[2] = static_cast<uint8_t>((lat >> 8) & 0xFF);
	payload[3] = static_cast<uint8_t>((lat >> 16) & 0xFF);
	payload[4] = static_cast<uint8_t>((lat >> 24) & 0xFF);

	payload[5] = static_cast<uint8_t>(lon & 0xFF);
	payload[6] = static_cast<uint8_t>((lon >> 8) & 0xFF);
	payload[7] = static_cast<uint8_t>((lon >> 16) & 0xFF);
	payload[8] = static_cast<uint8_t>((lon >> 24) & 0xFF);

	payload[9] = speed;
	payload[10] = telemetry.battPct;

	return payload;
}
