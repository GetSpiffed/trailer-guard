#pragma once

#include <Arduino.h>
#include <vector>

#include "services/FixSnapshot.h"

struct Telemetry {
	FixSnapshot fix;
	uint8_t battPct = 0;
};

class TelemetryService {
public:
	void begin(FixSnapshot* fix);
	void bindBatteryReader(uint8_t (*reader)());
	Telemetry collect() const;
	std::vector<uint8_t> encode(const Telemetry& telemetry) const;

private:
	FixSnapshot* fix_ = nullptr;
	uint8_t (*batteryReader_)() = nullptr;
};
