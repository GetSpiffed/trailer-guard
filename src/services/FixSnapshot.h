#pragma once

#include <Arduino.h>

struct FixSnapshot {
	bool hasFix = false;
	int32_t latE7 = 0;
	int32_t lonE7 = 0;
	uint8_t speedKmh = 0;
	uint32_t ageMs = 0;
};
