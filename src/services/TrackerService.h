#pragma once

#include <Arduino.h>

#include "drivers/GpsL76k.h"

class TrackerService {
public:
	bool isMoving(const CurrentFix& fix) const;
	uint32_t nextUplinkIntervalMs(const CurrentFix& fix) const;
	size_t buildPayload(uint8_t* out, size_t maxLen, const CurrentFix& fix, uint16_t battMv) const;
};
