#pragma once

#include <Arduino.h>

#include "drivers/GpsL76k.h"

// Service voor bewegingsdetectie en uplinkpayloads.
class TrackerService {
public:
	// Bepaal of het apparaat in beweging is op basis van GPS-fix.
	bool isMoving(const CurrentFix& fix) const;
	// Kies het volgende uplinkinterval op basis van beweging.
	uint32_t nextUplinkIntervalMs(const CurrentFix& fix) const;
	// Bouw de uplinkpayload uit fix- en batterijdata.
	size_t buildPayload(uint8_t* out, size_t maxLen, const CurrentFix& fix, uint16_t battMv) const;
};
