#include "services/TrackerService.h"

#include <TinyGPSPlus.h>

#include "config/AppConfig.h"

bool TrackerService::isMoving(const CurrentFix& fix) const {
	if (!fix.hasFix) return false;
	if (fix.hasSpeed && fix.speedMps > AppConfig::MOVE_SPEED_MPS) return true;
	if (fix.anchorValid) {
		double distance = TinyGPSPlus::distanceBetween(
				fix.anchorLat,
				fix.anchorLon,
				fix.lat,
				fix.lon);
		if (distance > AppConfig::MOVE_DISTANCE_M) return true;
	}
	return false;
}

uint32_t TrackerService::nextUplinkIntervalMs(const CurrentFix& fix) const {
	return isMoving(fix) ? AppConfig::UPLINK_INTERVAL_MS : AppConfig::KEEPALIVE_INTERVAL_MS;
}

size_t TrackerService::buildPayload(uint8_t* out, size_t maxLen, const CurrentFix& fix, uint16_t battMv) const {
	if (maxLen < 3) return 0;

	uint8_t flags = 0;
	if (fix.hasFix) flags |= 0x01;
	if (isMoving(fix)) flags |= 0x02;
	if (fix.hasSpeed && fix.hasCourse) flags |= 0x04;

	out[0] = flags;
	out[1] = static_cast<uint8_t>(battMv & 0xFF);
	out[2] = static_cast<uint8_t>((battMv >> 8) & 0xFF);

	if (fix.hasFix) {
		if (maxLen < 14) return 3;
		int32_t lat = static_cast<int32_t>(lroundf(fix.lat * 1000000.0f));
		int32_t lon = static_cast<int32_t>(lroundf(fix.lon * 1000000.0f));
		uint8_t sats = (fix.sats >= 0 && fix.sats <= 255) ? static_cast<uint8_t>(fix.sats) : 0;
		uint16_t hdop10 = (fix.hdop >= 0.0f) ? static_cast<uint16_t>(lroundf(fix.hdop * 10.0f)) : 0;

		memcpy(out + 3, &lat, sizeof(lat));
		memcpy(out + 7, &lon, sizeof(lon));
		out[11] = sats;
		out[12] = static_cast<uint8_t>(hdop10 & 0xFF);
		out[13] = static_cast<uint8_t>((hdop10 >> 8) & 0xFF);

		size_t len = 14;
		if (flags & 0x04) {
			if (maxLen < 18) return len;
			uint16_t speed10 = static_cast<uint16_t>(lroundf(fix.speedMps * 10.0f));
			uint16_t course10 = static_cast<uint16_t>(lroundf(fix.courseDeg * 10.0f));
			out[14] = static_cast<uint8_t>(speed10 & 0xFF);
			out[15] = static_cast<uint8_t>((speed10 >> 8) & 0xFF);
			out[16] = static_cast<uint8_t>(course10 & 0xFF);
			out[17] = static_cast<uint8_t>((course10 >> 8) & 0xFF);
			len = 18;
		}
		return len;
	}

	if (maxLen < 8) return 3;
	uint8_t sats = (fix.sats >= 0 && fix.sats <= 255) ? static_cast<uint8_t>(fix.sats) : 0;
	uint16_t hdop10 = (fix.hdop >= 0.0f) ? static_cast<uint16_t>(lroundf(fix.hdop * 10.0f)) : 0;
	uint16_t ageSec = 0xFFFF;
	if (fix.lastFixMs > 0) {
		uint32_t ageMs = millis() - fix.lastFixMs;
		ageSec = (ageMs > 0xFFFFu * 1000u) ? 0xFFFF : static_cast<uint16_t>(ageMs / 1000u);
	}
	out[3] = sats;
	out[4] = static_cast<uint8_t>(hdop10 & 0xFF);
	out[5] = static_cast<uint8_t>((hdop10 >> 8) & 0xFF);
	out[6] = static_cast<uint8_t>(ageSec & 0xFF);
	out[7] = static_cast<uint8_t>((ageSec >> 8) & 0xFF);
	return 8;
}
