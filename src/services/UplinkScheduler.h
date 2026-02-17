#pragma once

#include <Arduino.h>

class UplinkScheduler {
public:
	void begin(uint32_t intervalMs = 300000);
	void onLinkReady();
	bool loop(uint32_t nowMs) const;
	void markSent(uint32_t nowMs);
	uint32_t secondsUntilNext(uint32_t nowMs) const;

private:
	uint32_t intervalMs_ = 300000;
	uint32_t nextSendMs_ = 0;
	bool linkReady_ = false;
};
