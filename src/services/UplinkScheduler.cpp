#include "services/UplinkScheduler.h"

void UplinkScheduler::begin(uint32_t intervalMs) {
	intervalMs_ = intervalMs;
	nextSendMs_ = 0;
	linkReady_ = false;
}

void UplinkScheduler::onLinkReady() {
	linkReady_ = true;
	if (nextSendMs_ == 0) {
		nextSendMs_ = millis() + intervalMs_;
	}
}

bool UplinkScheduler::loop(uint32_t nowMs) const {
	if (!linkReady_ || nextSendMs_ == 0) return false;
	return nowMs >= nextSendMs_;
}

void UplinkScheduler::markSent(uint32_t nowMs) {
	nextSendMs_ = nowMs + intervalMs_;
}

uint32_t UplinkScheduler::secondsUntilNext(uint32_t nowMs) const {
	if (!linkReady_ || nextSendMs_ == 0 || nowMs >= nextSendMs_) return 0;
	return (nextSendMs_ - nowMs) / 1000;
}
