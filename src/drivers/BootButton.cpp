#include "drivers/BootButton.h"

#include "config/AppConfig.h"
#include "config/BoardPins.h"

void BootButton::begin() {
	pinMode(BoardPins::BOOT_PIN, INPUT_PULLUP);
	uint32_t now = millis();
	lastRawDown_ = (digitalRead(BoardPins::BOOT_PIN) == LOW);
	rawChangeMs_ = now;
	debouncedDown_ = lastRawDown_;

	if (debouncedDown_) {
		pressStartMs_ = now;
	}
	longArmed_ = false;
	tooLong_ = false;
	shortEvent_ = false;
	longTriggerEvent_ = false;
	heldMs_ = 0;
	initialized_ = true;
}

void BootButton::update() {
	if (!initialized_) begin();

	uint32_t now = millis();
	bool rawDown = (digitalRead(BoardPins::BOOT_PIN) == LOW);

	if (rawDown != lastRawDown_) {
		lastRawDown_ = rawDown;
		rawChangeMs_ = now;
	}

	bool prevDebounced = debouncedDown_;
	if ((now - rawChangeMs_) >= AppConfig::BOOT_DEBOUNCE_MS) {
		debouncedDown_ = rawDown;
	}

	if (!prevDebounced && debouncedDown_) {
		pressStartMs_ = now;
		longArmed_ = false;
		tooLong_ = false;
	}

	if (debouncedDown_) {
		heldMs_ = now - pressStartMs_;

		if (!longArmed_ && heldMs_ >= AppConfig::FORCE_OTAA_MIN_MS) {
			longArmed_ = true;
		}
		if (heldMs_ > AppConfig::FORCE_OTAA_MAX_MS) {
			tooLong_ = true;
		}
	}

	if (prevDebounced && !debouncedDown_) {
		uint32_t pressMs = now - pressStartMs_;

		if (longArmed_ && !tooLong_ && pressMs <= AppConfig::FORCE_OTAA_MAX_MS) {
			longTriggerEvent_ = true;
		} else if (pressMs >= AppConfig::BOOT_MIN_PRESS_MS && pressMs <= AppConfig::BOOT_MAX_SHORT_MS) {
			shortEvent_ = true;
		}

		heldMs_ = 0;
		longArmed_ = false;
		tooLong_ = false;
	}
}

bool BootButton::consumeShortPress() {
	if (!shortEvent_) return false;
	shortEvent_ = false;
	return true;
}

bool BootButton::consumeLongPressTrigger() {
	if (!longTriggerEvent_) return false;
	longTriggerEvent_ = false;
	return true;
}

bool BootButton::inHoldUi() const {
	return debouncedDown_;
}

uint32_t BootButton::getHeldMs() const {
	return heldMs_;
}
