#pragma once

#include <Arduino.h>

class BootButton {
public:
	void begin();
	void update();
	bool consumeShortPress();
	bool consumeLongPressTrigger();
	bool inHoldUi() const;
	uint32_t getHeldMs() const;

private:
	bool lastRawDown_ = false;
	uint32_t rawChangeMs_ = 0;
	bool debouncedDown_ = false;
	uint32_t pressStartMs_ = 0;
	bool longArmed_ = false;
	bool tooLong_ = false;
	bool shortEvent_ = false;
	bool longTriggerEvent_ = false;
	bool initialized_ = false;
	uint32_t heldMs_ = 0;
};
