#pragma once

#include <Arduino.h>

// Driver voor het uitlezen van de BOOT-knop met debounce.
class BootButton {
public:
	// Configureer de BOOT-knop GPIO.
	void begin();
	// Verwerk de knopstatus en genereer events.
	void update();
	// Verbruik een korte druk-event als die aanwezig is.
	bool consumeShortPress();
	// Verbruik een long-press trigger-event als die aanwezig is.
	bool consumeLongPressTrigger();
	// Geef aan of de UI in hold-modus staat.
	bool inHoldUi() const;
	// Geef de huidige holdduur in milliseconden terug.
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
