#pragma once

#include <Arduino.h>
#include <RadioLib.h>

// Driverwrapper voor de SX1262 LoRa-radio.
class LoraRadioSx1262 {
public:
	// Maak de radio-driver en modulebinding aan.
	LoraRadioSx1262();
	// Initialiseer de radio met TCXO- en voedinginstellingen.
	bool begin(float tcxoVoltage, int16_t& lastErr);

	// Geef toegang tot de onderliggende SX1262-instantie.
	SX1262& radio();

private:
	Module module_;
	SX1262 radio_;
};
