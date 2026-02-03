#pragma once

#include <Arduino.h>
#include <RadioLib.h>

class LoraRadioSx1262 {
public:
	LoraRadioSx1262();
	bool begin(float tcxoVoltage, int16_t& lastErr);

	SX1262& radio();

private:
	Module module_;
	SX1262 radio_;
};
