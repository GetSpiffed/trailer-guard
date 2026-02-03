#include "drivers/LoraRadioSx1262.h"

#include <SPI.h>

#include "config/BoardPins.h"

LoraRadioSx1262::LoraRadioSx1262()
		: module_(BoardPins::LORA_CS, BoardPins::LORA_DIO1, BoardPins::LORA_RST, BoardPins::LORA_BUSY)
		, radio_(&module_) {
}

bool LoraRadioSx1262::begin(float tcxoVoltage, int16_t& lastErr) {
	SPI.begin(BoardPins::LORA_SCK, BoardPins::LORA_MISO, BoardPins::LORA_MOSI, BoardPins::LORA_CS);

	pinMode(BoardPins::LORA_RST, OUTPUT);
	digitalWrite(BoardPins::LORA_RST, LOW);
	delay(50);
	digitalWrite(BoardPins::LORA_RST, HIGH);
	delay(50);

	int16_t st = radio_.begin();
	if (st != RADIOLIB_ERR_NONE) {
		lastErr = st;
		return false;
	}

	st = radio_.setTCXO(tcxoVoltage);
	if (st != RADIOLIB_ERR_NONE) {
		lastErr = st;
		return false;
	}

	st = radio_.setRegulatorDCDC();
	if (st != RADIOLIB_ERR_NONE) {
		lastErr = st;
		return false;
	}

	st = radio_.setDio2AsRfSwitch(true);
	if (st != RADIOLIB_ERR_NONE) {
		lastErr = st;
		return false;
	}

	radio_.setOutputPower(14);
	radio_.setCurrentLimit(140);

	lastErr = RADIOLIB_ERR_NONE;
	return true;
}

SX1262& LoraRadioSx1262::radio() {
	return radio_;
}
