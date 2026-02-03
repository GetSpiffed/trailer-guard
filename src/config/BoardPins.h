#pragma once

#include <stdint.h>

namespace BoardPins {
// PMU bus
// PMU I2C SDA-pin.
static constexpr uint8_t PMU_SDA = 42;
// PMU I2C SCL-pin.
static constexpr uint8_t PMU_SCL = 41;

// OLED + sensors bus
// I2C SDA-pin voor OLED en sensoren.
static constexpr uint8_t DEV_SDA = 17;
// I2C SCL-pin voor OLED en sensoren.
static constexpr uint8_t DEV_SCL = 18;

// BOOT button
// GPIO-pin voor de BOOT-knop.
static constexpr uint8_t BOOT_PIN = 0; // Button1 (BOOT)

// T-Beam S3 Supreme SX1262 pinmap
// SPI SCK-pin voor de LoRa-radio.
static constexpr int LORA_SCK = 12;
// SPI MISO-pin voor de LoRa-radio.
static constexpr int LORA_MISO = 13;
// SPI MOSI-pin voor de LoRa-radio.
static constexpr int LORA_MOSI = 11;
// Chip-select-pin voor de LoRa-radio.
static constexpr int LORA_CS = 10;
// Reset-pin voor de LoRa-radio.
static constexpr int LORA_RST = 5;
// DIO1-pin voor LoRa-radio interrupts.
static constexpr int LORA_DIO1 = 1;
// Busy-pin voor de LoRa-radio.
static constexpr int LORA_BUSY = 4;

// GPS (L76K)
// RX-pin van de ESP gekoppeld aan GNSS TX.
static constexpr uint8_t GPS_RX_PIN = 9;  // ESP RX <- GNSS TX
// TX-pin van de ESP gekoppeld aan GNSS RX.
static constexpr uint8_t GPS_TX_PIN = 8;  // ESP TX -> GNSS RX
// Enable-pin voor de GPS-module.
static constexpr uint8_t GPS_EN_PIN = 7;  // GPS_EN
// PPS-pin voor tijdpulsen (optioneel).
static constexpr uint8_t GPS_PPS_PIN = 6; // PPS (optional)
} // namespace BoardPins
