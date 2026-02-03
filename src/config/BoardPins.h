#pragma once

#include <stdint.h>

namespace BoardPins {
// PMU bus
static constexpr uint8_t PMU_SDA = 42;
static constexpr uint8_t PMU_SCL = 41;

// OLED + sensors bus
static constexpr uint8_t DEV_SDA = 17;
static constexpr uint8_t DEV_SCL = 18;

// BOOT button
static constexpr uint8_t BOOT_PIN = 0; // Button1 (BOOT)

// T-Beam S3 Supreme SX1262 pinmap
static constexpr int LORA_SCK = 12;
static constexpr int LORA_MISO = 13;
static constexpr int LORA_MOSI = 11;
static constexpr int LORA_CS = 10;
static constexpr int LORA_RST = 5;
static constexpr int LORA_DIO1 = 1;
static constexpr int LORA_BUSY = 4;

// GPS (L76K)
static constexpr uint8_t GPS_RX_PIN = 9;  // ESP RX <- GNSS TX
static constexpr uint8_t GPS_TX_PIN = 8;  // ESP TX -> GNSS RX
static constexpr uint8_t GPS_EN_PIN = 7;  // GPS_EN
static constexpr uint8_t GPS_PPS_PIN = 6; // PPS (optional)
} // namespace BoardPins
