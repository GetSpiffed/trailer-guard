#pragma once

#include <stdint.h>

namespace AppConfig {
// LoRaWAN settings
static constexpr uint8_t LORAWAN_FPORT = 1;
static constexpr uint32_t UPLINK_INTERVAL_MS = 60000;
static constexpr uint32_t JOIN_RETRY_MS = 10000;
static constexpr uint32_t RESTORE_TEST_DELAY_MS = 2000;
static constexpr uint8_t JOIN_MAX_ATTEMPTS = 8;
static constexpr uint8_t RADIO_INIT_MAX_ATTEMPTS = 5;
static constexpr float RADIO_TCXO_VOLTAGE = 1.6f;

// Tracker thresholds
static constexpr float MOVE_SPEED_MPS = 1.5f;
static constexpr float MOVE_DISTANCE_M = 30.0f;
static constexpr uint32_t KEEPALIVE_INTERVAL_MS = 86400000;

// Boot button timing
static constexpr uint32_t FORCE_OTAA_MIN_MS = 2000;
static constexpr uint32_t FORCE_OTAA_MAX_MS = 6000;
static constexpr uint32_t BOOT_DEBOUNCE_MS = 25;
static constexpr uint32_t BOOT_MIN_PRESS_MS = 20;
static constexpr uint32_t BOOT_MAX_SHORT_MS = 1900;

// GPS
static constexpr uint32_t GPS_BAUD = 9600;
} // namespace AppConfig
