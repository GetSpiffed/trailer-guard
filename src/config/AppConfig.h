#pragma once

#include <stdint.h>

namespace AppConfig {
// LoRaWAN settings
// Applicatiepoort voor LoRaWAN uplinks.
static constexpr uint8_t LORAWAN_FPORT = 1;
// Interval in milliseconden tussen uplinks tijdens beweging.
static constexpr uint32_t UPLINK_INTERVAL_MS = 60000;
// Wachttijd in milliseconden tussen joinpogingen.
static constexpr uint32_t JOIN_RETRY_MS = 10000;
// Vertraagde testtijd in milliseconden na session restore.
static constexpr uint32_t RESTORE_TEST_DELAY_MS = 2000;
// Maximaal aantal joinpogingen voordat falen wordt gemeld.
static constexpr uint8_t JOIN_MAX_ATTEMPTS = 8;
// Maximaal aantal radio-initialisatiepogingen.
static constexpr uint8_t RADIO_INIT_MAX_ATTEMPTS = 5;
// TCXO-voedingsspanning voor de SX1262.
static constexpr float RADIO_TCXO_VOLTAGE = 1.6f;

// Tracker thresholds
// Snelheidsdrempel in m/s om beweging te detecteren.
static constexpr float MOVE_SPEED_MPS = 1.5f;
// Afstandsdrempel in meters om beweging te detecteren.
static constexpr float MOVE_DISTANCE_M = 30.0f;
// Interval in milliseconden tussen keepalive-uplinks.
static constexpr uint32_t KEEPALIVE_INTERVAL_MS = 86400000;

// Boot button timing
// Minimale holdduur om een OTAA-reset te triggeren.
static constexpr uint32_t FORCE_OTAA_MIN_MS = 2000;
// Maximale holdduur voor een geldige OTAA-reset.
static constexpr uint32_t FORCE_OTAA_MAX_MS = 6000;
// Debounce-tijd voor de bootknop.
static constexpr uint32_t BOOT_DEBOUNCE_MS = 25;
// Minimale drukduur om een korte druk te registreren.
static constexpr uint32_t BOOT_MIN_PRESS_MS = 20;
// Maximale drukduur voor een korte druk.
static constexpr uint32_t BOOT_MAX_SHORT_MS = 1900;

// GPS
// Baudrate voor de GPS-seriële verbinding.
static constexpr uint32_t GPS_BAUD = 9600;
} // namespace AppConfig
