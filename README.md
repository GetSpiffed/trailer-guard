# T-Beam S3 GPS Anti-Theft Tracker

GPS anti-diefstal tracker op basis van LilyGO T-Beam-S3 Supreme (ESP32-S3).

## Features
- GNSS (L76K)
- LoRa (SX1262, 868 MHz)
- Motion wake-up via accelerometer (QMI8658A)
- Deep sleep voor laag verbruik
- Integratie met Home Assistant (via LoRa + gateway + MQTT)

## Status
- [ ] Board bring-up
- [ ] Accelerometer interrupt
- [ ] Deep sleep / wake
- [ ] GPS fix + parsing
- [ ] LoRa payload
- [ ] Home Assistant integratie

## Troubleshooting

### Join Fail (bijv. `1116 other`) + geen join attempts in TTN
- Dit project gebruikt de **SX1262** pinmap (SCK 12, MISO 13, MOSI 11, CS 10, RST 5, DIO1 1, BUSY 4). Als je doosje ook SX1262 zegt, hoort dit te matchen — controleer dan vooral de bedrading/pins en antenne-setup.【F:src/main.cpp†L37-L43】
- Zorg dat de **LoRa-antenne** is aangesloten vóór je zendt (hardware waarschuwing).【F:docs/t_beam_supreme_hw.md†L14-L18】
- Bevestig dat de **join flow** gestart wordt: na een korte BOOT-press gaat de app pas echt draaien en start de join- en uplink-flow.【F:src/main.cpp†L559-L614】【F:src/main.cpp†L646-L731】
- Verifieer je TTN keys (`JOIN_EUI`, `DEV_EUI`, `NWK_KEY`, `APP_KEY`) in `include/secrets.h`.【F:README.md†L22-L27】

### GPS geeft `NO UART`
- `NO UART` betekent dat er **geen bytes** binnenkomen op de GNSS UART (pin 8/9). De firmware verwacht GNSS TX op GPIO8 en GNSS RX op GPIO9 (zie pinmap).【F:src/main.cpp†L21-L27】【F:docs/t_beam_supreme_hw.md†L54-L56】
- GNSS heeft **ALDO4 power** nodig; de firmware schakelt die aan via de PMU. Als de PMU niet init of ALDO4 niet actief is, krijg je geen UART-data.【F:src/main.cpp†L145-L163】【F:docs/t_beam_supreme_hw.md†L138-L150】

## documentation
https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/docs/en/t_beam_supreme/t_beam_supreme_hw.md

## Secrets (TTN)
1) Kopieer het voorbeeldbestand en vul je keys in:
   - `cp include/secrets.example.h include/secrets.h`
2) Vul in `include/secrets.h` je:
   - `JOIN_EUI`, `DEV_EUI`, `NWK_KEY`, `APP_KEY`

## History cleanup (als keys eerder gecommit zijn)
1) Verwijder het bestand uit de git history:
   - `git filter-repo --path include/secrets.h --invert-paths`
2) Force push:
   - `git push --force --all`
   - `git push --force --tags`
3) Roteer TTN keys na cleanup.
