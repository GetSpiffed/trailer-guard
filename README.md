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
