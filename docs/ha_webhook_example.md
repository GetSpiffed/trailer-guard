# TTN → Home Assistant webhook via Tailscale

1. In The Things Stack (TTN v3): **Integrations → Webhooks → Add webhook**.
2. Kies **Custom webhook**.
3. URL (HTTPS): `https://<tailscale-fqdn>/api/webhook/ttn-lora`.
4. Method: `POST`.
5. Headers:
   - `Content-Type: application/json`
   - optioneel auth header/token van Home Assistant.
6. Body template (voorbeeld met decoded payload):

```json
{
  "device_id": "{{end_device_ids.device_id}}",
  "received_at": "{{received_at}}",
  "f_port": "{{uplink_message.f_port}}",
  "rssi": "{{uplink_message.rx_metadata[0].rssi}}",
  "snr": "{{uplink_message.rx_metadata[0].snr}}",
  "hasFix": "{{uplink_message.decoded_payload.hasFix}}",
  "lat": "{{uplink_message.decoded_payload.lat}}",
  "lon": "{{uplink_message.decoded_payload.lon}}",
  "speedKmh": "{{uplink_message.decoded_payload.speedKmh}}",
  "battPct": "{{uplink_message.decoded_payload.battPct}}"
}
```

Belangrijk JSON-pad in TTN event:
- `uplink_message.decoded_payload.hasFix`
- `uplink_message.decoded_payload.lat`
- `uplink_message.decoded_payload.lon`
- `uplink_message.decoded_payload.speedKmh`
- `uplink_message.decoded_payload.battPct`
