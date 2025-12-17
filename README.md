# PT100 + MAX31865 Temperature Data Logger (ESP32-S3, ESP-IDF)

This project logs PT100 temperature samples using a MAX31865 RTD front-end on an ESP32-S3.
It supports:
- 1–4 point **serial-console calibration** (fits a polynomial and stores coefficients in NVS)
- **FRAM-backed** circular buffer (wear-friendly)
- Periodic/batch **flush to SD card** over SPI (CSV files)
- **DS3231 RTC** timestamping (boot from RTC)
- **SNTP/NTP** time sync on the mesh **root** node
- **ESP-WIFI-MESH** streaming to a root node that forwards JSON over UART to a host

## Serial console commands

- `status`  
- `raw` (prints raw temp, resistance, and calibrated temp)
- `log interval <ms>` / `log show`
- `cal clear`
- `cal add <raw_c> <actual_c>`
- `cal list`
- `cal apply`
- `flush` (synchronously flushes all FRAM records to SD)

## Files written on SD

Daily CSV file:
- `/sdcard/pt100_log_YYYYMMDD.csv`

Columns:
`node_id,epoch_sec,ms,raw_temp_c,temp_c,resistance_ohm,seq,flags`

## Build / flash

```bash
idf.py set-target esp32s3
idf.py menuconfig   # configure pins, CS lines, Wi-Fi, mesh, etc.
idf.py build flash monitor
```

## Notes / assumptions

- MAX31865 defaults: **PT100 (R0=100Ω)** and **Rref=430Ω** (common breakout boards).
  If your board uses a different Rref, adjust in `max31865_reader.c`.
- DS3231 time is treated as **UTC**. If you need local time, apply TZ on the host side.
- Root node: enable `PT100 Logger -> Build as mesh root node` and set router SSID/password.
