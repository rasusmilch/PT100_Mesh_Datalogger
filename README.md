# PT100 + MAX31865 Temperature Data Logger (ESP32-S3, ESP-IDF)

This application turns an ESP32-S3 N8R2 into a UTC-centric PT100 data logger with FRAM-first buffering, SD-card CSV batching/verification, DS3231 RTC support, mesh streaming, and serial-console calibration.

## Hardware / wiring

All signal pins are configurable under `idf.py menuconfig -> PT100 Logger`:

- Shared SPI bus: `MOSI`, `MISO`, `SCLK`
- Chip-selects: `MAX31865 CS`, `FRAM CS`, `SD CS`
- DS3231 I2C: `SDA`, `SCL` (address selectable)

Configure mesh root/leaf role and Wi-Fi credentials in the same menu.

## Build / flash

```bash
idf.py set-target esp32s3
idf.py menuconfig   # configure pins, mesh role, batch sizes, flush thresholds
idf.py build flash monitor
```

## Logging pipeline (FRAM → SD with verification)

- Sensor task samples MAX31865 at `log interval` (NVS-backed).
- Each record is appended to a CRC-protected FRAM ring buffer; the persistent header tracks read/write indices and the next sequence number.
- The SD task builds large CSV batches (~64–256 KB, configurable) from FRAM without consuming it, then:
  1. Appends the batch to the daily CSV file (`YYYY-MM-DD.csv`) with `setvbuf` buffering.
  2. `fflush()` + `fsync()`.
  3. Reads back the appended region and checks SHA-256.
  4. On mismatch: truncates to the original size and leaves FRAM untouched.
  5. On success: consumes the matching FRAM records.
- CSV header (written once per day):  
  `seq,epoch_utc,iso8601_utc,raw_rtd_ohms,raw_temp_c,cal_temp_c,flags,node_id`

### Resume / crash safety

- On boot or day rollover the logger:
  1. Opens today’s file, truncates to the last newline if the tail was partial.
  2. Scans the tail to find the last written `seq`.
  3. Drops any FRAM records with `seq <= last_sd_seq` to guarantee no duplicates.
- If FRAM corruption is detected, consumption stops and the corrupted record can be skipped explicitly (see logs).

### Timekeeping

- DS3231 is treated as UTC and seeds system time at boot.
- Mesh root uses SNTP, updates DS3231, and broadcasts time over the mesh; leaves can request/broadcast time updates.

## Serial console commands

- `status`
- `raw` (one sample: raw/calibrated temp + resistance)
- `log interval <ms>`
- `log watermark <records>` (FRAM flush high-water mark)
- `log flush_period <ms>` (periodic SD flush interval)
- `log batch <bytes>` (target SD batch size)
- `log show`
- `cal clear`
- `cal add <raw_c> <actual_c>`
- `cal list`
- `cal apply` (1 point = offset-only; 2–4 points fit deg1–deg3)
- `flush` (best-effort FRAM→SD flush with verification)

All configuration changes persist to NVS.

## Mesh / host streaming

- Leaf nodes send samples upstream; logging to FRAM continues if mesh is down.
- Root node prints one JSON object per line over UART including `seq`, `epoch_utc`, `temps`, `resistance`, and `flags`.

## Test plan

1. **Calibration persistence**: Add points, apply, reboot, and confirm `status` shows the same coefficients and calibrated readings remain adjusted.
2. **Batching size**: Monitor SD writes (logic analyzer or debug prints) to confirm large buffered writes (64–256 KB) rather than per-sample writes.
3. **Crash during flush**: Pull power mid-flush; on reboot confirm the CSV tail is repaired and no duplicated `seq` appears on SD after FRAM drains.
4. **Tail repair**: Manually append a partial line to a CSV, reboot, and confirm it is truncated to the last newline automatically.
5. **Mesh resilience**: Run leaves with the root powered off; verify local FRAM accumulation and later FRAM→SD flush when root/SD returns.
6. **SD removal**: Unmount/remove SD, ensure FRAM continues to grow; remount and verify backlogged records flush with verified appends.
