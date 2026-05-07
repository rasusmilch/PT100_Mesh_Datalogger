# PT100 Plotter Config Workflow

## Config-first workflow
1. Start the tool: `python host_tools/pt100_plotter.py`.
2. Load an existing report config JSON (or create a new one) at startup.
3. Select logs/folder and then adjust range, plotting, PDF, and CSV actions.
4. Edit options only through **Options → Edit Options**, then explicitly save config.

The report config JSON is the source of truth for plotting and export behavior (including timezone handling, selected y-axis series, filtering controls, and PDF threshold rules).

## Input timezone versus display timezone
- **Input timezone** (`input_timezone_mode`) controls how Start/End text fields are parsed and autofilled.
  - `UTC`: Start/End fields are interpreted in UTC.
  - `Log/source time`: interpreted using log/source timezone when known.
  - `Same as display`: interpreted in whichever display timezone is configured.
- **Display timezone** (`display_timezone`) controls labels shown in plots, report summaries, and range previews.

This separation prevents accidental range shifts while still allowing operator-friendly display labels.

## Log/source time behavior
- Time-capable logs are trimmed by parsed minute boundaries.
- Record-ID-only logs are supported for plotting/reporting, but time-trim entry is unavailable and handled as full-range behavior.
- Single-node reports are expected; mixed/multi-node inputs are surfaced in report metadata so operators can detect unexpected aggregation.

## Why configured y-axis errors block output
The configured `y_axis_series` must be present and numeric in the selected data. The tool intentionally blocks plot/PDF/trimmed export when it is invalid instead of silently falling back. This is a safety guard to avoid incorrect analysis caused by implicit column substitution.

## Sensor fault threshold behavior
- `pdf_sensor_fault_threshold_percent` affects **PDF status-summary visibility** for SENSOR_FAULT rows where MAX31865 `fault_status` byte is zero.
- Below threshold: summary row may be omitted in PDF.
- At/above threshold: summary row is included.
- Any nonzero MAX31865 `fault_status` byte is always reported in PDF fault detail regardless of threshold.

## Plot data vs. summary-statistics filtering
- Plot traces show configured data series for the selected range.
- Statistics table values are computed on a filtered basis:
  - SENSOR_FAULT rows are excluded from stats.
  - For `cal_temp_c`, CAL_VALID==0 rows are excluded from stats.
  - NaN/unparseable values are dropped from stats.

This keeps visual trace fidelity while preventing known-bad values from distorting summary statistics.

## Trimmed CSV export behavior
- Export uses the same selected range and input-timezone parsing rules as plotting/PDF.
- Output removes internal helper columns (for example `__time`, `__source_file`, and other `__*` fields) while preserving operator-facing telemetry columns.

