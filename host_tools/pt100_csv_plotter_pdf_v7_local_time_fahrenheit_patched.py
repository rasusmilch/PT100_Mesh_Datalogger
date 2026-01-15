#!/usr/bin/env python3
"""
PT100 Mesh Logger CSV Plotter + PDF Report

- Loads 1+ CSV exports from PT100 nodes.
- Supports schema_ver 1 and 2.
  - schema_ver 2 adds: record_id (uint64-ish monotonic identifier).
- Plots selected series vs time.
- Optional trim by start/end time (minute resolution) with strict validation.
- Can plot across days by selecting multiple daily CSV files OR selecting a folder.
- Exports a PDF report (ReportLab) with a high-DPI plot image and summary table.

Dependencies:
  - python3
  - pandas
  - matplotlib
  - reportlab
  - tkinter (built-in on most Python installs)

Run:
  python pt100_csv_plotter_pdf_v3.py
"""

from __future__ import annotations

import re
import datetime
import glob
import math
import os
import tempfile
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.widgets import RangeSlider

from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.units import inch
from reportlab.platypus import SimpleDocTemplate, Spacer, Paragraph, Image, Table, TableStyle
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_CENTER

try:
    from zoneinfo import ZoneInfo
except ImportError:  # pragma: no cover - fallback for older Python
    ZoneInfo = None


# --- CSV log_record flags (from main/log_record.h) -------------------------
# We keep a built-in fallback so this host tool can run standalone.
# If the project header is present, we parse it at runtime to stay in sync.

# Known flag labels (short name -> human label).
_FLAG_LABELS: Dict[str, str] = {
    "TIME_VALID": "Time valid",
    "CAL_VALID": "Calibration valid",
    "SD_ERROR": "SD error flagged",
    "MESH_CONNECTED": "Mesh connected",
    "SENSOR_FAULT": "Sensor fault",
    "FRAM_FULL": "FRAM full",
    "RTD_EMA": "RTD EMA used",
}

# Flags that represent an error condition when set.
_ERROR_FLAG_SHORT_NAMES = {"SD_ERROR", "SENSOR_FAULT", "FRAM_FULL"}

# Built-in fallback definitions (mask -> short name).
_FALLBACK_FLAG_DEFS: List[Tuple[int, str]] = [
    (1 << 0, "TIME_VALID"),
    (1 << 1, "CAL_VALID"),
    (1 << 2, "SD_ERROR"),
    (1 << 3, "MESH_CONNECTED"),
    (1 << 4, "SENSOR_FAULT"),
    (1 << 5, "FRAM_FULL"),
    (1 << 6, "RTD_EMA"),
]


def _load_flag_definitions_from_header() -> List[Tuple[int, str]]:
    """Load log record flag bitmasks from the ESP32 project's header.

    Returns:
        A list of (mask, short_name) pairs. If the header is not found or cannot
        be parsed, returns the built-in fallback definitions.
    """
    header_candidates = [
        Path(__file__).resolve().parent / "log_record.h",
        Path(__file__).resolve().parents[1] / "main" / "log_record.h",
        Path.cwd() / "log_record.h",
        Path.cwd() / "main" / "log_record.h",
    ]
    header_path: Optional[Path] = None
    for candidate in header_candidates:
        if candidate.exists():
            header_path = candidate
            break

    if header_path is None:
        return list(_FALLBACK_FLAG_DEFS)

    try:
        header_text = header_path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        return list(_FALLBACK_FLAG_DEFS)

    # Match lines like:
    #   LOG_RECORD_FLAG_TIME_VALID = 1u << 0,
    pattern = re.compile(r"\bLOG_RECORD_FLAG_([A-Z0-9_]+)\s*=\s*1u\s*<<\s*(\d+)\s*,")
    parsed: List[Tuple[int, str]] = []
    for match in pattern.finditer(header_text):
        short_name = match.group(1).strip()
        shift = int(match.group(2))
        if 0 <= shift < 16:
            parsed.append((1 << shift, short_name))

    # If parsing failed, fall back.
    if not parsed:
        return list(_FALLBACK_FLAG_DEFS)

    # Sort by bit position to keep output stable.
    parsed.sort(key=lambda pair: pair[0])
    return parsed


# Resolved definitions used by this tool.
_LOG_RECORD_FLAG_DEFS: List[Tuple[int, str]] = _load_flag_definitions_from_header()


@dataclass
class LoadedLog:
    dataframe: pd.DataFrame
    time_column: str
    time_source: str  # "iso8601_local", "epoch_utc", or "mixed"
    tzinfo: Optional[datetime.tzinfo]
    source_files: List[str]
    dropped_no_time_rows: int


@dataclass
class DisplayTimeConfig:
    display_tz: Optional[datetime.tzinfo]
    display_tz_label: str


@dataclass
class StatsOptions:
    show_min: bool
    show_max: bool
    show_avg: bool
    show_std_band: bool


@dataclass
class HighlightOptions:
    highlight_outside_std: bool
    upper_limit: Optional[float]
    lower_limit: Optional[float]
    highlight_above: bool
    highlight_below: bool
    apply_to_rolling_mean: bool


@dataclass
class PlotOptions:
    overlay_raw_temp: bool
    smooth: bool
    enable_downsample: bool
    max_plot_points: int
    temp_unit: str
    stats: StatsOptions
    highlights: HighlightOptions
    display_time_config: DisplayTimeConfig
    time_source: str


def _human_series_label(series_name: str, temp_unit: str = "C") -> str:
    """Return a human-friendly label for a data series.

    Args:
        series_name: Column name from the CSV (e.g. 'cal_temp_c').
        temp_unit: Temperature unit for display ('C' or 'F'). Only applies to
            temperature series (cal_temp_c/raw_temp_c).

    Returns:
        A label suitable for plot titles/axes.
    """
    unit_upper = (temp_unit or "C").strip().upper()
    temp_suffix = "°F" if unit_upper == "F" else "°C"

    if series_name == "cal_temp_c":
        return f"Calibrated Temperature ({temp_suffix})"
    if series_name == "raw_temp_c":
        return f"Raw Temperature ({temp_suffix})"

    mapping = {
        "raw_rtd_ohms": "RTD Resistance (Ω)",
        "epoch_utc": "UTC Epoch (s)",
        "seq": "Sequence",
        "record_id": "Record ID",
    }
    return mapping.get(series_name, series_name)


def _get_local_tz() -> Optional[datetime.tzinfo]:
    try:
        from dateutil import tz as dateutil_tz

        return dateutil_tz.tzlocal()
    except Exception:
        return datetime.datetime.now().astimezone().tzinfo


def _resolve_display_tz(value: str) -> Optional[datetime.tzinfo]:
    value = (value or "").strip()
    if not value or value.lower().startswith("local"):
        return _get_local_tz()
    if value.upper() == "UTC":
        if ZoneInfo is not None:
            return ZoneInfo("UTC")
        return datetime.timezone.utc
    if ZoneInfo is not None:
        try:
            return ZoneInfo(value)
        except Exception:
            pass
    try:
        from dateutil import tz as dateutil_tz

        return dateutil_tz.gettz(value)
    except Exception:
        return None


def _format_tz_offset(offset: Optional[datetime.timedelta]) -> str:
    if offset is None:
        return "UTC+00:00"
    total_minutes = int(offset.total_seconds() // 60)
    sign = "+" if total_minutes >= 0 else "-"
    total_minutes = abs(total_minutes)
    hours, minutes = divmod(total_minutes, 60)
    return f"UTC{sign}{hours:02d}:{minutes:02d}"


def _format_display_tz_label(
    display_tz: Optional[datetime.tzinfo],
    start_dt: Optional[datetime.datetime],
    end_dt: Optional[datetime.datetime],
) -> str:
    if display_tz is None or start_dt is None or end_dt is None:
        return "n/a"

    start_tz = start_dt.tzname() or "Local"
    end_tz = end_dt.tzname() or "Local"
    start_offset = _format_tz_offset(start_dt.utcoffset())
    end_offset = _format_tz_offset(end_dt.utcoffset())

    if start_tz != end_tz or start_offset != end_offset:
        return f"{start_tz} ({start_offset}) / {end_tz} ({end_offset})"
    return f"{start_tz} ({start_offset})"


def _convert_time_series_to_display_tz(
    time_series: pd.Series,
    display_tz: Optional[datetime.tzinfo],
    time_source: str,
) -> pd.Series:
    parsed = pd.to_datetime(time_series, errors="coerce")
    if display_tz is None:
        return parsed

    try:
        if parsed.dt.tz is not None:
            return parsed.dt.tz_convert(display_tz)
    except Exception:
        return parsed

    if time_source == "epoch_utc":
        localized = parsed.dt.tz_localize(datetime.timezone.utc)
        return localized.dt.tz_convert(display_tz)

    if time_source in ("iso8601_local", "mixed"):
        local_tz = _get_local_tz()
        if local_tz is None:
            return parsed
        localized = parsed.dt.tz_localize(local_tz)
        return localized.dt.tz_convert(display_tz)

    return parsed

def _convert_temperature_series(series_c: pd.Series, temp_unit: str) -> pd.Series:
    """Convert a temperature series from °C to the desired display unit.

    Args:
        series_c: Temperature series in °C.
        temp_unit: Target unit ('C' or 'F').

    Returns:
        Series converted to the requested unit.
    """
    unit_upper = (temp_unit or "C").strip().upper()
    if unit_upper != "F":
        return series_c
    return series_c * 9.0 / 5.0 + 32.0

def _human_time_label(time_column: str) -> str:
    """Return a human-friendly label for the X-axis.

    Args:
        time_column: Internal X-axis column name. Expected values are '__time'
            (datetime) or '__x' (numeric record_id fallback).

    Returns:
        A label suitable for Matplotlib's x-axis.
    """
    if time_column == "__time":
        return "Time"
    if time_column == "__x":
        return "Record ID"
    return time_column


def _normalize_schema(df: pd.DataFrame) -> pd.DataFrame:
    """Normalize schema differences so plotting works across schema versions.

    This tool needs a consistent set of column names and data types across CSV
    schema versions. In particular, the 'flags' column is parsed as an integer
    bitmask (accepting either decimal or hex strings like '0x0042').
    """
    df = df.copy()

    if "schema_ver" in df.columns:
        df["schema_ver"] = pd.to_numeric(df["schema_ver"], errors="coerce")
    else:
        df["schema_ver"] = pd.NA

    if "record_id" not in df.columns:
        if "seq" in df.columns:
            df["record_id"] = pd.to_numeric(df["seq"], errors="coerce")
        else:
            df["record_id"] = pd.NA

    if "flags" in df.columns:
        def _parse_flags_cell(value: object) -> Optional[int]:
            if value is None or (isinstance(value, float) and math.isnan(value)):
                return None
            if isinstance(value, (int, np.integer)):
                return int(value)
            if isinstance(value, str):
                text = value.strip()
                if not text:
                    return None
                try:
                    return int(text, 0)
                except ValueError:
                    return None
            try:
                return int(value)  # type: ignore[arg-type]
            except Exception:
                return None

        parsed_flags = df["flags"].apply(_parse_flags_cell)
        df["flags"] = pd.array(parsed_flags, dtype="Int64")
    else:
        df["flags"] = pd.array([pd.NA] * len(df), dtype="Int64")

    if "node_id" not in df.columns:
        df["node_id"] = ""

    return df



def _pick_time_source(
    df: pd.DataFrame,
    local_tz: Optional[datetime.tzinfo],
) -> Tuple[pd.DataFrame, str, str, Optional[datetime.tzinfo], int]:
    """Choose the best available X-axis source and create either '__time' (datetime)
    or '__x' (numeric).

    Preference:
      1) iso8601_local if it has at least one parseable timestamp.
      2) epoch_utc (>0) otherwise.
      3) record_id (numeric) as a fallback when no usable timestamps exist.

    If both iso8601_local and epoch_utc are present and iso8601_local is parseable, we
    backfill missing/blank iso8601_local values with epoch_utc (converted to the same
    timezone as the iso values) so we don't silently drop valid samples.

    Returns: (filtered_df, x_column_name, time_source, tzinfo, dropped_no_time_rows)

    time_source values:
      - 'iso8601_local'
      - 'epoch_utc'
      - 'mixed' (iso present but had blanks backfilled from epoch)
      - 'record_id' (no usable timestamps; X axis is numeric record_id)
    """
    original_len = len(df)

    iso_series = None
    if "iso8601_local" in df.columns:
        iso_raw = df["iso8601_local"].astype(str).str.strip()
        iso_raw = iso_raw.where(iso_raw.ne(""), other=pd.NA)
        iso_series = pd.to_datetime(iso_raw, errors="coerce", utc=False)

    epoch_series = None
    if "epoch_utc" in df.columns:
        epoch_numeric = pd.to_numeric(df["epoch_utc"], errors="coerce")
        epoch_numeric = epoch_numeric.where(epoch_numeric.gt(0))
        epoch_series = pd.to_datetime(epoch_numeric, unit="s", errors="coerce", utc=True)

    tzinfo: Optional[datetime.tzinfo] = None
    time_source = "unknown"

    if iso_series is not None and iso_series.notna().any():
        first_ts = iso_series.dropna().iloc[0]
        tzinfo = getattr(first_ts, "tzinfo", None) or local_tz
        combined_time = iso_series
        try:
            if tzinfo is not None and combined_time.dt.tz is None:
                combined_time = combined_time.dt.tz_localize(tzinfo)
            elif tzinfo is not None and combined_time.dt.tz is not None:
                combined_time = combined_time.dt.tz_convert(tzinfo)
        except Exception:
            pass

        if epoch_series is not None and epoch_series.notna().any():
            if tzinfo is not None:
                epoch_converted = epoch_series.dt.tz_convert(tzinfo)
            else:
                epoch_converted = epoch_series.dt.tz_localize(None)
            combined_time = combined_time.copy().fillna(epoch_converted)
            time_source = "mixed" if iso_series.isna().any() else "iso8601_local"
        else:
            time_source = "iso8601_local"

        df = df.copy()
        df["__time"] = combined_time
        df = df.loc[df["__time"].notna()].copy()
        dropped_no_time_rows = original_len - len(df)
        return df, "__time", time_source, tzinfo, dropped_no_time_rows

    if epoch_series is not None and epoch_series.notna().any():
        tzinfo = datetime.timezone.utc
        df = df.copy()
        df["__time"] = epoch_series
        df = df.loc[df["__time"].notna()].copy()
        dropped_no_time_rows = original_len - len(df)
        return df, "__time", "epoch_utc", tzinfo, dropped_no_time_rows

    # Fallback: no usable timestamps. Use record_id as numeric X axis and sort by it.
    if "record_id" in df.columns:
        record_id_numeric = pd.to_numeric(df["record_id"], errors="coerce")
        if record_id_numeric.notna().any():
            df = df.copy()
            df["__x"] = record_id_numeric
            df = df.loc[df["__x"].notna()].copy()
            dropped_no_time_rows = original_len - len(df)
            return df, "__x", "record_id", None, dropped_no_time_rows

    raise ValueError(
        "No usable X-axis found. Need iso8601_local (parseable), epoch_utc (>0), or record_id."
    )


def _parse_user_time(text: str) -> Optional[datetime.datetime]:
    text = (text or "").strip()
    if not text:
        return None

    for fmt in ("%Y-%m-%d %H:%M", "%Y-%m-%dT%H:%M"):
        try:
            return datetime.datetime.strptime(text, fmt)
        except ValueError:
            continue
    raise ValueError('Invalid time format. Use "YYYY-MM-DD HH:MM" (minutes only).')


def _nearest_minute_string(minutes_index: pd.DatetimeIndex, target: datetime.datetime) -> str:
    target_ts = pd.Timestamp(target)
    if minutes_index.tz is not None and target_ts.tzinfo is None:
        target_ts = target_ts.tz_localize(minutes_index.tz)
    elif minutes_index.tz is None and target_ts.tzinfo is not None:
        target_ts = target_ts.tz_convert(None)

    pos = minutes_index.get_indexer([target_ts], method="nearest")[0]
    nearest = minutes_index[pos]
    return nearest.strftime("%Y-%m-%d %H:%M")


def _validate_and_trim_by_minute(
    df: pd.DataFrame,
    time_column: str,
    start_text: str,
    end_text: str,
    time_series: Optional[pd.Series] = None,
) -> Tuple[pd.DataFrame, str, str, str]:
    """Trim strictly against minute buckets that exist in the log.

    If the X axis is record_id-based (time_column == '__x'), trimming-by-time is not
    supported. In that mode, start/end must be blank and the full dataset is used.
    """
    if time_column == "__x":
        if (start_text or "").strip() or (end_text or "").strip():
            raise ValueError(
                "This log has no usable timestamps; X-axis uses record_id.\n"
                "Time trimming is unavailable. Clear start/end to plot the full range."
            )
        x_numeric = pd.to_numeric(df[time_column], errors="coerce")
        if x_numeric.isna().all():
            raise ValueError("record_id X-axis could not be parsed.")
        start_label = f"record_id {int(x_numeric.min())}"
        end_label = f"record_id {int(x_numeric.max())}"
        summary = f"{start_label} → {end_label} ({len(df):,} rows)"
        return df.copy(), start_label, end_label, summary

    if time_series is None:
        time_series = pd.to_datetime(df[time_column], errors="coerce")
    if time_series.isna().all():
        raise ValueError("Time column could not be parsed.")

    minutes = time_series.dt.floor("min")
    present_minutes = pd.DatetimeIndex(minutes.unique()).sort_values()

    user_start = _parse_user_time(start_text)
    user_end = _parse_user_time(end_text)

    if user_start and user_end and user_end < user_start:
        raise ValueError("End time is earlier than start time.")

    start_minute = None
    end_minute = None

    if user_start:
        start_ts = pd.Timestamp(user_start)
        if present_minutes.tz is not None and start_ts.tzinfo is None:
            start_ts = start_ts.tz_localize(present_minutes.tz)
        if start_ts not in present_minutes:
            nearest = _nearest_minute_string(present_minutes, user_start)
            raise ValueError(f"Start time not in log minutes. Nearest valid minute: {nearest}")
        start_minute = start_ts

    if user_end:
        end_ts = pd.Timestamp(user_end)
        if present_minutes.tz is not None and end_ts.tzinfo is None:
            end_ts = end_ts.tz_localize(present_minutes.tz)
        if end_ts not in present_minutes:
            nearest = _nearest_minute_string(present_minutes, user_end)
            raise ValueError(f"End time not in log minutes. Nearest valid minute: {nearest}")
        end_minute = end_ts

    mask = pd.Series(True, index=df.index)
    if start_minute is not None:
        mask &= minutes >= start_minute
    if end_minute is not None:
        mask &= minutes <= end_minute

    trimmed = df.loc[mask].copy()
    if trimmed.empty:
        raise ValueError("Trimming resulted in 0 rows.")

    trimmed_times = time_series.loc[trimmed.index]
    actual_start = pd.to_datetime(trimmed_times).min()
    actual_end = pd.to_datetime(trimmed_times).max()
    start_label = actual_start.strftime("%Y-%m-%d %H:%M")
    end_label = actual_end.strftime("%Y-%m-%d %H:%M")
    summary = f"{start_label} → {end_label} ({len(trimmed):,} rows)"
    return trimmed, start_label, end_label, summary


def _load_csv_files(file_paths: List[str]) -> LoadedLog:
    if not file_paths:
        raise ValueError("No files selected.")

    dataframes: List[pd.DataFrame] = []
    for path in file_paths:
        df = pd.read_csv(path)
        df = _normalize_schema(df)
        df["__source_file"] = os.path.basename(path)
        dataframes.append(df)

    combined = pd.concat(dataframes, ignore_index=True)
    local_tz = _get_local_tz()
    combined, time_column, time_source, tzinfo, dropped_no_time_rows = _pick_time_source(
        combined,
        local_tz=local_tz,
    )

    if time_column == "__time":
        combined = combined.sort_values(by=time_column).reset_index(drop=True)
    else:
        # record_id fallback: numeric X axis
        combined[time_column] = pd.to_numeric(combined[time_column], errors="coerce")
        combined = combined.sort_values(by=time_column).reset_index(drop=True)

    return LoadedLog(
        dataframe=combined,
        time_column=time_column,
        time_source=time_source,
        tzinfo=tzinfo,
        source_files=file_paths,
        dropped_no_time_rows=dropped_no_time_rows,
    )


def _compute_basic_stats(series: pd.Series) -> Dict[str, str]:
    numeric = pd.to_numeric(series, errors="coerce").dropna()
    if numeric.empty:
        return {"min": "n/a", "avg": "n/a", "max": "n/a", "std": "n/a"}
    return {
        "min": f"{numeric.min():.3f}",
        "avg": f"{numeric.mean():.3f}",
        "max": f"{numeric.max():.3f}",
        "std": f"{numeric.std(ddof=0):.3f}",
    }



def _format_flags_summary(df: pd.DataFrame) -> str:
    """Build a human-readable summary of the log_record flags.

    Args:
        df: Log dataframe (already normalized).

    Returns:
        A multi-line string suitable for the PDF summary table, or 'n/a' if the
        CSV does not contain usable flags.
    """
    if "flags" not in df.columns:
        return "n/a"

    flags_series = df["flags"]
    if flags_series is None:
        return "n/a"

    flags_numeric = pd.to_numeric(flags_series, errors="coerce").dropna()
    if flags_numeric.empty:
        return "n/a"

    # Convert to Python ints for stable bitwise ops.
    flags_int = flags_numeric.astype("int64")
    total = int(flags_int.shape[0])
    if total <= 0:
        return "n/a"

    def _pct(count: int) -> str:
        return f"{(100.0 * float(count) / float(total)):.1f}%"

    lines_out: List[str] = []
    for mask, short_name in _LOG_RECORD_FLAG_DEFS:
        label = _FLAG_LABELS.get(short_name, short_name)
        set_count = int(((flags_int & mask) != 0).sum())
        if short_name in ("TIME_VALID", "CAL_VALID"):
            cleared = total - set_count
            lines_out.append(
                f"{label}: {set_count}/{total} ({_pct(set_count)})"
                + (f"  [invalid: {cleared}/{total} ({_pct(cleared)})]" if cleared else "")
            )
        elif short_name in _ERROR_FLAG_SHORT_NAMES:
            lines_out.append(f"{label}: {set_count}/{total} ({_pct(set_count)})")
        else:
            lines_out.append(f"{label}: {set_count}/{total} ({_pct(set_count)})")

    return "\n".join(lines_out)


def _apply_flag_filters_for_stats(
    df: pd.DataFrame,
    y_series: pd.Series,
    y_name: str,
) -> Tuple[pd.Series, List[str]]:
    """Filter obviously-invalid samples from statistics based on log flags.

    We keep the plot series as-is (so faults remain visible), but compute
    statistics on samples that are likely meaningful.

    Rules:
      - Exclude SENSOR_FAULT from stats for all series.
      - For cal_temp_c, also require CAL_VALID.

    Args:
        df: Dataframe with a parsed Int64 'flags' column.
        y_series: Display-series values aligned to df.
        y_name: Column name being summarized.

    Returns:
        (filtered_series, notes) where notes describes what was excluded.
    """
    if "flags" not in df.columns:
        return y_series, []

    flags_series = df["flags"]
    flags_numeric = pd.to_numeric(flags_series, errors="coerce").fillna(0).astype("int64")
    include_mask = pd.Series(True, index=df.index)

    # Find masks (from header parsing, if available).
    sensor_fault_mask = 0
    cal_valid_mask = 0
    for mask, short_name in _LOG_RECORD_FLAG_DEFS:
        if short_name == "SENSOR_FAULT":
            sensor_fault_mask = mask
        if short_name == "CAL_VALID":
            cal_valid_mask = mask

    notes: List[str] = []

    if sensor_fault_mask:
        excluded_fault = (flags_numeric & sensor_fault_mask) != 0
        excluded_count = int(excluded_fault.sum())
        if excluded_count:
            include_mask &= ~excluded_fault
            notes.append(f"excluded SENSOR_FAULT rows: {excluded_count}")

    if y_name == "cal_temp_c" and cal_valid_mask:
        invalid_cal = (flags_numeric & cal_valid_mask) == 0
        invalid_count = int(invalid_cal.sum())
        if invalid_count:
            include_mask &= ~invalid_cal
            notes.append(f"excluded CAL_VALID==0 rows: {invalid_count}")

    filtered = y_series[include_mask]
    return filtered, notes


def _compute_numeric_stats(series: pd.Series) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
    numeric = pd.to_numeric(series, errors="coerce").dropna()
    if numeric.empty:
        return None, None, None, None
    return float(numeric.min()), float(numeric.mean()), float(numeric.max()), float(numeric.std(ddof=0))


def _downsample_positions_minmax(series_list: List[pd.Series], max_plot_points: int) -> np.ndarray:
    """Return sorted positional indices for plotting a large time series.

    This reduces point count while preserving short spikes and discontinuities.

    Strategy:
      - Split the series into buckets.
      - For each bucket, keep: first, last, min, and max (per bucket).
      - Also consider min/max across any additional series passed in (e.g. overlay).

    The output indices are monotonically increasing and unique.
    """
    if max_plot_points <= 0:
        raise ValueError(f"max_plot_points must be > 0, got {max_plot_points}")

    sample_count = int(series_list[0].shape[0])
    if sample_count <= max_plot_points:
        return np.arange(sample_count, dtype=np.int64)

    # We may emit up to 4 points per bucket (first/min/max/last). Use bucket_count
    # to roughly cap the total output size at max_plot_points.
    bucket_count = max(1, max_plot_points // 4)
    bucket_size = int(math.ceil(sample_count / bucket_count))

    selected_positions: List[int] = []
    start_pos = 0
    while start_pos < sample_count:
        end_pos = min(sample_count, start_pos + bucket_size)

        bucket_positions = {start_pos, end_pos - 1}

        for series in series_list:
            values = pd.to_numeric(series.iloc[start_pos:end_pos], errors="coerce").to_numpy(dtype=float, copy=False)
            if np.all(np.isnan(values)):
                continue
            # Use nan-safe argmin/argmax to preserve spikes even with missing data.
            bucket_positions.add(start_pos + int(np.nanargmin(values)))
            bucket_positions.add(start_pos + int(np.nanargmax(values)))

        selected_positions.extend(sorted(bucket_positions))
        start_pos = end_pos

    # Ensure uniqueness and strict ordering.
    return np.unique(np.asarray(selected_positions, dtype=np.int64))


def _mask_to_spans(x_values: pd.Series, mask: np.ndarray) -> List[Tuple[object, object]]:
    """Convert a boolean mask into contiguous spans on the X axis.

    NaN values in the mask are treated as False. Single-point spans expand to the
    next distinct sample when possible so the span has non-zero width.
    """
    mask_series = pd.Series(mask).fillna(False)
    mask_bool = mask_series.to_numpy(dtype=bool, copy=False)
    if mask_bool.size == 0:
        return []

    padded = np.concatenate(([False], mask_bool, [False]))
    starts = np.flatnonzero(padded[1:] & ~padded[:-1])
    ends = np.flatnonzero(~padded[1:] & padded[:-1]) - 1

    spans: List[Tuple[object, object]] = []
    last_index = len(x_values) - 1
    for start_idx, end_idx in zip(starts, ends):
        if start_idx > last_index:
            continue
        end_target = min(end_idx, last_index)
        x0 = x_values.iloc[start_idx]
        x1 = _next_distinct_x_value(x_values, end_target)
        if pd.isna(x0) or pd.isna(x1):
            continue
        spans.append((x0, x1))

    return _merge_spans(spans, _median_sample_delta(x_values))


def _next_distinct_x_value(x_values: pd.Series, end_idx: int) -> object:
    last_index = len(x_values) - 1
    if end_idx > last_index:
        end_idx = last_index
    end_value = x_values.iloc[end_idx]
    next_idx = min(end_idx + 1, last_index)
    while next_idx <= last_index and x_values.iloc[next_idx] == end_value:
        next_idx += 1
    if next_idx <= last_index:
        return x_values.iloc[next_idx]
    return _nudge_span_end(end_value)


def _nudge_span_end(value: object) -> object:
    if isinstance(value, (pd.Timestamp, datetime.datetime, np.datetime64)):
        return pd.Timestamp(value) + pd.Timedelta(seconds=1)
    try:
        return value + 1
    except Exception:
        return value


def _median_sample_delta(x_values: pd.Series) -> Optional[object]:
    diffs = pd.Series(x_values).diff().dropna()
    if diffs.empty:
        return None
    try:
        diffs = diffs[diffs > 0]
    except Exception:
        pass
    if diffs.empty:
        return None
    return diffs.median()


def _merge_spans(
    spans: List[Tuple[object, object]],
    gap_threshold: Optional[object],
) -> List[Tuple[object, object]]:
    if not spans:
        return []
    spans_sorted = sorted(spans, key=lambda span: span[0])
    merged = [spans_sorted[0]]
    for start, end in spans_sorted[1:]:
        last_start, last_end = merged[-1]
        if start <= last_end or _gap_within_threshold(start, last_end, gap_threshold):
            merged[-1] = (last_start, max(last_end, end))
        else:
            merged.append((start, end))
    return merged


def _gap_within_threshold(start: object, end: object, gap_threshold: Optional[object]) -> bool:
    if gap_threshold is None:
        return False
    try:
        return (start - end) <= gap_threshold
    except Exception:
        return False


def _add_highlight_spans(
    ax: plt.Axes,
    x_values: pd.Series,
    mask: np.ndarray,
    *,
    label: str,
    color: str,
    alpha: float,
) -> None:
    spans = _mask_to_spans(x_values, mask)
    if not spans:
        return
    for idx, (x0, x1) in enumerate(spans):
        ax.axvspan(
            x0,
            x1,
            facecolor=color,
            alpha=alpha,
            linewidth=0,
            antialiased=False,
            label=label if idx == 0 else "_nolegend_",
        )


def _build_figure(
    df: pd.DataFrame,
    time_column: str,
    y_name: str,
    plot_title: str,
    suptitle: Optional[str],
    options: PlotOptions,
) -> Tuple[plt.Figure, int, int]:
    """Build a Matplotlib figure for the selected series.

    Args:
        df: Trimmed dataframe.
        time_column: '__time' (datetime) or '__x' (numeric record_id fallback).
        y_name: Column name for the primary Y series.
        plot_title: Title for the axes.
        suptitle: Optional figure-level subtitle (e.g., nodes and time range).
        options: Plot options including smoothing, overlays, and time zone display settings.

    Returns:
        (figure, plotted_point_count, total_point_count)
    """
    total_points = int(df.shape[0])

    x_values = df[time_column]
    if time_column == "__time":
        x_values = _convert_time_series_to_display_tz(
            df[time_column],
            display_tz=options.display_time_config.display_tz,
            time_source=options.time_source,
        )

    y_series_c = pd.to_numeric(df[y_name], errors="coerce")
    y_series = (
        _convert_temperature_series(y_series_c, options.temp_unit)
        if y_name in ("cal_temp_c", "raw_temp_c")
        else y_series_c
    )

    raw_temp_series: Optional[pd.Series] = None
    if options.overlay_raw_temp and "raw_temp_c" in df.columns and y_name != "raw_temp_c":
        raw_temp_c = pd.to_numeric(df["raw_temp_c"], errors="coerce")
        raw_temp_series = _convert_temperature_series(raw_temp_c, options.temp_unit)

    # Compute smoothing on the full series first (so downsampling doesn't bias the trend).
    smoothed_series: Optional[pd.Series] = None
    window_size = 0
    valid_points = int(y_series.notna().sum())
    if options.smooth and valid_points >= 3:
        window_size = min(151, max(3, valid_points // 40))
        smoothed_series = y_series.rolling(
            window=window_size,
            center=True,
            min_periods=max(3, window_size // 4),
        ).mean()

    plot_positions = np.arange(total_points, dtype=np.int64)
    if options.enable_downsample and total_points > options.max_plot_points:
        series_for_downsample: List[pd.Series] = [y_series]
        if raw_temp_series is not None:
            series_for_downsample.append(raw_temp_series)
        plot_positions = _downsample_positions_minmax(
            series_list=series_for_downsample,
            max_plot_points=options.max_plot_points,
        )

    x_plot = x_values.iloc[plot_positions]
    y_plot = y_series.iloc[plot_positions]

    fig, ax = plt.subplots(figsize=(11, 6.2))
    ax.grid(True)
    ax.set_title(plot_title)

    if suptitle:
        fig.suptitle(suptitle, fontsize=10)

    if time_column == "__time" and options.display_time_config.display_tz_label not in ("", "n/a"):
        ax.set_xlabel(f"{_human_time_label(time_column)} ({options.display_time_config.display_tz_label})")
    else:
        ax.set_xlabel(_human_time_label(time_column))
    ax.set_ylabel(_human_series_label(y_name, temp_unit=options.temp_unit))

    if options.smooth and smoothed_series is not None:
        ax.plot(x_plot, y_plot, linewidth=0.7, alpha=0.7, label="data")
        ax.plot(x_plot, smoothed_series.iloc[plot_positions], linewidth=2.0, label="rolling mean")
    else:
        ax.plot(x_plot, y_plot, linewidth=1.2, label="data")

    if raw_temp_series is not None:
        ax.plot(x_plot, raw_temp_series.iloc[plot_positions], linewidth=0.9, alpha=0.7, label="raw_temp_c (data)")

    is_temperature = y_name in ("cal_temp_c", "raw_temp_c")
    if is_temperature:
        stats_min, stats_avg, stats_max, stats_std = _compute_numeric_stats(y_series)
        if stats_avg is not None:
            if options.stats.show_min and stats_min is not None:
                ax.axhline(stats_min, color="#8b0000", linestyle="--", linewidth=1.0, label="min")
            if options.stats.show_max and stats_max is not None:
                ax.axhline(stats_max, color="#8b0000", linestyle="--", linewidth=1.0, label="max")
            if options.stats.show_avg:
                ax.axhline(stats_avg, color="#1f77b4", linestyle="-.", linewidth=1.2, label="avg")
            if options.stats.show_std_band and stats_std is not None:
                ax.fill_between(
                    x_plot,
                    stats_avg - stats_std,
                    stats_avg + stats_std,
                    color="#1f77b4",
                    alpha=0.16,
                    label="±1σ",
                )

        highlight_series = y_series
        highlight_basis = "data"
        if options.highlights.apply_to_rolling_mean and smoothed_series is not None:
            highlight_series = smoothed_series
            highlight_basis = "rolling mean"

        if options.highlights.highlight_outside_std and stats_avg is not None and stats_std is not None:
            outside_full = (highlight_series < stats_avg - stats_std) | (highlight_series > stats_avg + stats_std)
            _add_highlight_spans(
                ax,
                x_values,
                outside_full,
                label=f"outside ±1σ ({highlight_basis})",
                color="#ff7f0e",
                alpha=0.16,
            )

        if options.highlights.upper_limit is not None:
            ax.axhline(
                options.highlights.upper_limit,
                color="#d62728",
                linestyle="--",
                linewidth=1.2,
                label="upper limit",
            )
        if options.highlights.highlight_above and options.highlights.upper_limit is not None:
            above_full = highlight_series > options.highlights.upper_limit
            _add_highlight_spans(
                ax,
                x_values,
                above_full,
                label="above upper",
                color="#d62728",
                alpha=0.16,
            )

        if options.highlights.lower_limit is not None:
            ax.axhline(
                options.highlights.lower_limit,
                color="#9467bd",
                linestyle="--",
                linewidth=1.2,
                label="lower limit",
            )
        if options.highlights.highlight_below and options.highlights.lower_limit is not None:
            below_full = highlight_series < options.highlights.lower_limit
            _add_highlight_spans(
                ax,
                x_values,
                below_full,
                label="below lower",
                color="#9467bd",
                alpha=0.16,
            )

    ax.legend()

    # Improve time axis readability: show local time with a concise formatter that
    # adds the date only when needed.
    if time_column == "__time":
        tzinfo = None
        try:
            tzinfo = x_values.dt.tz
        except Exception:
            tzinfo = None

        locator = mdates.AutoDateLocator(minticks=4, maxticks=10)
        ax.xaxis.set_major_locator(locator)
        ax.xaxis.set_major_formatter(mdates.ConciseDateFormatter(locator, tz=tzinfo))

    # Tight layout; reserve space if we used a suptitle.
    if suptitle:
        fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.94])
    else:
        fig.tight_layout()

    return fig, int(len(plot_positions)), total_points


def _node_ids_from_df(df: pd.DataFrame) -> str:
    if "node_id" not in df.columns:
        return "n/a"
    nodes = sorted({str(v) for v in df["node_id"].dropna().unique()})
    if not nodes:
        return "n/a"
    if len(nodes) <= 4:
        return ", ".join(nodes)
    return f"{nodes[0]} … ({len(nodes)} total)"


def _export_pdf_report(
    save_path: str,
    fig_png_path: str,
    source_files: List[str],
    summary_rows: List[List[str]],
    title: str,
    subtitle: str,
) -> None:
    half_inch = 0.5 * inch
    doc = SimpleDocTemplate(
        save_path,
        pagesize=letter,
        leftMargin=half_inch,
        rightMargin=half_inch,
        topMargin=half_inch,
        bottomMargin=half_inch,
    )

    styles_title = ParagraphStyle(
        "TitleStyle",
        fontName="Helvetica-Bold",
        fontSize=18,
        alignment=TA_CENTER,
        spaceAfter=8,
    )
    styles_sub = ParagraphStyle(
        "SubStyle",
        fontName="Helvetica",
        fontSize=10.5,
        alignment=TA_CENTER,
        textColor=colors.grey,
        spaceAfter=14,
    )
    styles_body = ParagraphStyle(
        "BodyStyle",
        fontName="Helvetica",
        fontSize=10,
        leading=13,
    )

    elements: List[object] = []
    elements.append(Paragraph(title, styles_title))
    elements.append(Paragraph(subtitle, styles_sub))

    table = Table(summary_rows, colWidths=[2.3 * inch, 4.7 * inch])
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (1, 0), colors.lightgrey),
                ("TEXTCOLOR", (0, 0), (1, 0), colors.black),
                ("FONTNAME", (0, 0), (1, 0), "Helvetica-Bold"),
                ("ALIGN", (0, 0), (1, 0), "LEFT"),
                ("FONTNAME", (0, 1), (1, -1), "Helvetica"),
                ("FONTSIZE", (0, 0), (1, -1), 9.5),
                ("BOTTOMPADDING", (0, 0), (1, 0), 7),
                ("TOPPADDING", (0, 0), (1, 0), 7),
                ("GRID", (0, 0), (-1, -1), 0.6, colors.grey),
            ]
        )
    )
    elements.append(table)
    elements.append(Spacer(1, 10))

    img = Image(fig_png_path, width=7.5 * inch, height=4.2 * inch)
    elements.append(img)
    elements.append(Spacer(1, 10))

    file_list = "\n".join([os.path.basename(p) for p in source_files[:12]])
    if len(source_files) > 12:
        file_list += f"\n… ({len(source_files)} files total)"
    elements.append(Paragraph(f"<b>Input file(s):</b><br/>{file_list}", styles_body))

    doc.build(elements)


def _export_pdf_report_vector(
    save_path: str,
    fig: plt.Figure,
    source_files: List[str],
    summary_rows: List[List[str]],
    title: str,
    subtitle: str,
) -> None:
    """Export a vector PDF using Matplotlib's PDF backend.

    This avoids rasterization artifacts and is typically better for printing.
    """
    # Page 1: plot (vector).
    # Reserve some headroom for a report header above the plot title.
    fig.subplots_adjust(top=0.82)
    fig.text(
        0.5,
        0.985,
        title,
        ha="center",
        va="top",
        fontsize=14,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.955,
        subtitle,
        ha="center",
        va="top",
        fontsize=9,
        color="gray",
    )

    with PdfPages(save_path) as pdf:
        pdf.savefig(fig)

        # Page 2: summary table + input file list.
        summary_fig = plt.figure(figsize=(8.5, 11.0))  # letter portrait
        ax = summary_fig.add_axes([0.05, 0.05, 0.90, 0.90])
        ax.axis("off")

        ax.text(
            0.5,
            1.02,
            "Summary",
            ha="center",
            va="bottom",
            fontsize=14,
            fontweight="bold",
            transform=ax.transAxes,
        )

        # Build a Matplotlib table (kept as vector text/lines in the PDF).
        # summary_rows includes a header row: ["Field", "Value"].
        cell_text = summary_rows[1:] if len(summary_rows) > 1 else []
        col_labels = summary_rows[0] if summary_rows else ["Field", "Value"]

        table = ax.table(
            cellText=cell_text,
            colLabels=col_labels,
            cellLoc="left",
            colLoc="left",
            loc="upper left",
            bbox=[0.0, 0.25, 1.0, 0.70],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9)

        # Input file list at the bottom.
        file_list = "\n".join([os.path.basename(p) for p in source_files[:20]])
        if len(source_files) > 20:
            file_list += f"\n… ({len(source_files)} files total)"

        ax.text(
            0.0,
            0.18,
            "Input file(s):",
            ha="left",
            va="top",
            fontsize=11,
            fontweight="bold",
            transform=ax.transAxes,
        )
        ax.text(
            0.0,
            0.15,
            file_list if file_list else "(none)",
            ha="left",
            va="top",
            fontsize=9,
            transform=ax.transAxes,
            family="monospace",
        )

        pdf.savefig(summary_fig)
        plt.close(summary_fig)


class PlotterApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PT100 CSV Plotter + PDF Report")

        self.selected_files: List[str] = []
        self.loaded: Optional[LoadedLog] = None

        self.start_time_text = tk.StringVar(value="")
        self.end_time_text = tk.StringVar(value="")
        self.display_tz_choice = tk.StringVar(value="Local")
        self.scenario_choice = tk.StringVar(value="General")

        self.y_choice = tk.StringVar(value="cal_temp_c")
        self.series_choices = ["cal_temp_c", "raw_temp_c", "raw_rtd_ohms"]
        self.temp_f = tk.BooleanVar(value=False)
        self.overlay_raw = tk.BooleanVar(value=False)
        self.smooth = tk.BooleanVar(value=True)
        self.enable_downsample = tk.BooleanVar(value=True)
        self.max_plot_points = tk.IntVar(value=20000)
        self.pdf_plot_dpi = tk.IntVar(value=600)
        self.pdf_vector = tk.BooleanVar(value=False)
        self.stats_show_min = tk.BooleanVar(value=False)
        self.stats_show_max = tk.BooleanVar(value=False)
        self.stats_show_avg = tk.BooleanVar(value=False)
        self.stats_show_std = tk.BooleanVar(value=False)
        self.highlight_outside_std = tk.BooleanVar(value=False)
        self.highlight_apply_to_rolling_mean = tk.BooleanVar(value=False)
        self.highlight_above = tk.BooleanVar(value=False)
        self.highlight_below = tk.BooleanVar(value=False)
        self.highlight_upper_limit = tk.StringVar(value="")
        self.highlight_lower_limit = tk.StringVar(value="")
        self._warned_aggregated = False

        self._build_ui()

    def _build_ui(self) -> None:
        frm = tk.Frame(self.root, padx=12, pady=12)
        frm.pack(fill="both", expand=True)

        tk.Label(frm, text="PT100 CSV file(s):").grid(row=0, column=0, sticky="w")
        self.file_label = tk.Label(frm, text="(none selected)", anchor="w", justify="left")
        self.file_label.grid(row=0, column=1, sticky="w")

        tk.Button(frm, text="Select CSV Files", command=self.select_files).grid(
            row=1, column=0, sticky="w", pady=(6, 0)
        )
        tk.Button(frm, text="Select Folder (all *.csv)", command=self.select_folder).grid(
            row=1, column=1, sticky="w", pady=(6, 0)
        )

        row = 2
        tk.Label(frm, text='Start time (YYYY-MM-DD HH:MM):').grid(row=row, column=0, sticky="w", pady=(12, 0))
        tk.Entry(frm, textvariable=self.start_time_text, width=26).grid(row=row, column=1, sticky="w", pady=(12, 0))
        row += 1

        tk.Label(frm, text='End time (YYYY-MM-DD HH:MM):').grid(row=row, column=0, sticky="w", pady=(6, 0))
        tk.Entry(frm, textvariable=self.end_time_text, width=26).grid(row=row, column=1, sticky="w", pady=(6, 0))
        row += 1

        tk.Button(frm, text="Select range…", command=self.open_range_selector).grid(row=row, column=1, sticky="w", pady=(4, 0))
        row += 1

        tk.Label(frm, text="Display time zone:").grid(row=row, column=0, sticky="w", pady=(10, 0))
        tz_choices = ["Local", "UTC"]
        tz_combo = ttk.Combobox(frm, textvariable=self.display_tz_choice, values=tz_choices, width=24)
        tz_combo.grid(row=row, column=1, sticky="w", pady=(10, 0))
        row += 1

        tk.Label(frm, text="Scenario:").grid(row=row, column=0, sticky="w", pady=(6, 0))
        scenario_combo = ttk.Combobox(frm, textvariable=self.scenario_choice, values=["General", "Thermal cycling"], width=20)
        scenario_combo.grid(row=row, column=1, sticky="w", pady=(6, 0))
        scenario_combo.bind("<<ComboboxSelected>>", self._handle_scenario_change)
        row += 1

        tk.Label(frm, text="Y-axis series:").grid(row=row, column=0, sticky="w", pady=(12, 0))
        tk.OptionMenu(frm, self.y_choice, *self.series_choices).grid(row=row, column=1, sticky="w", pady=(12, 0))
        row += 1
        tk.Checkbutton(frm, text="Display temperature in °F", variable=self.temp_f).grid(row=row, column=1, sticky="w")
        row += 1

        tk.Checkbutton(frm, text="Overlay raw_temp_c", variable=self.overlay_raw).grid(row=row, column=1, sticky="w")
        row += 1
        tk.Checkbutton(frm, text="Smooth (rolling mean, numeric series)", variable=self.smooth).grid(
            row=row,
            column=1,
            sticky="w",
        )
        row += 1
        tk.Checkbutton(frm, text="Downsample large datasets (preserve spikes)", variable=self.enable_downsample).grid(row=row, column=1, sticky="w")
        row += 1

        stats_frame = tk.LabelFrame(frm, text="Statistics", padx=8, pady=6)
        stats_frame.grid(row=row, column=0, columnspan=2, sticky="w", pady=(10, 0))
        tk.Checkbutton(stats_frame, text="Show minimum", variable=self.stats_show_min).grid(row=0, column=0, sticky="w")
        tk.Checkbutton(stats_frame, text="Show maximum", variable=self.stats_show_max).grid(row=0, column=1, sticky="w")
        tk.Checkbutton(stats_frame, text="Show average", variable=self.stats_show_avg).grid(row=1, column=0, sticky="w")
        tk.Checkbutton(stats_frame, text="Show standard deviation band (±1σ)", variable=self.stats_show_std).grid(
            row=1,
            column=1,
            sticky="w",
        )
        row += 1

        highlight_frame = tk.LabelFrame(frm, text="Out-of-bounds highlighting", padx=8, pady=6)
        highlight_frame.grid(row=row, column=0, columnspan=2, sticky="w", pady=(10, 0))
        tk.Checkbutton(
            highlight_frame,
            text="Highlight outside mean ± 1σ",
            variable=self.highlight_outside_std,
        ).grid(row=0, column=0, columnspan=2, sticky="w")
        tk.Checkbutton(
            highlight_frame,
            text="Compute highlight mask from rolling mean (if enabled)",
            variable=self.highlight_apply_to_rolling_mean,
        ).grid(row=1, column=0, columnspan=2, sticky="w")
        tk.Checkbutton(highlight_frame, text="Highlight above upper", variable=self.highlight_above).grid(
            row=2, column=0, sticky="w"
        )
        tk.Entry(highlight_frame, textvariable=self.highlight_upper_limit, width=10).grid(row=2, column=1, sticky="w")
        tk.Checkbutton(highlight_frame, text="Highlight below lower", variable=self.highlight_below).grid(
            row=3, column=0, sticky="w"
        )
        tk.Entry(highlight_frame, textvariable=self.highlight_lower_limit, width=10).grid(row=3, column=1, sticky="w")
        row += 1

        tk.Label(frm, text="Max plot points:").grid(row=row, column=0, sticky="e", pady=(10, 0))
        tk.Entry(frm, textvariable=self.max_plot_points, width=10).grid(row=row, column=1, sticky="w", pady=(10, 0))
        row += 1

        tk.Label(frm, text="PDF plot DPI (raster):").grid(row=row, column=0, sticky="e")
        tk.Entry(frm, textvariable=self.pdf_plot_dpi, width=10).grid(row=row, column=1, sticky="w")
        row += 1
        tk.Checkbutton(frm, text="Vector PDF (plot + summary as vector)", variable=self.pdf_vector).grid(
            row=row, column=1, sticky="w"
        )
        row += 1

        btn_row = row

        self.plot_btn = tk.Button(frm, text="Plot", command=self.plot, state=tk.DISABLED)
        self.plot_btn.grid(row=btn_row, column=0, sticky="w", pady=(15, 0))

        self.save_trim_btn = tk.Button(frm, text="Save Trimmed CSV", command=self.save_trimmed_csv, state=tk.DISABLED)
        self.save_trim_btn.grid(row=btn_row, column=1, sticky="w", pady=(15, 0))

        self.pdf_btn = tk.Button(frm, text="Export PDF Report", command=self.export_pdf, state=tk.DISABLED)
        self.pdf_btn.grid(row=btn_row + 1, column=0, sticky="w", pady=(8, 0))

        self.note_label = tk.Label(
            frm,
            text="Notes:\n"
                 "• Trimming matches minute buckets present in the log.\n"
                 "• X-axis labels are formatted without seconds.\n"
                 "• Multi-day plots: select multiple daily CSV files, or select a folder.\n"
                 "• Rows without usable timestamps are dropped (unless record_id fallback is used).",
            justify="left",
        )
        self.note_label.grid(row=btn_row + 2, column=0, columnspan=2, sticky="w", pady=(10, 0))

    def _handle_scenario_change(self, _event: Optional[tk.Event] = None) -> None:
        if self.scenario_choice.get() == "Thermal cycling":
            self.stats_show_avg.set(False)
            self.stats_show_std.set(False)
            self.highlight_outside_std.set(False)

    def _parse_optional_float(self, value: str, label: str) -> Optional[float]:
        text = (value or "").strip()
        if not text:
            return None
        try:
            return float(text)
        except ValueError:
            raise ValueError(f"Invalid {label} limit: {text}")

    def _get_display_time_config(
        self,
        start_dt: Optional[datetime.datetime],
        end_dt: Optional[datetime.datetime],
    ) -> DisplayTimeConfig:
        tz_value = self.display_tz_choice.get()
        display_tz = _resolve_display_tz(tz_value)
        if display_tz is None:
            raise ValueError(f"Invalid time zone: {tz_value}")
        label = _format_display_tz_label(display_tz, start_dt, end_dt)
        return DisplayTimeConfig(display_tz=display_tz, display_tz_label=label)

    def _autofill_time_range(self) -> None:
        if not self.loaded:
            return
        if self.loaded.time_column != "__time":
            self.start_time_text.set("")
            self.end_time_text.set("")
            return
        display_tz = _resolve_display_tz(self.display_tz_choice.get())
        if display_tz is None:
            return
        display_series = _convert_time_series_to_display_tz(
            self.loaded.dataframe[self.loaded.time_column],
            display_tz=display_tz,
            time_source=self.loaded.time_source,
        )
        minutes = display_series.dt.floor("min")
        if minutes.isna().all():
            return
        start_min = minutes.min()
        end_min = minutes.max()
        self.start_time_text.set(start_min.strftime("%Y-%m-%d %H:%M"))
        self.end_time_text.set(end_min.strftime("%Y-%m-%d %H:%M"))

    def _detect_aggregated(self, time_series: pd.Series) -> bool:
        if time_series.empty:
            return False
        diffs = pd.to_datetime(time_series).sort_values().diff().dropna()
        if diffs.empty:
            return False
        median_seconds = diffs.dt.total_seconds().median()
        return bool(median_seconds >= 60)

    def _collect_plot_options(
        self,
        display_time_config: DisplayTimeConfig,
        smooth: bool,
        overlay_raw: bool,
    ) -> PlotOptions:
        stats = StatsOptions(
            show_min=self.stats_show_min.get(),
            show_max=self.stats_show_max.get(),
            show_avg=self.stats_show_avg.get(),
            show_std_band=self.stats_show_std.get(),
        )
        highlight_upper = self._parse_optional_float(self.highlight_upper_limit.get(), "upper")
        highlight_lower = self._parse_optional_float(self.highlight_lower_limit.get(), "lower")
        highlights = HighlightOptions(
            highlight_outside_std=self.highlight_outside_std.get() and stats.show_std_band,
            upper_limit=highlight_upper,
            lower_limit=highlight_lower,
            highlight_above=self.highlight_above.get(),
            highlight_below=self.highlight_below.get(),
            apply_to_rolling_mean=self.highlight_apply_to_rolling_mean.get(),
        )
        temp_unit = "F" if self.temp_f.get() else "C"
        return PlotOptions(
            overlay_raw_temp=overlay_raw,
            smooth=smooth,
            enable_downsample=self.enable_downsample.get(),
            max_plot_points=int(self.max_plot_points.get()),
            temp_unit=temp_unit,
            stats=stats,
            highlights=highlights,
            display_time_config=display_time_config,
            time_source=self.loaded.time_source if self.loaded else "unknown",
        )

    def _ensure_valid_y_choice(self) -> None:
        if not self.loaded:
            return
        available = [name for name in self.series_choices if name in self.loaded.dataframe.columns]
        if not available:
            return
        preferred = "cal_temp_c" if "cal_temp_c" in available else available[0]
        if self.y_choice.get() not in available:
            self.y_choice.set(preferred)

    def _load_paths(self, file_paths: List[str]) -> None:
        if not file_paths:
            return
        self.selected_files = list(file_paths)
        try:
            self.loaded = _load_csv_files(self.selected_files)
        except Exception as exc:
            self.loaded = None
            messagebox.showerror("Load Error", str(exc))
            return

        file_list = "\n".join([os.path.basename(p) for p in self.selected_files[:6]])
        if len(self.selected_files) > 6:
            file_list += f"\n… ({len(self.selected_files)} total)"
        self.file_label.config(text=file_list)

        self.plot_btn.config(state=tk.NORMAL)
        self.save_trim_btn.config(state=tk.NORMAL)
        self.pdf_btn.config(state=tk.NORMAL)
        self._warned_aggregated = False
        self._ensure_valid_y_choice()
        self._autofill_time_range()

        if self.loaded.dropped_no_time_rows:
            messagebox.showinfo(
                "Info",
                f"Dropped {self.loaded.dropped_no_time_rows} row(s) without usable timestamps.",
            )

    def select_files(self) -> None:
        file_paths = filedialog.askopenfilenames(
            title="Select PT100 CSV file(s)",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
        )
        self._load_paths(list(file_paths))

    def select_folder(self) -> None:
        folder = filedialog.askdirectory(title="Select folder containing PT100 CSV files")
        if not folder:
            return
        file_paths = sorted(glob.glob(os.path.join(folder, "*.csv")))
        if not file_paths:
            messagebox.showerror("Load Error", "No .csv files found in the selected folder.")
            return
        self._load_paths(file_paths)

    def open_range_selector(self) -> None:
        if not self.loaded:
            messagebox.showerror("Range Selector", "No data loaded.")
            return

        df = self.loaded.dataframe
        time_column = self.loaded.time_column
        y_name = self.y_choice.get()
        y_series = pd.to_numeric(df.get(y_name, pd.Series(dtype=float)), errors="coerce")

        display_tz = _resolve_display_tz(self.display_tz_choice.get())
        if time_column == "__time":
            if display_tz is None:
                messagebox.showerror("Range Selector", f"Invalid time zone: {self.display_tz_choice.get()}")
                return
            display_series = _convert_time_series_to_display_tz(
                df[time_column],
                display_tz=display_tz,
                time_source=self.loaded.time_source,
            )
            x_plot = display_series.to_numpy()
            x_slider = mdates.date2num(display_series)
            present_minutes = display_series.dt.floor("min")
            start_dt = pd.to_datetime(display_series).min()
            end_dt = pd.to_datetime(display_series).max()
            display_config = self._get_display_time_config(start_dt=start_dt, end_dt=end_dt)
            x_label = f"{_human_time_label(time_column)} ({display_config.display_tz_label})"
        else:
            display_series = None
            x_slider = pd.to_numeric(df[time_column], errors="coerce").to_numpy(dtype=float, copy=False)
            x_plot = x_slider
            present_minutes = None
            display_config = DisplayTimeConfig(display_tz=None, display_tz_label="n/a")
            x_label = _human_time_label(time_column)

        if len(x_slider) == 0 or np.all(np.isnan(x_slider)):
            messagebox.showerror("Range Selector", "No usable X values for range selection.")
            return

        valid_mask = np.isfinite(x_slider)
        x_slider = x_slider[valid_mask]
        if time_column == "__time":
            x_plot = np.asarray(x_plot)[valid_mask]
        else:
            x_plot = x_slider
        y_series = y_series.to_numpy(dtype=float, copy=False)[valid_mask]

        fig = plt.Figure(figsize=(7.4, 3.4))
        ax = fig.add_subplot(111)
        ax.plot(x_plot, y_series, linewidth=0.8, alpha=0.8, label="preview")
        ax.set_xlabel(x_label)
        ax.set_ylabel(_human_series_label(y_name, temp_unit="F" if self.temp_f.get() else "C"))
        ax.grid(True)
        if time_column == "__time":
            locator = mdates.AutoDateLocator(minticks=4, maxticks=10)
            ax.xaxis.set_major_locator(locator)
            ax.xaxis.set_major_formatter(mdates.ConciseDateFormatter(locator, tz=display_tz))
        fig.tight_layout()

        selector_window = tk.Toplevel(self.root)
        selector_window.title("Select range")
        canvas = FigureCanvasTkAgg(fig, master=selector_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)

        slider_frame = tk.Frame(selector_window, padx=8, pady=0)
        slider_frame.pack(fill="x")
        slider_fig = plt.Figure(figsize=(7.4, 0.65))
        slider_ax = slider_fig.add_axes([0.08, 0.35, 0.84, 0.45])
        slider_canvas = FigureCanvasTkAgg(slider_fig, master=slider_frame)
        slider_canvas.draw()
        slider_canvas.get_tk_widget().pack(fill="x")

        slider = RangeSlider(slider_ax, "Range", float(np.min(x_slider)), float(np.max(x_slider)))
        slider.valtext.set_visible(False)
        slider.valtext.set_text("")
        if time_column == "__time" and display_tz is not None:
            try:
                start_candidate = _parse_user_time(self.start_time_text.get())
                end_candidate = _parse_user_time(self.end_time_text.get())
                if start_candidate and end_candidate:
                    start_ts = pd.Timestamp(start_candidate)
                    end_ts = pd.Timestamp(end_candidate)
                    if start_ts.tzinfo is None:
                        start_ts = start_ts.tz_localize(display_tz)
                    if end_ts.tzinfo is None:
                        end_ts = end_ts.tz_localize(display_tz)
                    slider.set_val((mdates.date2num(start_ts), mdates.date2num(end_ts)))
            except Exception:
                pass

        def _format_dt_label(value: float) -> str:
            if time_column != "__time":
                return f"{int(round(value))}"
            dt_val = mdates.num2date(value, tz=display_tz)
            return dt_val.strftime("%Y-%m-%d %H:%M")

        start_label_var = tk.StringVar(value=_format_dt_label(slider.val[0]))
        end_label_var = tk.StringVar(value=_format_dt_label(slider.val[1]))

        def _update_labels(val: Tuple[float, float]) -> None:
            start_label_var.set(_format_dt_label(val[0]))
            end_label_var.set(_format_dt_label(val[1]))

        def _on_slider_change(val: Tuple[float, float]) -> None:
            if time_column == "__time":
                x0_dt = mdates.num2date(val[0], tz=display_tz)
                x1_dt = mdates.num2date(val[1], tz=display_tz)
                ax.set_xlim(x0_dt, x1_dt)
            else:
                ax.set_xlim(val)
            _update_labels(val)
            slider.valtext.set_text("")
            canvas.draw_idle()

        slider.on_changed(_on_slider_change)

        label_frame = tk.Frame(selector_window, padx=8, pady=6)
        label_frame.pack(fill="x")
        tk.Label(label_frame, text="Start:").grid(row=0, column=0, sticky="w")
        tk.Label(label_frame, textvariable=start_label_var).grid(row=0, column=1, sticky="w")
        tk.Label(label_frame, text="End:").grid(row=1, column=0, sticky="w")
        tk.Label(label_frame, textvariable=end_label_var).grid(row=1, column=1, sticky="w")

        def _apply_range() -> None:
            if time_column != "__time" or present_minutes is None:
                self.start_time_text.set("")
                self.end_time_text.set("")
                messagebox.showinfo(
                    "Range Selection",
                    "This log uses record_id; time trimming is unavailable. The slider is for visual preview only.",
                )
                selector_window.destroy()
                return
            start_dt = mdates.num2date(slider.val[0], tz=display_tz)
            end_dt = mdates.num2date(slider.val[1], tz=display_tz)
            start_text = _nearest_minute_string(pd.DatetimeIndex(present_minutes.unique()).sort_values(), start_dt)
            end_text = _nearest_minute_string(pd.DatetimeIndex(present_minutes.unique()).sort_values(), end_dt)
            self.start_time_text.set(start_text)
            self.end_time_text.set(end_text)
            selector_window.destroy()

        action_frame = tk.Frame(selector_window, padx=8, pady=6)
        action_frame.pack(fill="x")
        tk.Button(action_frame, text="Apply range", command=_apply_range).pack(side="left")
        tk.Button(action_frame, text="Close", command=selector_window.destroy).pack(side="left", padx=6)

    def _get_trimmed_df(self) -> Tuple[pd.DataFrame, str, str, str, DisplayTimeConfig, Optional[pd.Series]]:
        if not self.loaded:
            raise ValueError("No data loaded.")
        display_series = None
        display_tz = None
        if self.loaded.time_column == "__time":
            display_tz = _resolve_display_tz(self.display_tz_choice.get())
            if display_tz is None:
                raise ValueError(f"Invalid time zone: {self.display_tz_choice.get()}")
            display_series = _convert_time_series_to_display_tz(
                self.loaded.dataframe[self.loaded.time_column],
                display_tz=display_tz,
                time_source=self.loaded.time_source,
            )
        trimmed, start_label, end_label, summary = _validate_and_trim_by_minute(
            df=self.loaded.dataframe,
            time_column=self.loaded.time_column,
            start_text=self.start_time_text.get(),
            end_text=self.end_time_text.get(),
            time_series=display_series,
        )
        if display_series is not None:
            display_series = display_series.loc[trimmed.index]
            display_config = self._get_display_time_config(
                start_dt=pd.to_datetime(display_series).min(),
                end_dt=pd.to_datetime(display_series).max(),
            )
        else:
            display_config = DisplayTimeConfig(display_tz=None, display_tz_label="n/a")
        return trimmed, start_label, end_label, summary, display_config, display_series

    def save_trimmed_csv(self) -> None:
        if not self.loaded:
            messagebox.showerror("Save Error", "No data loaded.")
            return
        try:
            trimmed, start_label, end_label, summary, _display_config, _display_series = self._get_trimmed_df()
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        default_name = f"pt100_trim_{start_label.replace(':','')}_{end_label.replace(':','')}.csv"
        save_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV Files", "*.csv")],
        )
        if not save_path:
            return

        try:
            trimmed.to_csv(save_path, index=False)
        except Exception as exc:
            messagebox.showerror("Save Error", str(exc))
            return

        messagebox.showinfo("Saved", f"Saved trimmed CSV:\n{save_path}\n\n{summary}")

    def plot(self) -> None:
        if not self.loaded:
            messagebox.showerror("Plot Error", "No data loaded.")
            return
        try:
            df, start_label, end_label, _summary, display_config, display_series = self._get_trimmed_df()
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        y_name = self.y_choice.get()
        if y_name not in df.columns:
            messagebox.showerror("Plot Error", f"Column not found: {y_name}")
            return

        overlay_raw = self.overlay_raw.get()
        if overlay_raw and "raw_temp_c" not in df.columns:
            self.overlay_raw.set(False)
            overlay_raw = False
            messagebox.showinfo("Overlay Disabled", "raw_temp_c is not available in this dataset.")

        smooth = self.smooth.get()
        if display_series is not None and self._detect_aggregated(display_series):
            if smooth and not self._warned_aggregated:
                messagebox.showinfo("Smoothing Disabled", "Input appears aggregated; smoothing disabled.")
                self._warned_aggregated = True
            smooth = False

        nodes = _node_ids_from_df(df)
        range_label = f"{start_label} → {end_label}"

        plot_title = _human_series_label(y_name, temp_unit="F" if self.temp_f.get() else "C")
        suptitle = f"Nodes: {nodes} — {range_label}" if nodes != "n/a" else range_label

        try:
            options = self._collect_plot_options(display_config, smooth=smooth, overlay_raw=overlay_raw)
        except Exception as exc:
            messagebox.showerror("Plot Error", str(exc))
            return

        fig, _plot_points, _total_points = _build_figure(
            df=df,
            time_column=self.loaded.time_column,
            y_name=y_name,
            plot_title=plot_title,
            suptitle=suptitle,
            options=options,
        )
        plt.show()

    def export_pdf(self) -> None:
        if not self.loaded:
            messagebox.showerror("PDF Error", "No data loaded.")
            return
        try:
            df, start_label, end_label, summary, display_config, display_series = self._get_trimmed_df()
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        y_name = self.y_choice.get()
        if y_name not in df.columns:
            messagebox.showerror("PDF Error", f"Column not found: {y_name}")
            return

        overlay_raw = self.overlay_raw.get()
        if overlay_raw and "raw_temp_c" not in df.columns:
            self.overlay_raw.set(False)
            overlay_raw = False
            messagebox.showinfo("Overlay Disabled", "raw_temp_c is not available in this dataset.")

        smooth = self.smooth.get()
        if display_series is not None and self._detect_aggregated(display_series):
            if smooth and not self._warned_aggregated:
                messagebox.showinfo("Smoothing Disabled", "Input appears aggregated; smoothing disabled.")
                self._warned_aggregated = True
            smooth = False

        today = datetime.date.today().isoformat()
        default_name = f"pt100_report_{today}.pdf"
        save_path = filedialog.asksaveasfilename(
            defaultextension=".pdf",
            initialfile=default_name,
            filetypes=[("PDF Files", "*.pdf")],
        )
        if not save_path:
            return

        use_vector_pdf = bool(self.pdf_vector.get())

        tmp_dir: Optional[str] = None
        fig_png_path: Optional[str] = None
        if not use_vector_pdf:
            tmp_dir = tempfile.mkdtemp(prefix="pt100_report_")
            fig_png_path = os.path.join(tmp_dir, "plot.png")

        nodes = _node_ids_from_df(df)
        title = "PT100 Temperature Log Report"
        range_label = f"{start_label} → {end_label}"
        subtitle = f"Nodes: {nodes} — {range_label}"

        temp_unit = "F" if self.temp_f.get() else "C"
        plot_title = _human_series_label(y_name, temp_unit=temp_unit)

        try:
            options = self._collect_plot_options(display_config, smooth=smooth, overlay_raw=overlay_raw)
        except Exception as exc:
            messagebox.showerror("PDF Error", str(exc))
            return

        fig, plot_points, total_points = _build_figure(
            df=df,
            time_column=self.loaded.time_column,
            y_name=y_name,
            plot_title=plot_title,
            suptitle=None,
            options=options,
        )

        y_series_c = pd.to_numeric(df.get(y_name, pd.Series(dtype=float)), errors="coerce")
        y_series_disp = (
            _convert_temperature_series(y_series_c, temp_unit)
            if y_name in ("cal_temp_c", "raw_temp_c")
            else y_series_c
        )
        stats_series, stats_notes = _apply_flag_filters_for_stats(df, y_series_disp, y_name)
        stats = _compute_basic_stats(stats_series)
        flags_summary = _format_flags_summary(df)

        series_label = _human_series_label(y_name, temp_unit=temp_unit)

        overlays = []
        if y_name in ("cal_temp_c", "raw_temp_c"):
            if options.stats.show_min:
                overlays.append("min")
            if options.stats.show_max:
                overlays.append("max")
            if options.stats.show_avg:
                overlays.append("avg")
            if options.stats.show_std_band:
                overlays.append("±1σ")
            highlights = []
            if options.highlights.highlight_outside_std:
                basis_label = "rolling mean" if options.highlights.apply_to_rolling_mean and options.smooth else "data"
                highlights.append(f"outside ±1σ ({basis_label})")
            if options.highlights.highlight_above and options.highlights.upper_limit is not None:
                highlights.append(f"> {options.highlights.upper_limit:g}")
            if options.highlights.highlight_below and options.highlights.lower_limit is not None:
                highlights.append(f"< {options.highlights.lower_limit:g}")
            if highlights:
                overlays.append(f"highlight: {', '.join(highlights)}")

        summary_rows = [
            ["Field", "Value"],
            ["Time source", self.loaded.time_source],
            ["Time zone", display_config.display_tz_label],
            ["Start", start_label],
            ["End", end_label],
            ["Series", series_label],
            ["min / avg / max / std", f"{stats['min']} / {stats['avg']} / {stats['max']} / {stats['std']}"],
        ]
        if flags_summary != "n/a":
            summary_rows.append(["Flags", flags_summary])
        if stats_notes:
            summary_rows.append(["Stats filtering", "; ".join(stats_notes)])

        if overlays:
            summary_rows.append(["Overlays", ", ".join(overlays)])

        try:
            if use_vector_pdf:
                _export_pdf_report_vector(
                    save_path=save_path,
                    fig=fig,
                    source_files=self.loaded.source_files,
                    summary_rows=summary_rows,
                    title=title,
                    subtitle=subtitle,
                )
            else:
                assert fig_png_path is not None
                fig.savefig(fig_png_path, dpi=int(self.pdf_plot_dpi.get()))
                _export_pdf_report(
                    save_path=save_path,
                    fig_png_path=fig_png_path,
                    source_files=self.loaded.source_files,
                    summary_rows=summary_rows,
                    title=title,
                    subtitle=subtitle,
                )
        except Exception as exc:
            messagebox.showerror("PDF Error", str(exc))
            return
        finally:
            plt.close(fig)
            if fig_png_path and tmp_dir:
                try:
                    if os.path.exists(fig_png_path):
                        os.remove(fig_png_path)
                    os.rmdir(tmp_dir)
                except Exception:
                    pass

        messagebox.showinfo("Success", f"PDF report saved:\n{save_path}")


def main() -> None:
    root = tk.Tk()
    app = PlotterApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()