#!/usr/bin/env python3
"""
PT100 Mesh Logger Log Plotter + PDF Report

- Loads 1+ CSV/PTLOG exports from PT100 nodes.
- Supports schema_ver 1, 2, and 3.
  - schema_ver 2 adds: record_id (uint64-ish monotonic identifier).
  - schema_ver 3 adds: fault_status (MAX31865 fault byte).
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
import json
import datetime
import glob
import math
import os
import tempfile
import zlib
import html
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple, Sequence

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

# MAX31865 fault-status bit labels (matches firmware Max31865FormatFault()).
_MAX31865_FAULT_BITS: List[Tuple[int, str]] = [
    (0x80, "RTD high threshold"),
    (0x40, "RTD low threshold"),
    (0x20, "REFIN- > 0.85*VBIAS"),
    (0x10, "REFIN- < 0.85*VBIAS (FORCE- open)"),
    (0x08, "RTDIN- < 0.85*VBIAS (FORCE- open)"),
    (0x04, "Over/undervoltage"),
    (0x01, "Fault summary bit (RTD LSB)"),
]

# Known flag labels (short name -> human label).
_FLAG_LABELS: Dict[str, str] = {
    "TIME_VALID": "Time valid",
    "CAL_VALID": "Calibration valid",
    "SD_ERROR": "SD error flagged",
    "MESH_CONNECTED": "Mesh connected",
    "SENSOR_FAULT": "Sensor fault",
    "FRAM_FULL": "FRAM full",
    "RTD_EMA": "RTD EMA used",
    "TIME_JUMP_BACK": "Time jump back (RTC sync)",
}

# Flags that represent an error condition when set.
_ERROR_FLAG_SHORT_NAMES = {"SD_ERROR", "SENSOR_FAULT", "FRAM_FULL"}

_FLAG_MEANINGS: Dict[str, str] = {
    "TIME_VALID": "Timestamp was considered valid.",
    "CAL_VALID": "Calibrated value was considered valid.",
    "SD_ERROR": "SD write or flush error was flagged.",
    "MESH_CONNECTED": "Node reported mesh connection status. Informational.",
    "SENSOR_FAULT": "Sensor read failed or sensor fault was detected.",
    "FRAM_FULL": "FRAM storage reached capacity.",
    "RTD_EMA": "RTD exponential moving average was used. Informational.",
    "TIME_JUMP_BACK": "RTC or timestamp moved backward. Review time continuity.",
}

_CONFIG_LABELS: Dict[str, str] = {
    "input_timezone_mode": "Input time zone",
    "display_timezone": "Display time zone",
    "y_axis_series": "Y-axis series",
    "display_temperature_f": "Display temperature in degrees F",
    "overlay_raw_temp_c": "Overlay raw temperature",
    "smooth_enabled": "Smooth data with rolling mean",
    "rolling_mean_divisor": "Rolling mean divisor",
    "downsample_enabled": "Downsample large datasets",
    "max_plot_points": "Maximum plot points",
    "pdf_plot_dpi": "PDF plot DPI",
    "vector_pdf": "Vector PDF output",
    "show_minimum": "Show minimum line",
    "show_maximum": "Show maximum line",
    "show_average": "Show average line",
    "show_std_band": "Show standard deviation band",
    "highlight_outside_std": "Highlight outside mean plus or minus 1 sigma",
    "highlight_mask_from_rolling_mean": "Apply highlight mask to rolling mean",
    "highlight_above_enabled": "Highlight above limit",
    "highlight_above_value": "Upper limit",
    "highlight_below_enabled": "Highlight below limit",
    "highlight_below_value": "Lower limit",
    "pdf_sensor_fault_threshold_percent": "PDF sensor fault threshold percent",
}


@dataclass
class FlagSummaryRow:
    short_name: str
    label: str
    count: int
    percent: float
    meaning: str
    is_problem: bool
    include_in_pdf_by_default: bool


@dataclass
class SensorFaultRowClassification:
    zero_fault_status_count: int
    nonzero_fault_status_counts_by_code: Dict[int, int]
    unparseable_fault_status_count: int
    total_sensor_fault_count: int
    zero_fault_status_percent: float


# Built-in fallback definitions (mask -> short name).
_FALLBACK_FLAG_DEFS: List[Tuple[int, str]] = [
    (1 << 0, "TIME_VALID"),
    (1 << 1, "CAL_VALID"),
    (1 << 2, "SD_ERROR"),
    (1 << 3, "MESH_CONNECTED"),
    (1 << 4, "SENSOR_FAULT"),
    (1 << 5, "FRAM_FULL"),
    (1 << 6, "RTD_EMA"),
    (1 << 7, "TIME_JUMP_BACK"),
]

_DEFAULT_ROLLING_MEAN_DIVISOR = 40
REPORT_CONFIG_VERSION = 1


@dataclass
class ReportConfig:
    input_timezone_mode: str
    display_timezone: str
    y_axis_series: str
    display_temperature_f: bool
    overlay_raw_temp_c: bool
    smooth_enabled: bool
    rolling_mean_divisor: int
    downsample_enabled: bool
    max_plot_points: int
    pdf_plot_dpi: int
    vector_pdf: bool
    show_minimum: bool
    show_maximum: bool
    show_average: bool
    show_std_band: bool
    highlight_outside_std: bool
    highlight_mask_from_rolling_mean: bool
    highlight_above_enabled: bool
    highlight_above_value: Optional[float]
    highlight_below_enabled: bool
    highlight_below_value: Optional[float]
    pdf_sensor_fault_threshold_percent: float


def create_default_report_config() -> ReportConfig:
    return ReportConfig(
        input_timezone_mode="Log/source time",
        display_timezone="Local",
        y_axis_series="cal_temp_c",
        display_temperature_f=False,
        overlay_raw_temp_c=False,
        smooth_enabled=True,
        rolling_mean_divisor=40,
        downsample_enabled=True,
        max_plot_points=20000,
        pdf_plot_dpi=600,
        vector_pdf=False,
        show_minimum=False,
        show_maximum=False,
        show_average=False,
        show_std_band=False,
        highlight_outside_std=False,
        highlight_mask_from_rolling_mean=False,
        highlight_above_enabled=False,
        highlight_above_value=None,
        highlight_below_enabled=False,
        highlight_below_value=None,
        pdf_sensor_fault_threshold_percent=0.10,
    )


def validate_report_config(config: ReportConfig) -> None:
    def _require_str(name: str, value: object) -> str:
        if not isinstance(value, str):
            raise ValueError(f"{name} must be a string")
        return value

    def _require_bool(name: str, value: object) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str) and value.strip().lower() in {"true", "false", "yes", "no", "0", "1"}:
            raise ValueError(f"{name} must be a boolean (true/false), not a string value")
        raise ValueError(f"{name} must be a boolean")

    def _require_int(name: str, value: object, minimum: Optional[int] = None) -> int:
        if not isinstance(value, int) or isinstance(value, bool):
            raise ValueError(f"{name} must be an integer")
        if minimum is not None and value < minimum:
            raise ValueError(f"{name} must be >= {minimum}")
        return value

    def _require_finite_number(name: str, value: object) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be numeric")
        value_f = float(value)
        if not math.isfinite(value_f):
            raise ValueError(f"{name} must be a finite number")
        return value_f

    allowed_input_modes = {"Log/source time", "UTC", "Local", "Same as display"}
    input_timezone_mode = _require_str("input_timezone_mode", config.input_timezone_mode)
    if input_timezone_mode not in allowed_input_modes:
        raise ValueError("input_timezone_mode must be one of: Log/source time, UTC, Local, Same as display")
    display_timezone = _require_str("display_timezone", config.display_timezone)
    if display_timezone not in {"Local", "UTC"} and _resolve_display_tz(display_timezone) is None:
        raise ValueError(
            f"Invalid timezone '{display_timezone}' for display_timezone. "
            "Check Edit Options and enter a valid IANA timezone (for example: America/Chicago) or use UTC/Local."
        )
    y_axis_series = _require_str("y_axis_series", config.y_axis_series)
    if not y_axis_series.strip():
        raise ValueError("y_axis_series must be a non-empty string")

    for key in (
        "display_temperature_f", "overlay_raw_temp_c", "smooth_enabled", "downsample_enabled", "vector_pdf",
        "show_minimum", "show_maximum", "show_average", "show_std_band", "highlight_outside_std",
        "highlight_mask_from_rolling_mean", "highlight_above_enabled", "highlight_below_enabled",
    ):
        _require_bool(key, getattr(config, key))

    _require_int("rolling_mean_divisor", config.rolling_mean_divisor, minimum=1)
    _require_int("max_plot_points", config.max_plot_points, minimum=1)
    _require_int("pdf_plot_dpi", config.pdf_plot_dpi, minimum=72)

    threshold = _require_finite_number("pdf_sensor_fault_threshold_percent", config.pdf_sensor_fault_threshold_percent)
    if threshold < 0:
        raise ValueError("pdf_sensor_fault_threshold_percent must be >= 0")

    if config.highlight_above_enabled:
        if config.highlight_above_value is None:
            raise ValueError("highlight_above_value must be set when highlight_above_enabled is true")
        _require_finite_number("highlight_above_value", config.highlight_above_value)
    elif config.highlight_above_value is not None:
        raise ValueError("Invalid highlight limit: highlight_above_value must be blank when highlight_above_enabled is false.")

    if config.highlight_below_enabled:
        if config.highlight_below_value is None:
            raise ValueError("highlight_below_value must be set when highlight_below_enabled is true")
        _require_finite_number("highlight_below_value", config.highlight_below_value)
    elif config.highlight_below_value is not None:
        raise ValueError("Invalid highlight limit: highlight_below_value must be blank when highlight_below_enabled is false.")


def save_report_config(config: ReportConfig, path: str) -> None:
    validate_report_config(config)
    payload = asdict(config)
    payload["config_version"] = REPORT_CONFIG_VERSION
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def load_report_config(path: str) -> ReportConfig:
    if not os.path.exists(path):
        raise ValueError(
            f"Missing config file: {path}. Check that the file exists and that you selected the correct configuration."
        )
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"Invalid JSON in config file '{path}'. Check for missing commas, quotes, or braces and try again."
        ) from exc
    if not isinstance(raw, dict):
        raise ValueError("Invalid report config format. Check that the config file contains a JSON object.")
    if "config_version" not in raw:
        raise ValueError(
            "Missing required config field 'config_version'. "
            "Check whether this config is from an older release and migrate it before loading."
        )
    config_version = raw.get("config_version")
    if config_version != REPORT_CONFIG_VERSION:
        raise ValueError(
            f"Unsupported config_version '{config_version}'. "
            f"This tool supports config_version {REPORT_CONFIG_VERSION}. Check that you selected a compatible config file."
        )
    raw = {k: v for k, v in raw.items() if k != "config_version"}
    try:
        config = ReportConfig(**raw)
    except TypeError as exc:
        raise ValueError(
            f"Missing required config field(s): {exc}. Check that all required settings are present in the config."
        ) from exc
    validate_report_config(config)
    return config


def build_plot_options_from_config(
    config: ReportConfig,
    loaded_log: LoadedLog,
    trimmed_dataframe: pd.DataFrame,
    display_time_config: DisplayTimeConfig,
) -> PlotOptions:
    validate_report_config(config)
    _validate_series_for_configured_output(trimmed_dataframe, config, require_overlay_check=False)

    stats = StatsOptions(
        show_min=config.show_minimum,
        show_max=config.show_maximum,
        show_avg=config.show_average,
        show_std_band=config.show_std_band,
    )
    highlights = HighlightOptions(
        highlight_outside_std=config.highlight_outside_std,
        upper_limit=config.highlight_above_value,
        lower_limit=config.highlight_below_value,
        highlight_above=config.highlight_above_enabled,
        highlight_below=config.highlight_below_enabled,
        apply_to_rolling_mean=config.highlight_mask_from_rolling_mean,
    )
    temp_unit = "F" if config.display_temperature_f else "C"
    return PlotOptions(
        overlay_raw_temp=config.overlay_raw_temp_c,
        smooth=config.smooth_enabled,
        rolling_mean_divisor=int(config.rolling_mean_divisor),
        enable_downsample=config.downsample_enabled,
        max_plot_points=int(config.max_plot_points),
        temp_unit=temp_unit,
        stats=stats,
        highlights=highlights,
        display_time_config=display_time_config,
        time_source=loaded_log.time_source,
    )


def _validate_series_for_configured_output(
    trimmed_df: pd.DataFrame,
    config: ReportConfig,
    *,
    require_overlay_check: bool,
) -> None:
    """Strict, audit-facing validation for configured y-axis and optional overlay."""
    y_name = config.y_axis_series
    if y_name not in trimmed_df.columns:
        raise ValueError(
            f"Configured y-axis series {y_name} is not present in the selected logs. "
            "Open Edit Options and choose a valid series or investigate the log data."
        )

    y_numeric = pd.to_numeric(trimmed_df[y_name], errors="coerce")
    if y_numeric.notna().sum() == 0:
        raise ValueError(
            f"Configured y-axis series {y_name} has no usable numeric values in the selected range. "
            "Open Edit Options and choose a valid series or investigate the log data."
        )

    if config.display_temperature_f and y_name not in ("cal_temp_c", "raw_temp_c"):
        raise ValueError(
            f"Configured y-axis series {y_name} is not a temperature series and cannot be displayed in degrees F. "
            "Disable Fahrenheit display or choose a temperature series."
        )

    if y_name == "cal_temp_c":
        if "flags" not in trimmed_df.columns:
            raise ValueError(
                "Configured y-axis series cal_temp_c cannot be used because calibration evidence is missing (flags column is unavailable). "
                "Open Edit Options and choose a valid series or investigate the calibration/log data."
            )
        flags_int = _parse_flags_series_to_int64(trimmed_df["flags"])
        if flags_int.isna().any():
            raise ValueError(
                "Configured y-axis series cal_temp_c cannot be used because calibration evidence is unparseable in the selected range. "
                "Open Edit Options and choose a valid series or investigate the calibration/log data."
            )
        cal_mask = _get_flag_mask("CAL_VALID", 1 << 1)
        if cal_mask <= 0:
            raise ValueError(
                "Configured y-axis series cal_temp_c cannot be used because calibration evidence is unavailable (CAL_VALID mask is undefined). "
                "Open Edit Options and choose a valid series or investigate the calibration/log data."
            )
        sensor_fault_mask = _get_flag_mask("SENSOR_FAULT", 1 << 4)
        eligible = pd.Series(True, index=trimmed_df.index)
        if sensor_fault_mask:
            eligible &= ((flags_int.astype("int64") & sensor_fault_mask) == 0)
        if not bool(((flags_int.astype("int64") & cal_mask) != 0)[eligible].all()):
            raise ValueError(
                "Configured y-axis series cal_temp_c cannot be used because the selected range is not fully calibration-valid. "
                "Open Edit Options and choose a valid series or investigate the calibration/log data."
            )

    if require_overlay_check and config.overlay_raw_temp_c:
        if "raw_temp_c" not in trimmed_df.columns:
            raise ValueError(
                "Overlay raw temperature is enabled, but raw_temp_c is not present in the selected logs. "
                "Disable overlay or investigate the log data."
            )
        raw_numeric = pd.to_numeric(trimmed_df["raw_temp_c"], errors="coerce")
        if raw_numeric.notna().sum() == 0:
            raise ValueError(
                "Overlay raw temperature is enabled, but raw_temp_c has no usable numeric values in the selected range. "
                "Disable overlay or investigate the log data."
            )


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


def _get_flag_mask(short_name: str, default_mask: int = 0) -> int:
    """Return the bitmask for a given LOG_RECORD_FLAG short name.

    Args:
        short_name: The enum suffix from log_record.h (e.g. 'CAL_VALID').
        default_mask: Fallback mask to return if the flag is not found.

    Returns:
        The bitmask value, or default_mask if unknown.
    """
    for mask, name in _LOG_RECORD_FLAG_DEFS:
        if name == short_name:
            return int(mask)
    return int(default_mask)

def _get_time_jump_back_mask(df: pd.DataFrame) -> np.ndarray:
    """Return a boolean mask for TIME_JUMP_BACK records."""
    if "flags" not in df.columns:
        return np.zeros(len(df), dtype=bool)
    mask = _get_flag_mask("TIME_JUMP_BACK", 1 << 7)
    flags_numeric = pd.to_numeric(df["flags"], errors="coerce").fillna(0).astype("int64")
    return ((flags_numeric & mask) != 0).to_numpy(dtype=bool, copy=False)


def _ensure_sequence_column(df: pd.DataFrame) -> pd.DataFrame:
    """Ensure a usable sequence column exists for acquisition ordering."""
    df = df.copy()
    if "seq" in df.columns:
        seq_series = pd.to_numeric(df["seq"], errors="coerce")
    elif "record_id" in df.columns:
        seq_series = pd.to_numeric(df["record_id"], errors="coerce")
    else:
        seq_series = pd.Series([pd.NA] * len(df), dtype="float64")

    df["__seq"] = pd.array(seq_series, dtype="Int64")
    if df["__seq"].isna().all():
        raise ValueError("Sequence ordering requires a valid 'seq' or 'record_id' column.")
    return df


def _compute_unwrapped_sequence(seq_series: pd.Series) -> pd.Series:
    """Return a monotonic sequence by unwrapping 32-bit wraps."""
    offset = 0
    previous_value: Optional[int] = None
    unwrapped: List[Optional[int]] = []
    for value in seq_series:
        if pd.isna(value):
            unwrapped.append(pd.NA)
            continue
        current = int(value)
        if previous_value is not None and current < previous_value:
            offset += 2 ** 32
        unwrapped.append(current + offset)
        previous_value = current
    return pd.Series(unwrapped, index=seq_series.index, dtype="Int64")


@dataclass
class LoadedLog:
    dataframe: pd.DataFrame
    time_column: str
    time_source: str  # "iso8601_local", "epoch_utc", or "mixed"
    tzinfo: Optional[datetime.tzinfo]
    source_files: List[str]
    dropped_no_time_rows: int
    file_headers: Dict[str, Dict[str, str]]
    audit_summary: "AuditSummary"
    source_timezone_label: Optional[str] = None
    source_timezone_warning: Optional[str] = None


@dataclass
class MetadataSegment:
    signature: int
    header_dict: Dict[str, str]
    file_names: List[str]
    time_range_utc: Optional[Tuple[int, int]] = None


@dataclass
class AuditSummary:
    device_serials: List[str]
    segments: List[MetadataSegment]
    calibration_summary: str
    calibration_status: str
    calibration_points: str
    calibration_warning: Optional[str]
    timezone_summary: str
    firmware_summary: str
    serial_source_note: Optional[str]


@dataclass
class DisplayTimeConfig:
    display_tz: Optional[datetime.tzinfo]
    display_tz_label: str


@dataclass
class TrimTimeConfig:
    input_timezone_mode: str
    input_timezone_label: str
    source_tz: Optional[datetime.tzinfo]


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
    rolling_mean_divisor: int
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


def _canonicalize_time_to_utc(
    time_series: pd.Series,
    time_source: str,
    source_tz: Optional[datetime.tzinfo],
) -> pd.Series:
    """Convert a source timestamp series into an aware UTC series.

    Args:
        time_series: Source timestamp series (aware or naive).
        time_source: Source mode label from _pick_time_source().
        source_tz: Preferred source timezone for naive timestamps.

    Returns:
        pandas Series of timezone-aware UTC timestamps.
    """
    parsed = pd.to_datetime(time_series, errors="coerce")
    if parsed.isna().all():
        return parsed
    if getattr(parsed.dt, "tz", None) is not None:
        return parsed.dt.tz_convert(datetime.timezone.utc)

    tz_for_localize = source_tz
    if tz_for_localize is None:
        if time_source == "epoch_utc":
            tz_for_localize = datetime.timezone.utc
        else:
            tz_for_localize = _get_local_tz()
    if tz_for_localize is None:
        raise ValueError("Unable to resolve timezone for naive timestamps.")
    localized = parsed.dt.tz_localize(tz_for_localize)
    return localized.dt.tz_convert(datetime.timezone.utc)

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


def _parse_int_cell(value: object) -> Optional[int]:
    """Parse decimal/hex-like values into int with tolerant input handling."""
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


def _normalize_schema(df: pd.DataFrame) -> pd.DataFrame:
    """Normalize schema differences so plotting works across schema versions.

    This tool needs a consistent set of column names and data types across CSV
    schema versions. In particular, integer-like text columns (for example
    'flags' and schema 3 'fault_status') are parsed as integers with base-auto
    behavior, so both decimal and hex strings such as '0x13' are accepted.
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
        parsed_flags = df["flags"].apply(_parse_int_cell)
        df["flags"] = pd.array(parsed_flags, dtype="Int64")
    else:
        df["flags"] = pd.array([pd.NA] * len(df), dtype="Int64")

    if "fault_status" in df.columns:
        parsed_fault_status = df["fault_status"].apply(_parse_int_cell)
        df["fault_status"] = pd.array(parsed_fault_status, dtype="Int64")
    else:
        df["fault_status"] = pd.array([0] * len(df), dtype="Int64")

    if "node_id" not in df.columns:
        df["node_id"] = ""

    return df




def _resolve_source_timezone_from_headers(headers_by_file: Dict[str, Dict[str, str]]) -> Tuple[Optional[datetime.tzinfo], Optional[str], Optional[str]]:
    """Resolve source timezone from PTLOG headers.

    Returns (tzinfo, label, warning).
    """
    if not headers_by_file:
        return None, None, None

    tz_values = {(h.get("timezone_posix") or "").strip() for h in headers_by_file.values()}
    tz_values.discard("")
    if not tz_values:
        return None, None, "Missing PTLOG timezone metadata; falling back to host local timezone for naive timestamps."
    if len(tz_values) > 1:
        return None, None, f"Inconsistent PTLOG timezone metadata across files: {sorted(tz_values)}"

    tz_text = next(iter(tz_values))
    dst_values = {(h.get("dst_enabled") or "").strip().lower() for h in headers_by_file.values()}
    dst_values.discard("")
    if len(dst_values) > 1:
        return None, None, f"Inconsistent PTLOG DST metadata across files: {sorted(dst_values)}"

    # Conservative parser for common forms first.
    normalized = tz_text.upper().replace("UTC", "GMT")
    m = re.fullmatch(r"GMT([+-])(\d{1,2})(?::?(\d{2}))?", normalized)
    if m:
        sign = -1 if m.group(1) == "-" else 1
        hours = int(m.group(2))
        minutes = int(m.group(3) or "0")
        offset = datetime.timedelta(hours=hours, minutes=minutes) * sign
        tz = datetime.timezone(offset, name=f"UTC{m.group(1)}{hours:02d}:{minutes:02d}")
        return tz, tz_text, None

    if ZoneInfo is not None:
        try:
            return ZoneInfo(tz_text), tz_text, None
        except Exception:
            pass

    return None, tz_text, f"PTLOG timezone metadata present but unparseable ({tz_text!r}); falling back to host local timezone for naive timestamps."

def _pick_time_source(
    df: pd.DataFrame,
    local_tz: Optional[datetime.tzinfo],
    source_tz_hint: Optional[datetime.tzinfo] = None,
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
        tzinfo = getattr(first_ts, "tzinfo", None) or source_tz_hint or local_tz
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


def _resolve_input_timezone(
    input_timezone_mode: str,
    display_tz: Optional[datetime.tzinfo],
    source_tz: Optional[datetime.tzinfo],
) -> Tuple[Optional[datetime.tzinfo], str]:
    """Resolve the timezone used to interpret GUI start/end values."""
    mode = (input_timezone_mode or "").strip().lower()
    if mode == "utc":
        return datetime.timezone.utc, "UTC"
    if mode == "local":
        local_tz = _get_local_tz()
        if local_tz is None:
            return None, "Local"
        return local_tz, "Local"
    if mode == "same as display":
        return display_tz, "Same as display"
    if mode == "log/source time":
        return source_tz, "Log/source time"
    raise ValueError(f"Unknown input timezone mode: {input_timezone_mode}")


def _parse_user_time_in_zone(
    text: str,
    input_timezone_mode: str,
    display_tz: Optional[datetime.tzinfo],
    source_tz: Optional[datetime.tzinfo],
) -> Optional[pd.Timestamp]:
    """Parse GUI text as an aware UTC timestamp based on selected input mode."""
    parsed = _parse_user_time(text)
    if parsed is None:
        return None
    tzinfo, label = _resolve_input_timezone(input_timezone_mode, display_tz, source_tz)
    if tzinfo is None:
        raise ValueError(f"Cannot interpret time for input mode '{label}' because its timezone is unavailable.")
    naive_ts = pd.Timestamp(parsed)
    try:
        aware = naive_ts.tz_localize(tzinfo, ambiguous="raise", nonexistent="raise")
    except Exception as exc:
        raise ValueError(
            f"Entered local time is ambiguous or invalid for timezone mode '{label}'. "
            "Choose a different minute or adjust timezone settings."
        ) from exc
    return aware.tz_convert(datetime.timezone.utc)


def _format_timestamp_for_input_timezone(
    timestamp_utc: pd.Timestamp,
    input_timezone_mode: str,
    display_tz: Optional[datetime.tzinfo],
    source_tz: Optional[datetime.tzinfo],
) -> str:
    """Format a UTC timestamp for insertion into GUI Start/End fields."""
    tzinfo, label = _resolve_input_timezone(input_timezone_mode, display_tz, source_tz)
    if tzinfo is None:
        raise ValueError(f"Cannot format time for input mode '{label}' because its timezone is unavailable.")
    ts_utc = pd.Timestamp(timestamp_utc)
    if ts_utc.tzinfo is None:
        ts_utc = ts_utc.tz_localize(datetime.timezone.utc)
    else:
        ts_utc = ts_utc.tz_convert(datetime.timezone.utc)
    return ts_utc.tz_convert(tzinfo).strftime("%Y-%m-%d %H:%M")


def _build_input_range_text_from_selected_utc(
    selected_start_utc: pd.Timestamp,
    selected_end_utc: pd.Timestamp,
    *,
    input_timezone_mode: str,
    display_tz: Optional[datetime.tzinfo],
    source_tz: Optional[datetime.tzinfo],
    ) -> Tuple[str, str]:
    """Format selected UTC range endpoints for the main Start/End input fields."""
    start_text = _format_timestamp_for_input_timezone(
        selected_start_utc,
        input_timezone_mode=input_timezone_mode,
        display_tz=display_tz,
        source_tz=source_tz,
    )
    end_text = _format_timestamp_for_input_timezone(
        selected_end_utc,
        input_timezone_mode=input_timezone_mode,
        display_tz=display_tz,
        source_tz=source_tz,
    )
    return start_text, end_text


def _apply_range_selection_to_main_form(
    apply_callback,
    *,
    selected_start_utc: pd.Timestamp,
    selected_end_utc: pd.Timestamp,
    input_timezone_mode: str,
    display_tz: Optional[datetime.tzinfo],
    source_tz: Optional[datetime.tzinfo],
) -> None:
    """Apply selected UTC endpoints to main Start/End inputs via callback."""
    start_text, end_text = _build_input_range_text_from_selected_utc(
        selected_start_utc,
        selected_end_utc,
        input_timezone_mode=input_timezone_mode,
        display_tz=display_tz,
        source_tz=source_tz,
    )
    apply_callback(start_text, end_text, mark_manual=True)

def _get_loaded_time_bounds_utc(loaded_log) -> Tuple[pd.Timestamp, pd.Timestamp]:
    if loaded_log.time_column == "__x":
        raise ValueError("Timestamp autofill is unavailable for record_id-only logs.")
    time_series_utc = _canonicalize_time_to_utc(
        loaded_log.dataframe[loaded_log.time_column],
        loaded_log.time_source,
        loaded_log.tzinfo,
    )
    valid_times_utc = time_series_utc.dropna()
    if valid_times_utc.empty:
        raise ValueError("Unable to autofill time range because no valid timestamps were found.")
    return valid_times_utc.min(), valid_times_utc.max()


def _parse_range_text_to_utc(
    start_text: str,
    end_text: str,
    config: ReportConfig,
    loaded_log,
    display_tz: Optional[datetime.tzinfo],
) -> Tuple[Optional[pd.Timestamp], Optional[pd.Timestamp]]:
    return (
        _parse_user_time_in_zone(
            start_text,
            input_timezone_mode=config.input_timezone_mode,
            display_tz=display_tz,
            source_tz=loaded_log.tzinfo,
        ),
        _parse_user_time_in_zone(
            end_text,
            input_timezone_mode=config.input_timezone_mode,
            display_tz=display_tz,
            source_tz=loaded_log.tzinfo,
        ),
    )


def _format_range_utc_for_input_fields(
    start_utc: pd.Timestamp,
    end_utc: pd.Timestamp,
    config: ReportConfig,
    loaded_log,
    display_tz: Optional[datetime.tzinfo],
) -> Tuple[str, str]:
    return _build_input_range_text_from_selected_utc(
        start_utc,
        end_utc,
        input_timezone_mode=config.input_timezone_mode,
        display_tz=display_tz,
        source_tz=loaded_log.tzinfo,
    )


def _parse_positive_int(raw_text: str, field_label: str, default_value: int) -> int:
    text = (raw_text or "").strip()
    if not text:
        return default_value
    try:
        parsed_value = int(text)
    except ValueError as exc:
        raise ValueError(f"{field_label} must be an integer >= 1") from exc
    if parsed_value < 1:
        raise ValueError(f"{field_label} must be an integer >= 1")
    return parsed_value


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
    time_series_utc: Optional[pd.Series] = None,
    input_timezone_mode: str = "Log/source time",
    display_tz: Optional[datetime.tzinfo] = None,
    source_tz: Optional[datetime.tzinfo] = None,
) -> Tuple[pd.DataFrame, str, str, str, str]:
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
        selected_label = "full range"
        return df.copy(), start_label, end_label, summary, selected_label

    if time_series_utc is None:
        time_series_utc = pd.to_datetime(df[time_column], errors="coerce", utc=True)
    if time_series_utc.isna().all():
        raise ValueError("Time column could not be parsed.")

    minutes = time_series_utc.dt.floor("min")
    present_minutes = pd.DatetimeIndex(minutes.unique()).sort_values()

    user_start = _parse_user_time_in_zone(
        start_text,
        input_timezone_mode=input_timezone_mode,
        display_tz=display_tz,
        source_tz=source_tz,
    )
    user_end = _parse_user_time_in_zone(
        end_text,
        input_timezone_mode=input_timezone_mode,
        display_tz=display_tz,
        source_tz=source_tz,
    )

    if user_start and user_end and user_end < user_start:
        raise ValueError("End time is earlier than start time.")

    start_minute = None
    end_minute = None

    if user_start:
        start_ts = pd.Timestamp(user_start)
        if start_ts not in present_minutes:
            nearest_utc = present_minutes[present_minutes.get_indexer([start_ts], method="nearest")[0]]
            nearest = _format_timestamp_for_input_timezone(
                nearest_utc,
                input_timezone_mode=input_timezone_mode,
                display_tz=display_tz,
                source_tz=source_tz,
            )
            raise ValueError(f"Start time not in log minutes. Nearest valid minute: {nearest}")
        start_minute = start_ts

    if user_end:
        end_ts = pd.Timestamp(user_end)
        if end_ts not in present_minutes:
            nearest_utc = present_minutes[present_minutes.get_indexer([end_ts], method="nearest")[0]]
            nearest = _format_timestamp_for_input_timezone(
                nearest_utc,
                input_timezone_mode=input_timezone_mode,
                display_tz=display_tz,
                source_tz=source_tz,
            )
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

    trimmed_times = time_series_utc.loc[trimmed.index]
    actual_start = pd.to_datetime(trimmed_times).min()
    actual_end = pd.to_datetime(trimmed_times).max()
    start_label = actual_start.strftime("%Y-%m-%d %H:%M")
    end_label = actual_end.strftime("%Y-%m-%d %H:%M")
    summary = f"{start_label} → {end_label} ({len(trimmed):,} rows)"
    selected_label = "full range"
    if user_start is not None or user_end is not None:
        selected_label = f"{(start_text or '').strip() or 'min'} → {(end_text or '').strip() or 'max'} ({input_timezone_mode})"
    return trimmed, start_label, end_label, summary, selected_label


def _validate_current_range_with_config(
    config: ReportConfig,
    loaded_log,
    start_text: str,
    end_text: str,
) -> Tuple[pd.DataFrame, str, str, str, str]:
    display_tz = _resolve_display_tz(config.display_timezone)
    if display_tz is None:
        raise ValueError(f"Configured display_timezone is invalid: {config.display_timezone}")
    time_series_utc = None
    if loaded_log.time_column == "__time":
        time_series_utc = _canonicalize_time_to_utc(
            loaded_log.dataframe[loaded_log.time_column],
            loaded_log.time_source,
            loaded_log.tzinfo,
        )
    return _validate_and_trim_by_minute(
        df=loaded_log.dataframe,
        time_column=loaded_log.time_column,
        start_text=start_text,
        end_text=end_text,
        time_series_utc=time_series_utc,
        input_timezone_mode=config.input_timezone_mode,
        display_tz=display_tz,
        source_tz=loaded_log.tzinfo,
    )


def _operator_trimmed_csv_dataframe(trimmed_df: pd.DataFrame) -> pd.DataFrame:
    """Return trimmed CSV rows intended for operator-facing export.

    Internal helper columns start with '__' and are excluded by default.
    """
    operator_columns = [name for name in trimmed_df.columns if not str(name).startswith("__")]
    return trimmed_df.loc[:, operator_columns].copy()



_CAL_STATUS_CALIBRATED = "CALIBRATED"
_CAL_STATUS_UNCALIBRATED = "UNCALIBRATED"
_CAL_STATUS_UNVERIFIED = "UNVERIFIED"


def _parse_header_int(header: Dict[str, str], key: str) -> Optional[int]:
    value = (header.get(key) or "").strip()
    if not value:
        return None
    try:
        return int(value, 10)
    except Exception:
        return None


def _calibration_status_from_header(header: Dict[str, str]) -> Tuple[str, Optional[int], Optional[int], bool]:
    points = _parse_header_int(header, "cal_points_count")
    applied = _parse_header_int(header, "cal_applied")
    missing_keys = (points is None) or (applied is None)
    if missing_keys:
        return _CAL_STATUS_UNVERIFIED, points, applied, True
    if applied == 1 and points > 0:
        return _CAL_STATUS_CALIBRATED, points, applied, False
    return _CAL_STATUS_UNCALIBRATED, points, applied, False


def _any_cal_valid_flags(df: pd.DataFrame) -> bool:
    if "flags" not in df.columns:
        return False
    flags_numeric = pd.to_numeric(df["flags"], errors="coerce").fillna(0).astype("int64")
    cal_mask = _get_flag_mask("CAL_VALID", 1 << 1)
    return bool(((flags_numeric & cal_mask) != 0).any())


def _parse_flags_series_to_int64(flags_series: pd.Series) -> pd.Series:
    """Return Int64 flags for each row; NA for unparseable values."""
    if pd.api.types.is_numeric_dtype(flags_series):
        numeric = pd.to_numeric(flags_series, errors="coerce")
        whole = numeric.where(numeric.isna() | (numeric % 1 == 0))
        return whole.astype("Int64")

    parsed = []
    for value in flags_series:
        if pd.isna(value):
            parsed.append(pd.NA)
            continue
        text = str(value).strip()
        if not text:
            parsed.append(pd.NA)
            continue
        try:
            parsed.append(int(text, 0))
            continue
        except Exception:
            pass
        try:
            as_float = float(text)
            if as_float.is_integer():
                parsed.append(int(as_float))
            else:
                parsed.append(pd.NA)
        except Exception:
            parsed.append(pd.NA)
    return pd.Series(parsed, index=flags_series.index, dtype="Int64")


def _cal_valid_fraction(df: pd.DataFrame) -> Optional[float]:
    """Return fraction of records with CAL_VALID asserted, or None if unavailable."""
    if "flags" not in df.columns:
        return None
    flags_int = _parse_flags_series_to_int64(df["flags"]).dropna()
    if flags_int.empty:
        return None
    cal_mask = _get_flag_mask("CAL_VALID", 1 << 1)
    return float(((flags_int & cal_mask) != 0).mean())


def _is_fully_calibrated(df: pd.DataFrame) -> bool:
    """Return True when every row is CAL_VALID and calibrated temperatures are usable."""
    if "flags" not in df.columns or "cal_temp_c" not in df.columns:
        return False

    flags = _parse_flags_series_to_int64(df["flags"])
    if flags.isna().any():
        return False

    cal_values = pd.to_numeric(df["cal_temp_c"], errors="coerce")
    if cal_values.isna().all():
        return False

    cal_mask = _get_flag_mask("CAL_VALID", 1 << 1)
    return bool((((flags.astype("int64") & cal_mask) != 0)).all())


def _format_applied_records_label(df: pd.DataFrame) -> str:
    fraction = _cal_valid_fraction(df)
    if fraction is None:
        return "unknown (flags unavailable)"
    pct = 100.0 * fraction
    if pct <= 0.0:
        return "no (0.0%)"
    return f"yes ({pct:.1f}%)"


def _segment_header_value(segments: List[MetadataSegment], key: str, default: str = "n/a") -> str:
    """Return a consistent header value across segments, else a 'varies' marker."""
    values = {((segment.header_dict.get(key) or "").strip()) for segment in segments}
    if not values:
        return default
    if len(values) > 1:
        return f"varies ({len(segments)} segments)"
    value = next(iter(values))
    return value if value else default


def _format_span_label(start_ts: pd.Timestamp, end_ts: pd.Timestamp) -> str:
    """Format a compact span label between two timestamps."""
    delta = end_ts - start_ts
    total_seconds = int(max(delta.total_seconds(), 0.0))
    days, rem = divmod(total_seconds, 86400)
    hours, rem = divmod(rem, 3600)
    minutes, seconds = divmod(rem, 60)

    parts: List[str] = []
    if days:
        parts.append(f"{days}d")
    if hours:
        parts.append(f"{hours}h")
    if minutes:
        parts.append(f"{minutes}m")
    if seconds or not parts:
        parts.append(f"{seconds}s")
    return " ".join(parts)


def _format_time_range_for_display(
    start_utc: pd.Timestamp,
    end_utc: pd.Timestamp,
    display_tz: Optional[datetime.tzinfo],
) -> Tuple[str, str, str]:
    """Return display start/end labels plus timezone name for plot/report headings."""
    start_ts = pd.Timestamp(start_utc)
    end_ts = pd.Timestamp(end_utc)
    timezone_label = "UTC"

    if display_tz is not None:
        start_ts = start_ts.tz_convert(display_tz)
        end_ts = end_ts.tz_convert(display_tz)
        timezone_label = start_ts.tzname() or "Local"

    return (
        start_ts.strftime("%Y-%m-%d %H:%M"),
        end_ts.strftime("%Y-%m-%d %H:%M"),
        timezone_label,
    )


def _build_report_subtitle(node_label: str, start_label: str, end_label: str, timezone_label: str) -> str:
    """Build a concise two-line report/plot subtitle."""
    return f"Nodes: {node_label}\nTime range: {start_label} \u2192 {end_label} {timezone_label}"


def _format_performed_utc_to_local_date(utc_text: str, display_tz: Optional[datetime.tzinfo]) -> str:
    """Convert a performed UTC timestamp string into a local YYYY-MM-DD date string."""
    if utc_text in ("", "n/a", "<unset>"):
        return utc_text

    timestamp = pd.to_datetime(utc_text, utc=True, errors="coerce")
    if pd.isna(timestamp):
        return utc_text

    if display_tz is not None:
        timestamp = timestamp.tz_convert(display_tz)
    return timestamp.strftime("%Y-%m-%d")


def _format_due_utc_midnight_to_date_preserve_day(utc_text: str) -> str:
    """Return due-date day as recorded, without timezone conversion."""
    if utc_text in ("", "n/a", "<unset>"):
        return utc_text
    if "T" in utc_text:
        return utc_text.split("T", 1)[0]
    return utc_text


_PTLOG_SIGNATURE_FIELDS = [
    "device_mac",
    "device_serial",
    "timezone_posix",
    "dst_enabled",
    "cal_last_utc",
    "cal_due_rule",
    "cal_due_utc",
    "cal_points_count",
    "cal_applied",
    "cal_method",
    "cal_context",
    "firmware_version",
    "firmware_build_date",
    "firmware_build_time",
]


def _parse_ptlog_header(path: str) -> Tuple[Dict[str, str], int]:
    header: Dict[str, str] = {}
    header_line_count = 0
    in_header = False
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("#PT100_LOG_V1"):
                in_header = True
            if not stripped.startswith("#"):
                if in_header:
                    break
                continue
            header_line_count += 1
            payload = stripped[1:].strip()
            if payload == "END_HEADER":
                break
            if not in_header:
                continue
            if "=" not in payload:
                continue
            key, value = payload.split("=", 1)
            key = key.strip()
            value = value.strip()
            if key:
                header[key] = value
    return header, header_line_count


def _is_ptlog_file(path: str) -> bool:
    if path.lower().endswith(".ptlog"):
        return True
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            for line in handle:
                stripped = line.strip()
                if not stripped:
                    continue
                return stripped.startswith("#PT100_LOG_V1")
    except Exception:
        return False
    return False


def _read_log_file(path: str) -> Tuple[pd.DataFrame, Dict[str, str]]:
    if _is_ptlog_file(path):
        header, _line_count = _parse_ptlog_header(path)
        return pd.read_csv(path, comment="#"), header
    return pd.read_csv(path), {}


def _file_crc32(path: str) -> int:
    crc = 0
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            crc = zlib.crc32(chunk, crc)
    return crc & 0xFFFFFFFF


def _validate_selected_paths(file_paths: List[str]) -> List[str]:
    deduped: List[str] = []
    seen_realpaths = set()
    duplicate_inputs = 0
    for raw in file_paths:
        real = os.path.realpath(raw)
        if real in seen_realpaths:
            duplicate_inputs += 1
            continue
        seen_realpaths.add(real)
        deduped.append(raw)
    if not deduped:
        raise ValueError("No files selected.")

    file_meta: List[Tuple[str, str, int, int]] = []
    for path in deduped:
        if not os.path.exists(path):
            raise ValueError(f"Selected file does not exist: {path}")
        if not os.path.isfile(path):
            raise ValueError(f"Selected path is not a file: {path}")
        try:
            size = os.path.getsize(path)
        except OSError as exc:
            raise ValueError(f"Cannot access file metadata: {path} ({exc})") from exc
        if size <= 0:
            raise ValueError(f"Selected file is empty: {path}")
        try:
            crc = _file_crc32(path)
        except OSError as exc:
            raise ValueError(f"Selected file is not readable: {path} ({exc})") from exc
        file_meta.append((path, os.path.basename(path).lower(), size, crc))

    by_name: Dict[str, List[str]] = {}
    by_content: Dict[Tuple[int, int], List[str]] = {}
    for path, name, size, crc in file_meta:
        by_name.setdefault(name, []).append(path)
        by_content.setdefault((size, crc), []).append(path)
    dup_names = [paths for paths in by_name.values() if len(paths) > 1]
    dup_content = [paths for paths in by_content.values() if len(paths) > 1]
    warn_bits = []
    # Exact duplicate path picks are silently deduplicated.
    if dup_names:
        warn_bits.append("duplicate basenames detected")
    if dup_content:
        warn_bits.append("duplicate file content detected")
    if warn_bits:
        raise ValueError("Likely accidental duplicate inputs: " + "; ".join(warn_bits))
    return deduped


def _parse_log_filename_sort_key(path: str) -> Tuple[str, int, int, str]:
    name = os.path.basename(path)
    match = re.match(r"^(\d{4}-\d{2}-\d{2})Z(?:-(\d+))?\.(ptlog|csv)$", name, flags=re.IGNORECASE)
    if match:
        date_part = match.group(1)
        rev = int(match.group(2) or "0")
        ext = match.group(3).lower()
        ext_priority = 0 if ext == "ptlog" else 1
        return (date_part, rev, ext_priority, name.lower())
    return ("9999-99-99", 999999, 999, name.lower())


def _dedupe_folder_file_paths(file_paths: List[str]) -> List[str]:
    has_ptlog = any(path.lower().endswith(".ptlog") for path in file_paths)
    if not has_ptlog:
        return sorted(file_paths, key=_parse_log_filename_sort_key)

    ptlog_stems = {Path(path).stem.lower() for path in file_paths if path.lower().endswith(".ptlog")}
    filtered: List[str] = []
    for path in file_paths:
        lower = path.lower()
        if lower.endswith(".csv") and Path(path).stem.lower() in ptlog_stems:
            continue
        filtered.append(path)
    return sorted(filtered, key=_parse_log_filename_sort_key)


def _metadata_signature(header: Dict[str, str]) -> int:
    payload_fields = []
    serial_value = header.get("device_mac") or header.get("device_serial") or ""
    payload_fields.append(serial_value)
    for field in _PTLOG_SIGNATURE_FIELDS[1:]:
        payload_fields.append(header.get(field, ""))
    payload = "\n".join(payload_fields).encode("utf-8")
    return zlib.crc32(payload) & 0xFFFFFFFF


def _device_serial_for_report(header: Dict[str, str]) -> str:
    serial = (header.get("device_serial") or "").strip()
    mac = (header.get("device_mac") or "").strip()
    if mac:
        return mac
    if serial:
        return serial
    return "n/a"


def _truncate_text(value: str, max_len: int = 40) -> str:
    value = (value or "").strip()
    if len(value) <= max_len:
        return value
    return value[: max_len - 1] + "…"


def _build_audit_summary(file_paths: List[str], headers_by_file: Dict[str, Dict[str, str]], combined: pd.DataFrame) -> AuditSummary:
    serials = sorted({_device_serial_for_report(headers_by_file.get(path, {})) for path in file_paths if _device_serial_for_report(headers_by_file.get(path, {})) != "n/a"})

    serial_source_note: Optional[str] = None
    for path in file_paths:
        header = headers_by_file.get(path, {})
        serial = (header.get("device_serial") or "").strip()
        mac = (header.get("device_mac") or "").strip()
        if serial and mac and serial != mac:
            serial_source_note = "Serial source: MAC (device_serial differs)"
            break

    groups: Dict[int, MetadataSegment] = {}
    statuses: List[str] = []
    points_values: List[Optional[int]] = []
    legacy_missing_keys = False
    for path in file_paths:
        header = headers_by_file.get(path, {})
        signature = _metadata_signature(header)
        file_name = os.path.basename(path)
        if signature not in groups:
            groups[signature] = MetadataSegment(signature=signature, header_dict=dict(header), file_names=[file_name])
        else:
            groups[signature].file_names.append(file_name)
        status, points, _applied, missing = _calibration_status_from_header(header)
        statuses.append(status)
        points_values.append(points)
        legacy_missing_keys = legacy_missing_keys or missing

    if "epoch_utc" in combined.columns:
        epoch_series = pd.to_numeric(combined["epoch_utc"], errors="coerce").dropna()
        if not epoch_series.empty:
            min_epoch = int(epoch_series.min())
            max_epoch = int(epoch_series.max())
            for segment in groups.values():
                segment.time_range_utc = (min_epoch, max_epoch)

    def _consistent(fields: List[str], fmt, default: str = "n/a") -> str:
        values = set()
        for segment in groups.values():
            header = segment.header_dict
            values.add(tuple(header.get(field, "") for field in fields))
        if len(values) != 1:
            return f"varies ({len(groups)} segments)"
        value_tuple = next(iter(values))
        if all(not v for v in value_tuple):
            return default
        return fmt(*value_tuple)

    unique_statuses = set(statuses)
    if unique_statuses == {_CAL_STATUS_CALIBRATED}:
        calibration_status = _CAL_STATUS_CALIBRATED
    elif _CAL_STATUS_UNVERIFIED in unique_statuses:
        calibration_status = _CAL_STATUS_UNVERIFIED
    else:
        calibration_status = _CAL_STATUS_UNCALIBRATED

    known_points = [value for value in points_values if value is not None]
    if not known_points:
        calibration_points = "unknown"
    elif all(value == known_points[0] for value in known_points):
        calibration_points = str(known_points[0])
    else:
        calibration_points = "varies"

    calibration_summary = _consistent(
        ["cal_last_utc", "cal_due_utc", "cal_method"],
        lambda last, due, method: f"last {last or 'n/a'} | due {due or 'n/a'} | method {_truncate_text(method or '<unset>')}",
    )
    timezone_summary = _consistent(
        ["timezone_posix", "dst_enabled"],
        lambda tz, dst: f"{tz or 'n/a'} (DST={dst or 'n/a'})",
    )
    firmware_summary = _consistent(
        ["firmware_version", "firmware_build_date", "firmware_build_time"],
        lambda ver, date, t: f"{ver or 'n/a'} {date or ''} {t or ''}".strip(),
    )

    calibration_warning: Optional[str] = None
    if calibration_status != _CAL_STATUS_CALIBRATED and _any_cal_valid_flags(combined):
        if legacy_missing_keys:
            calibration_warning = "CAL_VALID flags present, but calibration evidence missing; treated as Unverified."
        else:
            calibration_warning = "CAL_VALID flags present, but header calibration evidence is uncalibrated."

    return AuditSummary(
        device_serials=serials,
        segments=sorted(groups.values(), key=lambda seg: seg.file_names[0] if seg.file_names else ""),
        calibration_summary=calibration_summary,
        calibration_status=calibration_status,
        calibration_points=calibration_points,
        calibration_warning=calibration_warning,
        timezone_summary=timezone_summary,
        firmware_summary=firmware_summary,
        serial_source_note=serial_source_note,
    )


def _load_log_files(file_paths: List[str]) -> LoadedLog:
    file_paths = _validate_selected_paths(file_paths)

    sorted_paths = sorted(file_paths, key=_parse_log_filename_sort_key)

    dataframes: List[pd.DataFrame] = []
    headers_by_file: Dict[str, Dict[str, str]] = {}
    for path in sorted_paths:
        df, header = _read_log_file(path)
        if df.empty:
            raise ValueError(f"Selected file has no data rows: {path}")
        df = _normalize_schema(df)
        df["__source_file"] = os.path.basename(path)
        dataframes.append(df)
        headers_by_file[path] = header

    combined = pd.concat(dataframes, ignore_index=True)
    if "record_id" in combined.columns:
        record_numeric = pd.to_numeric(combined["record_id"], errors="coerce")
        if not record_numeric.notna().any():
            has_time_cols = any(col in combined.columns for col in ("iso8601_local", "epoch_utc", "__time"))
            if not has_time_cols:
                raise ValueError("No usable timestamp and no usable record_id in selected files.")

    local_tz = _get_local_tz()
    source_tz, source_tz_label, source_tz_warning = _resolve_source_timezone_from_headers(headers_by_file)
    combined, time_column, time_source, tzinfo, dropped_no_time_rows = _pick_time_source(
        combined,
        local_tz=local_tz,
        source_tz_hint=source_tz,
    )
    if tzinfo is None:
        tzinfo = source_tz

    combined = _ensure_sequence_column(combined)
    combined["__seq_unwrapped"] = _compute_unwrapped_sequence(combined["__seq"])
    missing_seq = int(combined["__seq_unwrapped"].isna().sum())
    if missing_seq:
        raise ValueError(
            f"Sequence ordering requires seq/record_id for every row; {missing_seq} rows are missing."
        )

    if time_column == "__x":
        combined[time_column] = pd.to_numeric(combined[time_column], errors="coerce")

    combined = combined.sort_values(by="__seq_unwrapped", kind="mergesort").reset_index(drop=True)
    if "node_id" in combined.columns and "record_id" in combined.columns:
        rec = pd.to_numeric(combined["record_id"], errors="coerce")
        node = combined["node_id"].astype(str).fillna("")
        pair_df = pd.DataFrame({"__node": node, "__rid": rec})
        dup_mask = rec.notna() & pair_df.duplicated(subset=["__node", "__rid"], keep=False)
        if bool(dup_mask.any()):
            raise ValueError(
                f"Duplicate records detected after concatenation by node_id + record_id ({int(dup_mask.sum())} rows)."
            )
    audit_summary = _build_audit_summary(sorted_paths, headers_by_file, combined)

    return LoadedLog(
        dataframe=combined,
        time_column=time_column,
        time_source=time_source,
        tzinfo=tzinfo,
        source_files=sorted_paths,
        dropped_no_time_rows=dropped_no_time_rows,
        file_headers=headers_by_file,
        audit_summary=audit_summary,
        source_timezone_label=source_tz_label,
        source_timezone_warning=source_tz_warning,
    )


def _parse_nonnegative_float(raw_text: str, field_label: str, default_value: float) -> float:
    text = (raw_text or "").strip()
    if not text:
        return float(default_value)
    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError(f"Invalid {field_label}: {text}") from exc
    if value < 0:
        raise ValueError(f"{field_label} must be non-negative.")
    return value


def _build_status_flag_rows(df: pd.DataFrame) -> List[FlagSummaryRow]:
    if "flags" not in df.columns:
        return []
    flags_numeric = pd.to_numeric(df["flags"], errors="coerce")
    flags_int = flags_numeric.dropna().astype("int64")
    total = int(flags_int.shape[0])
    if total <= 0:
        return []

    rows: List[FlagSummaryRow] = []
    for mask, short_name in _LOG_RECORD_FLAG_DEFS:
        set_count = int(((flags_int & mask) != 0).sum())
        if short_name in {"TIME_VALID", "CAL_VALID"}:
            count = total - set_count
        else:
            count = set_count
        percent = 100.0 * float(count) / float(total)
        rows.append(FlagSummaryRow(
            short_name=short_name,
            label=_FLAG_LABELS.get(short_name, short_name.replace("_", " ").title()),
            count=count,
            percent=percent,
            meaning=_FLAG_MEANINGS.get(short_name, ""),
            is_problem=short_name in _ERROR_FLAG_SHORT_NAMES or short_name in {"TIME_VALID", "CAL_VALID", "TIME_JUMP_BACK"},
            include_in_pdf_by_default=short_name in {"SD_ERROR", "FRAM_FULL", "TIME_JUMP_BACK", "TIME_VALID", "CAL_VALID"},
        ))
    return rows




def _format_percent_for_status_table(percent: float) -> str:
    """Format status percentage text for compact table display."""
    if percent > 0 and percent < 0.1:
        return f"{percent:.2f}%"
    return f"{percent:.2f}%"


def _filter_status_flag_rows_for_display(
    rows: Sequence[FlagSummaryRow],
    *,
    show_zero_count: bool = False,
) -> List[FlagSummaryRow]:
    """Return rows for UI display, defaulting to nonzero/problem-focused rows."""
    if show_zero_count:
        return list(rows)
    return [row for row in rows if row.count > 0]


def _default_status_detail_message(rows: Sequence[FlagSummaryRow]) -> str:
    if not rows:
        return "Meaning: No flags detected in current dataset."
    return f"Meaning: {rows[0].meaning or rows[0].label}"


def _human_config_label(key: str) -> str:
    return _CONFIG_LABELS.get(key, key.replace("_", " ").title())


def _status_table_values(row: FlagSummaryRow) -> Tuple[str, int, str]:
    return row.short_name, row.count, _format_percent_for_status_table(row.percent)


def _status_flag_meaning_for_key(key: str) -> str:
    return _FLAG_MEANINGS.get(key, key)

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


def _escape_reportlab_text(value: object) -> str:
    """Escape free-form text for safe ReportLab Paragraph markup."""
    if value is None:
        return ""
    return html.escape(str(value), quote=False)



def _classify_sensor_fault_rows(df: pd.DataFrame) -> Optional[SensorFaultRowClassification]:
    """Classify SENSOR_FAULT rows by decoded MAX31865 fault-status byte state."""
    if "flags" not in df.columns or "fault_status" not in df.columns:
        return None

    flags_numeric = pd.to_numeric(df["flags"], errors="coerce").fillna(0).astype("int64")
    sensor_fault_mask = _get_flag_mask("SENSOR_FAULT", 1 << 4)
    if sensor_fault_mask == 0:
        return None

    fault_rows = (flags_numeric & sensor_fault_mask) != 0
    total_sensor_fault_count = int(fault_rows.sum())
    if total_sensor_fault_count == 0:
        return SensorFaultRowClassification(0, {}, 0, 0, 0.0)

    fault_status_raw = pd.to_numeric(df.loc[fault_rows, "fault_status"], errors="coerce")
    parsed_mask = fault_status_raw.notna()
    fault_status_numeric = fault_status_raw.fillna(0).astype("int64")

    nonzero = fault_status_numeric[parsed_mask & (fault_status_numeric != 0)]
    nonzero_counts = {int(code) & 0xFF: int(count) for code, count in nonzero.value_counts().sort_index().items()}
    zero_count = int((parsed_mask & (fault_status_numeric == 0)).sum())
    unparseable_count = int((~parsed_mask).sum())
    zero_percent = (100.0 * float(zero_count) / float(len(df))) if len(df) else 0.0

    return SensorFaultRowClassification(
        zero_fault_status_count=zero_count,
        nonzero_fault_status_counts_by_code=nonzero_counts,
        unparseable_fault_status_count=unparseable_count,
        total_sensor_fault_count=total_sensor_fault_count,
        zero_fault_status_percent=zero_percent,
    )


def _format_flags_summary(
    df: pd.DataFrame,
    *,
    display_tz: Optional[datetime.tzinfo],
    time_source: str,
    sensor_fault_threshold_percent: float = 0.10,
) -> str:
    """Build a concise summary of *problem* log_record flags.

    This intentionally hides informational flags (for example, MESH_CONNECTED).
    If no problems are detected in the selected data, this returns 'n/a' so the
    report omits the row entirely.

    Args:
        df: Log dataframe (already normalized).

    Returns:
        A multi-line string suitable for the PDF summary table, or 'n/a' if the
        CSV does not contain usable flags or if no problems are present.
    """
    rows = _build_status_flag_rows(df)
    if not rows:
        return "n/a"

    total = int(pd.to_numeric(df["flags"], errors="coerce").dropna().shape[0])
    flags_int = pd.to_numeric(df["flags"], errors="coerce").dropna().astype("int64")

    def _pct(count: int) -> str:
        pct = 100.0 * float(count) / float(total)
        if count > 0 and pct < 0.1:
            return f"{pct:.2f}%"
        return f"{pct:.1f}%"

    problem_lines: List[str] = []

    for row in rows:
        short_name = row.short_name
        if short_name == "MESH_CONNECTED" or short_name == "RTD_EMA":
            continue
        if short_name == "SENSOR_FAULT" and row.count > 0:
            fault_class = _classify_sensor_fault_rows(df)
            if fault_class is not None:
                has_nonzero = bool(fault_class.nonzero_fault_status_counts_by_code)
                has_unparseable = fault_class.unparseable_fault_status_count > 0
                zero_below_threshold = (
                    fault_class.zero_fault_status_count > 0
                    and fault_class.zero_fault_status_percent < sensor_fault_threshold_percent
                )
                if not has_nonzero and not has_unparseable and zero_below_threshold:
                    continue
            elif row.percent < sensor_fault_threshold_percent:
                continue
        if short_name == "TIME_VALID" and row.count:
            problem_lines.append(f"Time invalid: {row.count}/{total} ({_pct(row.count)})")
            continue
        if short_name == "CAL_VALID" and row.count:
            problem_lines.append(f"Calibration invalid: {row.count}/{total} ({_pct(row.count)})")
            continue
        if short_name in _ERROR_FLAG_SHORT_NAMES and row.count:
            label = _FLAG_LABELS.get(short_name, short_name)
            problem_lines.append(f"{label}: {row.count}/{total} ({_pct(row.count)})")

    time_jump_mask = _get_time_jump_back_mask(df)
    time_jump_count = int(time_jump_mask.sum())
    if time_jump_count:
        first_index = int(np.flatnonzero(time_jump_mask)[0])
        first_time_label = "n/a"
        if "__time" in df.columns:
            display_times = _convert_time_series_to_display_tz(
                df["__time"],
                display_tz=display_tz,
                time_source=time_source,
            )
            first_time = display_times.iloc[first_index]
            if pd.notna(first_time):
                first_time_label = pd.Timestamp(first_time).strftime("%Y-%m-%d %H:%M:%S")

        first_seq_label = "n/a"
        if "__seq" in df.columns:
            first_seq = df["__seq"].iloc[first_index]
            if pd.notna(first_seq):
                first_seq_label = str(int(first_seq))

        problem_lines.append(
            f"Time jump back events: {time_jump_count} (first at {first_time_label}, seq {first_seq_label})"
        )

    if not problem_lines:
        return "n/a"

    return "\n".join(problem_lines)






def _decode_max31865_fault_status(fault_status: int) -> str:
    """Return a comma-separated label list for a MAX31865 fault-status byte."""
    labels = [label for bit, label in _MAX31865_FAULT_BITS if (fault_status & bit) != 0]
    if labels:
        return ", ".join(labels)
    return f"unknown(0x{fault_status:02X})"


def _format_fault_status_summary(
    df: pd.DataFrame,
    *,
    include_zero_fault_status: bool = True,
    sensor_fault_threshold_percent: Optional[float] = None,
) -> str:
    """Build a fault-status summary for rows flagged with SENSOR_FAULT."""
    fault_class = _classify_sensor_fault_rows(df)
    if fault_class is None or fault_class.total_sensor_fault_count == 0:
        return "n/a"

    lines: List[str] = []
    for code_int, count in fault_class.nonzero_fault_status_counts_by_code.items():
        label = _decode_max31865_fault_status(code_int)
        row_word = "row" if int(count) == 1 else "rows"
        lines.append(f"0x{code_int:02X}: {int(count)} {row_word} ({label})")

    include_zero = include_zero_fault_status
    if sensor_fault_threshold_percent is not None and fault_class.zero_fault_status_count > 0:
        if fault_class.zero_fault_status_percent < sensor_fault_threshold_percent:
            include_zero = False

    if include_zero and fault_class.zero_fault_status_count > 0:
        row_word = "row" if fault_class.zero_fault_status_count == 1 else "rows"
        lines.append(
            "Sensor read failure / no MAX31865 fault byte recorded: "
            f"{fault_class.zero_fault_status_count} {row_word}"
        )

    if fault_class.unparseable_fault_status_count > 0:
        row_word = "row" if fault_class.unparseable_fault_status_count == 1 else "rows"
        lines.append(f"Unparseable fault_status: {fault_class.unparseable_fault_status_count} {row_word}")

    return "\n".join(lines) if lines else "n/a"


def _prepare_series_for_statistics(
    df: pd.DataFrame,
    y_series: pd.Series,
    y_name: str,
) -> Tuple[pd.Series, List[str]]:
    """Prepare a statistics-ready series shared by plot overlays and PDF summaries.

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
        numeric = pd.to_numeric(y_series, errors="coerce").dropna()
        return numeric, []

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
    numeric = pd.to_numeric(filtered, errors="coerce").dropna()
    dropped_nan = int(filtered.shape[0] - numeric.shape[0])
    if dropped_nan > 0:
        notes.append(f"dropped NaN/unparseable rows: {dropped_nan}")
    return numeric, notes


def _apply_flag_filters_for_stats(
    df: pd.DataFrame,
    y_series: pd.Series,
    y_name: str,
) -> Tuple[pd.Series, List[str]]:
    """Backward-compatible alias for statistics series preparation."""
    return _prepare_series_for_statistics(df, y_series, y_name)


def _compute_numeric_stats(series: pd.Series) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
    if series.empty:
        return None, None, None, None
    return float(series.min()), float(series.mean()), float(series.max()), float(series.std(ddof=0))


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
            zorder=0.8,
            label=label if idx == 0 else "_nolegend_",
        )


def _add_start_date_label_if_multiday(ax: plt.Axes, x_values: pd.Series) -> None:
    """Add a left-side x-axis start date label when plotted data spans multiple days.

    Args:
        ax: Matplotlib axes to annotate.
        x_values: Datetime-like x-axis values (timezone-aware preferred).
    """
    if not _needs_multiday_date_label(x_values):
        return

    parsed = pd.to_datetime(x_values, errors="coerce")
    start_ts = parsed.min()

    offset = ax.xaxis.get_offset_text()
    y = offset.get_position()[1]

    start_text = next((txt for txt in ax.texts if txt.get_gid() == "pt100_start_date"), None)
    if start_text is None:
        start_text = ax.text(
            0.0,
            y,
            "",
            zorder=5,
        )
        start_text.set_gid("pt100_start_date")
        start_text.set_in_layout(True)
        start_text.set_clip_on(False)

    start_text.set_text(start_ts.strftime("%Y-%b-%d"))
    start_text.set_transform(offset.get_transform())
    start_text.set_ha("left")
    start_text.set_va(offset.get_va())
    start_text.set_position((0.0, y))
    start_text.set_fontproperties(offset.get_fontproperties())
    start_text.set_color(offset.get_color())


def _needs_multiday_date_label(x_values: pd.Series) -> bool:
    parsed = pd.to_datetime(x_values, errors="coerce")
    if parsed.empty:
        return False
    start_ts = parsed.min()
    end_ts = parsed.max()
    if pd.isna(start_ts) or pd.isna(end_ts):
        return False
    return start_ts.date() != end_ts.date()


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
    # Temperature unit conversion only applies to temperature series.
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
        window_size = min(151, max(3, valid_points // options.rolling_mean_divisor))
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

    time_jump_mask = _get_time_jump_back_mask(df)
    time_jump_positions = np.flatnonzero(time_jump_mask)
    if time_jump_positions.size:
        include_positions = set(plot_positions.tolist())
        for jump_index in time_jump_positions:
            include_positions.add(int(jump_index))
            if jump_index > 0:
                include_positions.add(int(jump_index - 1))
        plot_positions = np.unique(np.fromiter(include_positions, dtype=np.int64))

    x_plot = x_values.iloc[plot_positions]
    y_plot = y_series.iloc[plot_positions]

    fig, ax = plt.subplots(figsize=(11, 6.2))
    ax.grid(True)
    ax.set_title(plot_title)

    if suptitle:
        fig.suptitle(suptitle, fontsize=9)

    if time_column == "__time" and options.display_time_config.display_tz_label not in ("", "n/a"):
        ax.set_xlabel(f"{_human_time_label(time_column)} ({options.display_time_config.display_tz_label})")
    else:
        ax.set_xlabel(_human_time_label(time_column))
    ax.set_ylabel(_human_series_label(y_name, temp_unit=options.temp_unit))

    if options.smooth and smoothed_series is not None:
        ax.plot(x_plot, y_plot, linewidth=0.7, alpha=0.7, zorder=4.0, label="data")
        ax.plot(x_plot, smoothed_series.iloc[plot_positions], linewidth=2.0, zorder=5.0, label="rolling mean")
    else:
        ax.plot(x_plot, y_plot, linewidth=1.2, zorder=4.0, label="data")

    if raw_temp_series is not None:
        ax.plot(x_plot, raw_temp_series.iloc[plot_positions], linewidth=0.9, alpha=0.7, zorder=4.1, label="raw_temp_c (data)")

    if time_jump_positions.size:
        time_jump_label = _FLAG_LABELS.get("TIME_JUMP_BACK", "Time jump back (RTC sync)")
        for idx, jump_index in enumerate(time_jump_positions):
            x_jump = x_values.iloc[jump_index]
            ax.axvline(
                x_jump,
                color="#d62728",
                linestyle=":",
                linewidth=1.0,
                alpha=0.7,
                zorder=7.0,
                label=time_jump_label if idx == 0 else "_nolegend_",
            )
            y_jump = y_series.iloc[jump_index]
            if pd.notna(y_jump):
                ax.scatter([x_jump], [y_jump], color="#d62728", marker="x", s=35, zorder=7.1, label="_nolegend_")

    is_temperature = y_name in ("cal_temp_c", "raw_temp_c")
    if is_temperature:
        stats_source_series, _stats_notes = _prepare_series_for_statistics(df, y_series, y_name)
        stats_min, stats_avg, stats_max, stats_std = _compute_numeric_stats(stats_source_series)
        if stats_avg is not None:
            if options.stats.show_min and stats_min is not None:
                ax.axhline(stats_min, color="#8b0000", linestyle="--", linewidth=1.0, zorder=3.0, label="min")
            if options.stats.show_max and stats_max is not None:
                ax.axhline(stats_max, color="#8b0000", linestyle="--", linewidth=1.0, zorder=3.0, label="max")
            if options.stats.show_avg:
                ax.axhline(stats_avg, color="#1f77b4", linestyle="-.", linewidth=1.2, zorder=3.0, label="avg")
            if options.stats.show_std_band and stats_std is not None:
                ax.fill_between(
                    x_plot,
                    stats_avg - stats_std,
                    stats_avg + stats_std,
                    color="#1f77b4",
                    alpha=0.16,
                    zorder=1.0,
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
                zorder=3.0,
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
                zorder=3.0,
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
    # Ensure event markers render above the data line.
    if time_column == "__time" and "flags" in df.columns:
        time_jump_mask = _get_flag_mask("TIME_JUMP_BACK", 1 << 7)
        try:
            flags_int = df["flags"].fillna(0).astype("int64")
            jump_positions = np.flatnonzero((flags_int.values & time_jump_mask) != 0)
        except Exception:
            jump_positions = np.asarray([], dtype=np.int64)

        if jump_positions.size:
            jump_x_values = x_values.iloc[jump_positions]
            seen_keys = set()
            unique_jump_x: List[object] = []
            for item in jump_x_values:
                key = str(item)
                if key in seen_keys:
                    continue
                seen_keys.add(key)
                unique_jump_x.append(item)

            for idx, x0 in enumerate(unique_jump_x):
                ax.axvline(
                    x0,
                    color="0.10",
                    linewidth=3.0,
                    alpha=0.95,
                    zorder=7.0,
                    label="time jump back" if idx == 0 else "_nolegend_",
                )

    legend = ax.legend(framealpha=1.0)
    if legend is not None:
        legend.set_zorder(10.0)

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
        fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.90])
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


def _node_id_values(df: pd.DataFrame) -> List[str]:
    if "node_id" not in df.columns:
        return []
    return sorted({str(v) for v in df["node_id"].dropna().unique() if str(v).strip()})


def _export_pdf_report(
    save_path: str,
    fig_png_path: str,
    source_files: List[str],
    summary_rows: List[List[str]],
    calibration_rows: List[List[str]],
    title: str,
    subtitle: Optional[str],
    warning_text: Optional[str] = None,
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

    styles_warn = ParagraphStyle(
        "WarnStyle",
        fontName="Helvetica-Bold",
        fontSize=13.5,
        leading=16,
        alignment=TA_CENTER,
        textColor=colors.red,
        spaceAfter=12,
    )
    styles_body = ParagraphStyle(
        "BodyStyle",
        fontName="Helvetica",
        fontSize=10,
        leading=13,
    )

    elements: List[object] = []
    elements.append(Paragraph(_escape_reportlab_text(title), styles_title))
    elements.append(Spacer(1, 0.12 * inch))
    if subtitle:
        subtitle_text = _escape_reportlab_text(subtitle)
        if subtitle_text:
            elements.append(Paragraph(subtitle_text, styles_sub))
    if warning_text:
        warning_html = _escape_reportlab_text(warning_text).replace("\n", "<br/>")
        elements.append(Paragraph(warning_html, styles_warn))


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

    cal_table = Table(calibration_rows, colWidths=[2.3 * inch, 4.7 * inch])
    cal_table.setStyle(
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
    elements.append(cal_table)
    elements.append(Spacer(1, 8))

    img = Image(fig_png_path, width=7.5 * inch, height=4.2 * inch)
    elements.append(img)
    elements.append(Spacer(1, 10))

    file_list = "\n".join([_escape_reportlab_text(os.path.basename(p)) for p in source_files[:12]])
    if len(source_files) > 12:
        file_list += f"\n… ({len(source_files)} files total)"
    elements.append(Paragraph(f"<b>Input file(s):</b><br/>{file_list}", styles_body))

    doc.build(elements)


def _export_pdf_report_vector(
    save_path: str,
    fig: plt.Figure,
    source_files: List[str],
    summary_rows: List[List[str]],
    calibration_rows: List[List[str]],
    title: str,
    subtitle: Optional[str],
    warning_text: Optional[str] = None,
) -> None:
    """Export a vector PDF using Matplotlib's PDF backend.

    This avoids rasterization artifacts and is typically better for printing.
    """
    # Page 1: plot (vector).
    # Reserve some headroom for a report header above the plot title.
    fig.subplots_adjust(top=0.84)
    fig.text(
        0.5,
        0.985,
        title,
        ha="center",
        va="top",
        fontsize=14,
        fontweight="bold",
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

        
        summary_table_bbox = [0.0, 0.48, 1.0, 0.47]
        calibration_table_bbox = [0.0, 0.27, 1.0, 0.18]
        if warning_text:
            ax.text(
                0.5,
                0.96,
                str(warning_text),
                ha="center",
                va="top",
                fontsize=12,
                fontweight="bold",
                color="red",
                transform=ax.transAxes,
                wrap=True,
            )
            # Leave extra room above the tables for the warning.
            summary_table_bbox = [0.0, 0.48, 1.0, 0.37]
            calibration_table_bbox = [0.0, 0.27, 1.0, 0.18]

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
            bbox=summary_table_bbox,
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9)

        cal_cell_text = calibration_rows[1:] if len(calibration_rows) > 1 else []
        cal_col_labels = calibration_rows[0] if calibration_rows else ["Field", "Value"]
        cal_table = ax.table(
            cellText=cal_cell_text,
            colLabels=cal_col_labels,
            cellLoc="left",
            colLoc="left",
            loc="upper left",
            bbox=calibration_table_bbox,
        )
        cal_table.auto_set_font_size(False)
        cal_table.set_fontsize(9)

        # Input file list at the bottom.
        file_list = "\n".join([os.path.basename(p) for p in source_files[:20]])
        if len(source_files) > 20:
            file_list += f"\n… ({len(source_files)} files total)"

        ax.text(
            0.0,
            0.12,
            "Input file(s):",
            ha="left",
            va="top",
            fontsize=11,
            fontweight="bold",
            transform=ax.transAxes,
        )
        ax.text(
            0.0,
            0.09,
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
        self._base_title = "PT100 Log Plotter + PDF Report"
        self.root.title(self._base_title)

        self.selected_files: List[str] = []
        self.loaded: Optional[LoadedLog] = None

        self.start_time_text = tk.StringVar(value="")
        self.end_time_text = tk.StringVar(value="")
        self._has_manual_time_range = False
        self.series_choices = ["cal_temp_c", "raw_temp_c", "raw_rtd_ohms"]
        self._warned_aggregated = False
        self.config_path: Optional[str] = None
        self.report_config_loaded = False
        self.report_config: Optional[ReportConfig] = None
        self._config_dirty = False
        self.pdf_sensor_fault_threshold_text = tk.StringVar(value="0.10")
        self._range_validation_error: Optional[str] = None

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_exit_requested)
        self._refresh_action_enabled_state()
        self.root.after(0, self._show_startup_dialog)

    def _build_ui(self) -> None:
        self.root.geometry("640x760")
        self._build_menu()
        frm = tk.Frame(self.root, padx=10, pady=10)
        frm.pack(fill="both", expand=True)
        frm.columnconfigure(0, weight=1)

        config_frame = tk.LabelFrame(frm, text="1) Config", padx=8, pady=6)
        config_frame.grid(row=0, column=0, sticky="ew")
        config_frame.columnconfigure(1, weight=1)
        tk.Label(config_frame, text="Config file:").grid(row=0, column=0, sticky="w")
        self.config_summary_label = tk.Label(config_frame, text="(none loaded)", anchor="w", justify="left")
        self.config_summary_label.grid(row=0, column=1, sticky="ew")
        self.config_details_label = tk.Label(
            config_frame,
            text="Input time zone: n/a\nDisplay time zone: n/a\nY-axis series: n/a",
            justify="left",
            anchor="w",
        )
        self.config_details_label.grid(row=2, column=0, columnspan=2, sticky="w", pady=(6, 0))

        log_frame = tk.LabelFrame(frm, text="2) Log files", padx=8, pady=6)
        log_frame.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        log_frame.columnconfigure(1, weight=1)
        tk.Label(log_frame, text="Selected:").grid(row=0, column=0, sticky="nw")
        self.file_label = tk.Label(log_frame, text="(none selected)", anchor="w", justify="left")
        self.file_label.grid(row=0, column=1, sticky="ew")
        tk.Button(log_frame, text="Select Log Files", command=self.select_files).grid(row=1, column=0, sticky="w", pady=(6, 0))
        tk.Button(log_frame, text="Select Folder", command=self.select_folder).grid(row=1, column=1, sticky="w", pady=(6, 0))

        range_frame = tk.LabelFrame(frm, text="3) Range", padx=8, pady=6)
        range_frame.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        tk.Label(range_frame, text='Start time (YYYY-MM-DD HH:MM):').grid(row=0, column=0, sticky="w")
        self.start_time_entry = tk.Entry(range_frame, textvariable=self.start_time_text, width=22)
        self.start_time_entry.grid(row=0, column=1, sticky="w", padx=(6, 0))
        tk.Label(range_frame, text='End time (YYYY-MM-DD HH:MM):').grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.end_time_entry = tk.Entry(range_frame, textvariable=self.end_time_text, width=22)
        self.end_time_entry.grid(row=1, column=1, sticky="w", padx=(6, 0), pady=(6, 0))
        self.select_range_btn = tk.Button(range_frame, text="Select range…", command=self.open_range_selector, state=tk.DISABLED)
        self.select_range_btn.grid(row=0, column=2, rowspan=2, sticky="w", padx=(10, 0))
        self.start_time_text.trace_add("write", self._on_range_text_changed)
        self.end_time_text.trace_add("write", self._on_range_text_changed)

        action_frame = tk.LabelFrame(frm, text="4) Actions", padx=8, pady=6)
        action_frame.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        self._create_status_flags_section(frm, 4, 0)
        self.plot_btn = tk.Button(action_frame, text="Plot", command=self.plot, state=tk.DISABLED)
        self.plot_btn.pack(side="left")
        self.save_trim_btn = tk.Button(action_frame, text="Save Trimmed CSV", command=self.save_trimmed_csv, state=tk.DISABLED)
        self.save_trim_btn.pack(side="left", padx=(6, 0))
        self.pdf_btn = tk.Button(action_frame, text="Export PDF Report", command=self.export_pdf, state=tk.DISABLED)
        self.pdf_btn.pack(side="left", padx=(6, 0))

    def _build_menu(self) -> None:
        menu = tk.Menu(self.root)
        file_menu = tk.Menu(menu, tearoff=0)
        file_menu.add_command(label="Load Config...", command=self.load_config)
        file_menu.add_command(label="New Config...", command=self._create_new_config)
        file_menu.add_command(label="Save Config", command=self.save_config)
        file_menu.add_command(label="Save Config As...", command=self.save_config_as)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self._on_exit_requested)
        menu.add_cascade(label="File", menu=file_menu)

        options_menu = tk.Menu(menu, tearoff=0)
        options_menu.add_command(label="Edit Options...", command=self.edit_options)
        menu.add_cascade(label="Options", menu=options_menu)
        self.root.config(menu=menu)

    def _refresh_window_title(self) -> None:
        config_name = os.path.basename(self.config_path) if self.config_path else "(none)"
        dirty_suffix = " *" if self._config_dirty else ""
        self.root.title(f"{self._base_title} - {config_name}{dirty_suffix}")

    def _refresh_action_enabled_state(self) -> None:
        has_config = self.report_config is not None
        has_logs = self.loaded is not None
        range_ok = self._range_validation_error is None
        select_range_enabled = has_config and has_logs
        actions_enabled = has_config and has_logs and range_ok

        self.select_range_btn.config(state=tk.NORMAL if select_range_enabled else tk.DISABLED)
        action_state = tk.NORMAL if actions_enabled else tk.DISABLED
        for btn in (self.plot_btn, self.save_trim_btn, self.pdf_btn):
            btn.config(state=action_state)
    def _update_config_summary(self) -> None:
        name = os.path.basename(self.config_path) if self.config_path else "(none)"
        self.config_summary_label.config(text=name)
        threshold_text = "n/a"
        if self.report_config is not None:
            threshold_text = str(self.report_config.pdf_sensor_fault_threshold_percent)
        self.pdf_sensor_fault_threshold_text.set(threshold_text)
        self.config_details_label.config(
            text=(
                f"Input time zone: {self.report_config.input_timezone_mode if self.report_config else 'n/a'}\n"
                f"Display time zone: {self.report_config.display_timezone if self.report_config else 'n/a'}\n"
                f"Y-axis series: {self.report_config.y_axis_series if self.report_config else 'n/a'}\n"
                f"PDF sensor fault threshold percent: {threshold_text}"
            )
        )

    def _apply_report_config_to_ui(self, config: ReportConfig) -> None:
        self.report_config = config
        self.report_config_loaded = True
        self._update_config_summary()
        self._refresh_status_from_current_range()
        self._refresh_window_title()

    def _show_startup_dialog(self) -> None:
        dialog = tk.Toplevel(self.root)
        dialog.title("Report Configuration")
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.protocol("WM_DELETE_WINDOW", self._on_exit_requested)
        tk.Label(dialog, text="Load or create a report configuration to continue.").pack(padx=12, pady=(12, 8))

        def _load() -> None:
            path = filedialog.askopenfilename(title="Load report config", filetypes=[("JSON", "*.json"), ("All Files", "*.*")])
            if not path:
                return
            try:
                cfg = load_report_config(path)
            except Exception as exc:
                messagebox.showerror("Config Error", f"Failed to load config: {exc}")
                return
            self.config_path = path
            self._config_dirty = False
            self._apply_report_config_to_ui(cfg)
            dialog.destroy()

        def _create() -> None:
            path = filedialog.asksaveasfilename(title="Create report config", defaultextension=".json", filetypes=[("JSON", "*.json")])
            if not path:
                return
            cfg = create_default_report_config()
            try:
                save_report_config(cfg, path)
                loaded_cfg = load_report_config(path)
            except Exception as exc:
                messagebox.showerror("Config Error", f"Failed to create config: {exc}")
                return
            self.config_path = path
            self._config_dirty = False
            self._apply_report_config_to_ui(loaded_cfg)
            dialog.destroy()

        button_frame = tk.Frame(dialog)
        button_frame.pack(padx=12, pady=(0, 12))
        tk.Button(button_frame, text="Load Config", command=_load).pack(side="left")
        tk.Button(button_frame, text="Create New Config", command=_create).pack(side="left", padx=6)
        tk.Button(button_frame, text="Exit", command=self.root.destroy).pack(side="left")

    def _require_report_config(self) -> ReportConfig:
        if self.report_config is None:
            raise ValueError("No configuration is loaded. Load or create a configuration before continuing.")
        validate_report_config(self.report_config)
        return self.report_config

    def _create_new_config(self) -> None:
        path = filedialog.asksaveasfilename(title="Create report config", defaultextension=".json", filetypes=[("JSON", "*.json")])
        if not path:
            return
        cfg = create_default_report_config()
        try:
            save_report_config(cfg, path)
            loaded_cfg = load_report_config(path)
        except Exception as exc:
            messagebox.showerror("Config Error", f"Failed to create config file at '{path}'. Check file permissions and settings.\n\n{exc}")
            return
        self.config_path = path
        self._config_dirty = False
        self._apply_report_config_to_ui(loaded_cfg)

    def load_config(self) -> None:
        if self._config_dirty and not messagebox.askyesno("Unsaved Changes", "Discard unsaved config changes and load another file?"):
            return
        path = filedialog.askopenfilename(title="Load report config", filetypes=[("JSON", "*.json"), ("All Files", "*.*")])
        if not path:
            return
        try:
            cfg = load_report_config(path)
        except Exception as exc:
            messagebox.showerror("Config Error", f"Failed to load config file '{path}'.\n\n{exc}")
            return
        self.config_path = path
        self._config_dirty = False
        self._apply_report_config_to_ui(cfg)

    def save_config(self) -> None:
        if not self.config_path:
            self.save_config_as()
            return
        try:
            cfg = self._require_report_config()
            save_report_config(cfg, self.config_path)
        except Exception as exc:
            messagebox.showerror("Config Error", f"Failed to save config file '{self.config_path}'. Check the highlighted settings and file permissions.\n\n{exc}")
            return
        self.report_config = cfg
        self._config_dirty = False
        self._apply_report_config_to_ui(cfg)

    def save_config_as(self) -> None:
        if self.report_config is None:
            messagebox.showerror("Config Error", "No configuration is loaded. Load or create a configuration first.")
            return
        path = filedialog.asksaveasfilename(
            title="Save report config as",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            cfg = self._require_report_config()
            save_report_config(cfg, path)
        except Exception as exc:
            messagebox.showerror("Config Error", f"Failed to save config file '{path}'.\n\n{exc}")
            return
        self.config_path = path
        self.report_config = cfg
        self._config_dirty = False
        self._apply_report_config_to_ui(cfg)

    def edit_options(self) -> None:
        if not self.report_config_loaded:
            messagebox.showerror("Options", "Load a config first.")
            return
        self._create_options_dialog()

    def _get_available_y_axis_series_choices(self, cfg: Optional[ReportConfig] = None) -> List[str]:
        defaults = ["cal_temp_c", "raw_temp_c", "raw_rtd_ohms"]
        if self.loaded is None:
            choices = defaults[:]
        else:
            numeric_cols = [c for c in self.loaded.dataframe.columns if pd.api.types.is_numeric_dtype(self.loaded.dataframe[c])]
            choices = [c for c in numeric_cols if c in defaults or c not in {"record_id", "schema_ver", "flags", "fault_status"}]
            for d in defaults:
                if d not in choices:
                    choices.append(d)
        if cfg is not None and cfg.y_axis_series and cfg.y_axis_series not in choices:
            choices.append(cfg.y_axis_series)
        return choices

    def _collect_report_config_from_options_dialog(self, vars_map: Dict[str, tk.Variable]) -> ReportConfig:
        allowed_input_modes = {"Log/source time", "UTC", "Local", "Same as display"}
        input_mode = str(vars_map["input_timezone_mode"].get()).strip()
        if input_mode not in allowed_input_modes:
            raise ValueError("Input time zone mode must be one of: Log/source time, UTC, Local, Same as display.")
        above_enabled = bool(vars_map["highlight_above_enabled"].get())
        below_enabled = bool(vars_map["highlight_below_enabled"].get())
        above_text = str(vars_map["highlight_above_value"].get()).strip()
        below_text = str(vars_map["highlight_below_value"].get()).strip()
        above_val = None if not above_enabled else float(above_text)
        below_val = None if not below_enabled else float(below_text)
        return ReportConfig(
            input_timezone_mode=input_mode,
            display_timezone=str(vars_map["display_timezone"].get()).strip(),
            y_axis_series=str(vars_map["y_axis_series"].get()).strip(),
            display_temperature_f=bool(vars_map["display_temperature_f"].get()),
            overlay_raw_temp_c=bool(vars_map["overlay_raw_temp_c"].get()),
            smooth_enabled=bool(vars_map["smooth_enabled"].get()),
            rolling_mean_divisor=int(str(vars_map["rolling_mean_divisor"].get()).strip()),
            downsample_enabled=bool(vars_map["downsample_enabled"].get()),
            max_plot_points=int(str(vars_map["max_plot_points"].get()).strip()),
            pdf_plot_dpi=int(str(vars_map["pdf_plot_dpi"].get()).strip()),
            vector_pdf=bool(vars_map["vector_pdf"].get()),
            show_minimum=bool(vars_map["show_minimum"].get()),
            show_maximum=bool(vars_map["show_maximum"].get()),
            show_average=bool(vars_map["show_average"].get()),
            show_std_band=bool(vars_map["show_std_band"].get()),
            highlight_outside_std=bool(vars_map["highlight_outside_std"].get()),
            highlight_mask_from_rolling_mean=bool(vars_map["highlight_mask_from_rolling_mean"].get()),
            highlight_above_enabled=above_enabled,
            highlight_above_value=above_val,
            highlight_below_enabled=below_enabled,
            highlight_below_value=below_val,
            pdf_sensor_fault_threshold_percent=float(str(vars_map["pdf_sensor_fault_threshold_percent"].get()).strip()),
        )

    def _set_widget_enabled(self, widget: tk.Widget, enabled: bool) -> None:
        widget.config(state=tk.NORMAL if enabled else tk.DISABLED)

    def _create_options_dialog(self) -> None:
        cfg = self._require_report_config()
        dialog = tk.Toplevel(self.root)
        dialog.title("Edit Options")
        body = tk.Frame(dialog, padx=10, pady=10)
        body.pack(fill="both", expand=True)
        vars_map: Dict[str, tk.Variable] = {
            "input_timezone_mode": tk.StringVar(value=cfg.input_timezone_mode),
            "display_timezone": tk.StringVar(value=cfg.display_timezone),
            "y_axis_series": tk.StringVar(value=cfg.y_axis_series),
            "display_temperature_f": tk.BooleanVar(value=cfg.display_temperature_f),
            "overlay_raw_temp_c": tk.BooleanVar(value=cfg.overlay_raw_temp_c),
            "smooth_enabled": tk.BooleanVar(value=cfg.smooth_enabled),
            "rolling_mean_divisor": tk.StringVar(value=str(cfg.rolling_mean_divisor)),
            "downsample_enabled": tk.BooleanVar(value=cfg.downsample_enabled),
            "max_plot_points": tk.StringVar(value=str(cfg.max_plot_points)),
            "show_minimum": tk.BooleanVar(value=cfg.show_minimum),
            "show_maximum": tk.BooleanVar(value=cfg.show_maximum),
            "show_average": tk.BooleanVar(value=cfg.show_average),
            "show_std_band": tk.BooleanVar(value=cfg.show_std_band),
            "highlight_outside_std": tk.BooleanVar(value=cfg.highlight_outside_std),
            "highlight_mask_from_rolling_mean": tk.BooleanVar(value=cfg.highlight_mask_from_rolling_mean),
            "highlight_above_enabled": tk.BooleanVar(value=cfg.highlight_above_enabled),
            "highlight_above_value": tk.StringVar(value="" if cfg.highlight_above_value is None else str(cfg.highlight_above_value)),
            "highlight_below_enabled": tk.BooleanVar(value=cfg.highlight_below_enabled),
            "highlight_below_value": tk.StringVar(value="" if cfg.highlight_below_value is None else str(cfg.highlight_below_value)),
            "pdf_plot_dpi": tk.StringVar(value=str(cfg.pdf_plot_dpi)),
            "vector_pdf": tk.BooleanVar(value=cfg.vector_pdf),
            "pdf_sensor_fault_threshold_percent": tk.StringVar(value=str(cfg.pdf_sensor_fault_threshold_percent)),
        }
        r = 0
        sec = tk.LabelFrame(body, text="Time and series", padx=8, pady=6); sec.grid(row=r, column=0, sticky="ew"); r += 1
        sec.columnconfigure(1, weight=1)
        ttk.Combobox(sec, textvariable=vars_map["input_timezone_mode"], state="readonly", values=["Log/source time", "UTC", "Local", "Same as display"], width=26).grid(row=0, column=1, sticky="w")
        tk.Label(sec, text=_human_config_label("input_timezone_mode")).grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Combobox(sec, textvariable=vars_map["display_timezone"], values=["Local", "UTC", "America/Chicago"], width=26).grid(row=1, column=1, sticky="w", pady=(4, 0))
        tk.Label(sec, text=_human_config_label("display_timezone")).grid(row=1, column=0, sticky="w", padx=(0, 8), pady=(4, 0))
        ttk.Combobox(sec, textvariable=vars_map["y_axis_series"], values=self._get_available_y_axis_series_choices(cfg), width=26).grid(row=2, column=1, sticky="w", pady=(4, 0))
        tk.Label(sec, text=_human_config_label("y_axis_series")).grid(row=2, column=0, sticky="w", padx=(0, 8), pady=(4, 0))
        sec2 = tk.LabelFrame(body, text="Temperature and plotting", padx=8, pady=6); sec2.grid(row=r, column=0, sticky="ew", pady=(6, 0)); r += 1
        sec2.columnconfigure(0, weight=1)
        sec2.columnconfigure(1, weight=0)
        for i, key in enumerate(["display_temperature_f", "overlay_raw_temp_c", "smooth_enabled", "downsample_enabled"]):
            tk.Checkbutton(sec2, text=_human_config_label(key), variable=vars_map[key]).grid(row=i, column=0, sticky="w")
        tk.Label(sec2, text=_human_config_label("rolling_mean_divisor")).grid(row=0, column=1, sticky="w", padx=(16, 6))
        tk.Entry(sec2, textvariable=vars_map["rolling_mean_divisor"], width=10).grid(row=0, column=2, sticky="w")
        tk.Label(sec2, text=_human_config_label("max_plot_points")).grid(row=1, column=1, sticky="w", padx=(16, 6))
        tk.Entry(sec2, textvariable=vars_map["max_plot_points"], width=10).grid(row=1, column=2, sticky="w")
        sec3 = tk.LabelFrame(body, text="Statistics", padx=8, pady=6); sec3.grid(row=r, column=0, sticky="ew", pady=(6, 0)); r += 1
        stats_keys = ["show_minimum", "show_maximum", "show_average", "show_std_band"]
        for i, key in enumerate(stats_keys):
            tk.Checkbutton(sec3, text=_human_config_label(key), variable=vars_map[key]).grid(row=i // 2, column=i % 2, sticky="w", padx=(0, 12))
        sec4 = tk.LabelFrame(body, text="Out of bounds highlighting", padx=8, pady=6); sec4.grid(row=r, column=0, sticky="ew", pady=(6, 0)); r += 1
        tk.Checkbutton(sec4, text=_human_config_label("highlight_outside_std"), variable=vars_map["highlight_outside_std"]).grid(row=0, column=0, sticky="w", columnspan=2)
        tk.Checkbutton(sec4, text=_human_config_label("highlight_mask_from_rolling_mean"), variable=vars_map["highlight_mask_from_rolling_mean"]).grid(row=1, column=0, sticky="w", columnspan=2)
        e_above = tk.Entry(sec4, textvariable=vars_map["highlight_above_value"], width=10); e_below = tk.Entry(sec4, textvariable=vars_map["highlight_below_value"], width=10)
        tk.Checkbutton(sec4, text=_human_config_label("highlight_above_enabled"), variable=vars_map["highlight_above_enabled"]).grid(row=2, column=0, sticky="w")
        tk.Label(sec4, text=_human_config_label("highlight_above_value")).grid(row=2, column=1, sticky="e", padx=(6, 4)); e_above.grid(row=2, column=2, sticky="w")
        tk.Checkbutton(sec4, text=_human_config_label("highlight_below_enabled"), variable=vars_map["highlight_below_enabled"]).grid(row=3, column=0, sticky="w")
        tk.Label(sec4, text=_human_config_label("highlight_below_value")).grid(row=3, column=1, sticky="e", padx=(6, 4)); e_below.grid(row=3, column=2, sticky="w")
        def _sync_limits(*_args: object) -> None:
            self._set_widget_enabled(e_above, bool(vars_map["highlight_above_enabled"].get()))
            self._set_widget_enabled(e_below, bool(vars_map["highlight_below_enabled"].get()))
        vars_map["highlight_above_enabled"].trace_add("write", _sync_limits); vars_map["highlight_below_enabled"].trace_add("write", _sync_limits); _sync_limits()
        sec5 = tk.LabelFrame(body, text="PDF and output", padx=8, pady=6); sec5.grid(row=r, column=0, sticky="ew", pady=(6, 0)); r += 1
        tk.Label(sec5, text=_human_config_label("pdf_plot_dpi")).grid(row=0, column=0, sticky="w"); tk.Entry(sec5, textvariable=vars_map["pdf_plot_dpi"], width=10).grid(row=0, column=1, sticky="w")
        tk.Checkbutton(sec5, text=_human_config_label("vector_pdf"), variable=vars_map["vector_pdf"]).grid(row=0, column=2, sticky="w", padx=(12, 0))
        sec6 = tk.LabelFrame(body, text="Data quality", padx=8, pady=6); sec6.grid(row=r, column=0, sticky="ew", pady=(6, 0)); r += 1
        tk.Label(sec6, text=_human_config_label("pdf_sensor_fault_threshold_percent")).grid(row=0, column=0, sticky="w")
        tk.Entry(sec6, textvariable=vars_map["pdf_sensor_fault_threshold_percent"], width=10).grid(row=0, column=1, sticky="w", padx=(6, 0))
        tk.Label(sec6, text="Sensor read failures below this percentage are omitted from the PDF but remain visible in the status table.", justify="left", wraplength=540).grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))
        def _apply_options(close_after: bool) -> None:
            try:
                new_cfg = self._collect_report_config_from_options_dialog(vars_map)
                validate_report_config(new_cfg)
            except Exception as exc:
                messagebox.showerror("Options Validation", str(exc))
                return
            self._config_dirty = True
            self._apply_report_config_to_ui(new_cfg)
        button_row = tk.Frame(body); button_row.grid(row=r, column=0, sticky="e", pady=(10, 0))
        tk.Button(button_row, text="Apply", command=lambda: _apply_options(False)).pack(side="left", padx=(0, 6))
        tk.Button(button_row, text="Close", command=dialog.destroy).pack(side="left")

    def _create_status_flags_section(self, parent: tk.Widget, row: int, column: int = 0) -> None:
        status_frame = tk.LabelFrame(parent, text="Data Quality / Status Flags", padx=8, pady=6)
        status_frame.grid(row=row, column=column, sticky="ew", pady=(8, 0))
        status_frame.columnconfigure(0, weight=1)

        threshold_row = tk.Frame(status_frame)
        threshold_row.grid(row=0, column=0, sticky="w")
        tk.Label(threshold_row, text="PDF sensor fault threshold percent:").grid(row=0, column=0, sticky="w")
        tk.Label(threshold_row, textvariable=self.pdf_sensor_fault_threshold_text).grid(row=0, column=1, sticky="w", padx=(6, 0))

        tk.Label(status_frame, text="Sensor read failures below this threshold are hidden from the PDF but shown here.", justify="left", wraplength=520).grid(row=1, column=0, sticky="w")

        table_frame = tk.Frame(status_frame)
        table_frame.grid(row=2, column=0, sticky="ew", pady=(4, 0))
        table_frame.columnconfigure(0, weight=1)

        columns = ("flag", "count", "percent")
        self.status_flags_tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=5)
        self.status_flags_tree.heading("flag", text="Flag", anchor="w")
        self.status_flags_tree.heading("count", text="Count", anchor="e")
        self.status_flags_tree.heading("percent", text="Percent", anchor="e")
        self.status_flags_tree.column("flag", width=130, minwidth=110, stretch=False, anchor="w")
        self.status_flags_tree.column("count", width=80, minwidth=70, stretch=False, anchor="e")
        self.status_flags_tree.column("percent", width=90, minwidth=80, stretch=False, anchor="e")
        self.status_flags_tree.grid(row=0, column=0, sticky="ew")

        scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.status_flags_tree.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.status_flags_tree.configure(yscrollcommand=scrollbar.set)

        self.show_zero_status_flags = tk.BooleanVar(value=False)
        tk.Checkbutton(
            status_frame,
            text="Show rows with zero count",
            variable=self.show_zero_status_flags,
            command=self._refresh_status_from_current_range,
        ).grid(row=3, column=0, sticky="w", pady=(4, 0))
        self.status_flags_tree.bind("<<TreeviewSelect>>", self._on_status_flag_selected)
        self.status_summary_label = tk.Label(status_frame, text="Status: n/a", justify="left", anchor="w")
        self.status_summary_label.grid(row=4, column=0, sticky="w", pady=(4, 0))
        self.status_meaning_label = tk.Label(status_frame, text="Meaning: n/a", justify="left", anchor="w")
        self.status_meaning_label.grid(row=5, column=0, sticky="w")

    def _refresh_status_flags_table(self, df: Optional[pd.DataFrame]) -> None:
        rows = _build_status_flag_rows(df) if df is not None else []
        display_rows = _filter_status_flag_rows_for_display(rows, show_zero_count=self.show_zero_status_flags.get())
        self.status_flags_tree.delete(*self.status_flags_tree.get_children())

        if not rows:
            self.status_flags_tree.insert("", tk.END, values=("(no data)", "", ""))
            self.status_summary_label.config(text="Status: No flag data loaded.")
            self.status_meaning_label.config(text="Meaning: n/a")
            return

        if not display_rows:
            self.status_flags_tree.insert("", tk.END, values=("NONE", 0, "0.00%"))
            self.status_summary_label.config(text="Status: No flags detected in current trimmed range.")
            self.status_meaning_label.config(text="Meaning: No flags detected in current dataset.")
            return

        for row in display_rows:
            self.status_flags_tree.insert("", tk.END, values=_status_table_values(row))
        top = max(display_rows, key=lambda r: r.count)
        self.status_summary_label.config(text=f"Status: {top.short_name} {top.count} rows, {_format_percent_for_status_table(top.percent)}.")
        self.status_meaning_label.config(text=f"Meaning: {top.meaning or top.label}")

    def _on_status_flag_selected(self, _event: Optional[tk.Event] = None) -> None:
        selected = self.status_flags_tree.selection()
        if not selected:
            return
        vals = self.status_flags_tree.item(selected[0], "values")
        if not vals:
            return
        key = str(vals[0])
        self.status_meaning_label.config(text=f"Meaning: {_status_flag_meaning_for_key(key)}")

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
        cfg = self._require_report_config()
        tz_value = cfg.display_timezone
        display_tz = _resolve_display_tz(tz_value)
        if display_tz is None:
            raise ValueError(f"Invalid time zone: {tz_value}")
        label = _format_display_tz_label(display_tz, start_dt, end_dt)
        return DisplayTimeConfig(display_tz=display_tz, display_tz_label=label)

    def _apply_selected_time_range(
        self,
        start_text: str,
        end_text: str,
        *,
        mark_manual: bool,
    ) -> None:
        """Apply start/end text to the main form through one explicit update path."""
        self.start_time_text.set(start_text)
        self.end_time_text.set(end_text)
        self._has_manual_time_range = mark_manual
        self._refresh_status_from_current_range()
        # Ensure main-window Entry widgets repaint before/around dialog teardown.
        self.start_time_entry.update_idletasks()
        self.end_time_entry.update_idletasks()
        self.root.update_idletasks()

    def _autofill_time_range(self, *, force: bool = False) -> None:
        if self._has_manual_time_range and not force:
            return
        config = self._require_report_config()
        if self.loaded is None:
            return
        if self.loaded.time_column == "__x":
            self._apply_selected_time_range("", "", mark_manual=False)
            if hasattr(self, "status_summary_label"):
                self.status_summary_label.config(
                    text="Status: record_id-only log loaded; timestamp autofill is unavailable."
                )
            return
        selected_start_utc, selected_end_utc = _get_loaded_time_bounds_utc(self.loaded)
        display_tz = _resolve_display_tz(config.display_timezone)
        if display_tz is None:
            raise ValueError(
                f"Display timezone {config.display_timezone!r} could not be resolved. "
                "Open Edit Options and choose a valid timezone."
            )
        start_text, end_text = _format_range_utc_for_input_fields(
            selected_start_utc,
            selected_end_utc,
            config,
            self.loaded,
            display_tz,
        )
        _validate_current_range_with_config(config, self.loaded, start_text, end_text)
        self._apply_selected_time_range(start_text, end_text, mark_manual=False)

    def _detect_aggregated(self, time_series: pd.Series) -> bool:
        if time_series.empty:
            return False
        diffs = pd.to_datetime(time_series).sort_values().diff().dropna()
        if diffs.empty:
            return False
        median_seconds = diffs.dt.total_seconds().median()
        return bool(median_seconds >= 60)

    def _validate_loaded_config_for_action(self) -> ReportConfig:
        cfg = self._require_report_config()
        if not self.loaded:
            raise ValueError("No data loaded.")
        if cfg.y_axis_series not in self.loaded.dataframe.columns:
            raise ValueError(
                f"Configured y-axis series {cfg.y_axis_series} is not present in the selected log files. "
                "Open Edit Options and choose a series that exists in the loaded logs."
            )
        display_tz = _resolve_display_tz(cfg.display_timezone)
        if display_tz is None:
            raise ValueError(f"Configured display_timezone is invalid: {cfg.display_timezone}")
        input_tz, _ = _resolve_input_timezone(cfg.input_timezone_mode, display_tz, self.loaded.tzinfo)
        if input_tz is None:
            raise ValueError(f"Configured input_timezone_mode cannot be resolved: {cfg.input_timezone_mode}")
        return cfg

    def _on_exit_requested(self) -> None:
        if self._config_dirty and not messagebox.askyesno("Unsaved Changes", "Discard unsaved config changes and exit?"):
            return
        self.root.destroy()

    def _dataset_is_calibrated(self) -> bool:
        if not self.loaded:
            return False
        df = self.loaded.dataframe
        if "cal_temp_c" not in df.columns:
            return False
        fraction = _cal_valid_fraction(df)
        if fraction is not None:
            return fraction > 0.0
        return self.loaded.audit_summary.calibration_status == _CAL_STATUS_CALIBRATED

    def _load_paths(self, file_paths: List[str]) -> None:
        if not file_paths:
            return
        self.selected_files = list(file_paths)
        try:
            self.loaded = _load_log_files(self.selected_files)
        except Exception as exc:
            self.loaded = None
            messagebox.showerror("Load Error", str(exc))
            return

        file_list = "\n".join([os.path.basename(p) for p in self.selected_files[:6]])
        if len(self.selected_files) > 6:
            file_list += f"\n… ({len(self.selected_files)} total)"
        self.file_label.config(text=file_list)

        self._refresh_status_from_current_range()
        self._warned_aggregated = False
        self._has_manual_time_range = False
        self._autofill_time_range(force=True)

        if self.loaded.dropped_no_time_rows:
            messagebox.showinfo(
                "Info",
                f"Dropped {self.loaded.dropped_no_time_rows} row(s) without usable timestamps.",
            )

    def select_files(self) -> None:
        file_paths = filedialog.askopenfilenames(
            title="Select PT100 log file(s)",
            filetypes=[("PTLOG/CSV", "*.ptlog *.csv"), ("PTLOG", "*.ptlog"), ("CSV", "*.csv"), ("All Files", "*.*")],
        )
        self._load_paths(list(file_paths))

    def select_folder(self) -> None:
        folder = filedialog.askdirectory(title="Select folder containing PT100 log files")
        if not folder:
            return
        file_paths = glob.glob(os.path.join(folder, "*.ptlog")) + glob.glob(os.path.join(folder, "*.csv"))
        file_paths = _dedupe_folder_file_paths(file_paths)
        if not file_paths:
            messagebox.showerror("Load Error", "No .ptlog or .csv files found in the selected folder.")
            return
        self._load_paths(file_paths)

    def _on_range_text_changed(self, *_args: object) -> None:
        if not self.root.winfo_exists():
            return
        self._refresh_status_from_current_range()

    def _status_summary_for_rows(self, rows: Sequence[FlagSummaryRow], *, threshold_percent: float) -> str:
        nonzero_rows = [row for row in rows if row.count > 0]
        if not nonzero_rows:
            return "Status: No nonzero status flags in selected range."
        top = max(nonzero_rows, key=lambda r: r.count)
        summary = f"Status: {top.short_name} {top.count} rows, {_format_percent_for_status_table(top.percent)}."
        if top.short_name == "SENSOR_FAULT":
            relation = "Below" if top.percent < threshold_percent else "Included in PDF at/above"
            summary = f"{summary} {relation} threshold {_format_percent_for_status_table(threshold_percent)}."
        return summary

    def _refresh_status_from_current_range(self) -> None:
        self._range_validation_error = None
        if not self.loaded:
            self._refresh_status_flags_table(None)
            self._refresh_action_enabled_state()
            return
        if not self.report_config_loaded:
            self._refresh_status_flags_table(self.loaded.dataframe)
            self._refresh_action_enabled_state()
            return
        try:
            cfg = self._validate_loaded_config_for_action()
            trimmed, *_ = self._get_trimmed_df(cfg)
            self._refresh_status_flags_table(trimmed)
            rows = _build_status_flag_rows(trimmed)
            self.status_summary_label.config(
                text=self._status_summary_for_rows(rows, threshold_percent=float(cfg.pdf_sensor_fault_threshold_percent))
            )
        except Exception as exc:
            self._range_validation_error = str(exc)
            self._refresh_status_flags_table(None)
            self.status_summary_label.config(text="Status: Range is invalid. Fix Start and End before plotting or exporting.")
            self.status_meaning_label.config(text=f"Meaning: {exc}")
        self._refresh_action_enabled_state()

    def open_range_selector(self) -> None:
        try:
            cfg = self._validate_loaded_config_for_action()
        except Exception as exc:
            messagebox.showerror("Range Selector", str(exc))
            return

        df = self.loaded.dataframe
        time_column = self.loaded.time_column
        y_name = cfg.y_axis_series
        y_series = pd.to_numeric(df.get(y_name, pd.Series(dtype=float)), errors="coerce")

        display_tz = _resolve_display_tz(cfg.display_timezone)
        if time_column == "__time":
            if display_tz is None:
                messagebox.showerror("Range Selector", f"Invalid time zone: {cfg.display_timezone}")
                return
            display_series = _convert_time_series_to_display_tz(
                df[time_column],
                display_tz=display_tz,
                time_source=self.loaded.time_source,
            )
            x_plot = display_series.to_numpy()
            x_slider = mdates.date2num(display_series)
            present_minutes = display_series.dt.floor("min")
            present_minutes_index = pd.DatetimeIndex(present_minutes.unique()).sort_values()
            start_dt = pd.to_datetime(display_series).min()
            end_dt = pd.to_datetime(display_series).max()
            display_config = self._get_display_time_config(start_dt=start_dt, end_dt=end_dt)
            x_label = f"{_human_time_label(time_column)} ({display_config.display_tz_label})"
        else:
            display_series = None
            x_slider = pd.to_numeric(df[time_column], errors="coerce").to_numpy(dtype=float, copy=False)
            x_plot = x_slider
            present_minutes = None
            present_minutes_index = None
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
        ax.set_ylabel(_human_series_label(y_name, temp_unit="F" if cfg.display_temperature_f else "C"))
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
                start_candidate, end_candidate = _parse_range_text_to_utc(
                    self.start_time_text.get(),
                    self.end_time_text.get(),
                    cfg,
                    self.loaded,
                    display_tz,
                )
                if start_candidate and end_candidate:
                    start_ts = pd.Timestamp(start_candidate).tz_convert(display_tz)
                    end_ts = pd.Timestamp(end_candidate).tz_convert(display_tz)
                    slider_min = float(np.min(x_slider))
                    slider_max = float(np.max(x_slider))
                    start_num = max(slider_min, min(slider_max, mdates.date2num(start_ts)))
                    end_num = max(slider_min, min(slider_max, mdates.date2num(end_ts)))
                    if start_num > end_num:
                        start_num, end_num = end_num, start_num
                    slider.set_val((start_num, end_num))
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

        label_frame = tk.Frame(selector_window, padx=8, pady=6)
        label_frame.pack(fill="x")
        tk.Label(label_frame, text="Start:").grid(row=0, column=0, sticky="w")
        tk.Label(label_frame, textvariable=start_label_var).grid(row=0, column=1, sticky="w")
        tk.Label(label_frame, text="End:").grid(row=1, column=0, sticky="w")
        tk.Label(label_frame, textvariable=end_label_var).grid(row=1, column=1, sticky="w")
        _, input_mode_label = _resolve_input_timezone(
            cfg.input_timezone_mode,
            display_tz,
            self.loaded.tzinfo,
        )
        tk.Label(
            label_frame,
            text=f"Applies using: {input_mode_label}",
        ).grid(row=2, column=0, columnspan=2, sticky="w", pady=(4, 0))

        selected_start_utc: Optional[pd.Timestamp] = None
        selected_end_utc: Optional[pd.Timestamp] = None

        def _set_selected_utc_from_slider(val: Tuple[float, float]) -> None:
            nonlocal selected_start_utc, selected_end_utc
            if time_column != "__time" or display_tz is None:
                selected_start_utc = None
                selected_end_utc = None
                return
            start_dt = mdates.num2date(val[0], tz=display_tz)
            end_dt = mdates.num2date(val[1], tz=display_tz)
            selected_start_utc = pd.Timestamp(start_dt).tz_convert(datetime.timezone.utc)
            selected_end_utc = pd.Timestamp(end_dt).tz_convert(datetime.timezone.utc)

        _set_selected_utc_from_slider(slider.val)

        def _on_slider_change(val: Tuple[float, float]) -> None:
            if time_column == "__time":
                x0_dt = mdates.num2date(val[0], tz=display_tz)
                x1_dt = mdates.num2date(val[1], tz=display_tz)
                ax.set_xlim(x0_dt, x1_dt)
            else:
                ax.set_xlim(val)
            _update_labels(val)
            _set_selected_utc_from_slider(val)
            slider.valtext.set_text("")
            canvas.draw_idle()

        slider.on_changed(_on_slider_change)

        def _apply_range() -> None:
            if time_column != "__time" or present_minutes is None:
                self._apply_selected_time_range("", "", mark_manual=False)
                messagebox.showinfo(
                    "Range Selection",
                    "This log uses record_id; time trimming is unavailable. The slider is for visual preview only.",
                )
                selector_window.destroy()
                return
            if selected_start_utc is None or selected_end_utc is None:
                messagebox.showerror("Range Selector", "Unable to resolve selected time range.")
                return
            nearest_start_utc = present_minutes_index[present_minutes_index.get_indexer([selected_start_utc], method="nearest")[0]]
            nearest_end_utc = present_minutes_index[present_minutes_index.get_indexer([selected_end_utc], method="nearest")[0]]
            start_text, end_text = _format_range_utc_for_input_fields(
                nearest_start_utc,
                nearest_end_utc,
                cfg,
                self.loaded,
                display_tz,
            )
            _validate_current_range_with_config(cfg, self.loaded, start_text, end_text)
            self._apply_selected_time_range(start_text, end_text, mark_manual=True)
            selector_window.destroy()

        action_frame = tk.Frame(selector_window, padx=8, pady=6)
        action_frame.pack(fill="x")
        def _cancel_range() -> None:
            selector_window.destroy()

        selector_window.protocol("WM_DELETE_WINDOW", _cancel_range)

        tk.Button(action_frame, text="Apply", command=_apply_range).pack(side="left")
        tk.Button(action_frame, text="Cancel", command=_cancel_range).pack(side="left", padx=6)

    def _get_trimmed_df(self, cfg: ReportConfig) -> Tuple[pd.DataFrame, str, str, str, str, DisplayTimeConfig, Optional[pd.Series]]:
        if not self.loaded:
            raise ValueError("No data loaded.")
        display_series = None
        display_tz = None
        time_series_utc = None
        if self.loaded.time_column == "__time":
            display_tz = _resolve_display_tz(cfg.display_timezone)
            if display_tz is None:
                raise ValueError(f"Configured display_timezone is invalid: {cfg.display_timezone}")
            time_series_utc = _canonicalize_time_to_utc(
                self.loaded.dataframe[self.loaded.time_column],
                time_source=self.loaded.time_source,
                source_tz=self.loaded.tzinfo,
            )
            display_series = _convert_time_series_to_display_tz(
                time_series_utc,
                display_tz=display_tz,
                time_source="epoch_utc",
            )
        trimmed, start_label, end_label, summary, selected_label = _validate_and_trim_by_minute(
            df=self.loaded.dataframe,
            time_column=self.loaded.time_column,
            start_text=self.start_time_text.get(),
            end_text=self.end_time_text.get(),
            time_series_utc=time_series_utc,
            input_timezone_mode=cfg.input_timezone_mode,
            display_tz=display_tz,
            source_tz=self.loaded.tzinfo,
        )
        if display_series is not None:
            display_series = display_series.loc[trimmed.index]
            display_config = self._get_display_time_config(
                start_dt=pd.to_datetime(display_series).min(),
                end_dt=pd.to_datetime(display_series).max(),
            )
        else:
            display_config = DisplayTimeConfig(display_tz=None, display_tz_label="n/a")

        return trimmed, start_label, end_label, summary, selected_label, display_config, display_series

    def save_trimmed_csv(self) -> None:
        try:
            cfg = self._validate_loaded_config_for_action()
            trimmed, start_label, end_label, summary, _selected_label, _display_config, _display_series = self._get_trimmed_df(cfg)
            _validate_series_for_configured_output(trimmed, cfg, require_overlay_check=False)
            operator_trimmed = _operator_trimmed_csv_dataframe(trimmed)
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
            operator_trimmed.to_csv(save_path, index=False)
        except Exception as exc:
            messagebox.showerror("CSV Save Error", f"Failed to save CSV file '{save_path}'. Check file permissions and available disk space.\n\n{exc}")
            return

        messagebox.showinfo("Saved", f"Saved trimmed CSV:\n{save_path}\n\n{summary}")

    def plot(self) -> None:
        try:
            cfg = self._validate_loaded_config_for_action()
            df, start_label, end_label, _summary, selected_label, display_config, display_series = self._get_trimmed_df(cfg)
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        requested_y_name = cfg.y_axis_series
        y_name_effective = requested_y_name

        try:
            _validate_series_for_configured_output(df, cfg, require_overlay_check=True)
        except Exception as exc:
            messagebox.showerror("Plot Error", str(exc))
            return

        overlay_raw = cfg.overlay_raw_temp_c

        smooth = cfg.smooth_enabled
        if display_series is not None and self._detect_aggregated(display_series):
            if smooth and not self._warned_aggregated:
                messagebox.showinfo("Smoothing Disabled", "Input appears aggregated; smoothing disabled.")
                self._warned_aggregated = True
            smooth = False

        nodes = _node_ids_from_df(df)
        utc_times = pd.to_datetime(df[self.loaded.time_column], utc=True, errors="coerce")
        display_start, display_end, timezone_label = _format_time_range_for_display(
            utc_times.min(),
            utc_times.max(),
            display_config.display_tz,
        )
        plot_title = _human_series_label(y_name_effective, temp_unit="F" if cfg.display_temperature_f else "C")
        node_label = nodes if nodes != "n/a" else "n/a"
        suptitle = _build_report_subtitle(node_label, display_start, display_end, timezone_label)

        try:
            options = build_plot_options_from_config(cfg, self.loaded, df, display_config)
            options.smooth = smooth
            options.overlay_raw_temp = overlay_raw
        except Exception as exc:
            messagebox.showerror("Plot Error", str(exc))
            return

        fig, _plot_points, _total_points = _build_figure(
            df=df,
            time_column=self.loaded.time_column,
            y_name=y_name_effective,
            plot_title=plot_title,
            suptitle=suptitle,
            options=options,
        )
        plt.show()

    def export_pdf(self) -> None:
        try:
            cfg = self._validate_loaded_config_for_action()
            df, start_label, end_label, summary, selected_label, display_config, display_series = self._get_trimmed_df(cfg)
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        requested_y_name = cfg.y_axis_series
        y_name_effective = requested_y_name

        warning_text: Optional[str] = self.loaded.audit_summary.calibration_warning

        try:
            _validate_series_for_configured_output(df, cfg, require_overlay_check=True)
        except Exception as exc:
            messagebox.showerror("PDF Error", str(exc))
            return
        node_ids = _node_id_values(df)
        if len(node_ids) > 1:
            messagebox.showerror(
                "PDF Error",
                "Multiple node IDs are present in the selected range. "
                "Generate one report per node or add explicit multi-node report support. "
                f"Found {len(node_ids)} node IDs: {', '.join(node_ids[:8])}.",
            )
            return

        overlay_raw = cfg.overlay_raw_temp_c

        smooth = cfg.smooth_enabled
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

        use_vector_pdf = bool(cfg.vector_pdf)

        tmp_dir: Optional[str] = None
        fig_png_path: Optional[str] = None
        if not use_vector_pdf:
            tmp_dir = tempfile.mkdtemp(prefix="pt100_report_")
            fig_png_path = os.path.join(tmp_dir, "plot.png")

        nodes = _node_ids_from_df(df)
        title = "PT100 Temperature Log Report"
        utc_times = pd.to_datetime(df[self.loaded.time_column], utc=True, errors="coerce")
        display_start, display_end, timezone_label = _format_time_range_for_display(
            utc_times.min(),
            utc_times.max(),
            display_config.display_tz,
        )
        node_label = nodes if nodes != "n/a" else "n/a"
        subtitle = None

        temp_unit = "F" if cfg.display_temperature_f else "C"
        plot_title = _human_series_label(y_name_effective, temp_unit=temp_unit)

        try:
            options = build_plot_options_from_config(cfg, self.loaded, df, display_config)
            options.smooth = smooth
            options.overlay_raw_temp = overlay_raw
        except Exception as exc:
            messagebox.showerror("PDF Export Error", f"Failed to export PDF report to '{save_path}'. Check file permissions, disk space, and PDF settings.\n\n{exc}")
            return

        fig, plot_points, total_points = _build_figure(
            df=df,
            time_column=self.loaded.time_column,
            y_name=y_name_effective,
            plot_title=plot_title,
            suptitle=None,
            options=options,
        )

        y_series_c = pd.to_numeric(df.get(y_name_effective, pd.Series(dtype=float)), errors="coerce")
        y_series_disp = (
            _convert_temperature_series(y_series_c, temp_unit)
            if y_name_effective in ("cal_temp_c", "raw_temp_c")
            else y_series_c
        )
        stats_series, stats_notes = _prepare_series_for_statistics(df, y_series_disp, y_name_effective)
        if stats_series.empty:
            messagebox.showerror(
                "PDF Export Error",
                "Cannot compute summary statistics after filtering invalid rows "
                f"for '{y_name_effective}'. Please review flags/calibration and selected range."
            )
            plt.close(fig)
            return
        stats = _compute_basic_stats(stats_series)
        sensor_fault_threshold_percent = _parse_nonnegative_float(
            str(cfg.pdf_sensor_fault_threshold_percent),
            "PDF sensor read failure threshold percent",
            0.10,
        )
        flags_summary = _format_flags_summary(
            df,
            display_tz=display_config.display_tz,
            time_source=self.loaded.time_source,
            sensor_fault_threshold_percent=sensor_fault_threshold_percent,
        )

        series_label = _human_series_label(y_name_effective, temp_unit=temp_unit)
        if display_series is not None and not display_series.empty:
            start_ts = pd.to_datetime(display_series).min()
            end_ts = pd.to_datetime(display_series).max()
            span_str = _format_span_label(start_ts, end_ts)
            data_range_value = f"{display_start} → {display_end} (span {span_str})"
        else:
            data_range_value = f"{display_start} → {display_end} ({len(df):,} rows)"
        cal_points = _segment_header_value(self.loaded.audit_summary.segments, "cal_points_count")
        cal_last_utc = _segment_header_value(self.loaded.audit_summary.segments, "cal_last_utc")
        cal_due_utc = _segment_header_value(self.loaded.audit_summary.segments, "cal_due_utc")
        cal_last_local = _format_performed_utc_to_local_date(cal_last_utc, display_config.display_tz)
        cal_due_local = _format_due_utc_midnight_to_date_preserve_day(cal_due_utc)
        cal_method = _segment_header_value(self.loaded.audit_summary.segments, "cal_method", default="<unset>")

        overlays = []
        if y_name_effective in ("cal_temp_c", "raw_temp_c"):
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
            ["Metric", "Result"],
            ["Node ID", node_label],
            ["Reporting time zone", display_config.display_tz_label],
            ["Measurement window", data_range_value],
            ["Plotted series", series_label],
            ["Summary (min/mean/max/std dev)",
            f"{stats['min']} / {stats['avg']} / {stats['max']} / {stats['std']}"],
        ]
        if flags_summary != "n/a":
            summary_rows.append(["Data quality flags", flags_summary])
        fault_status_summary = _format_fault_status_summary(
            df,
            include_zero_fault_status=("Sensor fault:" in flags_summary),
            sensor_fault_threshold_percent=sensor_fault_threshold_percent,
        )
        if fault_status_summary != "n/a":
            summary_rows.append(["Sensor fault detail", fault_status_summary])

        calibration_rows = [
            ["Calibration item", "Recorded value"],
            ["Calibration applied to records", _format_applied_records_label(df)],
            ["Calibration points used", cal_points],
            ["Calibration performed (local date)", cal_last_local],
            ["Calibration due (date)", cal_due_local],
            ["Calibration method (operator notes)", cal_method],
        ]


        # Intentionally omit any "Overlays" row from the PDF summary. The report
        # should stay focused on time span, series, calibration, statistics, and
        # any flag issues.

        try:
            if use_vector_pdf:
                _export_pdf_report_vector(
                    save_path=save_path,
                    fig=fig,
                    source_files=self.loaded.source_files,
                    summary_rows=summary_rows,
                    calibration_rows=calibration_rows,
                    title=title,
                    subtitle=subtitle,
                    warning_text=warning_text,
                )
            else:
                assert fig_png_path is not None
                fig.savefig(fig_png_path, dpi=int(cfg.pdf_plot_dpi))
                _export_pdf_report(
                    save_path=save_path,
                    fig_png_path=fig_png_path,
                    source_files=self.loaded.source_files,
                    summary_rows=summary_rows,
                    calibration_rows=calibration_rows,
                    title=title,
                    subtitle=subtitle,
                    warning_text=warning_text,
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
