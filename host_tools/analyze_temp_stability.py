#!/usr/bin/env python3
"""Analyze PTLOG temperature drift and post-settle stability."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import signal
from scipy.stats import linregress


HEADER_PREFIX = "#"
SCHEMA_PREFIX = "schema_ver,"
SECONDS_PER_MINUTE = 60.0



def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments.

    Args:
        None.

    Returns:
        argparse.Namespace: Parsed command-line arguments.
    """
    argument_parser = argparse.ArgumentParser(
        description=(
            "Analyze PTLOG drift on a trailing time window and characterize "
            "temperature stability after the drift threshold is met."
        )
    )
    argument_parser.add_argument(
        "input_path",
        type=Path,
        help="Path to the .ptlog or CSV-style log file.",
    )
    argument_parser.add_argument(
        "--window-sec",
        type=float,
        required=True,
        help="Trailing regression window length in seconds used to compute drift.",
    )
    argument_parser.add_argument(
        "--drift-threshold",
        type=float,
        required=True,
        help="Absolute drift threshold in deg C/min used to declare stability.",
    )
    argument_parser.add_argument(
        "--hold-sec",
        type=float,
        default=0.0,
        help=(
            "Optional dwell time in seconds for which the rolling drift must stay "
            "inside the threshold before stability is declared."
        ),
    )
    argument_parser.add_argument(
        "--temp-column",
        choices=["auto", "cal_temp_c", "raw_temp_c"],
        default="auto",
        help="Temperature column to analyze. Default prefers calibrated temperature.",
    )
    argument_parser.add_argument(
        "--target-temp",
        type=float,
        default=None,
        help=(
            "Optional target/setpoint temperature in deg C. When provided with "
            "--setpoint-band, stable segments must also remain inside the band."
        ),
    )
    argument_parser.add_argument(
        "--setpoint-band",
        type=float,
        default=None,
        help=(
            "Optional half-band around --target-temp in deg C. Example: "
            "--target-temp 120 --setpoint-band 0.25 requires temperature to "
            "remain within 119.75 to 120.25 C."
        ),
    )
    argument_parser.add_argument(
        "--min-samples",
        type=int,
        default=5,
        help="Minimum number of samples required inside a drift window.",
    )
    argument_parser.add_argument(
        "--include-faulted",
        action="store_true",
        help="Keep samples whose fault_status is not 0x00.",
    )
    argument_parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for plots and summary files. Defaults beside the input file.",
    )
    argument_parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip plot generation.",
    )
    argument_parser.add_argument(
        "--plot-smoothing-sec",
        type=float,
        default=120.0,
        help=(
            "Smoothing span in seconds used only for plot overlays. "
            "Values <= 0 disable smoothing overlays."
        ),
    )
    argument_parser.add_argument(
        "--plot-smoothing-method",
        choices=["rolling", "savgol"],
        default="rolling",
        help=(
            "Smoothing method for plot overlays: rolling=centered moving average, "
            "savgol=Savitzky-Golay smoothing via scipy.signal.savgol_filter."
        ),
    )
    return argument_parser.parse_args()



def load_ptlog_dataframe(input_path: Path) -> Tuple[pd.DataFrame, Dict[str, str]]:
    """Load a PTLOG/CSV hybrid file into a DataFrame.

    Args:
        input_path: Path to the input PTLOG file.

    Returns:
        Tuple[pd.DataFrame, Dict[str, str]]: Parsed tabular data and extracted header fields.

    Raises:
        ValueError: If the CSV schema line cannot be found.
    """
    file_lines = input_path.read_text(encoding="utf-8", errors="replace").splitlines()

    header_fields: Dict[str, str] = {}
    schema_line_index: Optional[int] = None

    for line_index, line_text in enumerate(file_lines):
        if line_text.startswith(SCHEMA_PREFIX):
            schema_line_index = line_index
            break
        if line_text.startswith(HEADER_PREFIX):
            stripped_line = line_text[1:].strip()
            if "=" in stripped_line:
                field_name, field_value = stripped_line.split("=", 1)
                header_fields[field_name.strip()] = field_value.strip()

    if schema_line_index is None:
        raise ValueError(f"Could not find schema line in {input_path}.")

    csv_text = "\n".join(file_lines[schema_line_index:])
    dataframe = pd.read_csv(pd.io.common.StringIO(csv_text))
    return dataframe, header_fields



def choose_temperature_column(
    dataframe: pd.DataFrame,
    requested_column: str,
) -> str:
    """Choose the temperature column to analyze.

    Args:
        dataframe: Parsed measurement table.
        requested_column: User-requested column name or "auto".

    Returns:
        str: Name of the selected temperature column.

    Raises:
        ValueError: If the requested column is unavailable or empty.
    """
    candidate_columns = [requested_column] if requested_column != "auto" else ["cal_temp_c", "raw_temp_c"]

    for candidate_column in candidate_columns:
        if candidate_column in dataframe.columns:
            numeric_series = pd.to_numeric(dataframe[candidate_column], errors="coerce")
            if numeric_series.notna().any():
                return candidate_column

    available_columns = ", ".join(dataframe.columns)
    raise ValueError(
        "Could not choose a usable temperature column. "
        f"Requested={requested_column!r}, available columns: {available_columns}"
    )



def prepare_dataframe(
    raw_dataframe: pd.DataFrame,
    temperature_column: str,
    include_faulted: bool,
) -> pd.DataFrame:
    """Clean and normalize the measurement table.

    Args:
        raw_dataframe: Raw parsed measurement table.
        temperature_column: Selected temperature column name.
        include_faulted: Whether faulted samples should be retained.

    Returns:
        pd.DataFrame: Cleaned and time-sorted measurement data.
    """
    dataframe = raw_dataframe.copy()
    dataframe[temperature_column] = pd.to_numeric(dataframe[temperature_column], errors="coerce")
    dataframe["epoch_utc"] = pd.to_numeric(dataframe["epoch_utc"], errors="coerce")
    dataframe["fault_status"] = dataframe["fault_status"].astype(str)

    if "iso8601_local" in dataframe.columns:
        dataframe["timestamp"] = pd.to_datetime(dataframe["iso8601_local"], errors="coerce")
    else:
        dataframe["timestamp"] = pd.to_datetime(dataframe["epoch_utc"], unit="s", utc=True, errors="coerce")

    dataframe = dataframe.dropna(subset=["timestamp", temperature_column]).copy()

    if not include_faulted and "fault_status" in dataframe.columns:
        dataframe = dataframe[dataframe["fault_status"] == "0x00"].copy()

    dataframe = dataframe.sort_values("timestamp").reset_index(drop=True)
    dataframe["elapsed_sec"] = (
        dataframe["timestamp"] - dataframe["timestamp"].iloc[0]
    ).dt.total_seconds()
    dataframe["temperature_c"] = dataframe[temperature_column].astype(float)
    return dataframe



def compute_trailing_window_drift(
    dataframe: pd.DataFrame,
    window_sec: float,
    min_samples: int,
) -> pd.DataFrame:
    """Compute trailing-window linear drift for each sample.

    Args:
        dataframe: Cleaned measurement data.
        window_sec: Trailing window length in seconds.
        min_samples: Minimum required samples within a window.

    Returns:
        pd.DataFrame: Input data with drift and window metadata columns added.
    """
    drift_values_c_per_min = np.full(len(dataframe), np.nan, dtype=float)
    window_sample_counts = np.zeros(len(dataframe), dtype=int)
    window_start_seconds = np.full(len(dataframe), np.nan, dtype=float)

    elapsed_seconds = dataframe["elapsed_sec"].to_numpy(dtype=float)
    temperatures_c = dataframe["temperature_c"].to_numpy(dtype=float)

    window_start_index = 0
    for sample_index in range(len(dataframe)):
        current_time_seconds = elapsed_seconds[sample_index]
        while elapsed_seconds[window_start_index] < current_time_seconds - window_sec:
            window_start_index += 1

        window_elapsed_seconds = elapsed_seconds[window_start_index : sample_index + 1]
        window_temperatures_c = temperatures_c[window_start_index : sample_index + 1]
        window_sample_counts[sample_index] = len(window_elapsed_seconds)
        window_start_seconds[sample_index] = window_elapsed_seconds[0]

        if len(window_elapsed_seconds) < min_samples:
            continue
        if window_elapsed_seconds[-1] <= window_elapsed_seconds[0]:
            continue
        current_window_span_sec = window_elapsed_seconds[-1] - window_elapsed_seconds[0]
        if current_window_span_sec < window_sec:
            continue

        regression_result = linregress(window_elapsed_seconds, window_temperatures_c)
        drift_values_c_per_min[sample_index] = regression_result.slope * SECONDS_PER_MINUTE

    dataframe = dataframe.copy()
    dataframe["drift_c_per_min"] = drift_values_c_per_min
    dataframe["window_samples"] = window_sample_counts
    dataframe["window_start_elapsed_sec"] = window_start_seconds
    return dataframe



def compute_plot_smoothing_series(
    elapsed_seconds: np.ndarray,
    values: np.ndarray,
    smoothing_sec: float,
    method: str,
) -> np.ndarray:
    """Compute a smoothed series used only for plot overlays.

    Args:
        elapsed_seconds: Monotonic elapsed sample times in seconds.
        values: Data values to smooth for plotting.
        smoothing_sec: Target smoothing span in seconds.
        method: Smoothing method, either "rolling" or "savgol".

    Returns:
        np.ndarray: Smoothed values. Falls back to raw values for short or invalid inputs.
    """
    raw_values = np.asarray(values, dtype=float)
    elapsed_seconds = np.asarray(elapsed_seconds, dtype=float)

    if smoothing_sec <= 0.0 or len(raw_values) < 2 or len(elapsed_seconds) < 2:
        return raw_values.copy()

    sample_intervals = np.diff(elapsed_seconds)
    valid_intervals = sample_intervals[np.isfinite(sample_intervals) & (sample_intervals > 0.0)]
    if len(valid_intervals) == 0:
        return raw_values.copy()

    median_interval_sec = float(np.median(valid_intervals))
    if not np.isfinite(median_interval_sec) or median_interval_sec <= 0.0:
        return raw_values.copy()

    smoothing_samples = max(int(round(smoothing_sec / median_interval_sec)), 1)
    if smoothing_samples <= 1:
        return raw_values.copy()

    if method == "rolling":
        return (
            pd.Series(raw_values)
            .rolling(window=smoothing_samples, center=True, min_periods=1)
            .mean()
            .to_numpy(dtype=float)
        )

    if method == "savgol":
        smoothed_values = raw_values.copy()
        finite_mask = np.isfinite(raw_values)
        if not finite_mask.any():
            return smoothed_values

        finite_indices = np.flatnonzero(finite_mask)
        run_start = 0
        while run_start < len(finite_indices):
            run_end = run_start
            while (
                run_end + 1 < len(finite_indices)
                and finite_indices[run_end + 1] == finite_indices[run_end] + 1
            ):
                run_end += 1

            start_index = int(finite_indices[run_start])
            end_index = int(finite_indices[run_end]) + 1
            run_values = raw_values[start_index:end_index]
            run_length = len(run_values)

            window_length = min(smoothing_samples, run_length)
            if window_length % 2 == 0:
                window_length -= 1
            if window_length >= 3:
                polyorder = min(2, window_length - 1)
                if polyorder >= 1:
                    try:
                        smoothed_values[start_index:end_index] = signal.savgol_filter(
                            run_values,
                            window_length=window_length,
                            polyorder=polyorder,
                            mode="interp",
                        )
                    except ValueError:
                        pass
            run_start = run_end + 1
        return smoothed_values

    return raw_values.copy()


def find_stable_segments(
    dataframe: pd.DataFrame,
    drift_threshold_c_per_min: float,
    hold_sec: float,
    target_temp_c: Optional[float],
    setpoint_band_c: Optional[float],
) -> list[tuple[int, int]]:
    """Find contiguous drift-stable regions that satisfy the dwell requirement.

    Args:
        dataframe: Data with computed rolling drift.
        drift_threshold_c_per_min: Absolute drift threshold.
        hold_sec: Required continuous hold time inside threshold.
        target_temp_c: Optional target temperature in deg C.
        setpoint_band_c: Optional allowed half-band around the target in deg C.

    Returns:
        list[tuple[int, int]]: Inclusive (start_index, end_index) pairs for stable regions.
    """
    within_threshold_mask = (
        dataframe["drift_c_per_min"].abs() <= drift_threshold_c_per_min
    ) & dataframe["drift_c_per_min"].notna()

    if target_temp_c is not None and setpoint_band_c is not None:
        within_setpoint_band_mask = (
            (dataframe["temperature_c"] - target_temp_c).abs() <= setpoint_band_c
        ) & dataframe["temperature_c"].notna()
        within_threshold_mask = within_threshold_mask & within_setpoint_band_mask
    stable_indices = np.flatnonzero(within_threshold_mask.to_numpy(dtype=bool))
    if len(stable_indices) == 0:
        return []

    elapsed_seconds = dataframe["elapsed_sec"].to_numpy(dtype=float)
    stable_segments: list[tuple[int, int]] = []
    segment_start_index = int(stable_indices[0])
    previous_index = int(stable_indices[0])

    for current_index in stable_indices[1:]:
        current_index = int(current_index)
        if current_index != previous_index + 1:
            segment_duration_sec = elapsed_seconds[previous_index] - elapsed_seconds[segment_start_index]
            if segment_duration_sec >= hold_sec:
                stable_segments.append((segment_start_index, previous_index))
            segment_start_index = current_index
        previous_index = current_index

    segment_duration_sec = elapsed_seconds[previous_index] - elapsed_seconds[segment_start_index]
    if segment_duration_sec >= hold_sec:
        stable_segments.append((segment_start_index, previous_index))

    return stable_segments



def estimate_dominant_cycle(
    elapsed_seconds: np.ndarray,
    temperatures_c: np.ndarray,
) -> Dict[str, Optional[float]]:
    """Estimate the strongest oscillatory component in the stable region.

    Args:
        elapsed_seconds: Elapsed sample times in seconds.
        temperatures_c: Temperature values in deg C.

    Returns:
        Dict[str, Optional[float]]: Frequency-domain summary metrics.
    """
    if len(elapsed_seconds) < 8:
        return {
            "dominant_period_sec": None,
            "dominant_frequency_hz": None,
            "dominant_cycle_amplitude_c": None,
            "dominant_peak_power": None,
            "dominant_peak_to_median_power_ratio": None,
        }

    sample_intervals = np.diff(elapsed_seconds)
    median_interval_sec = float(np.median(sample_intervals))
    if not np.isfinite(median_interval_sec) or median_interval_sec <= 0.0:
        return {
            "dominant_period_sec": None,
            "dominant_frequency_hz": None,
            "dominant_cycle_amplitude_c": None,
            "dominant_peak_power": None,
            "dominant_peak_to_median_power_ratio": None,
        }

    uniform_times_sec = np.arange(elapsed_seconds[0], elapsed_seconds[-1] + median_interval_sec, median_interval_sec)
    uniform_temperatures_c = np.interp(uniform_times_sec, elapsed_seconds, temperatures_c)
    detrended_temperatures_c = signal.detrend(uniform_temperatures_c)

    if len(detrended_temperatures_c) < 8:
        return {
            "dominant_period_sec": None,
            "dominant_frequency_hz": None,
            "dominant_cycle_amplitude_c": None,
            "dominant_peak_power": None,
            "dominant_peak_to_median_power_ratio": None,
        }

    frequencies_hz, power_density = signal.welch(
        detrended_temperatures_c,
        fs=1.0 / median_interval_sec,
        nperseg=min(256, len(detrended_temperatures_c)),
        detrend="linear",
    )

    positive_frequency_mask = frequencies_hz > 0.0
    frequencies_hz = frequencies_hz[positive_frequency_mask]
    power_density = power_density[positive_frequency_mask]

    if len(frequencies_hz) == 0:
        return {
            "dominant_period_sec": None,
            "dominant_frequency_hz": None,
            "dominant_cycle_amplitude_c": None,
            "dominant_peak_power": None,
            "dominant_peak_to_median_power_ratio": None,
        }

    peak_index = int(np.argmax(power_density))
    dominant_frequency_hz = float(frequencies_hz[peak_index])
    dominant_peak_power = float(power_density[peak_index])
    median_power = float(np.median(power_density)) if len(power_density) else math.nan
    dominant_period_sec = 1.0 / dominant_frequency_hz if dominant_frequency_hz > 0.0 else None
    dominant_cycle_amplitude_c = math.sqrt(max(2.0 * dominant_peak_power, 0.0))
    peak_to_median_ratio = (
        dominant_peak_power / median_power
        if np.isfinite(median_power) and median_power > 0.0
        else None
    )

    return {
        "dominant_period_sec": dominant_period_sec,
        "dominant_frequency_hz": dominant_frequency_hz,
        "dominant_cycle_amplitude_c": dominant_cycle_amplitude_c,
        "dominant_peak_power": dominant_peak_power,
        "dominant_peak_to_median_power_ratio": peak_to_median_ratio,
    }



def estimate_autocorrelation_time_constant(
    elapsed_seconds: np.ndarray,
    temperatures_c: np.ndarray,
) -> Dict[str, Optional[float]]:
    """Estimate an approximate thermal time constant from autocorrelation decay.

    Args:
        elapsed_seconds: Elapsed sample times in seconds.
        temperatures_c: Temperature values in deg C.

    Returns:
        Dict[str, Optional[float]]: Autocorrelation-based time metrics.
    """
    if len(elapsed_seconds) < 8:
        return {
            "autocorr_1e_time_sec": None,
            "autocorr_ar1_tau_sec": None,
            "lag1_autocorr": None,
        }

    sample_intervals = np.diff(elapsed_seconds)
    median_interval_sec = float(np.median(sample_intervals))
    if not np.isfinite(median_interval_sec) or median_interval_sec <= 0.0:
        return {
            "autocorr_1e_time_sec": None,
            "autocorr_ar1_tau_sec": None,
            "lag1_autocorr": None,
        }

    uniform_times_sec = np.arange(elapsed_seconds[0], elapsed_seconds[-1] + median_interval_sec, median_interval_sec)
    uniform_temperatures_c = np.interp(uniform_times_sec, elapsed_seconds, temperatures_c)
    centered_temperatures_c = uniform_temperatures_c - np.mean(uniform_temperatures_c)

    if np.allclose(centered_temperatures_c, 0.0):
        return {
            "autocorr_1e_time_sec": 0.0,
            "autocorr_ar1_tau_sec": 0.0,
            "lag1_autocorr": 0.0,
        }

    autocorrelation_full = np.correlate(centered_temperatures_c, centered_temperatures_c, mode="full")
    autocorrelation = autocorrelation_full[autocorrelation_full.size // 2 :]
    autocorrelation = autocorrelation / autocorrelation[0]

    one_over_e_threshold = math.e ** -1
    below_threshold_indices = np.flatnonzero(autocorrelation <= one_over_e_threshold)
    autocorr_1e_time_sec = (
        float(below_threshold_indices[0] * median_interval_sec)
        if len(below_threshold_indices)
        else None
    )

    lag1_autocorr = float(autocorrelation[1]) if len(autocorrelation) > 1 else None
    autocorr_ar1_tau_sec = None
    if lag1_autocorr is not None and 0.0 < lag1_autocorr < 1.0:
        autocorr_ar1_tau_sec = -median_interval_sec / math.log(lag1_autocorr)

    return {
        "autocorr_1e_time_sec": autocorr_1e_time_sec,
        "autocorr_ar1_tau_sec": autocorr_ar1_tau_sec,
        "lag1_autocorr": lag1_autocorr,
    }



def analyze_stable_region(
    stable_dataframe: pd.DataFrame,
    target_temp_c: Optional[float],
) -> Dict[str, Any]:
    """Compute stability metrics for the region after the drift threshold is met.

    Args:
        stable_dataframe: Measurement data beginning at the stable start time.

    Returns:
        Dict[str, Any]: Summary metrics describing temperature wander and dynamics.
    """
    elapsed_seconds = stable_dataframe["elapsed_sec"].to_numpy(dtype=float)
    temperatures_c = stable_dataframe["temperature_c"].to_numpy(dtype=float)

    basic_regression = linregress(elapsed_seconds, temperatures_c)
    detrended_temperatures_c = temperatures_c - (
        basic_regression.intercept + basic_regression.slope * elapsed_seconds
    )

    cycle_summary = estimate_dominant_cycle(elapsed_seconds, temperatures_c)
    autocorr_summary = estimate_autocorrelation_time_constant(elapsed_seconds, temperatures_c)

    summary: Dict[str, Any] = {
        "sample_count": int(len(stable_dataframe)),
        "duration_sec": float(elapsed_seconds[-1] - elapsed_seconds[0]) if len(elapsed_seconds) > 1 else 0.0,
        "mean_temp_c": float(np.mean(temperatures_c)),
        "median_temp_c": float(np.median(temperatures_c)),
        "std_temp_c": float(np.std(temperatures_c, ddof=1)) if len(temperatures_c) > 1 else 0.0,
        "min_temp_c": float(np.min(temperatures_c)),
        "max_temp_c": float(np.max(temperatures_c)),
        "peak_to_peak_temp_c": float(np.ptp(temperatures_c)),
        "mean_abs_dev_c": float(np.mean(np.abs(temperatures_c - np.mean(temperatures_c)))),
        "stable_region_slope_c_per_min": float(basic_regression.slope * SECONDS_PER_MINUTE),
        "stable_region_r_squared": float(basic_regression.rvalue ** 2),
        "detrended_std_temp_c": float(np.std(detrended_temperatures_c, ddof=1)) if len(detrended_temperatures_c) > 1 else 0.0,
        "detrended_peak_to_peak_temp_c": float(np.ptp(detrended_temperatures_c)),
    }
    if target_temp_c is not None:
        temperature_error_c = temperatures_c - target_temp_c
        summary.update({
            "target_temp_c": float(target_temp_c),
            "mean_error_from_target_c": float(np.mean(temperature_error_c)),
            "median_error_from_target_c": float(np.median(temperature_error_c)),
            "max_abs_error_from_target_c": float(np.max(np.abs(temperature_error_c))),
        })
    summary.update(cycle_summary)
    summary.update(autocorr_summary)
    return summary



def format_summary_text(
    input_path: Path,
    header_fields: Dict[str, str],
    temperature_column: str,
    window_sec: float,
    drift_threshold_c_per_min: float,
    hold_sec: float,
    target_temp_c: Optional[float],
    setpoint_band_c: Optional[float],
    full_dataframe: pd.DataFrame,
    stable_segments: list[dict[str, Any]],
) -> str:
    """Build a human-readable report.

    Args:
        input_path: Source file path.
        header_fields: Parsed PTLOG header metadata.
        temperature_column: Selected temperature column.
        window_sec: Drift window length.
        drift_threshold_c_per_min: Stability threshold.
        hold_sec: Required stable dwell time.
        target_temp_c: Optional target temperature in deg C.
        setpoint_band_c: Optional allowed half-band around the target in deg C.
        full_dataframe: Full cleaned dataset.
        stable_segments: Stable-region summaries.

    Returns:
        str: Multi-line report text.
    """
    report_lines = []
    report_lines.append(f"Input file: {input_path}")
    report_lines.append(f"Selected temperature column: {temperature_column}")
    report_lines.append(f"Window length: {window_sec:.3f} s")
    report_lines.append(f"Drift threshold: {drift_threshold_c_per_min:.6f} C/min")
    report_lines.append(f"Hold time: {hold_sec:.3f} s")
    if target_temp_c is not None:
        report_lines.append(f"Target temperature: {target_temp_c:.6f} C")
    if setpoint_band_c is not None:
        report_lines.append(f"Setpoint band: +/-{setpoint_band_c:.6f} C")
    report_lines.append(f"Sample count analyzed: {len(full_dataframe)}")
    if header_fields.get("cal_applied"):
        report_lines.append(f"Header cal_applied: {header_fields['cal_applied']}")
    if header_fields.get("cal_method"):
        report_lines.append(f"Header cal_method: {header_fields['cal_method']}")
    report_lines.append("")

    if not stable_segments:
        report_lines.append("No stable region found that satisfies the drift threshold.")
        return "\n".join(report_lines)

    report_lines.append(f"Stable segments found: {len(stable_segments)}")
    report_lines.append("")

    ordered_keys = [
        "sample_count",
        "duration_sec",
        "mean_temp_c",
        "median_temp_c",
        "std_temp_c",
        "min_temp_c",
        "max_temp_c",
        "peak_to_peak_temp_c",
        "mean_abs_dev_c",
        "stable_region_slope_c_per_min",
        "stable_region_r_squared",
        "detrended_std_temp_c",
        "detrended_peak_to_peak_temp_c",
        "target_temp_c",
        "mean_error_from_target_c",
        "median_error_from_target_c",
        "max_abs_error_from_target_c",
        "dominant_period_sec",
        "dominant_frequency_hz",
        "dominant_cycle_amplitude_c",
        "dominant_peak_power",
        "dominant_peak_to_median_power_ratio",
        "lag1_autocorr",
        "autocorr_1e_time_sec",
        "autocorr_ar1_tau_sec",
    ]

    for segment_number, stable_segment in enumerate(stable_segments, start=1):
        report_lines.append(f"Stable segment {segment_number}")
        report_lines.append("-" * 80)
        report_lines.append(f"start_timestamp                  : {stable_segment['start_timestamp']}")
        report_lines.append(f"end_timestamp                    : {stable_segment['end_timestamp']}")
        report_lines.append(f"start_elapsed_sec                : {stable_segment['start_elapsed_sec']:.9f}")
        report_lines.append(f"end_elapsed_sec                  : {stable_segment['end_elapsed_sec']:.9f}")
        report_lines.append(f"start_drift_c_per_min            : {stable_segment['start_drift_c_per_min']:.9f}")
        report_lines.append(f"end_drift_c_per_min              : {stable_segment['end_drift_c_per_min']:.9f}")
        for metric_key in ordered_keys:
            metric_value = stable_segment['summary'].get(metric_key)
            if isinstance(metric_value, float):
                report_lines.append(f"{metric_key:32s}: {metric_value:.9f}")
            else:
                report_lines.append(f"{metric_key:32s}: {metric_value}")
        report_lines.append("")

    return "\n".join(report_lines)



def save_summary_files(
    output_dir: Path,
    summary_text: str,
    stable_segments: list[dict[str, Any]],
) -> None:
    """Write text and JSON summary files.

    Args:
        output_dir: Directory for output artifacts.
        summary_text: Human-readable summary text.
        stable_segments: Stable-region metrics to serialize as JSON.

    Returns:
        None.
    """
    (output_dir / "summary.txt").write_text(summary_text, encoding="utf-8")
    (output_dir / "summary.json").write_text(
        json.dumps(stable_segments, indent=2, sort_keys=True, default=str),
        encoding="utf-8",
    )



def plot_temperature_trace(
    dataframe: pd.DataFrame,
    stable_segments: list[dict[str, Any]],
    target_temp_c: Optional[float],
    setpoint_band_c: Optional[float],
    plot_smoothing_sec: float,
    plot_smoothing_method: str,
    output_path: Path,
) -> None:
    """Plot the full temperature trace with the detected stable point.

    Args:
        dataframe: Full cleaned dataset.
        stable_segments: Stable-region summaries.
        target_temp_c: Optional target temperature in deg C.
        setpoint_band_c: Optional allowed half-band around the target in deg C.
        plot_smoothing_sec: Smoothing span in seconds for plot overlays only.
        plot_smoothing_method: Plot overlay smoothing method.
        output_path: PNG output path.

    Returns:
        None.
    """
    figure, axis = plt.subplots(figsize=(12, 6))
    elapsed_seconds = dataframe["elapsed_sec"].to_numpy(dtype=float)
    temperatures_c = dataframe["temperature_c"].to_numpy(dtype=float)
    axis.plot(
        elapsed_seconds,
        temperatures_c,
        alpha=0.4,
        linewidth=1.0,
        label="Temperature (raw)",
    )
    if plot_smoothing_sec > 0.0:
        smoothed_temperatures_c = compute_plot_smoothing_series(
            elapsed_seconds=elapsed_seconds,
            values=temperatures_c,
            smoothing_sec=plot_smoothing_sec,
            method=plot_smoothing_method,
        )
        axis.plot(
            elapsed_seconds,
            smoothed_temperatures_c,
            linewidth=2.0,
            label="Temperature (smoothed)",
        )
    for segment_number, stable_segment in enumerate(stable_segments, start=1):
        axis.axvspan(
            stable_segment["start_elapsed_sec"],
            stable_segment["end_elapsed_sec"],
            alpha=0.15,
            label="Stable segment" if segment_number == 1 else None,
        )
    if target_temp_c is not None:
        axis.axhline(target_temp_c, linestyle=":", label="Target temp")
    if target_temp_c is not None and setpoint_band_c is not None:
        axis.axhline(target_temp_c + setpoint_band_c, linestyle="--", label="Upper band")
        axis.axhline(target_temp_c - setpoint_band_c, linestyle="--", label="Lower band")
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("Temperature (C)")
    axis.set_title("Temperature trace")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)



def plot_drift_trace(
    dataframe: pd.DataFrame,
    drift_threshold_c_per_min: float,
    plot_smoothing_sec: float,
    plot_smoothing_method: str,
    output_path: Path,
) -> None:
    """Plot trailing-window drift versus time.

    Args:
        dataframe: Dataset with computed drift.
        drift_threshold_c_per_min: Stability threshold.
        plot_smoothing_sec: Smoothing span in seconds for plot overlays only.
        plot_smoothing_method: Plot overlay smoothing method.
        output_path: PNG output path.

    Returns:
        None.
    """
    figure, axis = plt.subplots(figsize=(12, 6))
    elapsed_seconds = dataframe["elapsed_sec"].to_numpy(dtype=float)
    drift_c_per_min = dataframe["drift_c_per_min"].to_numpy(dtype=float)
    axis.plot(
        elapsed_seconds,
        drift_c_per_min,
        alpha=0.4,
        linewidth=1.0,
        label="Rolling drift (raw)",
    )
    if plot_smoothing_sec > 0.0:
        smoothed_drift_c_per_min = compute_plot_smoothing_series(
            elapsed_seconds=elapsed_seconds,
            values=drift_c_per_min,
            smoothing_sec=plot_smoothing_sec,
            method=plot_smoothing_method,
        )
        axis.plot(
            elapsed_seconds,
            smoothed_drift_c_per_min,
            linewidth=2.0,
            label="Rolling drift (smoothed)",
        )
    axis.axhline(drift_threshold_c_per_min, linestyle="--", label="+ threshold")
    axis.axhline(-drift_threshold_c_per_min, linestyle="--", label="- threshold")
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("Drift (C/min)")
    axis.set_title("Trailing-window drift")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)



def plot_stable_region_detail(
    stable_dataframe: pd.DataFrame,
    target_temp_c: Optional[float],
    setpoint_band_c: Optional[float],
    plot_smoothing_sec: float,
    plot_smoothing_method: str,
    output_path: Path,
) -> None:
    """Plot detailed stable-region behavior.

    Args:
        stable_dataframe: Dataset beginning at the stable start.
        target_temp_c: Optional target temperature in deg C.
        setpoint_band_c: Optional allowed half-band around the target in deg C.
        plot_smoothing_sec: Smoothing span in seconds for plot overlays only.
        plot_smoothing_method: Plot overlay smoothing method.
        output_path: PNG output path.

    Returns:
        None.
    """
    elapsed_seconds = stable_dataframe["elapsed_sec"].to_numpy(dtype=float)
    temperatures_c = stable_dataframe["temperature_c"].to_numpy(dtype=float)
    regression = linregress(elapsed_seconds, temperatures_c)
    fitted_temperatures_c = regression.intercept + regression.slope * elapsed_seconds

    figure, axis = plt.subplots(figsize=(12, 6))
    axis.plot(
        elapsed_seconds,
        temperatures_c,
        alpha=0.4,
        linewidth=1.0,
        label="Stable region temperature (raw)",
    )
    if plot_smoothing_sec > 0.0:
        smoothed_temperatures_c = compute_plot_smoothing_series(
            elapsed_seconds=elapsed_seconds,
            values=temperatures_c,
            smoothing_sec=plot_smoothing_sec,
            method=plot_smoothing_method,
        )
        axis.plot(
            elapsed_seconds,
            smoothed_temperatures_c,
            linewidth=2.0,
            label="Stable region temperature (smoothed)",
        )
    axis.plot(elapsed_seconds, fitted_temperatures_c, linestyle="--", label="Linear trend")
    if target_temp_c is not None:
        axis.axhline(target_temp_c, linestyle=":", label="Target temp")
    if target_temp_c is not None and setpoint_band_c is not None:
        axis.axhline(target_temp_c + setpoint_band_c, linestyle="--", label="Upper band")
        axis.axhline(target_temp_c - setpoint_band_c, linestyle="--", label="Lower band")
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("Temperature (C)")
    axis.set_title("Stable-region detail")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)



def main() -> int:
    """Run the command-line program.

    Args:
        None.

    Returns:
        int: Process exit code.
    """
    arguments = parse_arguments()

    if (arguments.target_temp is None) != (arguments.setpoint_band is None):
        raise ValueError(
            "--target-temp and --setpoint-band must be provided together when either is used."
        )
    if arguments.setpoint_band is not None and arguments.setpoint_band < 0.0:
        raise ValueError("--setpoint-band must be non-negative.")

    output_dir = arguments.output_dir
    if output_dir is None:
        output_dir = arguments.input_path.with_name(arguments.input_path.stem + "_analysis")
    output_dir.mkdir(parents=True, exist_ok=True)

    raw_dataframe, header_fields = load_ptlog_dataframe(arguments.input_path)
    temperature_column = choose_temperature_column(raw_dataframe, arguments.temp_column)
    dataframe = prepare_dataframe(raw_dataframe, temperature_column, arguments.include_faulted)

    if len(dataframe) < arguments.min_samples:
        raise ValueError(
            f"Not enough usable samples after filtering. Need at least {arguments.min_samples}, got {len(dataframe)}."
        )

    dataframe = compute_trailing_window_drift(
        dataframe,
        window_sec=arguments.window_sec,
        min_samples=arguments.min_samples,
    )

    stable_index_pairs = find_stable_segments(
        dataframe,
        drift_threshold_c_per_min=arguments.drift_threshold,
        hold_sec=arguments.hold_sec,
        target_temp_c=arguments.target_temp,
        setpoint_band_c=arguments.setpoint_band,
    )

    stable_segments: list[dict[str, Any]] = []
    longest_stable_dataframe = None
    longest_stable_duration_sec = -1.0
    for stable_start_index, stable_end_index in stable_index_pairs:
        stable_dataframe = dataframe.iloc[stable_start_index : stable_end_index + 1].copy().reset_index(drop=True)
        stable_summary = analyze_stable_region(
            stable_dataframe,
            target_temp_c=arguments.target_temp,
        )
        stable_duration_sec = stable_summary["duration_sec"]
        stable_segment = {
            "start_index": stable_start_index,
            "end_index": stable_end_index,
            "start_timestamp": str(dataframe.iloc[stable_start_index]["timestamp"]),
            "end_timestamp": str(dataframe.iloc[stable_end_index]["timestamp"]),
            "start_elapsed_sec": float(dataframe.iloc[stable_start_index]["elapsed_sec"]),
            "end_elapsed_sec": float(dataframe.iloc[stable_end_index]["elapsed_sec"]),
            "start_drift_c_per_min": float(dataframe.iloc[stable_start_index]["drift_c_per_min"]),
            "end_drift_c_per_min": float(dataframe.iloc[stable_end_index]["drift_c_per_min"]),
            "summary": stable_summary,
        }
        stable_segments.append(stable_segment)
        if stable_duration_sec > longest_stable_duration_sec:
            longest_stable_duration_sec = stable_duration_sec
            longest_stable_dataframe = stable_dataframe

    summary_text = format_summary_text(
        input_path=arguments.input_path,
        header_fields=header_fields,
        temperature_column=temperature_column,
        window_sec=arguments.window_sec,
        drift_threshold_c_per_min=arguments.drift_threshold,
        hold_sec=arguments.hold_sec,
        target_temp_c=arguments.target_temp,
        setpoint_band_c=arguments.setpoint_band,
        full_dataframe=dataframe,
        stable_segments=stable_segments,
    )

    save_summary_files(output_dir, summary_text, stable_segments)
    dataframe.to_csv(output_dir / "rolling_drift.csv", index=False)

    if not arguments.no_plots:
        plot_temperature_trace(
            dataframe,
            stable_segments,
            arguments.target_temp,
            arguments.setpoint_band,
            arguments.plot_smoothing_sec,
            arguments.plot_smoothing_method,
            output_dir / "temperature_trace.png",
        )
        plot_drift_trace(
            dataframe,
            arguments.drift_threshold,
            arguments.plot_smoothing_sec,
            arguments.plot_smoothing_method,
            output_dir / "drift_trace.png",
        )
        if longest_stable_dataframe is not None:
            plot_stable_region_detail(
                longest_stable_dataframe,
                arguments.target_temp,
                arguments.setpoint_band,
                arguments.plot_smoothing_sec,
                arguments.plot_smoothing_method,
                output_dir / "stable_region_detail.png",
            )

    print(summary_text)
    print("")
    print(f"Wrote analysis outputs to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
