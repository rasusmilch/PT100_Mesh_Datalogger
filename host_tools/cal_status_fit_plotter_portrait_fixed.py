
#!/usr/bin/env python3
"""GUI tool to parse PT100 `cal status` text and generate a PDF fit report."""

from __future__ import annotations

import datetime
import math
import os
import re
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.platypus import (
    Image,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet


COEFFICIENT_LETTERS = "abcdefghijklmnopqrstuvwxyz"


@dataclass
class CalibrationPoint:
    """One captured calibration point from the compact point list."""

    point_index: int
    reference_temp_c: Optional[float] = None
    ideal_reference_res_ohm: Optional[float] = None
    captured_raw_temp_c: Optional[float] = None
    captured_raw_res_ohm: Optional[float] = None
    capture_residual_temp_c: Optional[float] = None
    capture_residual_res_ohm: Optional[float] = None
    captured_raw_temp_stddev_c: Optional[float] = None
    captured_raw_res_stddev_ohm: Optional[float] = None
    captured_drift_c_per_min: Optional[float] = None
    captured_delta_c: Optional[float] = None


@dataclass
class FitRow:
    """One row from the printed fit table."""

    point_index: int
    target_temp_c: Optional[float] = None
    target_res_ohm: Optional[float] = None
    captured_temp_c: Optional[float] = None
    captured_res_ohm: Optional[float] = None
    model_temp_c: Optional[float] = None
    model_res_ohm: Optional[float] = None
    temp_residual_c: Optional[float] = None
    ohm_residual_ohm: Optional[float] = None
    correction_fit_domain: Optional[float] = None


@dataclass
class CalibrationReport:
    """Parsed `cal status` data needed for plotting and reporting."""

    source_name: str = "<pasted text>"
    raw_text: str = ""
    calibration_equation: str = ""
    coefficients: Dict[str, float] = field(default_factory=dict)
    fit_domain: str = ""
    model_type: str = ""
    polynomial_degree: Optional[int] = None
    calibration_points_used: Optional[int] = None
    model_parameters: Optional[int] = None
    degrees_of_freedom: Optional[int] = None
    calibration_method: str = ""
    effective_last_utc: str = ""
    due_date: str = ""
    metric_values: Dict[str, float] = field(default_factory=dict)
    metric_labels: Dict[str, str] = field(default_factory=dict)
    compact_points: List[CalibrationPoint] = field(default_factory=list)
    fit_rows: List[FitRow] = field(default_factory=list)


def _safe_float(value: Optional[str]) -> Optional[float]:
    """Parse a float-like string, returning None for blank or n/a."""
    if value is None:
        return None
    text = value.strip()
    if not text or text.lower() == "n/a":
        return None
    return float(text)


def _safe_int(value: Optional[str]) -> Optional[int]:
    """Parse an integer-like string, returning None for blank or n/a."""
    if value is None:
        return None
    text = value.strip()
    if not text or text.lower() == "n/a":
        return None
    return int(text)


def _clean_input_text(raw_text: str) -> str:
    """Normalize line endings and strip nulls."""
    normalized_text = raw_text.replace("\r\n", "\n").replace("\r", "\n")
    normalized_text = normalized_text.replace("\x00", "")
    return normalized_text


def _detect_multiple_outputs(raw_text: str) -> None:
    """Raise ValueError if the text appears to contain multiple cal-status dumps."""
    model_report_count = raw_text.count("Calibration Model Fit Report:")
    calibration_block_count = len(re.findall(r"(?m)^Calibration:\s*$", raw_text))
    fit_table_count = raw_text.count("Fit Table:")

    if model_report_count == 0:
        raise ValueError(
            "Could not find a 'Calibration Model Fit Report:' block in the input."
        )
    if model_report_count > 1:
        raise ValueError(
            "Multiple 'Calibration Model Fit Report:' blocks were found. "
            "This looks like more than one cal status output; refusing to proceed."
        )
    if fit_table_count > 1:
        raise ValueError(
            "Multiple 'Fit Table:' blocks were found. "
            "This looks like more than one cal status output; refusing to proceed."
        )
    if calibration_block_count > 1:
        raise ValueError(
            "Multiple top-level 'Calibration:' sections were found. "
            "This looks like more than one cal status output; refusing to proceed."
        )


def _extract_label_value(
    raw_text: str,
    label: str,
    *,
    flags: int = 0,
    take_last: bool = False,
) -> Optional[str]:
    """Extract a single `Label: value` field from text."""
    pattern = re.compile(rf"(?m)^\s*{re.escape(label)}:\s*(.+?)\s*$", flags)
    matches = pattern.findall(raw_text)
    if not matches:
        return None
    return matches[-1] if take_last else matches[0]


def _extract_metric_value(raw_text: str, label: str) -> Optional[float]:
    """Extract a numeric model-fit metric by its printed label."""
    text_value = _extract_label_value(raw_text, label)
    return _safe_float(text_value)


def _extract_compact_point_lines(raw_text: str) -> List[str]:
    """Return compact one-line point entries from the `cal status` output."""
    point_lines: List[str] = []
    for line in raw_text.splitlines():
        if re.match(r"^\s+\d+:\s+reference_temp_C=", line):
            point_lines.append(line.strip())
    return point_lines


def _parse_key_value_tokens(line: str) -> Dict[str, str]:
    """Parse key=value tokens from one compact calibration-point line."""
    tokens: Dict[str, str] = {}
    for match in re.finditer(r"([A-Za-z0-9_]+)=([^\s]+)", line):
        tokens[match.group(1)] = match.group(2)
    return tokens


def _parse_compact_points(raw_text: str) -> List[CalibrationPoint]:
    """Parse compact `1: reference_temp_C=...` calibration point lines."""
    parsed_points: List[CalibrationPoint] = []
    for line in _extract_compact_point_lines(raw_text):
        point_match = re.match(r"^(\d+):\s+", line)
        if point_match is None:
            continue
        point_index = int(point_match.group(1))
        tokens = _parse_key_value_tokens(line)
        parsed_points.append(
            CalibrationPoint(
                point_index=point_index,
                reference_temp_c=_safe_float(tokens.get("reference_temp_C")),
                ideal_reference_res_ohm=_safe_float(tokens.get("ideal_ref_res_Ohm")),
                captured_raw_temp_c=_safe_float(tokens.get("captured_raw_temp_avg_C")),
                captured_raw_res_ohm=_safe_float(tokens.get("captured_raw_res_avg_Ohm")),
                capture_residual_temp_c=_safe_float(tokens.get("residual_C")),
                capture_residual_res_ohm=_safe_float(tokens.get("residual_res_Ohm")),
                captured_raw_temp_stddev_c=_safe_float(
                    tokens.get("captured_raw_temp_stddev_C")
                ),
                captured_raw_res_stddev_ohm=_safe_float(
                    tokens.get("captured_raw_res_stddev_Ohm")
                ),
                captured_drift_c_per_min=_safe_float(
                    tokens.get("captured_drift_C_per_min")
                ),
                captured_delta_c=_safe_float(tokens.get("captured_delta_C")),
            )
        )
    parsed_points.sort(key=lambda point: point.point_index)
    return parsed_points


def _extract_fit_table_lines(raw_text: str) -> List[str]:
    """Extract numeric rows from the `Fit Table:` block."""
    marker = "Fit Table:"
    marker_index = raw_text.find(marker)
    if marker_index < 0:
        raise ValueError("Could not find the 'Fit Table:' block.")
    table_text = raw_text[marker_index:].splitlines()[1:]
    data_lines: List[str] = []
    for line in table_text:
        if re.match(r"^\s*\d+\s+", line):
            data_lines.append(line.rstrip())
            continue
        if data_lines:
            break
    if not data_lines:
        raise ValueError("The 'Fit Table:' block was found, but no data rows were parsed.")
    return data_lines


def _parse_fit_rows(raw_text: str) -> List[FitRow]:
    """Parse the printed model fit table."""
    fit_rows: List[FitRow] = []
    for line in _extract_fit_table_lines(raw_text):
        parts = re.split(r"\s+", line.strip())
        if len(parts) < 10:
            raise ValueError(f"Could not parse fit-table row: {line}")
        fit_rows.append(
            FitRow(
                point_index=int(parts[0]),
                target_temp_c=_safe_float(parts[1]),
                target_res_ohm=_safe_float(parts[2]),
                captured_temp_c=_safe_float(parts[3]),
                captured_res_ohm=_safe_float(parts[4]),
                model_temp_c=_safe_float(parts[5]),
                model_res_ohm=_safe_float(parts[6]),
                temp_residual_c=_safe_float(parts[7]),
                ohm_residual_ohm=_safe_float(parts[8]),
                correction_fit_domain=_safe_float(parts[9]),
            )
        )
    fit_rows.sort(key=lambda row: row.point_index)
    return fit_rows


def _extract_coefficients(raw_text: str) -> Dict[str, float]:
    """Extract coefficient lines such as `a: 0.123`."""
    coefficient_map: Dict[str, float] = {}
    for match in re.finditer(
        r"(?m)^\s*([a-z])\s*:\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*$",
        raw_text,
    ):
        coefficient_map[match.group(1)] = float(match.group(2))
    return dict(sorted(coefficient_map.items(), key=lambda item: item[0]))


def _parse_report(raw_text: str, source_name: str) -> CalibrationReport:
    """Parse a single `cal status` output into a report object."""
    normalized_text = _clean_input_text(raw_text)
    _detect_multiple_outputs(normalized_text)

    report = CalibrationReport(source_name=source_name, raw_text=normalized_text)
    report.calibration_equation = (
        _extract_label_value(normalized_text, "calibration_equation", take_last=True) or ""
    )
    report.coefficients = _extract_coefficients(normalized_text)
    report.fit_domain = _extract_label_value(normalized_text, "Fit domain") or ""
    report.model_type = _extract_label_value(normalized_text, "Model type") or ""
    report.polynomial_degree = _safe_int(
        _extract_label_value(normalized_text, "Polynomial degree")
    )
    report.calibration_points_used = _safe_int(
        _extract_label_value(normalized_text, "Calibration points used")
    )
    report.model_parameters = _safe_int(
        _extract_label_value(normalized_text, "Model parameters")
    )
    report.degrees_of_freedom = _safe_int(
        _extract_label_value(normalized_text, "Degrees of freedom")
    )
    report.calibration_method = (
        _extract_label_value(normalized_text, "Method", take_last=True) or ""
    )
    report.effective_last_utc = (
        _extract_label_value(normalized_text, "Effective last UTC") or ""
    )
    report.due_date = _extract_label_value(normalized_text, "Due date") or ""

    metric_order = [
        "Mean signed residual (ohms)",
        "Mean absolute residual (ohms)",
        "Root mean square error (ohms)",
        "Residual standard deviation (ohms)",
        "Maximum absolute residual (ohms)",
        "Sum of squared error (ohm^2)",
        "Mean signed residual (C)",
        "Mean absolute residual (C)",
        "Root mean square error (C)",
        "Residual standard deviation (C)",
        "Maximum absolute residual (C)",
        "Sum of squared error (C^2)",
        "R^2",
        "Adjusted R^2",
        "Largest applied correction (ohms)",
        "Largest applied correction (C)",
    ]
    for metric_label in metric_order:
        metric_value = _extract_metric_value(normalized_text, metric_label)
        if metric_value is not None:
            metric_key = re.sub(r"[^a-z0-9]+", "_", metric_label.lower()).strip("_")
            report.metric_values[metric_key] = metric_value
            report.metric_labels[metric_key] = metric_label

    report.compact_points = _parse_compact_points(normalized_text)
    report.fit_rows = _parse_fit_rows(normalized_text)

    if not report.fit_rows:
        raise ValueError("No fit-table rows were parsed.")
    if not report.compact_points:
        # Fit table is enough to plot and report, but warn through exception text only if truly empty.
        pass
    return report


def _is_resistance_domain(report: CalibrationReport) -> bool:
    """Return True if the fit domain is resistance-based."""
    return "resistance" in report.fit_domain.lower() or "ohm" in report.fit_domain.lower()


def _raw_domain_label(report: CalibrationReport) -> str:
    """Return the X-axis label for the calibration domain."""
    if _is_resistance_domain(report):
        return "Captured raw resistance (ohm)"
    return "Captured raw temperature (C)"


def _corrected_domain_label(report: CalibrationReport) -> str:
    """Return the Y-axis label for the corrected/target domain."""
    if _is_resistance_domain(report):
        return "Corrected / target resistance (ohm)"
    return "Corrected / target temperature (C)"


def _correction_label(report: CalibrationReport) -> str:
    """Return the correction-column label."""
    if _is_resistance_domain(report):
        return "Applied correction (ohm)"
    return "Applied correction (C)"


def _get_curve_axes_data(
    report: CalibrationReport,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return raw-domain X data plus target and model Y data for plotting."""
    x_values: List[float] = []
    target_values: List[float] = []
    model_values: List[float] = []
    capture_values: List[float] = []

    for row in report.fit_rows:
        if _is_resistance_domain(report):
            x_value = row.captured_res_ohm
            target_value = row.target_res_ohm
            model_value = row.model_res_ohm
            capture_value = row.captured_res_ohm
        else:
            x_value = row.captured_temp_c
            target_value = row.target_temp_c
            model_value = row.model_temp_c
            capture_value = row.captured_temp_c

        if x_value is None or target_value is None:
            continue
        x_values.append(x_value)
        target_values.append(target_value)
        model_values.append(model_value if model_value is not None else target_value)
        capture_values.append(capture_value if capture_value is not None else x_value)

    if not x_values:
        raise ValueError("Not enough fit-table data to build the calibration curve.")
    return (
        np.asarray(x_values, dtype=float),
        np.asarray(target_values, dtype=float),
        np.asarray(model_values, dtype=float),
        np.asarray(capture_values, dtype=float),
    )


def _evaluate_polynomial_curve(
    report: CalibrationReport,
    x_min: float,
    x_max: float,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Evaluate a polynomial calibration curve from printed coefficients."""
    if not report.coefficients:
        return None
    if not report.model_type:
        return None
    if report.model_type.lower() not in ("poly", "linear"):
        return None

    ordered_coefficients: List[float] = [
        report.coefficients[key] for key in sorted(report.coefficients.keys())
    ]
    if not ordered_coefficients:
        return None

    degree = len(ordered_coefficients) - 1
    x_padding = max(abs(x_max - x_min) * 0.05, 1e-6)
    curve_x = np.linspace(x_min - x_padding, x_max + x_padding, 600)
    curve_y = np.zeros_like(curve_x)

    for coefficient_index, coefficient_value in enumerate(ordered_coefficients):
        exponent = degree - coefficient_index
        curve_y += coefficient_value * np.power(curve_x, exponent)

    return curve_x, curve_y


def _build_curve_fallback(
    x_values: np.ndarray,
    model_values: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Create a simple piecewise line through model points."""
    sort_indices = np.argsort(x_values)
    return x_values[sort_indices], model_values[sort_indices]


def _display_source_name(report: CalibrationReport) -> str:
    """Return a cleaner source label for chart and PDF titles."""
    source_name = (report.source_name or "").strip()
    if not source_name or source_name == "<pasted text>":
        return "Pasted cal status input"
    return Path(source_name).stem


def _style_plot_axes(ax: plt.Axes) -> None:
    """Apply a clean, print-friendly style to a plot axis."""
    ax.grid(True, which="major", alpha=0.25)
    ax.set_axisbelow(True)


def _build_curve_figure(report: CalibrationReport) -> plt.Figure:
    """Build the main calibration curve figure."""
    x_values, target_values, model_values, _ = _get_curve_axes_data(report)

    polynomial_curve = _evaluate_polynomial_curve(
        report,
        x_min=float(np.min(x_values)),
        x_max=float(np.max(x_values)),
    )
    if polynomial_curve is None:
        curve_x, curve_y = _build_curve_fallback(x_values, model_values)
        curve_label = "Model fit (point-to-point)"
    else:
        curve_x, curve_y = polynomial_curve
        curve_label = "Model fit"

    figure = plt.figure(figsize=(7.2, 5.2))
    axes = figure.add_subplot(1, 1, 1)
    axes.plot(curve_x, curve_y, linewidth=2.0, label=curve_label)
    axes.scatter(
        x_values,
        target_values,
        marker="o",
        s=55,
        label="Calibration target points",
        zorder=3,
    )
    axes.scatter(
        x_values,
        model_values,
        marker="x",
        s=55,
        label="Model values at points",
        zorder=3,
    )

    for row in report.fit_rows:
        if _is_resistance_domain(report):
            x_value = row.captured_res_ohm
            y_value = row.target_res_ohm
        else:
            x_value = row.captured_temp_c
            y_value = row.target_temp_c
        if x_value is None or y_value is None:
            continue
        axes.annotate(
            str(row.point_index),
            (x_value, y_value),
            textcoords="offset points",
            xytext=(5, 5),
            fontsize=9,
        )

    axes.set_title("Calibration curve - target points and fitted model", pad=10)
    axes.set_xlabel(_raw_domain_label(report))
    axes.set_ylabel(_corrected_domain_label(report))
    _style_plot_axes(axes)
    axes.legend(loc="best", fontsize=9)
    figure.tight_layout(rect=(0.06, 0.08, 0.98, 0.96))
    return figure


def _set_residual_axis_scale(ax: plt.Axes, use_symlog: bool) -> None:
    """Configure the Y-axis scale for residual plots."""
    if use_symlog:
        ax.set_yscale("symlog", linthresh=1e-3)


def _build_residuals_figure(report: CalibrationReport, use_symlog: bool) -> plt.Figure:
    """Build the residual-plot figure."""
    model_temp_pairs: List[Tuple[float, float]] = []
    model_ohm_pairs: List[Tuple[float, float]] = []
    for row in report.fit_rows:
        if row.target_temp_c is not None and row.temp_residual_c is not None:
            model_temp_pairs.append((row.target_temp_c, row.temp_residual_c))
        if row.target_temp_c is not None and row.ohm_residual_ohm is not None:
            model_ohm_pairs.append((row.target_temp_c, row.ohm_residual_ohm))

    compact_by_index = {point.point_index: point for point in report.compact_points}
    capture_temp_pairs: List[Tuple[float, float]] = []
    capture_ohm_pairs: List[Tuple[float, float]] = []
    for row in report.fit_rows:
        compact_point = compact_by_index.get(row.point_index)
        if (
            compact_point is not None
            and row.target_temp_c is not None
            and compact_point.capture_residual_temp_c is not None
        ):
            capture_temp_pairs.append(
                (row.target_temp_c, compact_point.capture_residual_temp_c)
            )
        if (
            compact_point is not None
            and row.target_temp_c is not None
            and compact_point.capture_residual_res_ohm is not None
        ):
            capture_ohm_pairs.append(
                (row.target_temp_c, compact_point.capture_residual_res_ohm)
            )

    figure = plt.figure(figsize=(7.2, 6.1))
    top_axes = figure.add_subplot(2, 1, 1)
    bottom_axes = figure.add_subplot(2, 1, 2, sharex=top_axes)

    if model_temp_pairs:
        model_temp_array = np.asarray(model_temp_pairs, dtype=float)
        top_axes.axhline(0.0, linewidth=1.0, linestyle="--")
        top_axes.plot(
            model_temp_array[:, 0],
            model_temp_array[:, 1],
            marker="o",
            label="Model temp residual",
        )
    if capture_temp_pairs:
        capture_temp_array = np.asarray(capture_temp_pairs, dtype=float)
        top_axes.plot(
            capture_temp_array[:, 0],
            capture_temp_array[:, 1],
            marker="x",
            linestyle=":",
            label="Captured temp residual",
        )
    top_axes.set_ylabel("Residual (C)")
    top_axes.set_title("Residuals vs Reference Temperature", pad=8)
    _set_residual_axis_scale(top_axes, use_symlog)
    _style_plot_axes(top_axes)
    if model_temp_pairs or capture_temp_pairs:
        top_axes.legend(loc="best", fontsize=9)

    if model_ohm_pairs:
        model_ohm_array = np.asarray(model_ohm_pairs, dtype=float)
        bottom_axes.axhline(0.0, linewidth=1.0, linestyle="--")
        bottom_axes.plot(
            model_ohm_array[:, 0],
            model_ohm_array[:, 1],
            marker="o",
            label="Model ohm residual",
        )
    if capture_ohm_pairs:
        capture_ohm_array = np.asarray(capture_ohm_pairs, dtype=float)
        bottom_axes.plot(
            capture_ohm_array[:, 0],
            capture_ohm_array[:, 1],
            marker="x",
            linestyle=":",
            label="Captured ohm residual",
        )
    bottom_axes.set_xlabel("Reference temperature (C)")
    bottom_axes.set_ylabel("Residual (ohm)")
    _set_residual_axis_scale(bottom_axes, use_symlog)
    _style_plot_axes(bottom_axes)
    if model_ohm_pairs or capture_ohm_pairs:
        bottom_axes.legend(loc="best", fontsize=9)

    figure.tight_layout(rect=(0.08, 0.07, 0.98, 0.96))
    return figure


def _format_value(value: Optional[float], decimals: int = 6) -> str:
    """Format a float for tables."""
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "n/a"
    return f"{value:.{decimals}f}"


def _format_metric_value(metric_key: str, metric_value: float) -> str:
    """Format metric values with sane precision."""
    if "r_2" in metric_key or metric_key.endswith("r_2") or "adjusted_r_2" in metric_key:
        return f"{metric_value:.9f}"
    if abs(metric_value) >= 1000 or (metric_value != 0 and abs(metric_value) < 1e-3):
        return f"{metric_value:.9g}"
    return f"{metric_value:.9f}".rstrip("0").rstrip(".")


def _make_table(data: List[List[str]], column_widths: Sequence[float]) -> Table:
    """Create a consistently styled ReportLab table."""
    table = Table(data, colWidths=list(column_widths), repeatRows=1)
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e6e6e6")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.black),
                ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                ("FONTNAME", (0, 1), (-1, -1), "Helvetica"),
                ("FONTSIZE", (0, 0), (-1, -1), 8.5),
                ("LEADING", (0, 0), (-1, -1), 10),
                ("GRID", (0, 0), (-1, -1), 0.5, colors.grey),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("TOPPADDING", (0, 0), (-1, -1), 4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
            ]
        )
    )
    return table


def _page_footer(canvas, doc) -> None:  # type: ignore[no-untyped-def]
    """Draw a simple footer with the page number."""
    canvas.saveState()
    canvas.setFont("Helvetica", 9)
    canvas.setFillColor(colors.grey)
    page_width, _page_height = doc.pagesize
    canvas.drawCentredString(page_width / 2.0, 0.45 * inch, f"Page {doc.page}")
    canvas.restoreState()


def _render_figure_to_png(figure: plt.Figure, temp_dir: str, file_stem: str) -> str:
    """Save a figure to a temporary PNG file and return its path."""
    output_path = os.path.join(temp_dir, f"{file_stem}.png")
    figure.savefig(output_path, dpi=220, bbox_inches="tight")
    return output_path


def _build_summary_rows(report: CalibrationReport) -> List[List[str]]:
    """Build the summary statistics table."""
    summary_rows: List[List[str]] = [
        ["Item", "Value"],
        ["Input source", _display_source_name(report)],
        ["Calibration equation", report.calibration_equation or "n/a"],
        ["Fit domain", report.fit_domain or "n/a"],
        ["Model type", report.model_type or "n/a"],
        ["Polynomial degree", str(report.polynomial_degree) if report.polynomial_degree is not None else "n/a"],
        ["Calibration points used", str(report.calibration_points_used) if report.calibration_points_used is not None else str(len(report.fit_rows))],
        ["Model parameters", str(report.model_parameters) if report.model_parameters is not None else "n/a"],
        ["Degrees of freedom", str(report.degrees_of_freedom) if report.degrees_of_freedom is not None else "n/a"],
        ["Calibration method", report.calibration_method or "n/a"],
        ["Effective last UTC", report.effective_last_utc or "n/a"],
        ["Due date", report.due_date or "n/a"],
    ]
    return summary_rows


def _build_metric_rows(report: CalibrationReport) -> List[List[str]]:
    """Build the model-fit metrics table."""
    preferred_keys = [
        "mean_absolute_residual_ohms",
        "root_mean_square_error_ohms",
        "maximum_absolute_residual_ohms",
        "mean_absolute_residual_c",
        "root_mean_square_error_c",
        "maximum_absolute_residual_c",
        "r_2",
        "adjusted_r_2",
        "largest_applied_correction_ohms",
        "largest_applied_correction_c",
    ]
    present_keys = [key for key in preferred_keys if key in report.metric_values]
    for metric_key in report.metric_values:
        if metric_key not in present_keys:
            present_keys.append(metric_key)

    metric_rows: List[List[str]] = [["Metric", "Value"]]
    for metric_key in present_keys:
        metric_rows.append(
            [
                report.metric_labels.get(metric_key, metric_key),
                _format_metric_value(metric_key, report.metric_values[metric_key]),
            ]
        )
    return metric_rows


def _build_point_rows(report: CalibrationReport) -> List[List[str]]:
    """Build the per-point detail table."""
    compact_by_index = {point.point_index: point for point in report.compact_points}
    rows: List[List[str]] = [
        [
            "Pt",
            "Target C",
            "Target ohm",
            "Captured C",
            "Captured ohm",
            "Model C",
            "Model ohm",
            "Model res C",
            "Model res ohm",
            "Capture res C",
            "Capture res ohm",
            _correction_label(report),
        ]
    ]
    for row in report.fit_rows:
        compact_point = compact_by_index.get(row.point_index)
        rows.append(
            [
                str(row.point_index),
                _format_value(row.target_temp_c, 3),
                _format_value(row.target_res_ohm, 3),
                _format_value(row.captured_temp_c, 3),
                _format_value(row.captured_res_ohm, 3),
                _format_value(row.model_temp_c, 3),
                _format_value(row.model_res_ohm, 3),
                _format_value(row.temp_residual_c, 6),
                _format_value(row.ohm_residual_ohm, 6),
                _format_value(
                    compact_point.capture_residual_temp_c if compact_point else None, 6
                ),
                _format_value(
                    compact_point.capture_residual_res_ohm if compact_point else None, 6
                ),
                _format_value(row.correction_fit_domain, 6),
            ]
        )
    return rows


def _build_point_value_rows(report: CalibrationReport) -> List[List[str]]:
    """Build a portrait-friendly point-value table."""
    rows: List[List[str]] = [
        [
            "Pt",
            "Target C",
            "Target ohm",
            "Captured C",
            "Captured ohm",
            "Model C",
            "Model ohm",
        ]
    ]
    for row in report.fit_rows:
        rows.append(
            [
                str(row.point_index),
                _format_value(row.target_temp_c, 3),
                _format_value(row.target_res_ohm, 3),
                _format_value(row.captured_temp_c, 3),
                _format_value(row.captured_res_ohm, 3),
                _format_value(row.model_temp_c, 3),
                _format_value(row.model_res_ohm, 3),
            ]
        )
    return rows


def _build_point_residual_rows(report: CalibrationReport) -> List[List[str]]:
    """Build a portrait-friendly residual and correction table."""
    compact_by_index = {point.point_index: point for point in report.compact_points}
    rows: List[List[str]] = [
        [
            "Pt",
            "Model res C",
            "Model res ohm",
            "Capture res C",
            "Capture res ohm",
            _correction_label(report),
        ]
    ]
    for row in report.fit_rows:
        compact_point = compact_by_index.get(row.point_index)
        rows.append(
            [
                str(row.point_index),
                _format_value(row.temp_residual_c, 6),
                _format_value(row.ohm_residual_ohm, 6),
                _format_value(
                    compact_point.capture_residual_temp_c if compact_point else None, 6
                ),
                _format_value(
                    compact_point.capture_residual_res_ohm if compact_point else None, 6
                ),
                _format_value(row.correction_fit_domain, 6),
            ]
        )
    return rows


def export_pdf_report(
    report: CalibrationReport,
    save_path: str,
    *,
    use_symlog_residual_axis: bool,
) -> None:
    """Create a concise, professional multi-page PDF report."""
    curve_figure = _build_curve_figure(report)
    residuals_figure = _build_residuals_figure(report, use_symlog_residual_axis)

    temp_dir = tempfile.mkdtemp(prefix="cal_status_fit_report_")
    try:
        curve_png_path = _render_figure_to_png(curve_figure, temp_dir, "curve")
        residuals_png_path = _render_figure_to_png(
            residuals_figure, temp_dir, "residuals"
        )
    finally:
        plt.close(curve_figure)
        plt.close(residuals_figure)

    styles = getSampleStyleSheet()
    title_style = styles["Title"]
    heading_style = ParagraphStyle(
        "ReportHeading",
        parent=styles["Heading2"],
        spaceBefore=0,
        spaceAfter=8,
        keepWithNext=True,
    )
    body_style = ParagraphStyle(
        "BodyStyle",
        parent=styles["BodyText"],
        fontName="Helvetica",
        fontSize=9.5,
        leading=12,
        spaceAfter=6,
    )

    display_source_name = _display_source_name(report)
    summary_table = _make_table(_build_summary_rows(report), [2.0 * inch, 4.9 * inch])
    metric_table = _make_table(_build_metric_rows(report), [3.05 * inch, 3.35 * inch])
    point_values_table = _make_table(
        _build_point_value_rows(report),
        [0.45 * inch, 0.82 * inch, 0.90 * inch, 0.90 * inch, 0.98 * inch, 0.82 * inch, 0.90 * inch],
    )
    point_residuals_table = _make_table(
        _build_point_residual_rows(report),
        [0.45 * inch, 1.08 * inch, 1.12 * inch, 1.10 * inch, 1.22 * inch, 1.18 * inch],
    )

    elements = [
        Paragraph("PT100 Calibration Fit Report", title_style),
        Paragraph(f"<b>Input source:</b> {display_source_name}", body_style),
        Spacer(1, 0.08 * inch),
        Paragraph("Summary", heading_style),
        summary_table,
        PageBreak(),

        Paragraph("Calibration Curve", heading_style),
        Paragraph(
            "Polynomial fit with calibration target points and model values at each point.",
            body_style,
        ),
        Image(curve_png_path, width=6.7 * inch, height=4.95 * inch),
        PageBreak(),

        Paragraph("Residuals", heading_style),
        Paragraph(
            "Model-fit and captured residuals plotted against reference temperature.",
            body_style,
        ),
        Image(residuals_png_path, width=6.7 * inch, height=5.55 * inch),
        PageBreak(),

        Paragraph("Model Fit Statistics", heading_style),
        metric_table,
        Spacer(1, 0.16 * inch),
        Paragraph("Calibration Point Values", heading_style),
        point_values_table,
        Spacer(1, 0.16 * inch),
        Paragraph("Residuals and Applied Corrections", heading_style),
        point_residuals_table,
    ]

    document = SimpleDocTemplate(
        save_path,
        pagesize=letter,
        rightMargin=0.6 * inch,
        leftMargin=0.6 * inch,
        topMargin=0.6 * inch,
        bottomMargin=0.7 * inch,
        title="PT100 Calibration Fit Report",
        author="Nortech Systems, Inc.",
    )
    document.build(elements, onFirstPage=_page_footer, onLaterPages=_page_footer)

    for png_path in [curve_png_path, residuals_png_path]:
        if os.path.exists(png_path):
            os.remove(png_path)
    if os.path.isdir(temp_dir):
        os.rmdir(temp_dir)


class PasteTextDialog(simpledialog.Dialog):
    """Simple modal dialog for pasting raw `cal status` text."""

    def __init__(self, parent: tk.Misc, title: str = "Paste cal status text") -> None:
        self.pasted_text = ""
        super().__init__(parent, title)

    def body(self, master: tk.Misc) -> tk.Widget:
        ttk.Label(
            master,
            text="Paste one complete cal status output below:",
        ).grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        self.text_widget = tk.Text(master, width=110, height=28, wrap="word")
        self.text_widget.grid(row=1, column=0, padx=8, pady=(0, 8))
        return self.text_widget

    def apply(self) -> None:
        self.pasted_text = self.text_widget.get("1.0", "end-1c")


class CalibrationFitGui:
    """Tkinter GUI for loading, previewing, and exporting calibration fit reports."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PT100 cal status fit plotter")
        self.report: Optional[CalibrationReport] = None
        self.loaded_text: str = ""
        self.preview_window: Optional[tk.Toplevel] = None
        self.preview_canvas: Optional[FigureCanvasTkAgg] = None

        self.use_symlog_residual_axis = tk.BooleanVar(value=False)
        self.status_text = tk.StringVar(value="No cal status text loaded.")

        self._build_ui()

    def _build_ui(self) -> None:
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(3, weight=1)

        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        ttk.Button(button_frame, text="Open text file", command=self.open_text_file).grid(
            row=0, column=0, padx=(0, 6)
        )
        ttk.Button(button_frame, text="Paste text", command=self.paste_text).grid(
            row=0, column=1, padx=(0, 6)
        )
        ttk.Button(
            button_frame, text="Preview curve", command=self.preview_curve
        ).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(
            button_frame, text="Preview residuals", command=self.preview_residuals
        ).grid(row=0, column=3, padx=(0, 6))
        ttk.Button(button_frame, text="Export PDF", command=self.export_pdf).grid(
            row=0, column=4, padx=(0, 6)
        )
        ttk.Button(button_frame, text="Clear", command=self.clear_loaded_text).grid(
            row=0, column=5, padx=(0, 6)
        )

        options_frame = ttk.Frame(main_frame)
        options_frame.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        ttk.Checkbutton(
            options_frame,
            text="Use symlog axis for residual plots",
            variable=self.use_symlog_residual_axis,
        ).grid(row=0, column=0, sticky="w")

        ttk.Label(
            main_frame,
            textvariable=self.status_text,
            foreground="#333333",
        ).grid(row=2, column=0, sticky="w", pady=(0, 8))

        self.summary_text = tk.Text(main_frame, width=120, height=10, wrap="word")
        self.summary_text.grid(row=3, column=0, sticky="nsew")
        self.summary_text.configure(state="disabled")

        ttk.Label(
            main_frame,
            text="Loaded input preview:",
        ).grid(row=4, column=0, sticky="w", pady=(10, 4))

        self.input_preview_text = tk.Text(main_frame, width=120, height=18, wrap="word")
        self.input_preview_text.grid(row=5, column=0, sticky="nsew")
        self.input_preview_text.configure(state="disabled")
        main_frame.rowconfigure(5, weight=1)

    def clear_loaded_text(self) -> None:
        """Clear the current input and parsed report."""
        self.report = None
        self.loaded_text = ""
        self.status_text.set("No cal status text loaded.")
        self._set_text_widget(self.summary_text, "")
        self._set_text_widget(self.input_preview_text, "")
        self._close_preview_window()

    def _set_text_widget(self, widget: tk.Text, value: str) -> None:
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("1.0", value)
        widget.configure(state="disabled")

    def _update_report_views(self) -> None:
        """Refresh summary and input preview after parsing."""
        if self.report is None:
            return
        summary_lines = [
            f"Input source: {self.report.source_name}",
            f"Calibration equation: {self.report.calibration_equation or 'n/a'}",
            f"Fit domain: {self.report.fit_domain or 'n/a'}",
            f"Model type: {self.report.model_type or 'n/a'}",
            f"Polynomial degree: {self.report.polynomial_degree if self.report.polynomial_degree is not None else 'n/a'}",
            f"Calibration method: {self.report.calibration_method or 'n/a'}",
            f"Calibration points: {len(self.report.fit_rows)}",
        ]
        if self.report.metric_values:
            summary_lines.append("")
            summary_lines.append("Selected fit statistics:")
            for row in _build_metric_rows(self.report)[1:8]:
                summary_lines.append(f"  {row[0]}: {row[1]}")

        self._set_text_widget(self.summary_text, "\n".join(summary_lines))
        self._set_text_widget(self.input_preview_text, self.loaded_text[:20000])
        self.status_text.set(
            f"Loaded {self.report.source_name} with {len(self.report.fit_rows)} fit-table rows."
        )

    def _load_and_parse_text(self, raw_text: str, source_name: str) -> None:
        """Parse new input text and update the GUI."""
        try:
            parsed_report = _parse_report(raw_text, source_name)
        except Exception as exc:
            messagebox.showwarning("Parse warning", str(exc))
            return

        self.report = parsed_report
        self.loaded_text = raw_text
        self._update_report_views()

    def open_text_file(self) -> None:
        """Open a text file and parse it."""
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt *.log *.text"), ("All files", "*.*")]
        )
        if not file_path:
            return
        with open(file_path, "r", encoding="utf-8", errors="replace") as handle:
            raw_text = handle.read()
        self._load_and_parse_text(raw_text, os.path.basename(file_path))

    def paste_text(self) -> None:
        """Prompt for pasted text and parse it."""
        dialog = PasteTextDialog(self.root)
        pasted_text = dialog.pasted_text
        if not pasted_text.strip():
            return
        self._load_and_parse_text(pasted_text, "<pasted text>")

    def _close_preview_window(self) -> None:
        """Close any existing preview window."""
        if self.preview_canvas is not None:
            try:
                plt.close(self.preview_canvas.figure)
            except Exception:
                pass
            self.preview_canvas = None
        if self.preview_window is not None:
            try:
                self.preview_window.destroy()
            except Exception:
                pass
            self.preview_window = None

    def _show_figure_in_window(self, figure: plt.Figure, window_title: str) -> None:
        """Display a Matplotlib figure in a Tk window."""
        self._close_preview_window()
        preview_window = tk.Toplevel(self.root)
        preview_window.title(window_title)
        canvas = FigureCanvasTkAgg(figure, master=preview_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self.preview_window = preview_window
        self.preview_canvas = canvas

    def preview_curve(self) -> None:
        """Preview the calibration curve."""
        if self.report is None:
            messagebox.showerror("Preview error", "No cal status text loaded.")
            return
        try:
            figure = _build_curve_figure(self.report)
        except Exception as exc:
            messagebox.showerror("Preview error", str(exc))
            return
        self._show_figure_in_window(figure, "Calibration curve preview")

    def preview_residuals(self) -> None:
        """Preview the residual plots."""
        if self.report is None:
            messagebox.showerror("Preview error", "No cal status text loaded.")
            return
        try:
            figure = _build_residuals_figure(
                self.report,
                self.use_symlog_residual_axis.get(),
            )
        except Exception as exc:
            messagebox.showerror("Preview error", str(exc))
            return
        self._show_figure_in_window(figure, "Residual preview")

    def export_pdf(self) -> None:
        """Export the parsed report to PDF."""
        if self.report is None:
            messagebox.showerror("PDF error", "No cal status text loaded.")
            return
        initial_name = f"calibration_fit_report_{datetime.date.today().isoformat()}.pdf"
        save_path = filedialog.asksaveasfilename(
            defaultextension=".pdf",
            initialfile=initial_name,
            filetypes=[("PDF files", "*.pdf")],
        )
        if not save_path:
            return
        try:
            export_pdf_report(
                self.report,
                save_path,
                use_symlog_residual_axis=self.use_symlog_residual_axis.get(),
            )
        except Exception as exc:
            messagebox.showerror("PDF error", str(exc))
            return
        messagebox.showinfo("Success", f"PDF report saved:\n{save_path}")


def main() -> None:
    """Launch the GUI application."""
    root = tk.Tk()
    app = CalibrationFitGui(root)
    del app
    root.mainloop()


if __name__ == "__main__":
    main()
