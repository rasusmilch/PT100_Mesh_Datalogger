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

import datetime
import glob
import math
import os
import tempfile
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.units import inch
from reportlab.platypus import SimpleDocTemplate, Spacer, Paragraph, Image, Table, TableStyle
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_CENTER


@dataclass
class LoadedLog:
    dataframe: pd.DataFrame
    time_column: str
    time_source: str  # "iso8601_local", "epoch_utc", or "mixed"
    tzinfo: Optional[datetime.tzinfo]
    source_files: List[str]
    dropped_no_time_rows: int


def _human_series_label(series_name: str) -> str:
    mapping = {
        "cal_temp_c": "Calibrated Temperature (°C)",
        "raw_temp_c": "Raw Temperature (°C)",
        "raw_rtd_ohms": "RTD Resistance (Ω)",
        "epoch_utc": "UTC Epoch (s)",
        "seq": "Sequence",
        "record_id": "Record ID",
    }
    return mapping.get(series_name, series_name)


def _normalize_schema(df: pd.DataFrame) -> pd.DataFrame:
    """Normalize schema differences so plotting works across schema versions."""
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

    return df


def _pick_time_source(
    df: pd.DataFrame,
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
        tzinfo = getattr(first_ts, "tzinfo", None)

        if epoch_series is not None and epoch_series.notna().any():
            if tzinfo is not None:
                epoch_converted = epoch_series.dt.tz_convert(tzinfo)
            else:
                epoch_converted = epoch_series.dt.tz_localize(None)

            combined_time = iso_series.copy().fillna(epoch_converted)
            time_source = "mixed" if iso_series.isna().any() else "iso8601_local"
        else:
            combined_time = iso_series
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

    actual_start = pd.to_datetime(trimmed[time_column]).min()
    actual_end = pd.to_datetime(trimmed[time_column]).max()
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
    combined, time_column, time_source, tzinfo, dropped_no_time_rows = _pick_time_source(combined)

    if time_column == "__time":
        combined[time_column] = pd.to_datetime(combined[time_column], errors="coerce")
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

def _build_figure(
    df: pd.DataFrame,
    time_column: str,
    y_name: str,
    overlay_raw_temp: bool,
    smooth: bool,
    title: str,
    enable_downsample: bool,
    max_plot_points: int,
) -> Tuple[plt.Figure, int, int]:
    y_series = pd.to_numeric(df[y_name], errors="coerce")
    x_values = df[time_column]
    total_points = int(df.shape[0])

    # Compute smoothing on the full series first (so downsampling doesn't bias the trend).
    smoothed_series: Optional[pd.Series] = None
    if smooth and y_name in ("cal_temp_c", "raw_temp_c"):
        # Window scales with dataset size but is capped to keep the curve responsive.
        window_size = min(151, max(3, total_points // 40))
        smoothed_series = (
            y_series.rolling(window=window_size, center=True, min_periods=max(3, window_size // 4)).mean()
        )

    raw_temp_series: Optional[pd.Series] = None
    if overlay_raw_temp and "raw_temp_c" in df.columns and y_name != "raw_temp_c":
        raw_temp_series = pd.to_numeric(df["raw_temp_c"], errors="coerce")

    plot_positions = np.arange(total_points, dtype=np.int64)
    did_downsample = False
    if enable_downsample and total_points > max_plot_points:
        series_for_downsample: List[pd.Series] = [y_series]
        if raw_temp_series is not None:
            series_for_downsample.append(raw_temp_series)
        plot_positions = _downsample_positions_minmax(
            series_list=series_for_downsample, max_plot_points=max_plot_points
        )
        did_downsample = True

    x_plot = x_values.iloc[plot_positions]
    y_plot = y_series.iloc[plot_positions]

    fig, ax = plt.subplots(figsize=(11, 6.2))
    ax.grid(True)

    title_for_plot = title
    if did_downsample:
        title_for_plot = f"{title}\\nDownsampled for plotting: {len(plot_positions):,} of {total_points:,} points"
    ax.set_title(title_for_plot)

    ax.set_xlabel(_human_time_label(time_column))
    ax.set_ylabel(_human_series_label(y_name))

    if smooth and smoothed_series is not None:
        ax.plot(x_plot, y_plot, linewidth=0.7, alpha=0.6, label="raw (downsampled)" if did_downsample else "raw")
        ax.plot(
            x_plot,
            smoothed_series.iloc[plot_positions],
            linewidth=2.0,
            label=f"smooth (w={min(151, max(3, total_points // 40))})",
        )
    else:
        ax.plot(x_plot, y_plot, linewidth=1.2, label="data (downsampled)" if did_downsample else "data")

    if raw_temp_series is not None:
        ax.plot(x_plot, raw_temp_series.iloc[plot_positions], linewidth=0.9, alpha=0.7, label="raw_temp_c (overlay)")

    ax.legend()

    fig.autofmt_xdate(rotation=30, ha="right")
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

        self.y_choice = tk.StringVar(value="cal_temp_c")
        self.overlay_raw = tk.BooleanVar(value=False)
        self.smooth = tk.BooleanVar(value=True)
        self.enable_downsample = tk.BooleanVar(value=True)
        self.max_plot_points = tk.IntVar(value=20000)
        self.pdf_plot_dpi = tk.IntVar(value=600)
        self.pdf_vector = tk.BooleanVar(value=False)

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

        tk.Label(frm, text='Start time (YYYY-MM-DD HH:MM):').grid(row=2, column=0, sticky="w", pady=(12, 0))
        tk.Entry(frm, textvariable=self.start_time_text, width=26).grid(row=2, column=1, sticky="w", pady=(12, 0))

        tk.Label(frm, text='End time (YYYY-MM-DD HH:MM):').grid(row=3, column=0, sticky="w", pady=(6, 0))
        tk.Entry(frm, textvariable=self.end_time_text, width=26).grid(row=3, column=1, sticky="w", pady=(6, 0))

        tk.Label(frm, text="Y-axis series:").grid(row=4, column=0, sticky="w", pady=(12, 0))
        series_choices = ["cal_temp_c", "raw_temp_c", "raw_rtd_ohms", "record_id", "seq"]
        tk.OptionMenu(frm, self.y_choice, *series_choices).grid(row=4, column=1, sticky="w", pady=(12, 0))

        tk.Checkbutton(frm, text="Overlay raw_temp_c", variable=self.overlay_raw).grid(row=5, column=1, sticky="w")
        tk.Checkbutton(frm, text="Smooth (rolling mean)", variable=self.smooth).grid(row=6, column=1, sticky="w")
        tk.Checkbutton(frm, text="Downsample large datasets (preserve spikes)", variable=self.enable_downsample).grid(row=7, column=1, sticky="w")

        tk.Label(frm, text="Max plot points:").grid(row=8, column=0, sticky="e")
        tk.Entry(frm, textvariable=self.max_plot_points, width=10).grid(row=8, column=1, sticky="w")

        tk.Label(frm, text="PDF plot DPI (raster):").grid(row=9, column=0, sticky="e")
        tk.Entry(frm, textvariable=self.pdf_plot_dpi, width=10).grid(row=9, column=1, sticky="w")
        tk.Checkbutton(frm, text="Vector PDF (plot + summary as vector)", variable=self.pdf_vector).grid(row=10, column=1, sticky="w")

        btn_row = 11

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

    def _get_trimmed_df(self) -> Tuple[pd.DataFrame, str, str, str]:
        if not self.loaded:
            raise ValueError("No data loaded.")
        trimmed, start_label, end_label, summary = _validate_and_trim_by_minute(
            df=self.loaded.dataframe,
            time_column=self.loaded.time_column,
            start_text=self.start_time_text.get(),
            end_text=self.end_time_text.get(),
        )
        return trimmed, start_label, end_label, summary

    def save_trimmed_csv(self) -> None:
        if not self.loaded:
            messagebox.showerror("Save Error", "No data loaded.")
            return
        try:
            trimmed, start_label, end_label, summary = self._get_trimmed_df()
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
            df, _, _, summary = self._get_trimmed_df()
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        y_name = self.y_choice.get()
        if y_name not in df.columns:
            messagebox.showerror("Plot Error", f"Column not found: {y_name}")
            return

        nodes = _node_ids_from_df(df)
        title = f"PT100 Log — {_human_series_label(y_name)}\nNodes: {nodes} — {summary}"

        fig, _plot_points, _total_points = _build_figure(
            df=df,
            time_column=self.loaded.time_column,
            y_name=y_name,
            overlay_raw_temp=self.overlay_raw.get(),
            smooth=self.smooth.get(),
            title=title,
            enable_downsample=self.enable_downsample.get(),
            max_plot_points=int(self.max_plot_points.get()),
        )
        plt.show()

    def export_pdf(self) -> None:
        if not self.loaded:
            messagebox.showerror("PDF Error", "No data loaded.")
            return
        try:
            df, start_label, end_label, summary = self._get_trimmed_df()
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        y_name = self.y_choice.get()
        if y_name not in df.columns:
            messagebox.showerror("PDF Error", f"Column not found: {y_name}")
            return

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
        subtitle = f"Nodes: {nodes} — {summary}"

        fig_title = f"{_human_series_label(y_name)} vs Time"
        fig, plot_points, total_points = _build_figure(
            df=df,
            time_column=self.loaded.time_column,
            y_name=y_name,
            overlay_raw_temp=self.overlay_raw.get(),
            smooth=self.smooth.get(),
            title=fig_title,
            enable_downsample=self.enable_downsample.get(),
            max_plot_points=int(self.max_plot_points.get()),
        )

        stats = _compute_basic_stats(df.get("cal_temp_c", pd.Series(dtype=float)))

        summary_rows = [
            ["Field", "Value"],
            ["Time source", self.loaded.time_source],
            ["Start", start_label],
            ["End", end_label],
            ["Rows in trim", f"{len(df):,}"],
            ["Plot points", f"{plot_points:,}"],
            ["Downsampling", "yes" if plot_points != len(df) else "no"],
            ["Max plot points", f"{int(self.max_plot_points.get()):,}"],
            ["PDF plot mode", "vector" if use_vector_pdf else "raster"],
            ["PDF plot DPI (raster)", f"{int(self.pdf_plot_dpi.get())}" if not use_vector_pdf else "n/a"],
            ["Dropped no-time rows", f"{self.loaded.dropped_no_time_rows:,}"],
            ["Primary series", _human_series_label(y_name)],
            ["Overlay raw_temp_c", "yes" if self.overlay_raw.get() else "no"],
            ["Smoothing", "yes" if self.smooth.get() else "no"],
            ["cal_temp_c min/avg/max/std", f"{stats['min']} / {stats['avg']} / {stats['max']} / {stats['std']}"],
        ]

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
