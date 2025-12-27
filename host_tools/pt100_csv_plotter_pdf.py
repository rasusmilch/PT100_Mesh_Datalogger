#!/usr/bin/env python3
"""
PT100 Mesh Logger CSV Plotter + PDF Report

- Loads 1+ CSV exports from PT100 nodes (schema_ver, seq, epoch_utc, iso8601_local, ...)
- Plots selected series vs time
- Optional trim by start/end time (minute resolution) with strict validation
- Exports a PDF report (ReportLab) with a high-DPI plot image and summary table

Dependencies:
  - python3
  - pandas
  - matplotlib
  - reportlab

Run:
  python pt100_csv_plotter_pdf.py
"""

from __future__ import annotations

import datetime
import math
import os
import tempfile
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox

import pandas as pd
import matplotlib.pyplot as plt

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
    }
    return mapping.get(series_name, series_name)


def _pick_time_source(df: pd.DataFrame) -> Tuple[str, Optional[datetime.tzinfo], int]:
    """
    Pick the best time column:
      1) iso8601_local (if parseable and non-empty)
      2) epoch_utc (if > 0)
    Returns: (time_column, tzinfo, dropped_rows_without_time)
    """
    dropped_no_time_rows = 0

    if "iso8601_local" in df.columns:
        parsed = pd.to_datetime(df["iso8601_local"], errors="coerce", utc=False)
        usable = parsed.notna()
        if usable.any():
            # Preserve only rows with usable timestamps.
            dropped_no_time_rows = int((~usable).sum())
            df = df.loc[usable].copy()
            df["__time"] = parsed.loc[usable]
            tzinfo = None
            # If timestamps are tz-aware, pandas uses tz-aware dtype; grab tz if present.
            try:
                tzinfo = df["__time"].dt.tz  # type: ignore[attr-defined]
            except Exception:
                tzinfo = None
            return "__time", tzinfo, dropped_no_time_rows

    # Fallback to epoch_utc
    if "epoch_utc" in df.columns:
        epoch = pd.to_numeric(df["epoch_utc"], errors="coerce")
        usable = epoch.notna() & (epoch > 0)
        if usable.any():
            dropped_no_time_rows = int((~usable).sum())
            df = df.loc[usable].copy()
            # epoch_utc is UTC seconds.
            df["__time"] = pd.to_datetime(epoch.loc[usable], unit="s", utc=True)
            tzinfo = datetime.timezone.utc
            return "__time", tzinfo, dropped_no_time_rows

    raise ValueError(
        "No usable timestamp found. Need either iso8601_local (parseable) or epoch_utc (>0)."
    )


def _parse_user_time(text: str) -> Optional[datetime.datetime]:
    text = (text or "").strip()
    if not text:
        return None

    # Accept: "YYYY-MM-DD HH:MM" or "YYYY-MM-DDTHH:MM"
    for fmt in ("%Y-%m-%d %H:%M", "%Y-%m-%dT%H:%M"):
        try:
            return datetime.datetime.strptime(text, fmt)
        except ValueError:
            continue
    raise ValueError('Invalid time format. Use "YYYY-MM-DD HH:MM" (minutes only).')


def _nearest_minute_string(minutes_index: pd.DatetimeIndex, target: datetime.datetime) -> str:
    # minutes_index may be tz-aware; convert target accordingly if needed
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
    """
    Trimming is strict against minute buckets that exist in the log:
      - Start/end must be empty OR match one of the present minutes exactly.
      - If not present, reject and offer nearest valid minute.

    Returns: (trimmed_df, start_label, end_label, summary_text)
    """
    time_series = pd.to_datetime(df[time_column], errors="coerce")
    if time_series.isna().all():
        raise ValueError("Time column could not be parsed.")

    # Build minute buckets present in data.
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

    # Labels
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
        df["__source_file"] = os.path.basename(path)
        dataframes.append(df)

    combined = pd.concat(dataframes, ignore_index=True)

    time_column, tzinfo, dropped_no_time_rows = _pick_time_source(combined)

    # Sort by time.
    combined[time_column] = pd.to_datetime(combined[time_column], errors="coerce")
    combined = combined.sort_values(by=time_column).reset_index(drop=True)

    return LoadedLog(
        dataframe=combined,
        time_column=time_column,
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


def _build_figure(
    df: pd.DataFrame,
    time_column: str,
    y_name: str,
    overlay_raw_temp: bool,
    smooth: bool,
    title: str,
) -> plt.Figure:
    time_series = pd.to_datetime(df[time_column])
    y_primary = pd.to_numeric(df[y_name], errors="coerce")

    fig = plt.figure(figsize=(11, 6.2))
    ax = plt.gca()

    primary_label = _human_series_label(y_name)

    if smooth:
        sample_count = int(y_primary.shape[0])
        smoothing_window = max(5, min(151, sample_count // 40))
        y_smoothed = y_primary.rolling(window=smoothing_window, center=True, min_periods=1).mean()
        ax.plot(time_series, y_primary, linewidth=0.7, alpha=0.6, label=f"{primary_label} (raw)")
        ax.plot(time_series, y_smoothed, linewidth=2.0, label=f"{primary_label} (smoothed)")
    else:
        ax.plot(time_series, y_primary, linewidth=1.2, label=primary_label)

    if overlay_raw_temp and "raw_temp_c" in df.columns and y_name != "raw_temp_c":
        y_raw = pd.to_numeric(df["raw_temp_c"], errors="coerce")
        ax.plot(time_series, y_raw, linewidth=0.9, alpha=0.8, label=_human_series_label("raw_temp_c"))

    # Nice labels
    ax.set_title(title)
    ax.set_xlabel("Local Time")
    ax.set_ylabel(primary_label)

    # Format without seconds.
    import matplotlib.dates as mdates
    locator = mdates.AutoDateLocator(minticks=5, maxticks=12)
    formatter = mdates.DateFormatter("%Y-%m-%d %H:%M")
    ax.xaxis.set_major_locator(locator)
    ax.xaxis.set_major_formatter(formatter)

    ax.grid(True, linestyle=":", linewidth=0.7, alpha=0.7)
    ax.legend(loc="best")
    fig.autofmt_xdate(rotation=45)

    fig.tight_layout()
    return fig


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

    # Summary table.
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

    # Plot image
    img = Image(fig_png_path, width=7.5 * inch, height=4.2 * inch)
    elements.append(img)
    elements.append(Spacer(1, 10))

    # Files list (keep short)
    file_list = "\n".join([os.path.basename(p) for p in source_files[:12]])
    if len(source_files) > 12:
        file_list += f"\n… ({len(source_files)} files total)"
    elements.append(Paragraph(f"<b>Input file(s):</b><br/>{file_list}", styles_body))

    doc.build(elements)


class PlotterApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PT100 CSV Plotter + PDF Report")

        self.selected_files: List[str] = []
        self.loaded: Optional[LoadedLog] = None

        self.start_time_text = tk.StringVar(value="")
        self.end_time_text = tk.StringVar(value="")

        self.y_choice = tk.StringVar(value="cal_temp_c")
        self.overlay_raw = tk.BooleanVar(value=True)
        self.smooth = tk.BooleanVar(value=False)

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

        tk.Label(frm, text='Start time (YYYY-MM-DD HH:MM):').grid(row=2, column=0, sticky="w", pady=(12, 0))
        tk.Entry(frm, textvariable=self.start_time_text, width=26).grid(row=2, column=1, sticky="w", pady=(12, 0))

        tk.Label(frm, text='End time (YYYY-MM-DD HH:MM):').grid(row=3, column=0, sticky="w", pady=(6, 0))
        tk.Entry(frm, textvariable=self.end_time_text, width=26).grid(row=3, column=1, sticky="w", pady=(6, 0))

        tk.Label(frm, text="Y-axis series:").grid(row=4, column=0, sticky="w", pady=(12, 0))
        series_choices = ["cal_temp_c", "raw_temp_c", "raw_rtd_ohms", "seq"]
        tk.OptionMenu(frm, self.y_choice, *series_choices).grid(row=4, column=1, sticky="w", pady=(12, 0))

        tk.Checkbutton(frm, text="Overlay raw_temp_c", variable=self.overlay_raw).grid(row=5, column=1, sticky="w")
        tk.Checkbutton(frm, text="Smooth (rolling mean)", variable=self.smooth).grid(row=6, column=1, sticky="w")

        # Action buttons.
        btn_row = 7
        self.plot_btn = tk.Button(frm, text="Plot", command=self.plot, state=tk.DISABLED)
        self.plot_btn.grid(row=btn_row, column=0, sticky="w", pady=(15, 0))

        self.save_trim_btn = tk.Button(frm, text="Save Trimmed CSV", command=self.save_trimmed_csv, state=tk.DISABLED)
        self.save_trim_btn.grid(row=btn_row, column=1, sticky="w", pady=(15, 0))

        self.pdf_btn = tk.Button(frm, text="Export PDF Report", command=self.export_pdf, state=tk.DISABLED)
        self.pdf_btn.grid(row=btn_row + 1, column=0, sticky="w", pady=(8, 0))

        # Notes.
        self.note_label = tk.Label(
            frm,
            text="Notes:\n"
                 "• Trimming matches minute buckets present in the log.\n"
                 "• X-axis labels are formatted without seconds.\n"
                 "• Rows without usable timestamps are dropped.",
            justify="left",
        )
        self.note_label.grid(row=btn_row + 2, column=0, columnspan=2, sticky="w", pady=(10, 0))

    def select_files(self) -> None:
        file_paths = filedialog.askopenfilenames(
            title="Select PT100 CSV file(s)",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
        )
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
            df, start_label, end_label, summary = self._get_trimmed_df()
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        y_name = self.y_choice.get()
        if y_name not in df.columns:
            messagebox.showerror("Plot Error", f"Column not found: {y_name}")
            return

        nodes = _node_ids_from_df(df)
        title = f"PT100 Log — { _human_series_label(y_name) }\nNodes: {nodes} — {summary}"

        fig = _build_figure(
            df=df,
            time_column=self.loaded.time_column,
            y_name=y_name,
            overlay_raw_temp=self.overlay_raw.get(),
            smooth=self.smooth.get(),
            title=title,
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

        # Choose output PDF path.
        today = datetime.date.today().isoformat()
        default_name = f"pt100_report_{today}.pdf"
        save_path = filedialog.asksaveasfilename(
            defaultextension=".pdf",
            initialfile=default_name,
            filetypes=[("PDF Files", "*.pdf")],
        )
        if not save_path:
            return

        # Build plot PNG in temp dir.
        tmp_dir = tempfile.mkdtemp(prefix="pt100_report_")
        fig_png_path = os.path.join(tmp_dir, "plot.png")

        nodes = _node_ids_from_df(df)
        title = "PT100 Temperature Log Report"
        subtitle = f"Nodes: {nodes} — {summary}"

        fig_title = f"{_human_series_label(y_name)} vs Time"
        fig = _build_figure(
            df=df,
            time_column=self.loaded.time_column,
            y_name=y_name,
            overlay_raw_temp=self.overlay_raw.get(),
            smooth=self.smooth.get(),
            title=fig_title,
        )
        fig.savefig(fig_png_path, dpi=300)
        plt.close(fig)

        # Summary table data
        stats = _compute_basic_stats(df.get("cal_temp_c", pd.Series(dtype=float)))
        summary_rows = [
            ["Field", "Value"],
            ["Time source", "iso8601_local" if self.loaded.time_column == "__time" and "iso8601_local" in self.loaded.dataframe.columns else "epoch_utc"],
            ["Start (local, minute)", start_label],
            ["End (local, minute)", end_label],
            ["Rows plotted", f"{len(df):,}"],
            ["Dropped no-time rows", f"{self.loaded.dropped_no_time_rows:,}"],
            ["Primary series", _human_series_label(y_name)],
            ["Overlay raw_temp_c", "yes" if self.overlay_raw.get() else "no"],
            ["Smoothing", "yes" if self.smooth.get() else "no"],
            ["cal_temp_c min/avg/max/std", f"{stats['min']} / {stats['avg']} / {stats['max']} / {stats['std']}"],
        ]

        try:
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
