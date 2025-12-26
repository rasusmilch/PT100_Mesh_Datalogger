#!/usr/bin/env python3
"""
PT100 Mesh Logger CSV Plotter

- Loads one or more PT100 CSV export files.
- Plots selected channels (cal_temp_c, raw_temp_c, raw_rtd_ohms).
- Optional trimming by start/end time (minute resolution) using timestamps present in the log.
- X-axis labels are rendered WITHOUT seconds.

Designed in the same "select CSV -> plot -> save" spirit as your hydrostatic pressure
report generator script. fileciteturn1file0

Requirements:
  - Python 3.9+
  - pandas
  - matplotlib
  - tkinter (usually bundled on Windows/macOS; on some Linux distros install python3-tk)

Run:
  python pt100_csv_plotter.py
"""

from __future__ import annotations

import io
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import pandas as pd
import matplotlib.pyplot as plt

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox
except Exception as exc:  # pragma: no cover
    raise RuntimeError(
        "tkinter is required for the GUI. On Linux, try installing python3-tk."
    ) from exc


DEFAULT_TIME_INPUT_HINT = "YYYY-MM-DD HH:MM (local)"


@dataclass(frozen=True)
class LoadedLog:
    dataframe: pd.DataFrame
    time_column: str
    time_is_local: bool
    tzinfo: Optional[timezone]
    source_files: List[str]
    dropped_rows_no_time: int


def _safe_parse_user_time(user_text: str) -> Optional[datetime]:
    """Parse user-entered time. Accepts 'YYYY-MM-DD HH:MM' or ISO-like variants."""
    cleaned = (user_text or "").strip()
    if not cleaned:
        return None

    # Accept "YYYY-MM-DD HH:MM" or "YYYY-MM-DDTHH:MM"
    cleaned = cleaned.replace("T", " ").replace("/", "-")
    # If seconds are present, keep them; otherwise it's minute resolution.
    # fromisoformat accepts "YYYY-MM-DD HH:MM[:SS[.ffffff]]"
    try:
        return datetime.fromisoformat(cleaned)
    except ValueError:
        return None


def _pick_time_source(df: pd.DataFrame) -> Tuple[str, bool, Optional[timezone]]:
    """
    Choose primary time source:
      1) iso8601_local (preferred): tz-aware if offset present in string.
      2) epoch_utc: interpreted as seconds since epoch UTC, tz-aware UTC.
    Returns: (time_column_name, time_is_local, tzinfo)
    """
    if "iso8601_local" in df.columns:
        parsed = pd.to_datetime(df["iso8601_local"], errors="coerce", utc=False)
        if parsed.notna().any():
            df["__time"] = parsed
            tzinfo = None
            # If tz-aware, pandas dtype is DatetimeTZDtype; get tzinfo from first value.
            first_valid = df["__time"].dropna().iloc[0]
            if hasattr(first_valid, "tzinfo") and first_valid.tzinfo is not None:
                # Convert to a fixed offset tzinfo, since your strings carry an offset like "-06:00".
                # (This avoids pulling in pytz/dateutil timezone databases.)
                tzinfo = first_valid.tzinfo  # type: ignore[assignment]
            return "__time", True, tzinfo

    if "epoch_utc" in df.columns:
        # Some rows can have epoch_utc==0 before time is valid. Coerce those to NaT.
        epoch_series = pd.to_numeric(df["epoch_utc"], errors="coerce")
        epoch_series = epoch_series.where(epoch_series > 0)
        parsed = pd.to_datetime(epoch_series, unit="s", errors="coerce", utc=True)
        if parsed.notna().any():
            df["__time"] = parsed
            return "__time", False, timezone.utc

    raise ValueError("No usable timestamps found (iso8601_local and epoch_utc are empty).")


def _load_pt100_csv_files(file_paths: Sequence[str]) -> LoadedLog:
    if not file_paths:
        raise ValueError("No files selected.")

    dataframes: List[pd.DataFrame] = []
    for file_path in file_paths:
        df = pd.read_csv(file_path)
        dataframes.append(df)

    df_all = pd.concat(dataframes, ignore_index=True, sort=False)

    # Normalize expected columns (don't fail hard if extras exist).
    expected = [
        "schema_ver",
        "seq",
        "epoch_utc",
        "iso8601_local",
        "raw_rtd_ohms",
        "raw_temp_c",
        "cal_temp_c",
        "flags",
        "node_id",
    ]
    for column_name in expected:
        if column_name not in df_all.columns:
            # Keep going; plotter will only require a time column and at least one y column.
            df_all[column_name] = pd.NA

    time_column, time_is_local, tzinfo = _pick_time_source(df_all)

    dropped_rows_no_time = int(df_all[time_column].isna().sum())
    df_all = df_all[df_all[time_column].notna()].copy()

    # Sort by time then sequence (if available).
    sort_columns = [time_column]
    if "seq" in df_all.columns:
        sort_columns.append("seq")
    df_all.sort_values(sort_columns, inplace=True)

    return LoadedLog(
        dataframe=df_all,
        time_column=time_column,
        time_is_local=time_is_local,
        tzinfo=tzinfo,
        source_files=list(file_paths),
        dropped_rows_no_time=dropped_rows_no_time,
    )


def _minute_floor_series(time_series: pd.Series) -> pd.Series:
    # Pandas floor to minute.
    return pd.to_datetime(time_series).dt.floor("min")


def _validate_and_trim_by_minute(
    df: pd.DataFrame,
    time_column: str,
    tzinfo: Optional[timezone],
    start_text: str,
    end_text: str,
) -> Tuple[pd.DataFrame, Optional[datetime], Optional[datetime], str]:
    """
    Trim data by start/end, both matched to a minute bucket that exists in the log.
    If user supplies times, they must parse and their floored minute must be present in the log.
    """
    start_dt_naive = _safe_parse_user_time(start_text)
    end_dt_naive = _safe_parse_user_time(end_text)

    time_series = pd.to_datetime(df[time_column])
    time_minutes = _minute_floor_series(time_series)

    # Build set of valid minute buckets (tz-aware matches tz-aware; naive matches naive).
    valid_minutes = pd.Series(time_minutes.unique()).dropna().sort_values()
    if valid_minutes.empty:
        raise ValueError("No valid timestamps to trim against.")

    # Determine tz-awareness of data minutes by inspecting the first value.
    first_minute = valid_minutes.iloc[0]
    data_is_tz_aware = hasattr(first_minute, "tzinfo") and first_minute.tzinfo is not None

    def attach_tz_if_needed(dt_naive: datetime) -> datetime:
        if data_is_tz_aware:
            # Interpret user input as local wall time in the log's offset.
            # If tzinfo is missing (rare), fall back to the first timestamp tzinfo.
            chosen_tz = tzinfo or first_minute.tzinfo  # type: ignore[union-attr]
            return dt_naive.replace(tzinfo=chosen_tz)
        return dt_naive  # keep naive

    start_dt = attach_tz_if_needed(start_dt_naive) if start_dt_naive else None
    end_dt = attach_tz_if_needed(end_dt_naive) if end_dt_naive else None

    # Floor user selections to minute.
    start_minute = start_dt.replace(second=0, microsecond=0) if start_dt else None
    end_minute = end_dt.replace(second=0, microsecond=0) if end_dt else None

    # Validate membership.
    valid_set = set(valid_minutes.tolist())

    def nearest_minute_string(target_minute: datetime) -> str:
        # Find nearest available minute for debugging.
        deltas = valid_minutes.apply(lambda v: abs((v - target_minute).total_seconds()))
        nearest_idx = int(deltas.idxmin())
        nearest = valid_minutes.loc[nearest_idx]
        return str(nearest)

    if start_minute and start_minute not in valid_set:
        raise ValueError(
            f"Start time {start_minute} is not present in the log minute buckets.\n"
            f"Valid range: {valid_minutes.iloc[0]} .. {valid_minutes.iloc[-1]}\n"
            f"Nearest minute: {nearest_minute_string(start_minute)}"
        )
    if end_minute and end_minute not in valid_set:
        raise ValueError(
            f"End time {end_minute} is not present in the log minute buckets.\n"
            f"Valid range: {valid_minutes.iloc[0]} .. {valid_minutes.iloc[-1]}\n"
            f"Nearest minute: {nearest_minute_string(end_minute)}"
        )
    if start_minute and end_minute and start_minute > end_minute:
        raise ValueError("Start time must be <= end time.")

    # Apply trim (end is inclusive of its minute bucket).
    trimmed = df.copy()
    if start_minute:
        trimmed = trimmed[time_series >= start_minute]
    if end_minute:
        trimmed = trimmed[time_series < (end_minute + timedelta(minutes=1))]

    summary = "No trimming applied."
    if start_minute or end_minute:
        summary = f"Trimmed to: start={start_minute or '(none)'} end={end_minute or '(none)'}"

    return trimmed, start_minute, end_minute, summary


class PlotterApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PT100 CSV Plotter")

        self.selected_files: List[str] = []
        self.loaded: Optional[LoadedLog] = None

        self.start_time_text = tk.StringVar(value="")
        self.end_time_text = tk.StringVar(value="")

        self.y_choice = tk.StringVar(value="cal_temp_c")
        self.overlay_raw = tk.BooleanVar(value=True)
        self.smooth = tk.BooleanVar(value=False)

        self._build_ui()

    def _build_ui(self) -> None:
        frm = tk.Frame(self.root, padx=10, pady=10)
        frm.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # File selection row.
        tk.Button(frm, text="Select CSV File(s)", command=self.select_files).grid(
            row=0, column=0, sticky="w"
        )
        self.file_label = tk.Label(frm, text="No files selected")
        self.file_label.grid(row=0, column=1, sticky="w", padx=10)

        # Time trimming controls.
        tk.Label(frm, text=f"Start time ({DEFAULT_TIME_INPUT_HINT}):").grid(
            row=1, column=0, sticky="e", pady=(10, 0)
        )
        tk.Entry(frm, textvariable=self.start_time_text, width=28).grid(
            row=1, column=1, sticky="w", pady=(10, 0)
        )

        tk.Label(frm, text=f"End time ({DEFAULT_TIME_INPUT_HINT}):").grid(
            row=2, column=0, sticky="e"
        )
        tk.Entry(frm, textvariable=self.end_time_text, width=28).grid(
            row=2, column=1, sticky="w"
        )

        self.range_label = tk.Label(frm, text="Load a file to see valid time range.")
        self.range_label.grid(row=3, column=0, columnspan=2, sticky="w", pady=(5, 10))

        # Plot options.
        tk.Label(frm, text="Y-axis:").grid(row=4, column=0, sticky="e")
        tk.OptionMenu(frm, self.y_choice, "cal_temp_c", "raw_temp_c", "raw_rtd_ohms").grid(
            row=4, column=1, sticky="w"
        )

        tk.Checkbutton(frm, text="Overlay raw_temp_c", variable=self.overlay_raw).grid(
            row=5, column=1, sticky="w"
        )
        tk.Checkbutton(frm, text="Smooth (rolling mean)", variable=self.smooth).grid(
            row=6, column=1, sticky="w"
        )

        # Action buttons.
        self.plot_btn = tk.Button(frm, text="Plot", command=self.plot, state=tk.DISABLED)
        self.plot_btn.grid(row=7, column=0, sticky="w", pady=(15, 0))

        self.save_trim_btn = tk.Button(
            frm, text="Save Trimmed CSV", command=self.save_trimmed_csv, state=tk.DISABLED
        )
        self.save_trim_btn.grid(row=7, column=1, sticky="w", pady=(15, 0))

        # Notes.
        self.note_label = tk.Label(
            frm,
            text="Note: Time trimming matches minute buckets present in the log.\n"
                 "X-axis labels are formatted without seconds.",
            justify="left",
        )
        self.note_label.grid(row=8, column=0, columnspan=2, sticky="w", pady=(10, 0))

    def select_files(self) -> None:
        file_paths = filedialog.askopenfilenames(
            title="Select PT100 CSV file(s)",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
        )
        if not file_paths:
            return

        self.selected_files = list(file_paths)
        self.file_label.configure(text=f"{len(self.selected_files)} file(s) selected")
        try:
            self.loaded = _load_pt100_csv_files(self.selected_files)
        except Exception as exc:
            self.loaded = None
            messagebox.showerror("Load Error", str(exc))
            self.plot_btn.configure(state=tk.DISABLED)
            self.save_trim_btn.configure(state=tk.DISABLED)
            return

        df = self.loaded.dataframe
        time_series = pd.to_datetime(df[self.loaded.time_column])
        t0 = time_series.min()
        t1 = time_series.max()
        dropped = self.loaded.dropped_rows_no_time

        tz_note = "local" if self.loaded.time_is_local else "UTC"
        self.range_label.configure(
            text=f"Valid time range ({tz_note}): {t0} .. {t1}   | "
                 f"Rows dropped (no timestamp): {dropped}"
        )
        self.plot_btn.configure(state=tk.NORMAL)
        self.save_trim_btn.configure(state=tk.NORMAL)

    def _get_trimmed_df(self) -> Tuple[pd.DataFrame, str]:
        if not self.loaded:
            raise RuntimeError("No data loaded.")

        trimmed, _, _, summary = _validate_and_trim_by_minute(
            df=self.loaded.dataframe,
            time_column=self.loaded.time_column,
            tzinfo=self.loaded.tzinfo,
            start_text=self.start_time_text.get(),
            end_text=self.end_time_text.get(),
        )
        if trimmed.empty:
            raise ValueError("Trimming resulted in 0 rows.")
        return trimmed, summary

    def save_trimmed_csv(self) -> None:
        try:
            trimmed, summary = self._get_trimmed_df()
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        default_name = "pt100_trimmed.csv"
        save_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV Files", "*.csv")],
        )
        if not save_path:
            return

        trimmed.to_csv(save_path, index=False)
        messagebox.showinfo("Saved", f"Saved trimmed CSV.\n{summary}\n\n{save_path}")

    def plot(self) -> None:
        if not self.loaded:
            messagebox.showerror("Plot Error", "No data loaded.")
            return

        try:
            df, summary = self._get_trimmed_df()
        except Exception as exc:
            messagebox.showerror("Trim Error", str(exc))
            return

        time_series = pd.to_datetime(df[self.loaded.time_column])

        y_name = self.y_choice.get()
        if y_name not in df.columns:
            messagebox.showerror("Plot Error", f"Column not found: {y_name}")
            return

        y_primary = pd.to_numeric(df[y_name], errors="coerce")

        plt.figure(figsize=(12, 6))

        if self.smooth.get():
            sample_count = int(y_primary.shape[0])
            smoothing_window = max(5, min(151, sample_count // 40))  # similar spirit to your hydro plotter
            y_plot = y_primary.rolling(window=smoothing_window, center=True, min_periods=1).mean()
            plt.plot(time_series, y_primary, linewidth=0.7, alpha=0.6)
            plt.plot(time_series, y_plot, linewidth=2.0)
        else:
            plt.plot(time_series, y_primary, linewidth=1.2)

        if self.overlay_raw.get() and "raw_temp_c" in df.columns and y_name != "raw_temp_c":
            y_raw = pd.to_numeric(df["raw_temp_c"], errors="coerce")
            plt.plot(time_series, y_raw, linewidth=0.8, alpha=0.7)

        plt.title(f"PT100 log: {y_name} ({summary})")
        plt.xlabel("Time")
        plt.ylabel(y_name)

        ax = plt.gca()

        # Format without seconds.
        import matplotlib.dates as mdates
        locator = mdates.AutoDateLocator(minticks=5, maxticks=12)
        formatter = mdates.DateFormatter("%Y-%m-%d %H:%M")
        ax.xaxis.set_major_locator(locator)
        ax.xaxis.set_major_formatter(formatter)

        plt.xticks(rotation=45, ha="right")
        plt.tight_layout()
        plt.show()


def main() -> None:
    root = tk.Tk()
    app = PlotterApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
