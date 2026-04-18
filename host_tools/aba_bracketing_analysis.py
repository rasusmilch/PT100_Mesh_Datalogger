#!/usr/bin/env python3
"""Analyze A-B-A bracketing runs from a CSV file.

This script groups records by run number, fits a linear model using the first and
last records in each run, evaluates that model at the middle record timestamp,
and reports the difference between the expected and measured middle record values.

Expected CSV columns:
    run,bore,timestamp,mean_temp_C,mean_ohm

Example row:
    1,A,2026-04-15T13:48:32Z,-49.105,80.445

Behavior:
    - Incomplete or malformed CSV rows are skipped with a warning.
    - Only complete alternating three-record runs are analyzed.
    - A complete alternating run must sort by timestamp into X-Y-X, where the
      first and last bore labels match and the middle bore label differs.
    - Invalid runs are skipped with a warning instead of aborting the program.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


@dataclass(frozen=True)
class BoreRecord:
    """Represents one bore measurement record from the CSV file.

    Attributes:
        run_id: Run identifier from the CSV.
        bore_label: Bore label, for example "A" or "B".
        timestamp_utc: Timestamp parsed as a UTC datetime.
        mean_temp_c: Mean temperature value from the CSV.
        mean_ohm: Mean resistance value from the CSV.
        source_row_number: Original CSV row number for diagnostics.
    """

    run_id: str
    bore_label: str
    timestamp_utc: datetime.datetime
    mean_temp_c: float
    mean_ohm: float
    source_row_number: int


@dataclass(frozen=True)
class RunAnalysisResult:
    """Holds the A-B-A bracketing analysis result for one run.

    Attributes:
        run_id: Run identifier.
        first_record: First record of the run.
        middle_record: Middle record of the run.
        last_record: Last record of the run.
        elapsed_seconds: Time between first and last record.
        middle_offset_seconds: Time from the first record to the middle record.
        expected_middle_temp_c: Temperature predicted at the middle timestamp.
        expected_middle_ohm: Resistance predicted at the middle timestamp.
        temp_difference_c: Measured minus expected temperature at the middle
            record.
        ohm_difference: Measured minus expected resistance at the middle record.
        temp_slope_c_per_hour: Linear temperature slope across the bracket.
        ohm_slope_per_hour: Linear resistance slope across the bracket.
    """

    run_id: str
    first_record: BoreRecord
    middle_record: BoreRecord
    last_record: BoreRecord
    elapsed_seconds: float
    middle_offset_seconds: float
    expected_middle_temp_c: float
    expected_middle_ohm: float
    temp_difference_c: float
    ohm_difference: float
    temp_slope_c_per_hour: float
    ohm_slope_per_hour: float


@dataclass(frozen=True)
class SummaryStatistics:
    """Stores summary statistics for a numeric series.

    Attributes:
        count: Number of values.
        mean: Arithmetic mean.
        standard_deviation: Population standard deviation if count > 1, else 0.
        minimum: Minimum value.
        maximum: Maximum value.
        median: Median value.
        mean_absolute_value: Mean absolute value.
        root_mean_square: Root mean square value.
    """

    count: int
    mean: float
    standard_deviation: float
    minimum: float
    maximum: float
    median: float
    mean_absolute_value: float
    root_mean_square: float


@dataclass(frozen=True)
class AggregateAnalysis:
    """Stores aggregate statistics across all analyzed runs.

    Attributes:
        temp_difference_stats_c: Summary statistics for middle temperature
            differences, using measured minus expected.
        ohm_difference_stats: Summary statistics for middle resistance
            differences, using measured minus expected.
        elapsed_time_stats_minutes: Summary statistics for first-to-last run
            durations in minutes.
        midpoint_offset_stats_minutes: Summary statistics for first-to-middle
            offsets in minutes.
        midpoint_fraction_stats: Summary statistics for the middle timestamp
            position expressed as a fraction of the total first-to-last span.
    """

    temp_difference_stats_c: SummaryStatistics
    ohm_difference_stats: SummaryStatistics
    elapsed_time_stats_minutes: SummaryStatistics
    midpoint_offset_stats_minutes: SummaryStatistics
    midpoint_fraction_stats: SummaryStatistics


def emit_warning(message: str) -> None:
    """Emit a non-fatal warning to stderr.

    Args:
        message: Warning text to print.
    """
    print(f"Warning: {message}", file=sys.stderr)


def parse_command_line_arguments() -> argparse.Namespace:
    """Parse command line arguments.

    Returns:
        Parsed argparse namespace.
    """
    argument_parser = argparse.ArgumentParser(
        description=(
            "Perform A-B-A bracketing analysis on a CSV file containing run, "
            "bore, timestamp, mean_temp_C, and mean_ohm columns. Incomplete "
            "rows and invalid runs are skipped with warnings."
        )
    )
    argument_parser.add_argument(
        "csv_path",
        type=Path,
        help="Path to the input CSV file.",
    )
    argument_parser.add_argument(
        "--output-csv",
        type=Path,
        default=None,
        help="Optional path to write per-run analysis results as CSV.",
    )
    return argument_parser.parse_args()


def parse_iso8601_utc(timestamp_text: str) -> datetime.datetime:
    """Parse an ISO-8601 timestamp with trailing Z into a UTC datetime.

    Args:
        timestamp_text: Timestamp text, for example "2026-04-17T19:34:16Z".

    Returns:
        Parsed timezone-aware UTC datetime.

    Raises:
        ValueError: If the timestamp cannot be parsed.
    """
    normalized_text = timestamp_text.strip()
    if normalized_text.endswith("Z"):
        normalized_text = normalized_text[:-1] + "+00:00"
    parsed_datetime = datetime.datetime.fromisoformat(normalized_text)
    if parsed_datetime.tzinfo is None:
        raise ValueError(f"Timestamp is missing timezone information: {timestamp_text}")
    return parsed_datetime.astimezone(datetime.timezone.utc)


def load_records_from_csv(csv_path: Path) -> List[BoreRecord]:
    """Load and validate records from the input CSV file.

    Incomplete or malformed rows are skipped with a warning.

    Args:
        csv_path: Path to the input CSV file.

    Returns:
        List of valid BoreRecord instances.

    Raises:
        ValueError: If required columns are missing or no valid rows are found.
    """
    required_columns = {"run", "bore", "timestamp", "mean_temp_C", "mean_ohm"}
    records: List[BoreRecord] = []

    with csv_path.open("r", encoding="utf-8", newline="") as csv_file:
        csv_reader = csv.DictReader(csv_file)
        if csv_reader.fieldnames is None:
            raise ValueError("CSV file is missing a header row.")

        missing_columns = required_columns - set(csv_reader.fieldnames)
        if missing_columns:
            missing_columns_text = ", ".join(sorted(missing_columns))
            raise ValueError(f"CSV file is missing required columns: {missing_columns_text}")

        for source_row_number, row in enumerate(csv_reader, start=2):
            run_id = str(row.get("run", "")).strip()
            bore_label = str(row.get("bore", "")).strip()
            timestamp_text = str(row.get("timestamp", "")).strip()
            mean_temp_text = str(row.get("mean_temp_C", "")).strip()
            mean_ohm_text = str(row.get("mean_ohm", "")).strip()

            if not run_id or not bore_label or not timestamp_text or not mean_temp_text or not mean_ohm_text:
                emit_warning(
                    f"Skipping row {source_row_number}: incomplete row values "
                    f"(run={run_id!r}, bore={bore_label!r}, timestamp={timestamp_text!r}, "
                    f"mean_temp_C={mean_temp_text!r}, mean_ohm={mean_ohm_text!r})."
                )
                continue

            try:
                timestamp_utc = parse_iso8601_utc(timestamp_text)
                mean_temp_c = float(mean_temp_text)
                mean_ohm = float(mean_ohm_text)
            except Exception as error:  # pylint: disable=broad-except
                emit_warning(f"Skipping row {source_row_number}: failed to parse row: {error}")
                continue

            records.append(
                BoreRecord(
                    run_id=run_id,
                    bore_label=bore_label,
                    timestamp_utc=timestamp_utc,
                    mean_temp_c=mean_temp_c,
                    mean_ohm=mean_ohm,
                    source_row_number=source_row_number,
                )
            )

    if not records:
        raise ValueError("No valid data rows were found in the CSV file.")

    return records


def group_records_by_run(records: Sequence[BoreRecord]) -> Dict[str, List[BoreRecord]]:
    """Group records by run identifier.

    Args:
        records: Input records.

    Returns:
        Dictionary mapping run identifiers to lists of records.
    """
    grouped_records: Dict[str, List[BoreRecord]] = {}
    for record in records:
        grouped_records.setdefault(record.run_id, []).append(record)
    return grouped_records


def compute_linear_interpolation(
    first_time_seconds: float,
    first_value: float,
    last_time_seconds: float,
    last_value: float,
    evaluation_time_seconds: float,
) -> Tuple[float, float]:
    """Compute a straight-line prediction and slope.

    Args:
        first_time_seconds: Time coordinate of the first point.
        first_value: Value at the first point.
        last_time_seconds: Time coordinate of the last point.
        last_value: Value at the last point.
        evaluation_time_seconds: Time coordinate where the model is evaluated.

    Returns:
        Tuple of (predicted_value, slope_per_second).

    Raises:
        ValueError: If the first and last time coordinates are equal.
    """
    if math.isclose(first_time_seconds, last_time_seconds):
        raise ValueError("Cannot fit a line when the first and last timestamps are identical.")

    slope_per_second = (last_value - first_value) / (last_time_seconds - first_time_seconds)
    predicted_value = first_value + slope_per_second * (evaluation_time_seconds - first_time_seconds)
    return predicted_value, slope_per_second


def get_sorted_run_records(run_records: Sequence[BoreRecord]) -> List[BoreRecord]:
    """Sort run records by timestamp.

    Args:
        run_records: Records belonging to a run.

    Returns:
        Time-sorted records.
    """
    return sorted(run_records, key=lambda record: record.timestamp_utc)


def validate_complete_alternating_run(run_id: str, sorted_records: Sequence[BoreRecord]) -> None:
    """Validate that a run is a complete alternating three-record bracket.

    A valid run must contain exactly three records that sort into X-Y-X by bore
    label, where the first and last labels match and the middle label differs.

    Args:
        run_id: Run identifier.
        sorted_records: Time-sorted records for the run.

    Raises:
        ValueError: If the run is incomplete or not alternating.
    """
    if len(sorted_records) != 3:
        raise ValueError(
            f"Skipping run {run_id}: expected exactly three records, found {len(sorted_records)}."
        )

    first_record, middle_record, last_record = sorted_records

    if first_record.bore_label != last_record.bore_label:
        raise ValueError(
            f"Skipping run {run_id}: first bore {first_record.bore_label!r} does not match "
            f"last bore {last_record.bore_label!r}."
        )

    if middle_record.bore_label == first_record.bore_label:
        raise ValueError(
            f"Skipping run {run_id}: middle bore {middle_record.bore_label!r} does not alternate "
            f"from outer bore {first_record.bore_label!r}."
        )


def analyze_single_run(run_id: str, run_records: Sequence[BoreRecord]) -> RunAnalysisResult:
    """Analyze one complete alternating three-record run.

    Args:
        run_id: Run identifier.
        run_records: Records belonging to the run.

    Returns:
        Per-run analysis result.

    Raises:
        ValueError: If the run does not meet the required shape.
    """
    sorted_records = get_sorted_run_records(run_records)
    validate_complete_alternating_run(run_id, sorted_records)

    first_record, middle_record, last_record = sorted_records

    first_time_seconds = first_record.timestamp_utc.timestamp()
    middle_time_seconds = middle_record.timestamp_utc.timestamp()
    last_time_seconds = last_record.timestamp_utc.timestamp()

    expected_middle_temp_c, temp_slope_per_second = compute_linear_interpolation(
        first_time_seconds=first_time_seconds,
        first_value=first_record.mean_temp_c,
        last_time_seconds=last_time_seconds,
        last_value=last_record.mean_temp_c,
        evaluation_time_seconds=middle_time_seconds,
    )
    expected_middle_ohm, ohm_slope_per_second = compute_linear_interpolation(
        first_time_seconds=first_time_seconds,
        first_value=first_record.mean_ohm,
        last_time_seconds=last_time_seconds,
        last_value=last_record.mean_ohm,
        evaluation_time_seconds=middle_time_seconds,
    )

    elapsed_seconds = last_time_seconds - first_time_seconds
    middle_offset_seconds = middle_time_seconds - first_time_seconds

    return RunAnalysisResult(
        run_id=run_id,
        first_record=first_record,
        middle_record=middle_record,
        last_record=last_record,
        elapsed_seconds=elapsed_seconds,
        middle_offset_seconds=middle_offset_seconds,
        expected_middle_temp_c=expected_middle_temp_c,
        expected_middle_ohm=expected_middle_ohm,
        temp_difference_c=middle_record.mean_temp_c - expected_middle_temp_c,
        ohm_difference=middle_record.mean_ohm - expected_middle_ohm,
        temp_slope_c_per_hour=temp_slope_per_second * 3600.0,
        ohm_slope_per_hour=ohm_slope_per_second * 3600.0,
    )


def summarize_numeric_values(values: Sequence[float]) -> SummaryStatistics:
    """Compute useful summary statistics for a numeric series.

    Args:
        values: Numeric values to summarize.

    Returns:
        Summary statistics.

    Raises:
        ValueError: If values is empty.
    """
    if not values:
        raise ValueError("Cannot summarize an empty value list.")

    mean_value = statistics.fmean(values)
    standard_deviation = statistics.pstdev(values) if len(values) > 1 else 0.0
    mean_absolute_value = statistics.fmean(abs(value) for value in values)
    root_mean_square = math.sqrt(statistics.fmean(value * value for value in values))

    return SummaryStatistics(
        count=len(values),
        mean=mean_value,
        standard_deviation=standard_deviation,
        minimum=min(values),
        maximum=max(values),
        median=statistics.median(values),
        mean_absolute_value=mean_absolute_value,
        root_mean_square=root_mean_square,
    )


def build_aggregate_analysis(results: Sequence[RunAnalysisResult]) -> AggregateAnalysis:
    """Build aggregate statistics across analyzed runs.

    Args:
        results: Per-run analysis results.

    Returns:
        Aggregate analysis object.
    """
    temp_differences_c = [result.temp_difference_c for result in results]
    ohm_differences = [result.ohm_difference for result in results]
    elapsed_minutes = [result.elapsed_seconds / 60.0 for result in results]
    midpoint_offset_minutes = [result.middle_offset_seconds / 60.0 for result in results]
    midpoint_fractions = [
        result.middle_offset_seconds / result.elapsed_seconds
        for result in results
        if not math.isclose(result.elapsed_seconds, 0.0)
    ]

    return AggregateAnalysis(
        temp_difference_stats_c=summarize_numeric_values(temp_differences_c),
        ohm_difference_stats=summarize_numeric_values(ohm_differences),
        elapsed_time_stats_minutes=summarize_numeric_values(elapsed_minutes),
        midpoint_offset_stats_minutes=summarize_numeric_values(midpoint_offset_minutes),
        midpoint_fraction_stats=summarize_numeric_values(midpoint_fractions),
    )


def format_summary_statistics(label: str, statistics_result: SummaryStatistics, unit: str) -> str:
    """Format summary statistics for console output.

    Args:
        label: Display label.
        statistics_result: Statistics to format.
        unit: Unit suffix.

    Returns:
        Formatted multi-line string.
    """
    return (
        f"{label}\n"
        f"  count:      {statistics_result.count}\n"
        f"  mean:       {statistics_result.mean:+.9f} {unit}\n"
        f"  std dev:    {statistics_result.standard_deviation:.9f} {unit}\n"
        f"  min:        {statistics_result.minimum:+.9f} {unit}\n"
        f"  max:        {statistics_result.maximum:+.9f} {unit}\n"
        f"  median:     {statistics_result.median:+.9f} {unit}\n"
        f"  mean |x|:   {statistics_result.mean_absolute_value:.9f} {unit}\n"
        f"  RMS:        {statistics_result.root_mean_square:.9f} {unit}"
    )


def write_per_run_results_csv(output_csv_path: Path, results: Sequence[RunAnalysisResult]) -> None:
    """Write per-run analysis results to a CSV file.

    Args:
        output_csv_path: Destination path.
        results: Per-run analysis results.
    """
    field_names = [
        "run",
        "first_bore",
        "middle_bore",
        "last_bore",
        "first_timestamp_utc",
        "middle_timestamp_utc",
        "last_timestamp_utc",
        "elapsed_minutes",
        "middle_offset_minutes",
        "middle_fraction_of_total",
        "first_mean_temp_c",
        "middle_mean_temp_c",
        "last_mean_temp_c",
        "expected_middle_temp_c",
        "temp_difference_c_measured_minus_expected",
        "temp_slope_c_per_hour",
        "first_mean_ohm",
        "middle_mean_ohm",
        "last_mean_ohm",
        "expected_middle_ohm",
        "ohm_difference_measured_minus_expected",
        "ohm_slope_per_hour",
    ]

    with output_csv_path.open("w", encoding="utf-8", newline="") as output_csv_file:
        csv_writer = csv.DictWriter(output_csv_file, fieldnames=field_names)
        csv_writer.writeheader()
        for result in results:
            middle_fraction_of_total = (
                result.middle_offset_seconds / result.elapsed_seconds
                if not math.isclose(result.elapsed_seconds, 0.0)
                else float("nan")
            )
            csv_writer.writerow(
                {
                    "run": result.run_id,
                    "first_bore": result.first_record.bore_label,
                    "middle_bore": result.middle_record.bore_label,
                    "last_bore": result.last_record.bore_label,
                    "first_timestamp_utc": result.first_record.timestamp_utc.isoformat().replace("+00:00", "Z"),
                    "middle_timestamp_utc": result.middle_record.timestamp_utc.isoformat().replace("+00:00", "Z"),
                    "last_timestamp_utc": result.last_record.timestamp_utc.isoformat().replace("+00:00", "Z"),
                    "elapsed_minutes": f"{result.elapsed_seconds / 60.0:.9f}",
                    "middle_offset_minutes": f"{result.middle_offset_seconds / 60.0:.9f}",
                    "middle_fraction_of_total": f"{middle_fraction_of_total:.9f}",
                    "first_mean_temp_c": f"{result.first_record.mean_temp_c:.9f}",
                    "middle_mean_temp_c": f"{result.middle_record.mean_temp_c:.9f}",
                    "last_mean_temp_c": f"{result.last_record.mean_temp_c:.9f}",
                    "expected_middle_temp_c": f"{result.expected_middle_temp_c:.9f}",
                    "temp_difference_c_measured_minus_expected": f"{result.temp_difference_c:.9f}",
                    "temp_slope_c_per_hour": f"{result.temp_slope_c_per_hour:.9f}",
                    "first_mean_ohm": f"{result.first_record.mean_ohm:.9f}",
                    "middle_mean_ohm": f"{result.middle_record.mean_ohm:.9f}",
                    "last_mean_ohm": f"{result.last_record.mean_ohm:.9f}",
                    "expected_middle_ohm": f"{result.expected_middle_ohm:.9f}",
                    "ohm_difference_measured_minus_expected": f"{result.ohm_difference:.9f}",
                    "ohm_slope_per_hour": f"{result.ohm_slope_per_hour:.9f}",
                }
            )


def print_per_run_table(results: Sequence[RunAnalysisResult]) -> None:
    """Print a concise per-run table to stdout.

    Args:
        results: Per-run analysis results.
    """
    header_fields = [
        "run",
        "first",
        "middle",
        "last",
        "elapsed_min",
        "mid_frac",
        "temp_diff_C",
        "ohm_diff",
    ]
    print("\nPer-run results")
    print("-" * 110)
    print(
        f"{header_fields[0]:>8}  {header_fields[1]:>6}  {header_fields[2]:>6}  {header_fields[3]:>6}  "
        f"{header_fields[4]:>12}  {header_fields[5]:>8}  {header_fields[6]:>14}  {header_fields[7]:>14}"
    )
    print("-" * 110)

    for result in results:
        middle_fraction = (
            result.middle_offset_seconds / result.elapsed_seconds
            if not math.isclose(result.elapsed_seconds, 0.0)
            else float("nan")
        )
        print(
            f"{result.run_id:>8}  "
            f"{result.first_record.bore_label:>6}  "
            f"{result.middle_record.bore_label:>6}  "
            f"{result.last_record.bore_label:>6}  "
            f"{result.elapsed_seconds / 60.0:12.3f}  "
            f"{middle_fraction:8.4f}  "
            f"{result.temp_difference_c:+14.9f}  "
            f"{result.ohm_difference:+14.9f}"
        )


def main() -> int:
    """Run the command line program.

    Returns:
        Process exit code.
    """
    arguments = parse_command_line_arguments()

    try:
        records = load_records_from_csv(arguments.csv_path)
        grouped_records = group_records_by_run(records)

        analysis_results: List[RunAnalysisResult] = []
        for run_id in sorted(grouped_records, key=lambda value: str(value)):
            try:
                analysis_results.append(
                    analyze_single_run(
                        run_id=run_id,
                        run_records=grouped_records[run_id],
                    )
                )
            except ValueError as error:
                emit_warning(str(error))

        print(f"Input file: {arguments.csv_path}")

        if not analysis_results:
            emit_warning("No complete alternating three-record runs were found after filtering.")
            print("Runs analyzed: 0")
            return 0

        aggregate_analysis = build_aggregate_analysis(analysis_results)

        print(f"Runs analyzed: {len(analysis_results)}")
        print_per_run_table(analysis_results)
        print()
        print(format_summary_statistics(
            "Temperature difference at middle record (measured - expected)",
            aggregate_analysis.temp_difference_stats_c,
            "C",
        ))
        print()
        print(format_summary_statistics(
            "Resistance difference at middle record (measured - expected)",
            aggregate_analysis.ohm_difference_stats,
            "ohm",
        ))
        print()
        print(format_summary_statistics(
            "Total run span (first to last)",
            aggregate_analysis.elapsed_time_stats_minutes,
            "min",
        ))
        print()
        print(format_summary_statistics(
            "First-to-middle offset",
            aggregate_analysis.midpoint_offset_stats_minutes,
            "min",
        ))
        print()
        print(format_summary_statistics(
            "Middle timestamp fraction of total span",
            aggregate_analysis.midpoint_fraction_stats,
            "fraction",
        ))

        if arguments.output_csv is not None:
            write_per_run_results_csv(arguments.output_csv, analysis_results)
            print(f"\nPer-run CSV written to: {arguments.output_csv}")

        return 0

    except Exception as error:  # pylint: disable=broad-except
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
