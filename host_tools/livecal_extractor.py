#!/usr/bin/env python3
"""Interactive parser for PT100 `cal livecal` console lines.

This script reads pasted `cal livecal:` lines from standard input and prints a
CSV fragment in the form:

    ,,timestamp,mean_temp_C,mean_ohm,std_temp_C,std_ohm,drift_C_per_min

Example input:
    cal livecal: ts=2026-04-17T20:32:38Z n=120 raw_ohm=118.838 std_ohm=0.006 mean_ohm=118.840 cal_temp=48.428C std_temp=0.016C mean_temp=48.433C drift=0.003C/min delta=0.005C fault=none (0x00)

Example output:
    2026-04-17T20:32:38Z,48.433,118.840,0.016,0.006,0.003

The user can prepend run and bore manually in the CSV file.
"""

from __future__ import annotations

import re
import sys


LIVE_CAL_PATTERN = re.compile(
    r"""
    ts=(?P<timestamp>\S+)
    .*?
    std_ohm=(?P<std_ohm>-?\d+(?:\.\d+)?)
    .*?
    mean_ohm=(?P<mean_ohm>-?\d+(?:\.\d+)?)
    .*?
    std_temp=(?P<std_temp>-?\d+(?:\.\d+)?)C
    .*?
    mean_temp=(?P<mean_temp>-?\d+(?:\.\d+)?)C
    .*?
    drift=(?P<drift>-?\d+(?:\.\d+)?)C/min
    """,
    re.VERBOSE,
)


def parse_livecal_line(livecal_line: str) -> str:
    """Parses one `cal livecal:` line into a CSV fragment.

    Args:
      livecal_line: Raw console line containing `cal livecal:` data.

    Returns:
      A CSV string in the form:
      timestamp,mean_temp_C,mean_ohm,std_temp_C,std_ohm,drift_C_per_min

    Raises:
      ValueError: If the line does not match the expected format.
    """
    match = LIVE_CAL_PATTERN.search(livecal_line.strip())
    if not match:
        raise ValueError("Input line did not match expected `cal livecal:` format.")

    timestamp = match.group("timestamp")
    mean_temp_c = match.group("mean_temp")
    mean_ohm = match.group("mean_ohm")
    std_temp_c = match.group("std_temp")
    std_ohm = match.group("std_ohm")
    drift_c_per_min = match.group("drift")

    return (
        f",,"
        f"{timestamp},"
        f"{mean_temp_c},"
        f"{mean_ohm},"
        f"{std_temp_c},"
        f"{std_ohm},"
        f"{drift_c_per_min}"
    )


def main() -> int:
    """Runs the interactive prompt until the user exits."""
    print("Paste `cal livecal:` lines one at a time.")
    print("Type 'quit' or 'exit' to stop.")
    print()

    while True:
        try:
            user_input = input("> ").strip()
        except EOFError:
            print()
            return 0
        except KeyboardInterrupt:
            print()
            return 0

        if not user_input:
            continue

        if user_input.lower() in {"quit", "exit"}:
            return 0

        try:
            csv_fragment = parse_livecal_line(user_input)
            print(csv_fragment)
        except ValueError as error:
            print(f"Error: {error}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())