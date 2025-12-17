#!/usr/bin/env python3
"""
Read JSON lines from the mesh root over serial and store into SQLite.

Expected lines (one per sample):
{"type":"temp","node":"AA:BB:CC:DD:EE:FF","ts":1700000000,"temp_c":12.345,"raw_c":12.300,"r_ohm":104.567,"seq":123}
"""

from __future__ import annotations

import argparse
import json
import sqlite3
import sys
import time
from pathlib import Path
from typing import Any, Dict

import serial  # pip install pyserial


CREATE_SQL = """
CREATE TABLE IF NOT EXISTS temp_samples (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  node_id TEXT NOT NULL,
  ts_epoch INTEGER NOT NULL,
  temp_c REAL,
  raw_c REAL,
  r_ohm REAL,
  seq INTEGER,
  received_epoch INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_temp_samples_node_ts
  ON temp_samples(node_id, ts_epoch);
"""


def init_db(db_path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(str(db_path))
    connection.executescript(CREATE_SQL)
    connection.commit()
    return connection


def insert_sample(connection: sqlite3.Connection, sample: Dict[str, Any]) -> None:
    connection.execute(
        "INSERT INTO temp_samples(node_id, ts_epoch, temp_c, raw_c, r_ohm, seq, received_epoch) "
        "VALUES(?,?,?,?,?,?,?)",
        (
            sample.get("node", ""),
            int(sample.get("ts", 0)),
            float(sample.get("temp_c", "nan")),
            float(sample.get("raw_c", "nan")),
            float(sample.get("r_ohm", "nan")),
            int(sample.get("seq", 0)),
            int(time.time()),
        ),
    )
    connection.commit()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--db", type=Path, default=Path("pt100_mesh.sqlite3"))
    parser.add_argument("--echo", action="store_true", help="Print decoded samples to stdout")
    args = parser.parse_args()

    connection = init_db(args.db)

    with serial.Serial(args.port, args.baud, timeout=1) as serial_port:
        print(f"Listening on {args.port} @ {args.baud} baud. DB={args.db}")
        while True:
            line = serial_port.readline()
            if not line:
                continue
            try:
                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                # Ignore non-JSON lines.
                if not text.startswith("{"):
                    continue
                sample = json.loads(text)
                if sample.get("type") != "temp":
                    continue
                insert_sample(connection, sample)
                if args.echo:
                    print(sample)
            except KeyboardInterrupt:
                break
            except Exception as exc:
                print(f"parse/store error: {exc}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
