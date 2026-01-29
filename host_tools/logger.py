import argparse
import ctypes
import os
import re
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import serial
from serial.tools import list_ports

LOG_LEVEL_COLORS = {
    "E": "\x1b[31m",
    "W": "\x1b[33m",
    "I": "\x1b[32m",
    "D": "\x1b[36m",
    "V": "\x1b[90m",
}
ANSI_RESET = "\x1b[0m"
LOG_PATTERN = re.compile(r"^([EWDIV]) \(\d+\)")
ANSI_ESCAPE_PATTERN = re.compile(r"\x1b\[")


@dataclass
class ReconnectConfig:
    enabled: bool
    initial_seconds: float
    max_seconds: float
    backoff: float


@dataclass
class ResetConfig:
    enabled: bool
    delay_seconds: float
    method: str
    sequence: Optional[str]


@dataclass
class BufferConfig:
    buffer_lines: int
    flush_interval_seconds: float
    fsync: bool


class SerialSessionLogger:
    def __init__(
        self,
        port: str,
        baud: int,
        output_base: str,
        reconnect: ReconnectConfig,
        reset: ResetConfig,
        buffer_config: BufferConfig,
        auto_color: bool,
        vid: Optional[int],
        pid: Optional[int],
        serial_number: Optional[str],
    ):
        self.port = port
        self.baud = baud
        self.output_base = output_base
        self.reconnect = reconnect
        self.reset = reset
        self.buffer_config = buffer_config
        self.auto_color = auto_color
        self.vid = vid
        self.pid = pid
        self.serial_number = serial_number

        self._stop_requested = False
        self._txt_file = None
        self._log_buffer = []
        self._last_flush_time = time.monotonic()
        self._warned_no_color = False
        self._color_enabled = auto_color
        self._connected_once = False

    def run(self) -> int:
        self._setup_color()
        self._setup_signal_handlers()
        txt_filename = self._open_output_file()
        self._write_marker(
            f"=== LOGGER STARTED {self._timestamp()} output={txt_filename} ==="
        )

        backoff_seconds = self.reconnect.initial_seconds

        while not self._stop_requested:
            port = self._resolve_port()
            if not port:
                message = "No serial port found matching criteria"
                if not self.reconnect.enabled:
                    self._write_marker(f"=== LOGGER STOPPED {self._timestamp()} error={message} ===")
                    self._flush_buffer()
                    return 1
                self._write_marker(f"=== DISCONNECTED {self._timestamp()} error={message} ===")
                self._flush_buffer()
                time.sleep(backoff_seconds)
                backoff_seconds = min(
                    backoff_seconds * self.reconnect.backoff, self.reconnect.max_seconds
                )
                continue

            try:
                with serial.Serial(port, self.baud, timeout=1) as ser:
                    self._handle_connected(port, ser)
                    backoff_seconds = self.reconnect.initial_seconds
                    self._read_loop(ser)
            except (serial.SerialException, OSError, UnicodeDecodeError) as exc:
                self._write_marker(
                    f"=== DISCONNECTED {self._timestamp()} error={exc} ==="
                )
                self._flush_buffer()
                if not self.reconnect.enabled:
                    self._write_marker(
                        f"=== LOGGER STOPPED {self._timestamp()} error={exc} ==="
                    )
                    self._flush_buffer()
                    return 1
                time.sleep(backoff_seconds)
                backoff_seconds = min(
                    backoff_seconds * self.reconnect.backoff, self.reconnect.max_seconds
                )
            except Exception as exc:
                self._write_marker(
                    f"=== DISCONNECTED {self._timestamp()} error={exc} ==="
                )
                self._flush_buffer()
                self._write_marker(
                    f"=== LOGGER STOPPED {self._timestamp()} error={exc} ==="
                )
                self._flush_buffer()
                raise

        self._write_marker(f"=== LOGGER STOPPED {self._timestamp()} ===")
        self._flush_buffer()
        return 0

    def _setup_signal_handlers(self) -> None:
        signal.signal(signal.SIGINT, self._handle_stop_signal)
        signal.signal(signal.SIGTERM, self._handle_stop_signal)

    def _handle_stop_signal(self, sig, frame) -> None:
        self._stop_requested = True

    def _setup_color(self) -> None:
        if not self.auto_color:
            self._color_enabled = False
            return
        if os.name != "nt":
            return
        try:
            handle = ctypes.windll.kernel32.GetStdHandle(-11)
            mode = ctypes.c_uint32()
            if ctypes.windll.kernel32.GetConsoleMode(handle, ctypes.byref(mode)) == 0:
                raise OSError("Unable to get console mode")
            if ctypes.windll.kernel32.SetConsoleMode(
                handle, mode.value | 0x0004
            ) == 0:
                raise OSError("Unable to enable virtual terminal processing")
        except Exception as exc:
            self._color_enabled = False
            if not self._warned_no_color:
                print(
                    f"Color disabled: failed to enable Windows ANSI support ({exc})"
                )
                self._warned_no_color = True

    def _open_output_file(self) -> str:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        txt_filename = f"{self.output_base}_{timestamp}.txt"
        self._txt_file = open(txt_filename, "w", encoding="utf-8")
        print(f"Logging to {txt_filename}")
        return txt_filename

    def _resolve_port(self) -> Optional[str]:
        if self.port != "auto":
            return self.port
        matches = []
        for port_info in list_ports.comports():
            if self.vid is not None and port_info.vid != self.vid:
                continue
            if self.pid is not None and port_info.pid != self.pid:
                continue
            if self.serial_number is not None and (
                port_info.serial_number != self.serial_number
            ):
                continue
            matches.append(port_info.device)
        if not matches:
            return None
        if len(matches) > 1:
            print(f"Multiple ports matched {matches}, using {matches[0]}")
        return matches[0]

    def _handle_connected(self, port: str, ser: serial.Serial) -> None:
        if self._connected_once:
            self._write_marker(
                f"=== RECONNECTED {self._timestamp()} port={port} ==="
            )
        else:
            self._write_marker(f"=== CONNECTED {self._timestamp()} port={port} ===")
            self._connected_once = True
        if self.reset.enabled:
            self._reset_target(ser)
            time.sleep(self.reset.delay_seconds)

    def _reset_target(self, ser: serial.Serial) -> None:
        if self.reset.sequence:
            self._apply_reset_sequence(ser, self.reset.sequence)
            return
        if self.reset.method == "rts":
            ser.rts = True
            time.sleep(0.1)
            ser.rts = False
        elif self.reset.method == "dtr_rts":
            ser.dtr = False
            ser.rts = True
            time.sleep(0.1)
            ser.rts = False
            ser.dtr = True
        else:
            raise ValueError(f"Unsupported reset method: {self.reset.method}")

    def _apply_reset_sequence(self, ser: serial.Serial, sequence: str) -> None:
        steps = [step.strip() for step in sequence.split(",") if step.strip()]
        for step in steps:
            if step.startswith("wait="):
                time.sleep(float(step.split("=", 1)[1]))
                continue
            if ":" in step:
                action, wait_value = step.split(":", 1)
                wait_seconds = float(wait_value)
            else:
                action = step
                wait_seconds = 0.0
            if "=" not in action:
                raise ValueError(
                    "Reset sequence steps must be like rts=1 or dtr=0, optionally with :seconds"
                )
            line, value = action.split("=", 1)
            level = value.strip() in {"1", "true", "True"}
            line = line.strip().lower()
            if line == "rts":
                ser.rts = level
            elif line == "dtr":
                ser.dtr = level
            else:
                raise ValueError(f"Unknown reset line: {line}")
            if wait_seconds:
                time.sleep(wait_seconds)

    def _read_loop(self, ser: serial.Serial) -> None:
        while not self._stop_requested:
            line = ser.readline()
            if not line:
                continue
            try:
                decoded = line.decode().strip()
            except UnicodeDecodeError:
                raise
            if not decoded:
                continue
            timestamp = self._timestamp()
            formatted = f"<{timestamp}> {decoded}"
            self._write_console_line(decoded, timestamp)
            self._log_buffer.append(formatted)
            self._flush_if_needed()

    def _write_console_line(self, decoded: str, timestamp: str) -> None:
        prefix = f"<{timestamp}> "
        if self._color_enabled and not ANSI_ESCAPE_PATTERN.search(decoded):
            match = LOG_PATTERN.match(decoded)
            if match:
                color = LOG_LEVEL_COLORS.get(match.group(1))
                if color:
                    print(f"{prefix}{color}{decoded}{ANSI_RESET}")
                    return
        print(f"{prefix}{decoded}")

    def _write_marker(self, text: str) -> None:
        print(text)
        if self._txt_file:
            self._txt_file.write(text + "\n")

    def _flush_if_needed(self) -> None:
        now = time.monotonic()
        if len(self._log_buffer) >= self.buffer_config.buffer_lines:
            self._flush_buffer()
        elif now - self._last_flush_time >= self.buffer_config.flush_interval_seconds:
            self._flush_buffer()

    def _flush_buffer(self) -> None:
        if not self._txt_file or not self._log_buffer:
            self._last_flush_time = time.monotonic()
            return
        for line in self._log_buffer:
            self._txt_file.write(line + "\n")
        self._log_buffer.clear()
        self._txt_file.flush()
        if self.buffer_config.fsync:
            os.fsync(self._txt_file.fileno())
        self._last_flush_time = time.monotonic()

    def _timestamp(self) -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _build_parser() -> argparse.ArgumentParser:
    description = """Serial logger with reconnect, reset, and IDF-style color support."""
    epilog = """
Reconnect behavior:
  When --reconnect is enabled (default), the logger retries opening the port
  forever using exponential backoff controlled by:
    --reconnect-initial-seconds, --reconnect-backoff, --reconnect-max-seconds

Reset behavior:
  Reset is enabled by default. Use --no-reset to skip toggling RTS/DTR.
  --reset-method selects a simple reset (rts or dtr_rts). You can provide a
  custom --reset-sequence for complex boards. Sequence syntax examples:
    rts=1:0.1,rts=0:0.1
    dtr=0:0.05,rts=1:0.1,rts=0:0.05,dtr=1:0.05
    wait=0.2,rts=1:0.1

Color behavior:
  Auto-coloring is enabled by default. Use --disable-auto-color to disable.

Buffering and durability:
  Lines are buffered in memory before being written to disk. Use --fsync to
  force a disk flush on each buffer flush.

Examples:
  python host_tools/logger.py --port auto --vid 0x10C4 --pid 0xEA60 -o testlog
  python host_tools/logger.py --port COM3 --no-reset -o testlog
  python host_tools/logger.py --port /dev/ttyUSB0 --disable-auto-color -o testlog
  python host_tools/logger.py --port /dev/ttyUSB0 --fsync -o testlog
"""
    parser = argparse.ArgumentParser(
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog,
    )
    parser.add_argument("--port", required=True, help="Serial port or 'auto'.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate.")
    parser.add_argument("--vid", type=lambda x: int(x, 0), help="USB VID (hex or dec).")
    parser.add_argument("--pid", type=lambda x: int(x, 0), help="USB PID (hex or dec).")
    parser.add_argument(
        "--serial-number",
        dest="serial_number",
        help="USB serial number for auto port selection.",
    )
    parser.add_argument(
        "--reconnect",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable or disable reconnect attempts (default: enabled).",
    )
    parser.add_argument(
        "--reconnect-initial-seconds",
        type=float,
        default=1.0,
        help="Initial reconnect delay in seconds.",
    )
    parser.add_argument(
        "--reconnect-max-seconds",
        type=float,
        default=15.0,
        help="Maximum reconnect delay in seconds.",
    )
    parser.add_argument(
        "--reconnect-backoff",
        type=float,
        default=1.5,
        help="Reconnect backoff multiplier.",
    )
    parser.add_argument(
        "--no-reset",
        dest="no_reset",
        action="store_true",
        help="Disable reset on connect (default: reset enabled).",
    )
    parser.add_argument(
        "--reset-delay-seconds",
        type=float,
        default=0.2,
        help="Delay after reset before reading.",
    )
    parser.add_argument(
        "--reset-method",
        choices=["rts", "dtr_rts"],
        default="rts",
        help="Reset method when reset is enabled.",
    )
    parser.add_argument(
        "--reset-sequence",
        help="Custom reset sequence (overrides --reset-method).",
    )
    parser.add_argument(
        "--disable-auto-color",
        action="store_true",
        help="Disable IDF-style auto-color output.",
    )
    parser.add_argument(
        "--buffer-lines",
        type=int,
        default=600,
        help="Number of lines to buffer before flush.",
    )
    parser.add_argument(
        "--flush-interval-seconds",
        type=float,
        default=600,
        help="Maximum time between buffer flushes.",
    )
    parser.add_argument(
        "--fsync",
        action="store_true",
        help="Call os.fsync() after flushing buffer.",
    )
    parser.add_argument(
        "-o",
        "--output",
        required=True,
        help="Base name for output files (no extension).",
    )
    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    reconnect = ReconnectConfig(
        enabled=args.reconnect,
        initial_seconds=args.reconnect_initial_seconds,
        max_seconds=args.reconnect_max_seconds,
        backoff=args.reconnect_backoff,
    )
    reset = ResetConfig(
        enabled=not args.no_reset,
        delay_seconds=args.reset_delay_seconds,
        method=args.reset_method,
        sequence=args.reset_sequence,
    )
    buffer_config = BufferConfig(
        buffer_lines=args.buffer_lines,
        flush_interval_seconds=args.flush_interval_seconds,
        fsync=args.fsync,
    )

    logger = SerialSessionLogger(
        port=args.port,
        baud=args.baud,
        output_base=args.output,
        reconnect=reconnect,
        reset=reset,
        buffer_config=buffer_config,
        auto_color=not args.disable_auto_color,
        vid=args.vid,
        pid=args.pid,
        serial_number=args.serial_number,
    )
    return logger.run()


if __name__ == "__main__":
    sys.exit(main())
