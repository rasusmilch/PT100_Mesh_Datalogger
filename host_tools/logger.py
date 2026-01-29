import argparse
import ctypes
import os
import queue
import re
import signal
import sys
import threading
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


@dataclass
class InputConfig:
    interactive: Optional[bool]
    input_eol: str
    input_encoding: str
    local_echo: bool
    input_log: bool
    escape_prefix: str


class StdinLineReader(threading.Thread):
    """Reads stdin lines into a queue for interactive serial input."""

    def __init__(self, stop_event: threading.Event, line_queue: queue.Queue[str]):
        super().__init__(daemon=True)
        self._stop_event = stop_event
        self._line_queue = line_queue

    def run(self) -> None:
        while not self._stop_event.is_set():
            line = sys.stdin.readline()
            if line == "":
                break
            cleaned = line.rstrip("\r\n")
            self._line_queue.put(cleaned)


class SerialSessionLogger:
    def __init__(
        self,
        port: str,
        baud: int,
        output_base: str,
        reconnect: ReconnectConfig,
        reset: ResetConfig,
        buffer_config: BufferConfig,
        input_config: InputConfig,
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
        self.input_config = input_config
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
        self._interactive_enabled = False
        self._input_queue: Optional[queue.Queue[str]] = None
        self._input_stop_event: Optional[threading.Event] = None
        self._input_thread: Optional[StdinLineReader] = None

    def run(self) -> int:
        self._setup_color()
        self._setup_signal_handlers()
        self._setup_input_reader()
        txt_filename = self._open_output_file()
        self._write_marker(
            f"=== LOGGER STARTED {self._timestamp()} output={txt_filename} ==="
        )

        backoff_seconds = self.reconnect.initial_seconds

        try:
            while not self._stop_requested:
                port = self._resolve_port()
                if not port:
                    message = "No serial port found matching criteria"
                    if not self.reconnect.enabled:
                        self._write_marker(
                            f"=== LOGGER STOPPED {self._timestamp()} error={message} ==="
                        )
                        self._flush_buffer()
                        return 1
                    self._write_marker(
                        f"=== DISCONNECTED {self._timestamp()} error={message} ==="
                    )
                    self._flush_buffer()
                    time.sleep(backoff_seconds)
                    backoff_seconds = min(
                        backoff_seconds * self.reconnect.backoff,
                        self.reconnect.max_seconds,
                    )
                    continue

                try:
                    with serial.Serial(port, self.baud, timeout=0.1) as ser:
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
                        backoff_seconds * self.reconnect.backoff,
                        self.reconnect.max_seconds,
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
        finally:
            self._shutdown_input_reader()

        self._write_marker(f"=== LOGGER STOPPED {self._timestamp()} ===")
        self._flush_buffer()
        return 0

    def _setup_input_reader(self) -> None:
        interactive = self.input_config.interactive
        if interactive is None:
            interactive = sys.stdin.isatty() and sys.stdout.isatty()
        self._interactive_enabled = bool(interactive)
        if not self._interactive_enabled:
            return
        self._input_queue = queue.Queue()
        self._input_stop_event = threading.Event()
        self._input_thread = StdinLineReader(self._input_stop_event, self._input_queue)
        self._input_thread.start()

    def _shutdown_input_reader(self) -> None:
        if not self._input_stop_event:
            return
        self._input_stop_event.set()

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
            self._drain_input_queue(ser)
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

    def _drain_input_queue(self, ser: serial.Serial) -> None:
        if not self._interactive_enabled or not self._input_queue:
            return
        max_per_tick = 50
        for _ in range(max_per_tick):
            try:
                user_line = self._input_queue.get_nowait()
            except queue.Empty:
                break
            self._send_user_line(ser, user_line)

    def _send_user_line(self, ser: serial.Serial, user_line: str) -> None:
        """Send a line from stdin to the serial device or handle escape commands."""
        if self._handle_escape_command(ser, user_line):
            return
        payload = self._encode_user_line(user_line)
        if payload is None:
            return
        ser.write(payload)
        ser.flush()
        if self.input_config.local_echo:
            self._print_local_echo(user_line)
        if self.input_config.input_log:
            self._write_marker(self._format_tx_marker(user_line), print_console=False)

    def _encode_user_line(self, user_line: str) -> Optional[bytes]:
        """Encode the user line with configured encoding and EOL settings."""
        eol_bytes = self._eol_bytes()
        try:
            encoded = user_line.encode(self.input_config.input_encoding)
        except UnicodeEncodeError as exc:
            print(f"TX encode error: {exc}")
            return None
        return encoded + eol_bytes

    def _eol_bytes(self) -> bytes:
        eol = self.input_config.input_eol
        if eol == "crlf":
            return b"\r\n"
        if eol == "cr":
            return b"\r"
        return b"\n"

    def _handle_escape_command(self, ser: serial.Serial, command_line: str) -> bool:
        """Handle escape-prefixed commands. Returns True if consumed."""
        prefix = self.input_config.escape_prefix
        if not command_line.startswith(prefix):
            return False
        command = command_line[len(prefix) :].strip()
        if command in {".", "quit"}:
            self._stop_requested = True
            return True
        if command == "help":
            self._print_escape_help(prefix)
            return True
        if command == "reset":
            try:
                self._reset_target(ser)
                time.sleep(self.reset.delay_seconds)
            except Exception as exc:
                print(f"Reset failed: {exc}")
            return True
        if command == "reconnect":
            ser.close()
            raise serial.SerialException("User requested reconnect")
        print(f"Unknown escape command: {command_line!r}")
        return True

    def _print_escape_help(self, prefix: str) -> None:
        """Print quick help for escape commands."""
        print("Escape commands:")
        print(f"  {prefix}. or {prefix}quit    Quit logger")
        print(f"  {prefix}reset              Reset target now")
        print(f"  {prefix}reconnect          Force reconnect")
        print(f"  {prefix}help               Show this help")

    def _format_tx_marker(self, user_line: str) -> str:
        return f">>> <{self._timestamp()}> {user_line}"

    def _print_local_echo(self, user_line: str) -> None:
        timestamp = self._timestamp()
        message = f">>> <{timestamp}> {user_line}"
        if self._color_enabled:
            print(f"\x1b[90m{message}{ANSI_RESET}")
        else:
            print(message)

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

    def _write_marker(self, text: str, print_console: bool = True) -> None:
        if print_console:
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
        "--interactive",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Enable interactive stdin-to-serial input (default: auto TTY).",
    )
    parser.add_argument(
        "--input-eol",
        choices=["lf", "crlf", "cr"],
        default="lf",
        help="End-of-line to append for interactive input.",
    )
    parser.add_argument(
        "--input-encoding",
        default="utf-8",
        help="Encoding for interactive input.",
    )
    parser.add_argument(
        "--local-echo",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Echo sent lines locally (default: enabled).",
    )
    parser.add_argument(
        "--input-log",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Log sent lines to the output file (default: enabled).",
    )
    parser.add_argument(
        "--escape-prefix",
        default="~",
        help="Prefix for interactive escape commands.",
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
    input_config = InputConfig(
        interactive=args.interactive,
        input_eol=args.input_eol,
        input_encoding=args.input_encoding,
        local_echo=args.local_echo,
        input_log=args.input_log,
        escape_prefix=args.escape_prefix,
    )

    logger = SerialSessionLogger(
        port=args.port,
        baud=args.baud,
        output_base=args.output,
        reconnect=reconnect,
        reset=reset,
        buffer_config=buffer_config,
        input_config=input_config,
        auto_color=not args.disable_auto_color,
        vid=args.vid,
        pid=args.pid,
        serial_number=args.serial_number,
    )
    return logger.run()


if __name__ == "__main__":
    sys.exit(main())
