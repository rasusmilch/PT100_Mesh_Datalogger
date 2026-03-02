#!/usr/bin/env python3
"""
host_tools/logger.py

Resilient serial logger/monitor with optional ESP32 reset on connect and optional
two-way interactive input (stdin -> serial).

Key behaviors:
- Keeps running across unplug/replug, target reboot, or transient serial errors.
- Optional reset-on-connect (default) or --no-reset (like ESP-IDF monitor).
- Colorized output similar to ESP-IDF monitor (optional).
- Does NOT clear the serial input buffer on open/reopen.
"""

from __future__ import annotations

import argparse
import os
import queue
import re
import signal
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional, Sequence, Tuple

import serial
from serial.tools import list_ports


@dataclass(frozen=True)
class SerialConfig:
    """Serial port configuration."""
    port: str
    baudrate: int
    bytesize: int = serial.EIGHTBITS
    parity: str = serial.PARITY_NONE
    stopbits: float = serial.STOPBITS_ONE
    read_timeout_sec: float = 0.1
    write_timeout_sec: float = 2.0
    xonxoff: bool = False
    rtscts: bool = False
    dsrdtr: bool = False


@dataclass(frozen=True)
class OutputConfig:
    """Output formatting and logging configuration."""
    output_path: str
    append: bool
    host_timestamps: bool
    decode_encoding: str
    decode_errors: str
    flush_lines: int
    flush_interval_sec: float
    fsync_on_flush: bool
    print_to_stdout: bool


@dataclass(frozen=True)
class ReconnectConfig:
    """Reconnect/backoff configuration."""
    reconnect: bool
    reconnect_delay_sec: float
    port_scan_delay_sec: float


@dataclass(frozen=True)
class ResetConfig:
    """Reset configuration."""
    enabled: bool
    mode: str
    pulse_sec: float
    post_delay_sec: float
    custom_sequence: str


@dataclass(frozen=True)
class ColorConfig:
    """Color output configuration."""
    enabled: bool


@dataclass(frozen=True)
class InteractiveConfig:
    """Two-way communication configuration."""
    enabled: bool
    input_eol: str
    input_encoding: str
    input_errors: str
    local_echo: bool
    input_log: bool


class Ansi:
    """Minimal ANSI color helpers."""
    RESET = "\x1b[0m"
    BOLD = "\x1b[1m"
    DIM = "\x1b[2m"

    RED = "\x1b[31m"
    GREEN = "\x1b[32m"
    YELLOW = "\x1b[33m"
    MAGENTA = "\x1b[35m"
    CYAN = "\x1b[36m"


def _maybe_enable_windows_ansi() -> None:
    """Best-effort enable ANSI colors on Windows consoles."""
    if os.name != "nt":
        return
    try:
        import colorama  # type: ignore

        colorama.just_fix_windows_console()
    except Exception:
        return


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _local_now_iso() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _is_tty(stream) -> bool:
    try:
        return stream.isatty()
    except Exception:
        return False


def _format_host_prefix(include_timestamp: bool) -> str:
    if not include_timestamp:
        return ""
    return f"[{_local_now_iso()}] "


def _parse_reset_sequence(sequence: str) -> Sequence[Tuple[str, str]]:
    """Parse 'R0|D1|W0.5' into a list of (opcode, arg)."""
    if not sequence.strip():
        return []
    steps: list[Tuple[str, str]] = []
    for raw_part in sequence.split("|"):
        part = raw_part.strip()
        if not part:
            continue
        opcode = part[0].upper()
        arg = part[1:].strip()
        if opcode not in ("R", "D", "U", "W"):
            raise ValueError(f"Invalid reset opcode '{opcode}' in '{part}'")
        if opcode in ("R", "D") and arg not in ("0", "1"):
            raise ValueError(f"Invalid reset arg for '{opcode}': '{arg}' (expected 0 or 1)")
        if opcode == "U" and not re.fullmatch(r"[01],[01]", arg):
            raise ValueError(f"Invalid reset arg for 'U': '{arg}' (expected 0,0 / 0,1 / 1,0 / 1,1)")
        if opcode == "W":
            float(arg)  # validate
        steps.append((opcode, arg))
    return steps


def _set_control_lines(ser: serial.Serial, dtr: Optional[bool], rts: Optional[bool]) -> None:
    """Set DTR/RTS control lines (best-effort)."""
    if dtr is not None:
        ser.dtr = dtr
    if rts is not None:
        ser.rts = rts


def _apply_custom_reset_sequence(ser: serial.Serial, sequence: str, log_fn) -> None:
    """Apply custom reset sequence like esp-idf-monitor/esptool."""
    steps = _parse_reset_sequence(sequence)
    for opcode, arg in steps:
        if opcode == "D":
            value = (arg == "1")
            log_fn(f"reset: DTR={'ASSERT' if value else 'DEASSERT'}")
            _set_control_lines(ser, dtr=value, rts=None)
        elif opcode == "R":
            value = (arg == "1")
            log_fn(f"reset: RTS={'ASSERT' if value else 'DEASSERT'}")
            _set_control_lines(ser, dtr=None, rts=value)
        elif opcode == "U":
            dtr_str, rts_str = arg.split(",")
            dtr_val = (dtr_str == "1")
            rts_val = (rts_str == "1")
            log_fn(f"reset: DTR={'ASSERT' if dtr_val else 'DEASSERT'} RTS={'ASSERT' if rts_val else 'DEASSERT'}")
            _set_control_lines(ser, dtr=dtr_val, rts=rts_val)
        elif opcode == "W":
            seconds = float(arg)
            log_fn(f"reset: wait {seconds:.3f}s")
            time.sleep(seconds)


def _esp32_run_reset(ser: serial.Serial, pulse_sec: float, post_delay_sec: float, log_fn) -> None:
    """
    Reset ESP32 into normal run mode.

    Critical detail: many ESP32 dev boards include circuitry where asserting BOTH DTR and RTS
    together won't reset. Also, DTR/RTS are typically active-low on these boards. :contentReference[oaicite:5]{index=5}
    So we deassert DTR first (GPIO0 high), then pulse RTS (EN).
    """
    try:
        _set_control_lines(ser, dtr=False, rts=False)
        time.sleep(0.05)
        log_fn("reset: ESP32 run reset (pulse RTS/EN with DTR deasserted)")
        _set_control_lines(ser, dtr=False, rts=True)
        time.sleep(pulse_sec)
        _set_control_lines(ser, dtr=False, rts=False)
        time.sleep(post_delay_sec)
    except Exception as exc:
        log_fn(f"reset: failed: {exc}")


def _esp32_bootloader_reset(ser: serial.Serial, log_fn) -> None:
    """Reset ESP32 into ROM bootloader: hold GPIO0 low while toggling EN."""
    try:
        _set_control_lines(ser, dtr=True, rts=False)
        time.sleep(0.05)
        log_fn("reset: ESP32 bootloader reset (GPIO0 low + pulse EN)")
        _set_control_lines(ser, dtr=True, rts=True)
        time.sleep(0.10)
        _set_control_lines(ser, dtr=True, rts=False)
        time.sleep(0.05)
        _set_control_lines(ser, dtr=False, rts=False)
        time.sleep(0.10)
    except Exception as exc:
        log_fn(f"reset: failed: {exc}")


class SessionLogger:
    """Runs the reconnecting serial logger/monitor."""

    def __init__(
        self,
        serial_config: SerialConfig,
        output_config: OutputConfig,
        reconnect_config: ReconnectConfig,
        reset_config: ResetConfig,
        color_config: ColorConfig,
        interactive_config: InteractiveConfig,
        port_regex: str,
        port_vid_pid: str,
        port_serial_number: str,
    ) -> None:
        self._serial_config = serial_config
        self._output_config = output_config
        self._reconnect_config = reconnect_config
        self._reset_config = reset_config
        self._color_config = color_config
        self._interactive_config = interactive_config

        self._port_regex = re.compile(port_regex) if port_regex else None
        self._port_vid_pid = port_vid_pid
        self._port_serial_number = port_serial_number

        self._stop_event = threading.Event()
        self._input_queue: "queue.Queue[bytes]" = queue.Queue()
        self._log_file_lock = threading.Lock()
        self._last_flush_time = time.monotonic()
        self._lines_since_flush = 0

        self._output_file = self._open_output_file()
        self._stdin_thread: Optional[threading.Thread] = None
        self._flush_thread: Optional[threading.Thread] = None
        self._close_lock = threading.Lock()
        self._closed = False

    def stop(self) -> None:
        self._stop_event.set()

    def _open_output_file(self):
        resolved_path = self._resolve_output_path(self._output_config.output_path)
        mode = "a" if self._output_config.append else "w"
        os.makedirs(os.path.dirname(resolved_path) or ".", exist_ok=True)
        return open(resolved_path, mode, encoding="utf-8", errors="replace", buffering=1)

    @staticmethod
    def _resolve_output_path(output_arg: str) -> str:
        # If user gives a directory, create a timestamped file inside it.
        if output_arg.endswith(os.sep) or (os.path.isdir(output_arg) and not os.path.isfile(output_arg)):
            os.makedirs(output_arg, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            return os.path.join(output_arg, f"serial_{stamp}.log")

        # If it looks like a "base name" without extension, append timestamp.
        base, ext = os.path.splitext(output_arg)
        if ext == "":
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            return f"{output_arg}_{stamp}.log"

        return output_arg

    def _log_meta(self, message: str) -> None:
        prefix = _format_host_prefix(include_timestamp=True)
        line = f"{prefix}{message}\n"
        with self._log_file_lock:
            self._output_file.write(line)
        self._lines_since_flush += 1
        self._maybe_flush_due()
        if self._output_config.print_to_stdout:
            sys.stdout.write(line)
            sys.stdout.flush()

    def _format_with_color(self, line: str) -> str:
        if not self._color_config.enabled:
            return line

        match = re.match(r"^(\[[0-9:\-\s]+\]\s+)?([EWDIV])\s+\(\d+\)\s+", line)
        if match:
            level = match.group(2)
            color = {
                "E": Ansi.RED,
                "W": Ansi.YELLOW,
                "I": Ansi.GREEN,
                "D": Ansi.CYAN,
                "V": Ansi.MAGENTA,
            }.get(level, Ansi.RESET)
            return f"{color}{line}{Ansi.RESET}"

        if "rst:" in line and "boot:" in line:
            return f"{Ansi.BOLD}{line}{Ansi.RESET}"

        return line

    def _write_output_line(self, raw_line: str) -> None:
        stripped = raw_line.rstrip("\r\n")
        host_prefix = _format_host_prefix(self._output_config.host_timestamps)
        file_line = f"{host_prefix}{stripped}\n"

        with self._log_file_lock:
            self._output_file.write(file_line)

        self._lines_since_flush += 1
        self._maybe_flush_due()

        if self._output_config.print_to_stdout:
            display_line = f"{host_prefix}{stripped}"
            sys.stdout.write(self._format_with_color(display_line) + "\n")
            sys.stdout.flush()

    def _flush(self) -> None:
        with self._log_file_lock:
            if self._closed:
                return
            self._output_file.flush()
            if self._output_config.fsync_on_flush:
                os.fsync(self._output_file.fileno())
        self._lines_since_flush = 0
        self._last_flush_time = time.monotonic()

    def _maybe_flush_due(self) -> None:
        now = time.monotonic()
        flush_due_to_lines = self._output_config.flush_lines > 0 and self._lines_since_flush >= self._output_config.flush_lines
        flush_due_to_time = self._output_config.flush_interval_sec > 0 and (now - self._last_flush_time) >= self._output_config.flush_interval_sec
        if flush_due_to_lines or flush_due_to_time:
            self._flush()

    def _start_stdin_thread_if_enabled(self) -> None:
        if not self._interactive_config.enabled or self._stdin_thread is not None:
            return

        def reader() -> None:
            try:
                while not self._stop_event.is_set():
                    user_text = sys.stdin.readline()
                    if user_text == "":
                        return
                    user_text = user_text.rstrip("\r\n")
                    if not user_text:
                        continue

                    # Local commands start with "!" (handled by main loop):
                    #   !reset         -> run reset
                    #   !bootloader    -> bootloader reset
                    payload = (user_text + "\n").encode("utf-8", errors="replace") if user_text.startswith("!") else (
                        (user_text + self._interactive_config.input_eol).encode(
                            self._interactive_config.input_encoding,
                            errors=self._interactive_config.input_errors,
                        )
                    )
                    self._input_queue.put(payload)

                    if self._interactive_config.local_echo:
                        sys.stdout.write(f"{Ansi.DIM}>> {user_text}{Ansi.RESET}\n")
                        sys.stdout.flush()

                    if self._interactive_config.input_log:
                        self._log_meta(f"stdin: {user_text}")
            except BaseException as exc:
                sys.stderr.write(f"logger: stdin-reader failed: {exc}\n")
                sys.stderr.flush()
                self.stop()
                try:
                    self._flush()
                except Exception:
                    pass

        self._stdin_thread = threading.Thread(target=reader, name="stdin-reader", daemon=True)
        self._stdin_thread.start()

    def _start_flush_thread_if_needed(self) -> None:
        if self._flush_thread is not None:
            return
        if self._output_config.flush_lines <= 0 and self._output_config.flush_interval_sec <= 0:
            return

        def flush_timer() -> None:
            try:
                while not self._stop_event.is_set():
                    if self._output_config.flush_interval_sec > 0 and self._lines_since_flush > 0:
                        if (time.monotonic() - self._last_flush_time) >= self._output_config.flush_interval_sec:
                            self._flush()
                    self._stop_event.wait(0.5)
            except BaseException as exc:
                sys.stderr.write(f"logger: flush-timer failed: {exc}\n")
                sys.stderr.flush()
                self.stop()
                try:
                    self._flush()
                except Exception:
                    pass

        self._flush_thread = threading.Thread(target=flush_timer, name="flush-timer", daemon=True)
        self._flush_thread.start()

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self.stop()

            if self._stdin_thread is not None and self._stdin_thread.is_alive():
                self._stdin_thread.join(timeout=1.0)
            if self._flush_thread is not None and self._flush_thread.is_alive():
                self._flush_thread.join(timeout=1.0)

            try:
                self._flush()
            finally:
                with self._log_file_lock:
                    self._output_file.close()
                    self._closed = True

    def _resolve_port(self) -> Optional[str]:
        if self._serial_config.port.lower() != "auto":
            return self._serial_config.port

        ports = list(list_ports.comports())
        if not ports:
            return None

        def matches(port_info) -> bool:
            device_name = port_info.device or ""
            description = port_info.description or ""
            hwid = port_info.hwid or ""
            serial_number = getattr(port_info, "serial_number", None) or ""

            if self._port_regex:
                if not (self._port_regex.search(device_name) or self._port_regex.search(description) or self._port_regex.search(hwid)):
                    return False

            if self._port_serial_number and self._port_serial_number != serial_number:
                return False

            if self._port_vid_pid:
                vid_pid_norm = self._port_vid_pid.lower().replace("0x", "")
                if ":" not in vid_pid_norm:
                    return False
                expected_vid_str, expected_pid_str = vid_pid_norm.split(":", 1)
                expected_vid = int(expected_vid_str, 16)
                expected_pid = int(expected_pid_str, 16)
                if port_info.vid != expected_vid or port_info.pid != expected_pid:
                    return False

            return True

        for port_info in ports:
            if matches(port_info):
                return port_info.device

        return ports[0].device

    def _open_serial(self, port: str) -> serial.Serial:
        # Open the serial port while minimizing unintended DTR/RTS reset pulses.
        # Create without opening so we can set cached DTR/RTS state first.
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = self._serial_config.baudrate
        ser.bytesize = self._serial_config.bytesize
        ser.parity = self._serial_config.parity
        ser.stopbits = self._serial_config.stopbits
        ser.timeout = self._serial_config.read_timeout_sec
        ser.write_timeout = self._serial_config.write_timeout_sec
        ser.xonxoff = self._serial_config.xonxoff
        ser.rtscts = self._serial_config.rtscts
        ser.dsrdtr = self._serial_config.dsrdtr

        # Pre-set control lines BEFORE opening to avoid the common "assert on open"
        # behavior that can reset ESP32 boards via auto-reset circuitry.
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass

        ser.open()

        # After open, avoid extra toggling when --no-reset is requested.
        # If reset is enabled, we still normalize to a known state.
        if self._reset_config.enabled:
            try:
                _set_control_lines(ser, dtr=False, rts=False)
            except Exception:
                pass

        return ser

    def _perform_reset_if_enabled(self, ser: serial.Serial) -> None:
        if not self._reset_config.enabled:
            return

        mode = self._reset_config.mode
        if mode == "esp32":
            _esp32_run_reset(
                ser,
                pulse_sec=self._reset_config.pulse_sec,
                post_delay_sec=self._reset_config.post_delay_sec,
                log_fn=self._log_meta,
            )
            return
        if mode == "esp32-bootloader":
            _esp32_bootloader_reset(ser, log_fn=self._log_meta)
            return
        if mode == "custom":
            _apply_custom_reset_sequence(ser, self._reset_config.custom_sequence, self._log_meta)
            return
        if mode == "rts":
            self._log_meta("reset: pulse RTS")
            _set_control_lines(ser, dtr=None, rts=True)
            time.sleep(self._reset_config.pulse_sec)
            _set_control_lines(ser, dtr=None, rts=False)
            time.sleep(self._reset_config.post_delay_sec)
            return
        if mode == "dtr":
            self._log_meta("reset: pulse DTR")
            _set_control_lines(ser, dtr=True, rts=None)
            time.sleep(self._reset_config.pulse_sec)
            _set_control_lines(ser, dtr=False, rts=None)
            time.sleep(self._reset_config.post_delay_sec)
            return

        self._log_meta(f"reset: unknown mode '{mode}', skipping")

    def _send_pending_input(self, ser: serial.Serial) -> None:
        while True:
            try:
                payload = self._input_queue.get_nowait()
            except queue.Empty:
                return

            if payload.startswith(b"!"):
                command_text = payload.decode("utf-8", errors="replace").strip()
                if command_text == "!reset":
                    self._log_meta("local: !reset")
                    _esp32_run_reset(ser, pulse_sec=self._reset_config.pulse_sec, post_delay_sec=self._reset_config.post_delay_sec, log_fn=self._log_meta)
                elif command_text == "!bootloader":
                    self._log_meta("local: !bootloader")
                    _esp32_bootloader_reset(ser, log_fn=self._log_meta)
                else:
                    self._log_meta(f"local: unknown command '{command_text}'")
                continue

            try:
                ser.write(payload)
                ser.flush()
            except Exception as exc:
                self._log_meta(f"stdin: send failed: {exc}")

    def run(self) -> int:
        self._start_stdin_thread_if_enabled()
        self._start_flush_thread_if_needed()
        self._log_meta(f"logger: started utc={_utc_now_iso()}")

        try:
            while not self._stop_event.is_set():
                port = self._resolve_port()
                if not port:
                    self._log_meta("logger: no serial ports found; waiting...")
                    if not self._reconnect_config.reconnect:
                        return 2
                    time.sleep(self._reconnect_config.port_scan_delay_sec)
                    continue

                self._log_meta(f"logger: opening {port} @ {self._serial_config.baudrate}")
                try:
                    with self._open_serial(port) as ser:
                        self._log_meta("logger: connected")
                        self._perform_reset_if_enabled(ser)

                        while not self._stop_event.is_set():
                            if self._interactive_config.enabled:
                                self._send_pending_input(ser)

                            raw = ser.readline()
                            if not raw:
                                continue

                            text = raw.decode(self._output_config.decode_encoding, errors=self._output_config.decode_errors)
                            self._write_output_line(text)

                except (serial.SerialException, OSError) as exc:
                    self._log_meta(f"logger: disconnected: {exc}")
                    if not self._reconnect_config.reconnect:
                        self._log_meta("logger: reconnect disabled; exiting")
                        return 1
                    time.sleep(self._reconnect_config.reconnect_delay_sec)

            self._log_meta("logger: stopping")
            return 0
        finally:
            self.close()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Resilient serial logger/monitor (reconnect + optional reset + optional stdin->serial).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    serial_group = parser.add_argument_group("Serial port")
    serial_group.add_argument("-p", "--port", default="auto", help="COM3, /dev/ttyUSB0, or 'auto' to scan.")
    serial_group.add_argument("-b", "--baudrate", type=int, default=115200, help="Baud rate.")
    serial_group.add_argument("--port-regex", default="", help="When --port=auto, prefer ports matching this regex.")
    serial_group.add_argument("--port-vid-pid", default="", help="When --port=auto, prefer VID:PID (e.g. 10C4:EA60).")
    serial_group.add_argument("--port-serial-number", default="", help="When --port=auto, prefer this USB-serial serial number.")
    serial_group.add_argument("--read-timeout-sec", type=float, default=0.1, help="Read timeout; smaller improves interactive responsiveness.")

    output_group = parser.add_argument_group("Output")
    output_group.add_argument("-o", "--output", default="logs/", help="Output path: directory, filename, or base name.")
    output_group.add_argument("--append", action="store_true", help="Append instead of overwrite.")
    output_group.add_argument("--host-timestamps", action="store_true", help="Prefix each line with host local timestamp.")
    output_group.add_argument("--no-stdout", action="store_true", help="Do not print to stdout.")
    output_group.add_argument("--encoding", default="utf-8", help="Decode serial bytes using this encoding.")
    output_group.add_argument("--errors", default="replace", choices=["strict", "replace", "ignore"], help="Decode error behavior.")
    output_group.add_argument("--flush-lines", type=int, default=0, help="Flush to disk after N lines (<=0 disables line-based flushing).")
    output_group.add_argument("--flush-interval-sec", type=float, default=600.0, help="Flush to disk after N seconds (<=0 disables time-based flushing).")
    fsync_group = output_group.add_mutually_exclusive_group()
    fsync_group.add_argument("--fsync", dest="fsync", action="store_true", help="Force data to disk with os.fsync() on each flush.")
    fsync_group.add_argument("--no-fsync", dest="fsync", action="store_false", help="Disable os.fsync() on flush for higher throughput.")
    parser.set_defaults(fsync=True)

    reconnect_group = parser.add_argument_group("Reconnect")
    reconnect_group.add_argument("--no-reconnect", action="store_true", help="Exit on first disconnect.")
    reconnect_group.add_argument("--reconnect-delay-sec", type=float, default=0.5, help="Delay between reconnect attempts.")
    reconnect_group.add_argument("--port-scan-delay-sec", type=float, default=0.5, help="When --port=auto and none found, scan every N seconds.")

    reset_group = parser.add_argument_group("Reset")
    reset_group.add_argument("--no-reset", action="store_true", help="Do not reset on connect (like idf.py monitor --no-reset).")
    reset_group.add_argument("--reset-mode", default="esp32", choices=["esp32", "esp32-bootloader", "rts", "dtr", "custom"], help="Reset method.")
    reset_group.add_argument("--reset-pulse-sec", type=float, default=0.10, help="Pulse width for reset line(s).")
    reset_group.add_argument("--reset-post-delay-sec", type=float, default=0.20, help="Delay after reset before reading.")
    reset_group.add_argument("--reset-sequence", default="", help="Custom sequence for --reset-mode=custom, e.g. 'R0|D1|W0.5'.")

    color_group = parser.add_argument_group("Color")
    color_group.add_argument("--no-color", action="store_true", help="Disable ANSI color.")
    color_group.add_argument("--color", action="store_true", help="Force ANSI color even if stdout not a TTY.")

    interactive_group = parser.add_argument_group("Interactive (two-way)")
    interactive_group.add_argument("--interactive", action="store_true", help="Forward stdin lines to the device.")
    interactive_group.add_argument("--input-eol", default="\n", help="EOL appended to each stdin line.")
    interactive_group.add_argument("--input-encoding", default="utf-8", help="Encoding for stdin -> serial.")
    interactive_group.add_argument("--input-errors", default="replace", choices=["strict", "replace", "ignore"], help="Encoding error behavior.")
    interactive_group.add_argument("--local-echo", action="store_true", help="Echo stdin locally with a '>>' prefix.")
    interactive_group.add_argument("--input-log", action="store_true", help="Log stdin lines into the log file as metadata.")

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    _maybe_enable_windows_ansi()

    parser = _build_parser()
    args = parser.parse_args(argv)

    color_enabled = (not args.no_color) and (args.color or _is_tty(sys.stdout))

    serial_config = SerialConfig(
        port=args.port,
        baudrate=args.baudrate,
        read_timeout_sec=args.read_timeout_sec,
    )
    output_config = OutputConfig(
        output_path=args.output,
        append=args.append,
        host_timestamps=args.host_timestamps,
        decode_encoding=args.encoding,
        decode_errors=args.errors,
        flush_lines=args.flush_lines,
        flush_interval_sec=args.flush_interval_sec,
        fsync_on_flush=args.fsync,
        print_to_stdout=(not args.no_stdout),
    )
    reconnect_config = ReconnectConfig(
        reconnect=(not args.no_reconnect),
        reconnect_delay_sec=args.reconnect_delay_sec,
        port_scan_delay_sec=args.port_scan_delay_sec,
    )
    reset_config = ResetConfig(
        enabled=(not args.no_reset),
        mode=args.reset_mode,
        pulse_sec=args.reset_pulse_sec,
        post_delay_sec=args.reset_post_delay_sec,
        custom_sequence=args.reset_sequence,
    )
    color_config = ColorConfig(enabled=color_enabled)
    interactive_config = InteractiveConfig(
        enabled=args.interactive,
        input_eol=args.input_eol,
        input_encoding=args.input_encoding,
        input_errors=args.input_errors,
        local_echo=args.local_echo,
        input_log=args.input_log,
    )

    session_logger = SessionLogger(
        serial_config=serial_config,
        output_config=output_config,
        reconnect_config=reconnect_config,
        reset_config=reset_config,
        color_config=color_config,
        interactive_config=interactive_config,
        port_regex=args.port_regex,
        port_vid_pid=args.port_vid_pid,
        port_serial_number=args.port_serial_number,
    )

    def _handle_signal(_signum, _frame) -> None:
        session_logger.stop()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        return session_logger.run()
    except BaseException:
        session_logger.close()
        raise


if __name__ == "__main__":
    raise SystemExit(main())
