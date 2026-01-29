import argparse
import serial
import time
import signal
import sys
from datetime import datetime, timedelta

# === Configuration ===
LOG_BUFFER_LIMIT = 600  # Max number of lines before flushing
LOG_FLUSH_INTERVAL_MINUTES = 10  # Max time before flushing, in minutes

# === Globals for cleanup ===
ser = None
txt_file = None
start_time = None  # Time when the first valid line is logged
log_buffer = []  # In-memory buffer for CSV rows
last_flush_time = datetime.now()

def generate_timestamped_filenames(base_name):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{base_name}_{timestamp}.txt"

def open_serial_port(port, baudrate):
    print(f"Opening serial port {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(3)
    ser.reset_input_buffer()
    return ser

def open_output_files(base_name):
    global txt_file
    txt_filename = generate_timestamped_filenames(base_name)

    txt_file = open(txt_filename, "w")
    print(f"Logging to {txt_filename}")
    return txt_filename

def parse_and_buffer_line(line):
    global start_time, log_buffer, last_flush_time

    now = datetime.now()
    if start_time is None:
        start_time = now

    timestamp_str = now.strftime("%Y-%m-%d %H:%M:%S")

    print(f"<{timestamp_str}> {line}")
    log_buffer.append(f"<{timestamp_str}> {line}")

    flush_due_to_lines = len(log_buffer) >= LOG_BUFFER_LIMIT
    flush_due_to_time = (now - last_flush_time) >= timedelta(minutes=LOG_FLUSH_INTERVAL_MINUTES)

    if flush_due_to_lines or flush_due_to_time:
        flush_buffer()

def flush_buffer():
    global log_buffer, last_flush_time
    if log_buffer:
        for line in log_buffer:
            txt_file.write(line + "\n")
        log_buffer.clear()
        last_flush_time = datetime.now()

def handle_signal(sig, frame):
    print("\nReceived Ctrl+C, sending stopLogger and exiting...")
    try:
        if ser and ser.is_open:
            ser.write(b"stopLogger;")
            time.sleep(1)
    except Exception as e:
        print(f"Error sending stopLogger: {e}")
    finally:
        flush_buffer()
        close_all()
        sys.exit(0)

def close_all():
    global txt_file, ser
    if txt_file:
        txt_file.close()
    if ser and ser.is_open:
        ser.close()

def start_logging(args):
    global ser, last_flush_time
    ser = open_serial_port(args.port, args.baudrate)
    open_output_files(args.output)
    last_flush_time = datetime.now()

    signal.signal(signal.SIGINT, handle_signal)

    # ser.write(b"startLogger;")
    print("Logging started. Press Ctrl+C to stop.")

    while True:
        try:
            if ser.in_waiting:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    parse_and_buffer_line(line)
        except Exception as e:
            print(f"Error reading/parsing line: {e}")
            flush_buffer()
            close_all()
            break

def parse_args():
    parser = argparse.ArgumentParser(description="Serial temperature logger")
    parser.add_argument("-p", "--port", required=True, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baudrate", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("-o", "--output", required=True, help="Base name for output files (no extension)")
    return parser.parse_args()

def main():
    args = parse_args()
    start_logging(args)

if __name__ == "__main__":
    main()
