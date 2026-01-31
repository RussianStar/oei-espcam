import argparse
import os
import sys
import time

import serial


def wait_for_port(path, timeout_s):
    end = time.time() + timeout_s
    while time.time() < end:
        if os.path.exists(path):
            return True
        time.sleep(0.1)
    return False


def open_serial(port, baud):
    ser = serial.Serial(port, baudrate=baud, timeout=1)
    try:
        ser.setDTR(True)
        ser.setRTS(False)
    except Exception:
        pass
    return ser


def main():
    parser = argparse.ArgumentParser(description="Stream ESP32 debug output over USB serial")
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--wait", type=float, default=0.0, help="wait for port (seconds)")
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("--raw", action="store_true", help="print raw bytes")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds")
    args = parser.parse_args()

    if args.wait > 0:
        if not wait_for_port(args.port, args.wait):
            raise TimeoutError(f"port not found: {args.port}")

    ser = open_serial(args.port, args.baud)

    if args.reset:
        ser.close()
        time.sleep(0.3)
        if args.wait > 0:
            if not wait_for_port(args.port, args.wait):
                raise TimeoutError(f"port not found after reset: {args.port}")
        ser = open_serial(args.port, args.baud)

    end_time = time.time() + args.duration if args.duration > 0 else None

    while True:
        if end_time is not None and time.time() > end_time:
            break
        if args.raw:
            data = ser.read(1024)
            if data:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
            continue
        line = ser.readline()
        if not line:
            continue
        ts = time.strftime("%H:%M:%S")
        try:
            text = line.decode("utf-8", errors="replace").rstrip("\r\n")
        except Exception:
            text = repr(line)
        print(f"[{ts}] {text}")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
