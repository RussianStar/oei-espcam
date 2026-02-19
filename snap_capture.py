import argparse
import glob
import os
import struct
import sys
import time

import serial

MAGIC = b"JPG0"
END = b"END0"


def _log_bytes(log_f, data):
    if log_f is None or not data:
        return
    try:
        log_f.write(data)
        log_f.flush()
    except Exception:
        pass


def read_exact(ser, n, max_idle_s=10.0, log_f=None):
    buf = bytearray()
    last_progress = time.time()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        _log_bytes(log_f, chunk)
        if chunk:
            buf += chunk
            last_progress = time.time()
            continue
        if time.time() - last_progress > max_idle_s:
            raise TimeoutError(f"timeout reading {n} bytes (got {len(buf)})")
    return bytes(buf)


def sync_magic(ser, timeout_s, log_prefix=None, log_f=None):
    end = time.time() + timeout_s
    token_window = bytearray()
    line_buf = bytearray()
    while time.time() < end:
        b = ser.read(1)
        _log_bytes(log_f, b)
        if not b:
            continue
        if log_prefix is not None:
            if b == b"\n":
                if line_buf:
                    text = line_buf.decode("utf-8", errors="replace").rstrip("\r\n")
                    ts = time.strftime("%H:%M:%S")
                    print(f"[{ts}] {log_prefix}{text}")
                    sys.stdout.flush()
                    line_buf.clear()
            else:
                # Collect only printable-ish bytes to avoid binary garbage.
                if 32 <= b[0] <= 126 or b in (b"\r", b"\t"):
                    line_buf += b
        token_window = (token_window + b)[-4:]
        if bytes(token_window) == MAGIC:
            return True
    return False


def wait_for_any_token(ser, tokens, timeout_s, log_prefix=None, log_f=None):
    end = time.time() + timeout_s
    token_window = bytearray()
    line_buf = bytearray()
    max_len = max(len(t) for t in tokens)
    while time.time() < end:
        b = ser.read(1)
        _log_bytes(log_f, b)
        if not b:
            continue
        if log_prefix is not None:
            # Try to print any line-oriented logs while waiting.
            if b == b"\n":
                if line_buf:
                    text = line_buf.decode("utf-8", errors="replace").rstrip("\r\n")
                    ts = time.strftime("%H:%M:%S")
                    print(f"[{ts}] {log_prefix}{text}")
                    sys.stdout.flush()
                    line_buf.clear()
            else:
                if 32 <= b[0] <= 126 or b in (b"\r", b"\t"):
                    line_buf += b
        token_window = (token_window + b)[-max_len:]
        for t in tokens:
            if bytes(token_window).endswith(t):
                return True
    return False


def wait_for_port(path, timeout_s):
    end = time.time() + timeout_s
    while time.time() < end:
        if path:
            if os.path.exists(path):
                return path
        else:
            candidates = discover_serial_ports()
            if candidates:
                return candidates[0]
        time.sleep(0.1)
    return None


def discover_serial_ports():
    if os.name != "posix":
        return []

    candidates = []
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyS*"):
        candidates.extend(sorted(glob.glob(pattern)))
    dedup = []
    seen = set()
    for path in candidates:
        if path in seen:
            continue
        dedup.append(path)
        seen.add(path)
    return dedup


def resolve_port(path):
    if path and os.path.exists(path):
        return path
    ports = discover_serial_ports()
    if ports:
        return ports[0]
    return path


def open_serial(port, baud, write_timeout):
    ser = serial.Serial(port, baudrate=baud, timeout=1, write_timeout=write_timeout)
    try:
        # Keep DTR asserted so the device RX path stays active.
        ser.setDTR(True)
        ser.setRTS(False)
    except Exception:
        pass
    return ser


def print_boot_log_until_ready(ser, timeout_s, log_f=None):
    end = time.time() + timeout_s
    ready = False
    while time.time() < end:
        line = ser.readline()
        _log_bytes(log_f, line)
        if not line:
            continue
        ts = time.strftime("%H:%M:%S")
        text = line.decode("utf-8", errors="replace").rstrip("\r\n")
        print(f"[{ts}] {text}")
        sys.stdout.flush()
        if "READY snap_usb" in text or "BOOT" in text or "Send: SNAP" in text:
            ready = True
            break
    return ready


def main():
    parser = argparse.ArgumentParser(description="Reset, show boot log, then SNAP")
    parser.add_argument("port", nargs="?", default=None)
    parser.add_argument("out", nargs="?", default="out.jpg")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--wait", type=float, default=0.0)
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("--sleep", action="store_true")
    parser.add_argument("--sleep-only", action="store_true")
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--ack-timeout", type=float, default=5.0)
    parser.add_argument("--jpg-timeout", type=float, default=5.0)
    parser.add_argument("--write-timeout", type=float, default=2.0)
    parser.add_argument("--pre-snap-delay", type=float, default=0.2)
    parser.add_argument("--ping", action="store_true", help="send PING/ACK before SNAP")
    parser.add_argument("--ping-timeout", type=float, default=2.0)
    parser.add_argument("--post-ready-delay", type=float, default=0.5)
    parser.add_argument("--log-path", type=str, default="", help="write raw serial bytes to file")
    args = parser.parse_args()

    if args.wait > 0:
        wait_port = wait_for_port(args.port, args.wait) if args.port else wait_for_port(None, args.wait)
        if not wait_port:
            raise TimeoutError(f"port not found: {args.port or 'auto-detect'}")
        if not args.port and wait_port != args.port:
            print(f"auto-selected port: {wait_port}")
        args.port = wait_port
    else:
        if args.port and not os.path.exists(args.port):
            raise TimeoutError(f"port not found: {args.port}")
        args.port = resolve_port(args.port)
    if not args.port:
        raise TimeoutError("no serial port found; connect device and retry")

    ser = None
    log_f = None
    try:
        if args.log_path:
            try:
                log_f = open(args.log_path, "ab")
            except OSError as exc:
                print(f"WARNING: cannot open log file {args.log_path}: {exc}")
                log_f = None
        ser = open_serial(args.port, args.baud, args.write_timeout)

        if args.reset:
            try:
                ser.close()
            except OSError:
                pass
            time.sleep(0.3)
            if args.wait > 0:
                wait_port = wait_for_port(None if not args.port else args.port, args.wait)
                if not wait_port:
                    raise TimeoutError(f"port not found after reset: {args.port}")
                args.port = wait_port
            ser = open_serial(args.port, args.baud, args.write_timeout)

        time.sleep(0.2)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

        print("Waiting for READY...")
        ready = print_boot_log_until_ready(ser, args.ready_timeout, log_f=log_f)
        if not ready:
            print("READY not seen; trying sync probe")
            try:
                ser.write(b"PING\n")
                ser.flush()
            except serial.SerialTimeoutException:
                # try a quick reopen once if tx path is wedged
                ser.close()
                time.sleep(0.2)
                ser = open_serial(args.port, args.baud, args.write_timeout)
                ser.write(b"PING\n")
                ser.flush()
            if wait_for_any_token(ser, [b"ACK\n", b"ACK\r\n", b"ACK"], 2.0, log_prefix="", log_f=log_f):
                print("ACK seen; continuing")
            else:
                raise TimeoutError("timeout waiting for READY")
        print("READY seen")
        if args.post_ready_delay > 0:
            time.sleep(args.post_ready_delay)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

        if args.ping:
            print("Send: PING")
            time.sleep(0.1)
            ok = False
            for attempt in range(2):
                try:
                    ser.write(b"PING\n")
                    ser.flush()
                except serial.SerialTimeoutException:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    time.sleep(0.2)
                    ser = open_serial(args.port, args.baud, args.write_timeout)
                    ser.write(b"PING\n")
                    ser.flush()
                if wait_for_any_token(ser, [b"ACK\n", b"ACK\r\n", b"ACK"], args.ping_timeout, log_prefix="", log_f=log_f):
                    ok = True
                    break
                time.sleep(0.2)
            if not ok:
                raise TimeoutError("timeout waiting for ACK (PING)")

        if args.sleep_only:
            ser.write(b"SLEEP\n")
            ser.flush()
            if not wait_for_any_token(ser, [b"ACK\n", b"ACK\r\n", b"ACK"], 3.0, log_prefix="", log_f=log_f):
                raise TimeoutError("timeout waiting for ACK (SLEEP)")
            print("camera deinit requested")
            return

        time.sleep(args.pre_snap_delay)
        print("Send: SNAP")
        try:
            ser.write(b"SNAP\n")
            ser.flush()
        except serial.SerialTimeoutException:
            # Reopen once and retry if USB/JTAG is temporarily wedged.
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(0.2)
            ser = open_serial(args.port, args.baud, args.write_timeout)
            ser.write(b"SNAP\n")
            ser.flush()

        if not wait_for_any_token(ser, [b"ACK\n", b"ACK\r\n", b"ACK"], args.ack_timeout, log_prefix="", log_f=log_f):
            raise TimeoutError("timeout waiting for ACK")

        if not sync_magic(ser, args.jpg_timeout, log_prefix="", log_f=log_f):
            raise TimeoutError("timeout waiting for JPG0")

        (length,) = struct.unpack("<I", read_exact(ser, 4, max_idle_s=5.0, log_f=log_f))
        if length == 0:
            raise RuntimeError("device reported capture failure (length=0)")
        if length > 10 * 1024 * 1024:
            raise RuntimeError(f"invalid length: {length}")

        jpg = read_exact(ser, length, max_idle_s=15.0, log_f=log_f)
        tail = read_exact(ser, 4, max_idle_s=5.0, log_f=log_f)
        if tail != END:
            raise RuntimeError(f"bad tail: {tail!r}")

        with open(args.out, "wb") as f:
            f.write(jpg)

        if args.sleep:
            ser.write(b"SLEEP\n")
            ser.flush()
            if not wait_for_any_token(ser, [b"ACK\n", b"ACK\r\n", b"ACK"], 3.0, log_prefix="", log_f=log_f):
                raise TimeoutError("timeout waiting for ACK (SLEEP)")

        print(f"wrote {args.out} ({len(jpg)} bytes)")
    finally:
        if log_f is not None:
            try:
                log_f.close()
            except Exception:
                pass
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


def _main():
    try:
        main()
    except KeyboardInterrupt:
        # Avoid noisy tracebacks on Ctrl-C.
        print("\nInterrupted.")

if __name__ == "__main__":
    _main()
