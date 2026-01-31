import os
import struct
import sys
import time

import serial

MAGIC = b"JPG0"
END = b"END0"


def read_exact(ser, n, max_idle_s=10.0):
    buf = bytearray()
    last_progress = time.time()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
            last_progress = time.time()
            continue
        if time.time() - last_progress > max_idle_s:
            raise TimeoutError(f"timeout reading {n} bytes (got {len(buf)})")
    return bytes(buf)


def sync_magic(ser, timeout_s):
    end = time.time() + timeout_s
    window = bytearray()
    while time.time() < end:
        b = ser.read(1)
        if not b:
            continue
        window = (window + b)[-4:]
        if bytes(window) == MAGIC:
            return True
    return False


def wait_for_token(ser, token, timeout_s):
    end = time.time() + timeout_s
    window = bytearray()
    n = len(token)
    while time.time() < end:
        b = ser.read(1)
        if not b:
            continue
        window = (window + b)[-n:]
        if bytes(window) == token:
            return True
    return False


def wait_for_port(path, timeout_s):
    end = time.time() + timeout_s
    while time.time() < end:
        if os.path.exists(path):
            return True
        time.sleep(0.1)
    return False


def open_serial(port):
    ser = serial.Serial(port, baudrate=115200, timeout=1)
    try:
        ser.setDTR(True)
        ser.setRTS(False)
    except Exception:
        pass
    return ser


def main():
    args = sys.argv[1:]
    port = "/dev/ttyACM0"
    out = "out.jpg"
    sleep_after = False
    sleep_only = False
    do_reset = False
    wait_port_s = 0.0
    port_set = False
    out_set = False
    for a in args:
        if a == "--sleep":
            sleep_after = True
        elif a == "--sleep-only":
            sleep_only = True
        elif a == "--reset":
            do_reset = True
        elif a.startswith("--wait="):
            try:
                wait_port_s = float(a.split("=", 1)[1])
            except ValueError:
                pass
        elif not port_set:
            port = a
            port_set = True
        elif not out_set:
            out = a
            out_set = True

    if wait_port_s > 0:
        if not wait_for_port(port, wait_port_s):
            raise TimeoutError(f"port not found: {port}")

    ser = open_serial(port)
    # baudrate is ignored for USB CDC on many systems; keep it anyway.

    if do_reset:
        ser.close()
        time.sleep(0.3)
        if wait_port_s > 0:
            if not wait_for_port(port, wait_port_s):
                raise TimeoutError(f"port not found after reset: {port}")
        ser = open_serial(port)

    time.sleep(0.2)

    # Wait for READY banner so the device is fully booted before sending SNAP.
    wait_for_token(ser, b"READY snap_usb", 5.0)
    ser.reset_input_buffer()

    if sleep_only:
        ser.write(b"SLEEP\n")
        ser.flush()
        if not wait_for_token(ser, b"ACK\n", 3.0):
            raise TimeoutError("timeout waiting for ACK (SLEEP)")
        print("camera deinit requested")
        return

    ser.write(b"SNAP\n")
    ser.flush()

    if not wait_for_token(ser, b"ACK\n", 5.0):
        raise TimeoutError("timeout waiting for ACK")

    if not sync_magic(ser, 5.0):
        raise TimeoutError("timeout waiting for JPG0")

    ser.timeout = 1
    (length,) = struct.unpack("<I", read_exact(ser, 4, max_idle_s=5.0))
    if length == 0:
        raise RuntimeError("device reported capture failure (length=0)")
    if length > 10 * 1024 * 1024:
        raise RuntimeError(f"invalid length: {length}")
    jpg = read_exact(ser, length, max_idle_s=15.0)
    tail = read_exact(ser, 4, max_idle_s=5.0)
    if tail != END:
        raise RuntimeError(f"bad tail: {tail!r}")

    with open(out, "wb") as f:
        f.write(jpg)

    if sleep_after:
        ser.write(b"SLEEP\n")
        ser.flush()
        if not wait_for_token(ser, b"ACK\n", 3.0):
            raise TimeoutError("timeout waiting for ACK (SLEEP)")

    print(f"wrote {out} ({len(jpg)} bytes)")


if __name__ == "__main__":
    main()
