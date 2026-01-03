import struct
import sys
import time

import serial

MAGIC = b"JPG0"
END = b"END0"


def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise TimeoutError(f"timeout reading {n} bytes (got {len(buf)})")
        buf += chunk
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


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    out = sys.argv[2] if len(sys.argv) > 2 else "out.jpg"

    ser = serial.Serial(port, baudrate=115200, timeout=1)
    # baudrate is ignored for USB CDC on many systems; keep it anyway.

    # Avoid toggling control lines (some host stacks reset boards on open).
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass

    time.sleep(0.2)

    # Wait for READY banner so the device is fully booted before sending SNAP.
    wait_for_token(ser, b"READY snap_usb", 5.0)
    ser.reset_input_buffer()

    ser.write(b"SNAP\n")
    ser.flush()

    if not wait_for_token(ser, b"ACK\n", 3.0):
        raise TimeoutError("timeout waiting for ACK")

    if not sync_magic(ser, 2.0):
        raise TimeoutError("timeout waiting for JPG0")

    ser.timeout = 5
    (length,) = struct.unpack("<I", read_exact(ser, 4))
    if length == 0:
        raise RuntimeError("device reported capture failure (length=0)")
    if length > 10 * 1024 * 1024:
        raise RuntimeError(f"invalid length: {length}")
    jpg = read_exact(ser, length)
    tail = read_exact(ser, 4)
    if tail != END:
        raise RuntimeError(f"bad tail: {tail!r}")

    with open(out, "wb") as f:
        f.write(jpg)

    print(f"wrote {out} ({len(jpg)} bytes)")


if __name__ == "__main__":
    main()
