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


def sync_magic(ser):
    window = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("timeout waiting for JPG0")
        window = (window + b)[-4:]
        if bytes(window) == MAGIC:
            return


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    out = sys.argv[2] if len(sys.argv) > 2 else "out.jpg"

    ser = serial.Serial(port, baudrate=115200, timeout=5)
    # baudrate is ignored for USB CDC on many systems; keep it anyway.

    # Avoid toggling control lines (some host stacks reset boards on open).
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass

    time.sleep(0.2)
    ser.reset_input_buffer()

    ser.write(b"SNAP\n")
    ser.flush()

    sync_magic(ser)
    (length,) = struct.unpack("<I", read_exact(ser, 4))
    jpg = read_exact(ser, length)
    tail = read_exact(ser, 4)
    if tail != END:
        raise RuntimeError(f"bad tail: {tail!r}")

    with open(out, "wb") as f:
        f.write(jpg)

    print(f"wrote {out} ({len(jpg)} bytes)")


if __name__ == "__main__":
    main()
