import os
import glob
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


def discover_serial_ports():
    if os.name != "posix":
        return []

    candidates = []
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyS*"):
        candidates.extend(sorted(glob.glob(pattern)))
    # Keep first available in deterministic priority order.
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

    for candidate in discover_serial_ports():
        return candidate

    return path


def is_usb_uart_bridge(port):
    return isinstance(port, str) and "/ttyUSB" in port


def wait_for_port(path, timeout_s):
    end = time.time() + timeout_s
    while time.time() < end:
        if path:
            if os.path.exists(path):
                return path
        else:
            for candidate in discover_serial_ports():
                if candidate:
                    return candidate
        time.sleep(0.1)
    return None


def open_serial(port):
    ser = serial.Serial(port, baudrate=115200, timeout=1)
    try:
        # CH34x/CP210x reset wiring on ESP32-CAM can enter download mode if DTR is asserted.
        if is_usb_uart_bridge(port):
            ser.setDTR(False)
            ser.setRTS(False)
        else:
            ser.setDTR(True)
            ser.setRTS(False)
    except Exception:
        pass
    return ser


def set_dtr_rts(ser, dtr=None, rts=None):
    try:
        if dtr is not None:
            ser.setDTR(dtr)
        if rts is not None:
            ser.setRTS(rts)
        return True
    except Exception:
        return False


def pulse_reset_signals(port):
    try:
        ser = open_serial(port)
    except Exception:
        return False

    ok = False
    try:
        if set_dtr_rts(ser, dtr=False, rts=False):
            time.sleep(0.05)
            if set_dtr_rts(ser, dtr=True, rts=False):
                ok = True
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return ok


def wait_for_port_state(path, timeout_s, should_exist):
    end = time.time() + timeout_s
    while time.time() < end:
        if os.path.exists(path) == should_exist:
            return True
        time.sleep(0.05)
    return False


def reopen_after_reset(port, wait_s):
    # Prefer firmware reset first; safer on ESP32-CAM usb-uart bridges.
    soft_reset_sent = False
    try:
        with open_serial(port) as ser:
            ser.write(b"RESET\n")
            ser.flush()
            soft_reset_sent = True
    except Exception:
        pass

    # Fallback to line toggles on non-ttyUSB only; ttyUSB tends to land in boot:0x3.
    if not soft_reset_sent and not is_usb_uart_bridge(port):
        pulse_reset_signals(port)

    # Some adapters drop/recreate the tty on reset, some do not.
    if wait_s > 0:
        wait_for_port_state(port, min(1.0, wait_s), should_exist=False)
        if not wait_for_port_state(port, max(0.0, wait_s - 1.0), should_exist=True):
            raise TimeoutError(f"port did not come back after reset: {port}")

    time.sleep(0.2)
    return open_serial(port)


def main():
    args = sys.argv[1:]
    port = None
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
        resolved_port = wait_for_port(port if port_set else None, wait_port_s)
        if not resolved_port:
            target = "auto-detect" if not port_set else port
            raise TimeoutError(f"port not found: {target}")
        if not port_set and port != resolved_port:
            print(f"auto-selected port: {resolved_port}")
        port = resolved_port
    else:
        if port_set and port and not os.path.exists(port):
            raise TimeoutError(f"port not found: {port}")
        port = resolve_port(port if port_set else None)
    if not port:
        raise TimeoutError("no serial port found; connect device and retry")

    ser = open_serial(port)
    # baudrate is ignored for USB CDC on many systems; keep it anyway.

    if do_reset:
        ser.close()
        ser = reopen_after_reset(port, wait_port_s if wait_port_s > 0 else 2.0)

    time.sleep(0.2)

    # Wait for READY banner so the device is fully booted before sending SNAP.
    if not wait_for_token(ser, b"READY snap_usb", 5.0):
        print("READY not seen; trying sync probe")
        ser.write(b"PING\n")
        ser.flush()
        if not wait_for_token(ser, b"ACK\n", 2.0):
            raise TimeoutError("timeout waiting for READY")
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
