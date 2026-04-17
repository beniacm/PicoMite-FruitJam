#!/usr/bin/env python3
"""Upload a BASIC program to PicoMite Fruit Jam via AUTOSAVE then RUN silently.

AUTOSAVE N starts a no-echo program-capture mode on the device. Program text is
streamed in, then Ctrl-Z (0x1A) ends and flushes to program flash. The console
echo is suppressed so RUN produces a clean screen with only the program's own
drawing visible on HDMI.

Usage:  python3 autosave_run.py [script.bas] [--port /dev/ttyACMx] [--no-run]
"""
import serial, time, sys, argparse, glob, subprocess

def find_port():
    for p in sorted(glob.glob('/dev/ttyACM*')):
        try:
            info = subprocess.check_output(['udevadm', 'info', p], text=True, stderr=subprocess.DEVNULL)
            if 'PicoMite' in info or 'ID_VENDOR_ID=2e8a' in info:
                return p
        except Exception:
            continue
    return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('script')
    ap.add_argument('--port', default=None)
    ap.add_argument('--no-run', action='store_true')
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        print("No PicoMite CDC port found"); sys.exit(1)
    with open(args.script) as f:
        src = f.read()

    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b'\r')
    time.sleep(0.2)
    ser.read(ser.in_waiting)

    # Enter no-echo autosave mode
    ser.write(b'AUTOSAVE N\r')
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    # Stream the program — AUTOSAVE has a 100ms idle timeout that
    # disables no-echo mode, so send the whole buffer + Ctrl-Z atomically.
    payload = b''
    for line in src.splitlines():
        line = line.rstrip()
        if not line:
            continue
        payload += line.encode() + b'\r'
    payload += b'\x1a'  # Ctrl-Z terminates autosave and flushes to flash
    ser.write(payload)
    ser.flush()
    time.sleep(2.0)  # flash sector erase + write
    saved = ser.read(ser.in_waiting).decode(errors='replace')
    print(f"[saved]\n{saved.strip()}")

    if args.no_run:
        ser.close()
        return

    # Clear any pending output + HDMI console text, then RUN
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b'CLS\r')
    time.sleep(0.2); ser.read(ser.in_waiting)
    ser.write(b'RUN\r')
    ser.close()
    print("RUN issued")

if __name__ == '__main__':
    main()
