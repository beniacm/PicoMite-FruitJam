#!/usr/bin/env python3
"""Capture composited frame from PicoMite via CDC + XMODEM-CRC.

Flow:
  1. Open CDC port, send FRAMEDUMP.
  2. Wait for 'FJFD READY' line.
  3. Drive XMODEM-CRC receive.
  4. Parse 32-byte header, optional 512-byte palette, raw pixels.
  5. Render PNG via PIL.

Usage:
    python3 frame_capture.py [script.bas] [out.png] [--port /dev/ttyACMx]
"""
import serial, time, sys, struct, os, argparse

SOH, EOT, ACK, NAK, CAN, CRCREQ = 0x01, 0x04, 0x06, 0x15, 0x18, ord('C')
PKT = 128

def find_port():
    import glob, subprocess
    for p in sorted(glob.glob('/dev/ttyACM*')):
        try:
            info = subprocess.check_output(['udevadm', 'info', p], text=True, stderr=subprocess.DEVNULL)
            if 'PicoMite' in info or 'ID_VENDOR_ID=2e8a' in info:
                return p
        except Exception:
            continue
    for p in sorted(glob.glob('/dev/ttyACM*')):
        return p
    return None

def connect(port):
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.3)
    ser.reset_input_buffer()
    return ser

def send_line(ser, line):
    ser.write(line.encode() + b'\r\n')
    ser.flush()

def wait_for_prompt(ser, timeout=3):
    """Drain until idle; return what was drained."""
    end = time.time() + timeout
    buf = b''
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            end = time.time() + 0.3
        else:
            time.sleep(0.02)
    return buf

def load_program(ser, src_path):
    with open(src_path) as f:
        src = f.read()
    send_line(ser, 'NEW')
    wait_for_prompt(ser, 1)
    lineno = 10
    for line in src.strip().split('\n'):
        line = line.rstrip()
        if not line:
            continue
        ser.write(f'{lineno} {line}\r\n'.encode())
        time.sleep(0.04)
        lineno += 10
    wait_for_prompt(ser, 1)
    send_line(ser, 'RUN')
    # give the program a moment to draw, but it likely ends at drawing commands
    time.sleep(2.5)
    wait_for_prompt(ser, 1)

def crc16_ccitt(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def xmodem_recv_crc(ser):
    """Receive an XMODEM-CRC stream. Returns bytes (packets concatenated, may be
    padded with 0x1A at the tail)."""
    ser.timeout = 2
    data = b''
    expected = 1
    # Kick off CRC mode — may need to retry until sender starts
    for attempt in range(30):
        ser.write(bytes([CRCREQ]))
        ser.flush()
        b = ser.read(1)
        if not b:
            continue
        if b[0] == SOH:
            break
    else:
        raise RuntimeError("Sender did not start transmission")

    while True:
        if not b:
            raise RuntimeError("Timeout waiting for SOH/EOT")
        if b[0] == EOT:
            ser.write(bytes([ACK]))
            ser.flush()
            return data
        if b[0] == CAN:
            raise RuntimeError("Sender cancelled")
        if b[0] != SOH:
            # junk — NAK and continue
            ser.write(bytes([NAK]))
            b = ser.read(1)
            continue
        hdr = ser.read(2)
        if len(hdr) != 2:
            ser.write(bytes([NAK]))
            b = ser.read(1)
            continue
        seq, nseq = hdr[0], hdr[1]
        payload = ser.read(PKT)
        crc_bytes = ser.read(2)
        if len(payload) != PKT or len(crc_bytes) != 2:
            ser.write(bytes([NAK]))
            b = ser.read(1)
            continue
        crc_recv = (crc_bytes[0] << 8) | crc_bytes[1]
        crc_calc = crc16_ccitt(payload)
        ok = ((seq ^ nseq) == 0xFF) and (crc_recv == crc_calc) and (seq == expected)
        if ok:
            data += payload
            expected = (expected + 1) & 0xFF
            ser.write(bytes([ACK]))
        else:
            ser.write(bytes([NAK]))
        ser.flush()
        b = ser.read(1)

def parse_blob(blob):
    if blob[0:4] != b'FJFD':
        raise RuntimeError(f"Bad magic: {blob[0:4]!r}")
    mode = blob[4]
    bpp  = blob[5]
    w    = blob[6] | (blob[7] << 8)
    h    = blob[8] | (blob[9] << 8)
    total = blob[10] | (blob[11] << 8) | (blob[12] << 16) | (blob[13] << 24)
    transparent  = blob[14]
    transparents = blob[15]
    off = 32
    palette = None
    # SCREENMODE5 = 34 (8bpp indexed), SCREENMODE4 = 33 (16bpp RGB555)
    is_mode5 = (mode == 34 or mode == 5)
    if is_mode5:
        palette = list(struct.unpack('<256H', blob[off:off+512]))
        off += 512
    pix_len = w * h * bpp
    pixels = blob[off:off+pix_len]
    return dict(mode=mode, bpp=bpp, w=w, h=h, total=total,
                transparent=transparent, transparents=transparents,
                palette=palette, pixels=pixels)

def rgb555_to_rgb(v):
    r = (v >> 10) & 0x1F
    g = (v >> 5) & 0x1F
    b = v & 0x1F
    return (r * 255 // 31, g * 255 // 31, b * 255 // 31)

def render_png(info, path):
    try:
        from PIL import Image
    except ImportError:
        print("PIL missing; pip install pillow to render PNG")
        return
    img = Image.new('RGB', (info['w'], info['h']))
    px = img.load()
    if info['mode'] in (5, 34):
        pal = [rgb555_to_rgb(v) for v in info['palette']]
        for y in range(info['h']):
            for x in range(info['w']):
                px[x, y] = pal[info['pixels'][y * info['w'] + x]]
    else:
        for y in range(info['h']):
            for x in range(info['w']):
                o = (y * info['w'] + x) * 2
                v = info['pixels'][o] | (info['pixels'][o + 1] << 8)
                px[x, y] = rgb555_to_rgb(v)
    img.save(path)
    print(f"Saved {path}")

def render_ascii(info):
    w, h = info['w'], info['h']
    sx = max(1, w // 80)
    sy = max(1, h // 30)
    chars = ' .:-=+*#%@'
    for y in range(0, h, sy):
        row = ''
        for x in range(0, w, sx):
            if info['mode'] in (5, 34):
                v = info['palette'][info['pixels'][y * w + x]]
            else:
                o = (y * w + x) * 2
                v = info['pixels'][o] | (info['pixels'][o + 1] << 8)
            r, g, b = rgb555_to_rgb(v)
            lum = (r*299 + g*587 + b*114) // 1000
            row += chars[lum * (len(chars) - 1) // 255]
        print(row)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('script', nargs='?')
    ap.add_argument('out', nargs='?', default='/tmp/frame.png')
    ap.add_argument('--port', default=None)
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        print("No /dev/ttyACM* port found"); sys.exit(1)
    print(f"Opening {port}")
    ser = connect(port)

    if args.script and args.script.endswith('.bas'):
        print(f"Loading {args.script}")
        load_program(ser, args.script)

    # Issue FRAMEDUMP and wait for READY
    ser.reset_input_buffer()
    send_line(ser, 'FRAMEDUMP')
    deadline = time.time() + 15
    buf = b''
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            if b'FJFD READY' in buf:
                break
        else:
            time.sleep(0.02)
    else:
        print("No FJFD READY; got:", buf[-200:].decode('latin1', errors='replace'))
        sys.exit(2)

    print("Starting XMODEM-CRC receive…")
    # Some bytes may have arrived after the READY banner but before SOH; reset input.
    # Drain any trailing bytes first.
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    try:
        raw = xmodem_recv_crc(ser)
    except Exception as e:
        print("XMODEM failed:", e); sys.exit(3)

    print(f"Received {len(raw)} bytes")
    info = parse_blob(raw)
    print(f"mode={info['mode']} w={info['w']} h={info['h']} bpp={info['bpp']} "
          f"T={info['transparent']} Ts={info['transparents']}")
    render_ascii(info)
    render_png(info, args.out)
    # read any trailing "FJFD OK"
    time.sleep(0.2)
    trailer = ser.read(ser.in_waiting).decode('latin1', errors='replace')
    if trailer.strip():
        print("Trailer:", trailer.strip())
    ser.close()

if __name__ == '__main__':
    main()
