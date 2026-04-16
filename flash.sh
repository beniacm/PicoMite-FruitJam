#!/bin/bash
# Build, reboot to bootloader, flash
set -e
export PATH="/home/me/projects/rpi/toolchain/bin:$PATH"
export PICO_SDK_PATH="/home/me/projects/rpi/pico-sdk"

# Build
echo "Building..."
make -C /home/me/projects/rpi/PicoMite-FruitJam/build -j$(nproc) 2>&1 | tail -3

# Reboot to bootloader via USB vendor reset interface (most reliable)
echo "Rebooting to bootloader..."
python3 -c "
import sys
# Method 1: USB vendor reset interface (works even without CDC)
try:
    import usb.core
    dev = usb.core.find(idVendor=0x2e8a, idProduct=0x000a)
    if dev:
        dev.ctrl_transfer(0x41, 0x01, 0, 2, None, timeout=5000)
        print('Reboot via USB vendor interface')
        sys.exit(0)
except: pass
# Method 2: CDC null byte (works when serial port is available)
import serial
for port in ['/dev/ttyACM1', '/dev/ttyACM0']:
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        ser.write(b'\x00')
        ser.close()
        print(f'Reboot via CDC {port}')
        sys.exit(0)
    except: pass
print('No device found, use BOOT+reset')
sys.exit(1)
" 2>/dev/null || true

# Wait for bootloader
echo "Waiting for bootloader..."
for i in $(seq 1 30); do
    if picotool info 2>/dev/null | grep -q RP2350; then
        echo "Found!"
        sleep 1
        picotool load -x /home/me/projects/rpi/PicoMite-FruitJam/build/PicoMite.uf2 2>&1 | tail -3
        echo "Flashed and rebooted."
        exit 0
    fi
    sleep 1
done
echo "Timeout - no bootloader found"
exit 1
