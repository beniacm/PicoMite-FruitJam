#!/bin/bash
# Build, reboot to bootloader via CDC, flash, and optionally run a command
set -e
export PATH="/home/me/projects/rpi/toolchain/bin:$PATH"
export PICO_SDK_PATH="/home/me/projects/rpi/pico-sdk"

# Build
echo "Building..."
make -C /home/me/projects/rpi/PicoMite-FruitJam/build -j$(nproc) 2>&1 | tail -3

# Try CDC reboot first (send null byte to trigger reset_usb_boot)
echo "Rebooting to bootloader via CDC..."
python3 -c "
import serial, time, sys
for port in ['/dev/ttyACM1', '/dev/ttyACM0']:
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        ser.write(b'\x00')
        ser.close()
        print(f'Reboot sent via {port}')
        sys.exit(0)
    except: pass
print('No CDC port found, use BOOT+reset')
sys.exit(1)
" 2>/dev/null || true

# Wait for bootloader
echo "Waiting for bootloader..."
for i in $(seq 1 30); do
    if picotool info 2>/dev/null | grep -q RP2350 || ls /dev/disk/by-id/*RP* 2>/dev/null | grep -q RP2350; then
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
