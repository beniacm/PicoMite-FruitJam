#!/bin/bash
# Reboot Fruit Jam to bootloader via USB vendor reset interface
echo "Rebooting to bootloader..."
python3 -c "
import sys
try:
    import usb.core
    dev = usb.core.find(idVendor=0x2e8a, idProduct=0x000a)
    if dev:
        dev.ctrl_transfer(0x41, 0x01, 0, 2, None, timeout=5000)
        print('Reboot command sent via USB vendor interface')
        sys.exit(0)
except: pass
import serial
for port in ['/dev/ttyACM1', '/dev/ttyACM0']:
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        ser.write(b'\x00')
        ser.close()
        print(f'Reboot via CDC {port}')
        sys.exit(0)
    except: pass
print('No device found - use BOOT+reset')
sys.exit(1)
" 2>/dev/null

sleep 3
if picotool info 2>/dev/null | grep -q RP2350; then
    echo "Board in bootloader mode"
else
    echo "Board not in bootloader - may need BOOT+reset"
fi
