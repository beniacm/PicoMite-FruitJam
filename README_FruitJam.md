# PicoMite HDMI for Adafruit Fruit Jam

PicoMite MMBasic 6.02.02b0 ported to the [Adafruit Fruit Jam](https://www.adafruit.com/product/6363) (RP2350B) with HDMI display, USB keyboard via USB-A host port, I2S audio, and WiFi via ESP32-C6.

Based on [PicoMiteAllVersions](https://github.com/UKTailwind/PicoMiteAllVersions) with [PR#25](https://github.com/UKTailwind/PicoMiteAllVersions/pull/25) TLV320DAC3100 codec support by inindev, plus PIO-USB keyboard host.

## Quick Start

1. Hold **BOOT** button, connect USB-C to PC
2. Flash `PicoMiteHDMI_FruitJam.uf2` via `picotool load -x PicoMiteHDMI_FruitJam.uf2` or copy to the RPI-RP2 drive
3. Connect HDMI monitor and USB keyboard to the USB-A port
4. At the `>` prompt, run: `CONFIGURE FRUIT JAM`
5. Power cycle the board

## What Works

| Feature | Status | Details |
|---------|--------|---------|
| HDMI output | Working | 640x480 @ 60Hz via HSTX (GP12-19) |
| USB keyboard | Working | Direct USB-A port via PIO-USB (GP1/GP2) |
| USB mouse | Working | Via USB-A port |
| I2S audio | Working | TLV320DAC3100 codec, speaker + headphone jack |
| SD card | Working | GP34(CLK), GP35(MOSI), GP36(MISO), GP39(CS) |
| WiFi | Working | ESP32-C6 via NINA SPI protocol |
| NTP time sync | Working | `WEB NTP offset` sets RTC |
| TCP client | Working | Connect, send/receive, close |
| Heartbeat LED | Working | GP29 |
| Flash save (F1) | Working | Keyboard briefly disconnects, reconnects after ~2s |

## Audio Test

```basic
PLAY TONE 440, 440, 1000
PLAY VOLUME 50, 50
```

## WiFi

The Fruit Jam has an ESP32-C6 co-processor running NINA firmware for WiFi. Commands match the PicoMiteWEB syntax.

### WiFi Commands

```basic
' Connect to WiFi
WEB CONNECT "SSID", "password"
PRINT MM.INFO(IP ADDRESS)

' Sync time via NTP (offset = hours from UTC)
WEB NTP 2

' TCP client
WEB OPEN TCP CLIENT "hostname", port
DIM INTEGER r%(100)
WEB TCP CLIENT REQUEST "data to send", r%()
PRINT "Received bytes:", r%(0)
WEB CLOSE TCP CLIENT

' Disconnect
WEB DISCONNECT
```

### WiFi Info Functions

| Function | Returns |
|----------|---------|
| `MM.INFO(IP ADDRESS)` | IP address string (e.g., "192.168.1.100") |
| `MM.INFO(WIFI STATUS)` | Connection status (3=connected, 6=disconnected) |
| `MM.INFO(WIFI VERSION)` | NINA firmware version string |

### TCP CLIENT REQUEST

The response is stored in an integer array:
- `r%(0)` = number of bytes received
- `r%(1)..r%(n)` = response data as raw bytes (8 bytes per integer, little-endian)

Optional timeout (ms): `WEB TCP CLIENT REQUEST "data", r%(), 10000`

## Pin Configuration (set by CONFIGURE FRUIT JAM)

| Function | Pins |
|----------|------|
| HDMI (HSTX) | CK=GP13, D0=GP15, D1=GP17, D2=GP19 |
| USB keyboard (PIO-USB) | D+=GP1, D-=GP2, VBUS=GP11 |
| I2S audio | BCLK=GP26, WS=GP27, DIN=GP24, MCLK=GP25 |
| SD card (SPI) | SCK=GP34, MOSI=GP35, MISO=GP36, CS=GP39 |
| System I2C | SDA=GP20, SCL=GP21 |
| Serial console | TX=GP8, RX=GP9 (UART1, optional) |
| Heartbeat LED | GP29 |
| ESP32-C6 WiFi (SPI1) | SCK=GP30, MOSI=GP31, MISO=GP28, CS=GP46 |
| ESP32-C6 control | ACK=GP3, RESET=GP22 (shared with codec) |
| Codec reset | GP22 (shared with ESP32-C6) |

## Known Limitations

### Flash save keyboard reconnect
Flash write operations (F1 save, CONFIGURE, OPTION commands) temporarily disconnect the USB keyboard. It reconnects automatically after ~2 seconds. This is inherent to PIO-USB: flash erase disables interrupts for ~50ms per sector, and PIO-USB's SOF timer (software-based) can't fire during that period. Hardware USB (used by the standard HDMIUSB build) doesn't have this issue since its SOF is hardware-generated.

### PIO-USB timing sensitivity
The Pico-PIO-USB library has known infinite loop bugs when timing is disrupted ([#192](https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/192), [#197](https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/197)). In rare cases, `pio_usb_bus_usb_transfer()` or `pio_usb_bus_start_receive()` can hang. If the board becomes unresponsive, power cycle it.

### Software reset
The `__uninitialized_ram` section doesn't survive watchdog resets on RP2350B. Watchdog scratch registers are used to preserve `_excep_code` across resets.

## Building from Source

### Prerequisites
- ARM GCC toolchain (14.x recommended)
- Pico SDK 2.2.0
- [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) (cloned into the source directory)

### Build

```bash
git clone https://github.com/UKTailwind/PicoMiteAllVersions.git PicoMite-FruitJam
cd PicoMite-FruitJam
git clone https://github.com/sekigon-gonnoc/Pico-PIO-USB.git
# Apply Fruit Jam patches (this repository's changes)
mkdir build && cd build
cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j$(nproc)
```

### Key Changes from Upstream PicoMite

| File | Change |
|------|--------|
| CMakeLists.txt | FRUITJAM build target with PIO-USB, ESP32 WiFi, CDC serial |
| esp32_wifi.c | WiFi via ESP32-C6 NINA SPI protocol (connect, TCP, NTP) |
| usb_cdc_descriptors.c | USB CDC device descriptors for serial over USB-C |
| configuration.h | FLASH_TARGET_OFFSET 1MB (firmware ~960KB) |
| Custom.c | TLV320DAC3100 codec init (MCLK PWM, I2C register sequence) |
| PicoMite.c | PIO-USB init, HSTX bus priority, clk_hstx config, codec calls, flash save stop/restart |
| MM_Misc.c | CONFIGURE FRUIT JAM command, HMDI→HDMI typo fix |
| FileIO.c | HDMI pin defaults in ResetOptions |
| Serial.c | CDC host stubs for CFG_TUH_CDC=0 |
| usb_host_files/tusb_config.h | PIO-USB config (BOARD_TUH_RHPORT=1) |
| Pico-PIO-USB/src/pio_usb_host.c | Fix stop/restart for flash compatibility (issue #192) |

## Alternative: Hardware USB via USB-C Hub

If you prefer hardware USB (no PIO-USB limitations), build with the standard HDMIUSB target and the Fruit Jam board:

```bash
cmake -DCOMPILE=HDMIUSB -DBOARD=adafruit_fruit_jam -DPICO_SDK_PATH=/path/to/pico-sdk ..
```

Connect a USB keyboard via a **powered USB-C hub** to the Fruit Jam's USB-C port. Serial console on GP8/GP9 (UART1) for initial setup. See [PR#25](https://github.com/UKTailwind/PicoMiteAllVersions/pull/25) for details.

## Credits

- [PicoMite MMBasic](https://github.com/UKTailwind/PicoMiteAllVersions) by Geoff Graham and Peter Mather
- [PR#25 Fruit Jam support](https://github.com/UKTailwind/PicoMiteAllVersions/pull/25) by inindev (TLV320DAC3100 codec, CONFIGURE command)
- [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) by sekigon-gonnoc
- [fruitjam-doom](https://github.com/adafruit/fruitjam-doom) (HDMI reference implementation)
