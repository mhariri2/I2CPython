# CH341 display probe

This repository contains a Windows-first Python script that first verifies CH341 availability, scans the I2C bus, and then tries several common I2C display controller families over a CH341-based adapter to write `HELLO WORLD`.

## What it tries

- SSD1306 at `0x3C` and `0x3D`
- SH1106 at `0x3C` and `0x3D`
- ST7032 text LCD at `0x3E` and `0x3F`
- HD44780 text LCDs through PCF8574 backpacks at `0x20-0x27` and `0x38-0x3F`
- Multiple CH341 stream modes intended to cover the common I2C speed encodings

The script now runs in three phases:

- Opens CH341 device `0` and verifies that at least one I2C stream mode is usable
- Scans I2C addresses `0x03` through `0x77` for each supported CH341 stream mode and prints the responding devices
- Restricts the display loop to controller/address combinations that match the devices found during the scan

After each write attempt, the script waits three seconds. Press `s` during that window if the panel shows the text. If not, it automatically moves to the next discovered candidate.

## Requirements

- Windows
- Python 3.11+
- `CH341DLL.DLL` available either next to the script or in `C:\Windows\System32`

## Run

```powershell
python .\probe_ch341_display.py
```

## Important limitation

Many 240x240 RGB modules label SPI lines as `SCL` and `SDA`. If your panel is one of those, SDA/SCL alone is not enough to drive the display because a TFT controller such as ST7789 usually also needs `CS`, `DC`, and often `RST`.