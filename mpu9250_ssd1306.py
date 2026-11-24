#!/usr/bin/env python3
"""Display live MPU9250 readings on the project's CH341 SSD1306 display."""

import argparse
import math
import time
from typing import Any

from PIL import Image, ImageDraw, ImageFont

from mpu9250Controls import (
    SMBus,
    find_ch341_bus,
    init_mag,
    init_mpu,
    read_mag,
    read_mpu,
)


WIDTH = 128
HEIGHT = 64
UPDATE_SECONDS = 0.25
DIRECTIONS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    """Use a compact font that fits the full sensor dashboard."""
    try:
        return ImageFont.truetype(r"C:\Windows\Fonts\consola.ttf", size)
    except OSError:
        return ImageFont.load_default()


def _vector_text(label: str, values: tuple[float, float, float], decimals: int) -> str:
    value_format = f"{{:+.{decimals}f}}"
    return f"{label} " + " ".join(value_format.format(value) for value in values)


def _heading(magnetometer: tuple[float, float, float]) -> tuple[float, str]:
    degrees = math.degrees(math.atan2(magnetometer[1], magnetometer[0])) % 360.0
    direction = DIRECTIONS[round(degrees / 45.0) % len(DIRECTIONS)]
    return degrees, direction


def create_screen(
    accel: tuple[float, float, float],
    gyro: tuple[float, float, float],
    temperature: float,
    magnetometer: tuple[float, float, float] | None,
    font: ImageFont.FreeTypeFont | ImageFont.ImageFont | None = None,
) -> Image.Image:
    """Render one sensor sample into a 128x64 monochrome display image."""
    image = Image.new("1", (WIDTH, HEIGHT), 0)
    draw = ImageDraw.Draw(image)
    text_font = font or _font(9)
    lines = [
        "MPU9250 LIVE",
        _vector_text("A", accel, 2),
        _vector_text("G", gyro, 1),
        _vector_text("M", magnetometer, 1) if magnetometer else "M  --  --  --",
        f"T {temperature:5.1f} C",
    ]
    if magnetometer:
        degrees, direction = _heading(magnetometer)
        lines[-1] += f"  H {degrees:03.0f}{direction}"
    else:
        lines[-1] += "  H ---"

    line_height = 11
    for index, line in enumerate(lines):
        draw.text((1, index * line_height), line, font=text_font, fill=1)
    draw.line((0, 55, WIDTH - 1, 55), fill=1)
    draw.text((1, 56), "A:g  G:dps  M:uT", font=text_font, fill=1)
    return image


def _read_sample(bus: Any, magnetometer_adjustment: list[float] | None):
    accel, gyro, temperature = read_mpu(bus)
    magnetometer = (
        read_mag(bus, magnetometer_adjustment)
        if magnetometer_adjustment
        else None
    )
    return accel, gyro, temperature, magnetometer


def run(bus_number: int | None, interval: float) -> None:
    """Initialize both devices and continuously refresh the display."""
    import SSDI2cDisplayBAse as display

    selected_bus = find_ch341_bus() if bus_number is None else bus_number
    display_open = False
    try:
        display.ch341_open()
        display_open = True
        display.ssd1306_init()
        display.ssd1306_clear()

        with SMBus(selected_bus) as bus:
            who_am_i = init_mpu(bus)
            magnetometer_adjustment = init_mag(bus)
            print(
                f"MPU9250 0x{who_am_i:02X} on bus {selected_bus}; "
                f"magnetometer={'yes' if magnetometer_adjustment else 'no'}"
            )
            text_font = _font(9)
            try:
                while True:
                    sample = _read_sample(bus, magnetometer_adjustment)
                    display.ssd1306_display(create_screen(*sample, font=text_font))
                    time.sleep(interval)
            except KeyboardInterrupt:
                print("\nStopping display.")
    finally:
        if display_open:
            display.ssd1306_command(0xAE)
            display.ch341_close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bus",
        nargs="?",
        type=int,
        help="I2C bus number; omit to auto-detect a CH341 bus",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=UPDATE_SECONDS,
        help=f"seconds between updates (default: {UPDATE_SECONDS})",
    )
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be greater than zero")
    run(args.bus, args.interval)


if __name__ == "__main__":
    main()
