#!/usr/bin/env python3
"""
Render "Hello World" on a 240x240 ST7789-compatible panel via CH341T I2C.

This script talks to WCH CH341 DLL directly on Windows and assumes your display
board accepts an I2C control byte where:
- 0x00 indicates command stream
- 0x40 indicates data stream

Many 240x240 RGB modules are SPI-only. If this script cannot find or drive the
panel over I2C, the module may require SPI wiring and protocol.
"""

from __future__ import annotations

import argparse
import ctypes
import os
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List

from PIL import Image, ImageDraw, ImageFont


# ST7789 commands
SWRESET = 0x01
SLPOUT = 0x11
COLMOD = 0x3A
MADCTL = 0x36
CASET = 0x2A
RASET = 0x2B
RAMWR = 0x2C
INVON = 0x21
NORON = 0x13
DISPON = 0x29


@dataclass
class I2CDisplayConfig:
    device_index: int = 0
    address: int = 0x3E
    width: int = 240
    height: int = 240
    rotation: int = 0
    dll_path: str | None = None


class CH341I2C:
    def __init__(self, device_index: int = 0, dll_path: str | None = None) -> None:
        self.device_index = device_index
        self.dll = self._load_dll(dll_path)
        self._bind_api()

        handle = self.dll.CH341OpenDevice(self.device_index)
        if handle in (0, -1):
            raise OSError(
                f"Failed to open CH341 device index {self.device_index}. "
                "Check cable, driver, and index."
            )

        # 0x00 = I2C mode at standard speed for CH341 stream API.
        if self.dll.CH341SetStream(self.device_index, 0x00) == 0:
            self.dll.CH341CloseDevice(self.device_index)
            raise OSError("CH341SetStream failed. Unable to initialize I2C mode.")

        # Some CH341 DLL variants expose configurable I/O timeouts.
        if hasattr(self.dll, "CH341SetTimeout"):
            try:
                self.dll.CH341SetTimeout(self.device_index, 100, 100)
            except Exception:
                pass

    @staticmethod
    def _load_dll(dll_path: str | None) -> ctypes.WinDLL:
        candidates = []
        if dll_path:
            candidates.append(dll_path)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates.extend(
            [
                os.path.join(script_dir, "CH341DLLA64.DLL"),
                os.path.join(script_dir, "CH341DLL.DLL"),
                "CH341DLLA64.DLL",
                "CH341DLL.DLL",
            ]
        )

        last_error: Exception | None = None
        for candidate in candidates:
            try:
                return ctypes.WinDLL(candidate)
            except OSError as exc:
                last_error = exc

        raise OSError(
            "Could not load CH341 DLL. Put CH341DLLA64.DLL next to this script "
            "or pass --dll C:/path/to/CH341DLLA64.DLL"
        ) from last_error

    def _bind_api(self) -> None:
        self.dll.CH341OpenDevice.argtypes = [ctypes.c_ulong]
        self.dll.CH341OpenDevice.restype = ctypes.c_long

        self.dll.CH341CloseDevice.argtypes = [ctypes.c_ulong]
        self.dll.CH341CloseDevice.restype = ctypes.c_long

        self.dll.CH341SetStream.argtypes = [ctypes.c_ulong, ctypes.c_ulong]
        self.dll.CH341SetStream.restype = ctypes.c_long

        self.dll.CH341StreamI2C.argtypes = [
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_void_p,
        ]
        self.dll.CH341StreamI2C.restype = ctypes.c_long

        if hasattr(self.dll, "CH341SetTimeout"):
            self.dll.CH341SetTimeout.argtypes = [ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]
            self.dll.CH341SetTimeout.restype = ctypes.c_long

    def close(self) -> None:
        self.dll.CH341CloseDevice(self.device_index)

    def write(self, addr_7bit: int, payload: bytes) -> None:
        tx = bytes([(addr_7bit << 1) & 0xFE]) + payload
        buf = (ctypes.c_ubyte * len(tx))(*tx)
        ok = self.dll.CH341StreamI2C(self.device_index, len(tx), buf, 0, None)
        if ok == 0:
            raise OSError(f"I2C write failed at address {hex(addr_7bit)}")

    def probe(self, addr_7bit: int) -> bool:
        tx = bytes([(addr_7bit << 1) & 0xFE])
        buf = (ctypes.c_ubyte * len(tx))(*tx)
        ok = self.dll.CH341StreamI2C(self.device_index, len(tx), buf, 0, None)
        return ok != 0


class I2CSt7789:
    def __init__(self, cfg: I2CDisplayConfig) -> None:
        self.cfg = cfg
        self.link = CH341I2C(device_index=cfg.device_index, dll_path=cfg.dll_path)

    def close(self) -> None:
        self.link.close()

    def _write(self, control: int, payload: Iterable[int] | bytes, chunk: int = 16) -> None:
        data = payload if isinstance(payload, (bytes, bytearray)) else bytes(payload)
        for i in range(0, len(data), chunk):
            frame = bytes([control]) + data[i : i + chunk]
            self.link.write(self.cfg.address, frame)

    def cmd(self, command: int, data: Iterable[int] | None = None) -> None:
        self._write(0x00, [command])
        if data is not None:
            self._write(0x40, data)

    def init(self) -> None:
        self.cmd(SWRESET)
        time.sleep(0.15)

        self.cmd(SLPOUT)
        time.sleep(0.15)

        self.cmd(COLMOD, [0x55])  # 16-bit color

        madctl = {
            0: 0x00,
            90: 0x60,
            180: 0xC0,
            270: 0xA0,
        }.get(self.cfg.rotation, 0x00)
        self.cmd(MADCTL, [madctl])

        self.cmd(INVON)
        self.cmd(NORON)
        time.sleep(0.01)
        self.cmd(DISPON)
        time.sleep(0.05)

    def set_window(self, x0: int, y0: int, x1: int, y1: int) -> None:
        self.cmd(CASET, [(x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF])
        self.cmd(RASET, [(y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF])
        self.cmd(RAMWR)

    @staticmethod
    def _to_rgb565_bytes(img: Image.Image) -> bytes:
        rgb = img.convert("RGB")
        out = bytearray()
        raw = rgb.tobytes()
        for i in range(0, len(raw), 3):
            r = raw[i]
            g = raw[i + 1]
            b = raw[i + 2]
            value = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            out.append((value >> 8) & 0xFF)
            out.append(value & 0xFF)
        return bytes(out)

    def display_image(self, img: Image.Image) -> None:
        if img.size != (self.cfg.width, self.cfg.height):
            img = img.resize((self.cfg.width, self.cfg.height), Image.Resampling.BICUBIC)

        self.set_window(0, 0, self.cfg.width - 1, self.cfg.height - 1)
        pixel_bytes = self._to_rgb565_bytes(img)
        self._write(0x40, pixel_bytes)

    def display_image_at(self, img: Image.Image, x: int, y: int) -> None:
        w, h = img.size
        if x < 0 or y < 0 or (x + w) > self.cfg.width or (y + h) > self.cfg.height:
            raise ValueError("Patch image is out of display bounds")

        self.set_window(x, y, x + w - 1, y + h - 1)
        pixel_bytes = self._to_rgb565_bytes(img)
        self._write(0x40, pixel_bytes)


def scan_i2c(device_index: int, dll_path: str | None) -> List[int]:
    found: List[int] = []
    link = CH341I2C(device_index=device_index, dll_path=dll_path)
    try:
        for addr in range(0x03, 0x78):
            if link.probe(addr):
                found.append(addr)
    finally:
        link.close()
    return found


def make_hello_image(width: int = 180, height: int = 64) -> Image.Image:
    img = Image.new("RGB", (width, height), color=(8, 16, 32))
    draw = ImageDraw.Draw(img)

    # Draw a simple gradient banner to prove color writes are working.
    for y in range(0, min(56, height)):
        c = min(255, 40 + (y * 3))
        draw.line([(0, y), (width - 1, y)], fill=(c, 30, 10))

    try:
        title_font = ImageFont.truetype("arial.ttf", 36)
        body_font = ImageFont.truetype("arial.ttf", 22)
    except OSError:
        title_font = ImageFont.load_default()
        body_font = ImageFont.load_default()

    title = "Hello World"
    subtitle = "CH341 + I2C"

    tw, th = draw.textbbox((0, 0), title, font=title_font)[2:]
    sw, sh = draw.textbbox((0, 0), subtitle, font=body_font)[2:]

    tx = (width - tw) // 2
    ty = max(4, (height - th) // 2 - 10)
    sx = (width - sw) // 2
    sy = ty + th + 14

    draw.text((tx, ty), title, font=title_font, fill=(255, 255, 255))
    draw.text((sx, sy), subtitle, font=body_font, fill=(180, 210, 255))

    draw.rectangle([(4, 4), (width - 5, height - 5)], outline=(90, 140, 255), width=2)

    return img


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Write Hello World to a 240x240 I2C display")
    p.add_argument("--index", type=int, default=0, help="CH341 device index (default: 0)")
    p.add_argument("--dll", type=str, default=None, help="Path to CH341 DLL file")
    p.add_argument("--addr", type=lambda s: int(s, 0), default=0x3E, help="I2C address, e.g. 0x3E")
    p.add_argument("--rotation", type=int, default=0, choices=[0, 90, 180, 270], help="Display rotation")
    p.add_argument("--scan", action="store_true", help="Scan and print I2C addresses, then continue")
    p.add_argument(
        "--full-frame",
        action="store_true",
        help="Send a full 240x240 frame (slower, more likely to stall on CH341)",
    )
    p.add_argument(
        "--skip-sanity-check",
        action="store_true",
        help="Skip I2C bus sanity validation before sending display data",
    )
    return p.parse_args()


def sanity_check_bus(device_index: int, dll_path: str | None) -> None:
    devices = scan_i2c(device_index, dll_path)
    if len(devices) == 0:
        raise OSError("No responding I2C addresses detected.")

    # CH341 + floating lines or no target often reports ACK for almost all addresses.
    if len(devices) > 32:
        raise OSError(
            "I2C bus sanity check failed: too many addresses ACKed "
            f"({len(devices)}). This usually means no real I2C target is present "
            "or wiring/protocol is incorrect."
        )


def main() -> int:
    args = parse_args()

    if args.scan:
        print(f"Scanning I2C via CH341 device index {args.index}...")
        devices = scan_i2c(args.index, args.dll)
        print("Note: CH341 scan can report false positives on some driver versions.")
        if not devices:
            print("No I2C devices found.")
        else:
            print("Found:", ", ".join(hex(a) for a in devices))

    cfg = I2CDisplayConfig(
        device_index=args.index,
        address=args.addr,
        rotation=args.rotation,
        dll_path=args.dll,
    )

    if not args.skip_sanity_check:
        try:
            sanity_check_bus(args.index, args.dll)
        except OSError as exc:
            print(f"I2C sanity check failed: {exc}")
            print("This strongly suggests no valid I2C display target is reachable.")
            print("If you still want to force a transfer test, use --skip-sanity-check.")
            return 1

    display = I2CSt7789(cfg)

    try:
        display.init()
        if args.full_frame:
            frame = make_hello_image(cfg.width, cfg.height)
            display.display_image(frame)
        else:
            patch = make_hello_image()
            px = (cfg.width - patch.width) // 2
            py = (cfg.height - patch.height) // 2
            display.display_image_at(patch, px, py)
        print(
            f"Transfer sent to I2C address {hex(cfg.address)} using CH341 device "
            f"index {cfg.device_index}."
        )
        print(
            "Note: CH341StreamI2C does not always provide reliable ACK-level "
            "verification on all driver/DLL combinations."
        )
        return 0
    except KeyboardInterrupt:
        print("Aborted by user while transferring display data.")
        return 130
    except OSError as exc:
        print(f"I2C communication failed: {exc}")
        print("Tips:")
        print("- Try --scan to verify address")
        print("- Ensure CH341 driver and CH341DLLA64.DLL are installed")
        print("- Confirm your display board truly supports I2C (many are SPI-only)")
        return 1
    finally:
        display.close()


if __name__ == "__main__":
    sys.exit(main())
