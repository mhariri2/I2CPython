#!/usr/bin/env python3
"""
Drive two servos (channels 0 and 1) on a PCA9685 16-channel 12-bit PWM
controller over I2C, using a CH341T USB-I2C adapter on Windows.

Talks to the WCH CH341 DLL directly via ctypes (no fcntl/smbus2 dependency,
which is Linux-only).

Wiring assumption: CH341T SDA/SCL -> PCA9685 SDA/SCL, servos on PCA9685
channel 0 and channel 1 outputs, PCA9685 V+ powered from an external 5-6V
supply (not from the CH341T), grounds common.
"""

from __future__ import annotations

import argparse
import ctypes
import os
import sys
import time

EXPECTED_DEVICE_OBJECT = r"\Device\USBPDO-8"


class CH341I2C:
    """Minimal CH341 I2C bus wrapper via CH341DLL (Windows)."""

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

    def write_read(self, addr_7bit: int, payload: bytes, read_len: int) -> bytes:
        tx = bytes([(addr_7bit << 1) & 0xFE]) + payload
        tx_buf = (ctypes.c_ubyte * len(tx))(*tx)
        rx_buf = (ctypes.c_ubyte * read_len)()
        ok = self.dll.CH341StreamI2C(self.device_index, len(tx), tx_buf, read_len, rx_buf)
        if ok == 0:
            raise OSError(f"I2C write_read failed at address {hex(addr_7bit)}")
        return bytes(rx_buf)


# PCA9685 register map
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06

MODE1_SLEEP = 0x10
MODE1_AUTOINCREMENT = 0x20
MODE1_RESTART = 0x80
MODE2_OUTDRV = 0x04


class PCA9685:
    def __init__(self, bus: CH341I2C, address: int = 0x40) -> None:
        self.bus = bus
        self.address = address

    def _write8(self, reg: int, value: int) -> None:
        self.bus.write(self.address, bytes([reg, value & 0xFF]))

    def _read8(self, reg: int) -> int:
        return self.bus.write_read(self.address, bytes([reg]), 1)[0]

    def verify(self) -> tuple[bool, int, int]:
        mode1 = self._read8(PCA9685_MODE1)
        mode2 = self._read8(PCA9685_MODE2)
        detected = mode1 != 0xFF and mode2 != 0xFF
        return detected, mode1, mode2

    def reset(self) -> None:
        self._write8(PCA9685_MODE1, MODE1_AUTOINCREMENT)
        time.sleep(0.005)

    def set_pwm_freq(self, freq_hz: float) -> None:
        """Set PWM frequency (Hz). Typical servo range is 50-60 Hz."""
        prescale_val = round(25_000_000.0 / (4096 * freq_hz)) - 1
        prescale_val = max(3, min(255, prescale_val))

        old_mode = self._read8(PCA9685_MODE1)
        sleep_mode = (old_mode & 0x7F) | MODE1_SLEEP
        self._write8(PCA9685_MODE1, sleep_mode)
        self._write8(PCA9685_PRESCALE, prescale_val)
        self._write8(PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        self._write8(PCA9685_MODE1, old_mode | MODE1_RESTART | MODE1_AUTOINCREMENT)
        self._write8(PCA9685_MODE2, MODE2_OUTDRV)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        reg = PCA9685_LED0_ON_L + 4 * channel
        payload = bytes(
            [
                reg,
                on & 0xFF,
                (on >> 8) & 0xFF,
                off & 0xFF,
                (off >> 8) & 0xFF,
            ]
        )
        self.bus.write(self.address, payload)

    def set_pulse_us(self, channel: int, pulse_us: float, freq_hz: float) -> None:
        period_us = 1_000_000.0 / freq_hz
        off = round((pulse_us / period_us) * 4096)
        off = max(0, min(4095, off))
        self.set_pwm(channel, 0, off)


class Servo:
    """Maps 0-180 degree angles to PCA9685 pulse widths."""

    def __init__(
        self,
        pca: PCA9685,
        channel: int,
        freq_hz: float = 50.0,
        min_us: float = 500.0,
        max_us: float = 2500.0,
    ) -> None:
        self.pca = pca
        self.channel = channel
        self.freq_hz = freq_hz
        self.min_us = min_us
        self.max_us = max_us

    def set_angle(self, angle_deg: float) -> None:
        angle_deg = max(0.0, min(180.0, angle_deg))
        pulse_us = self.min_us + (self.max_us - self.min_us) * (angle_deg / 180.0)
        self.pca.set_pulse_us(self.channel, pulse_us, self.freq_hz)


def sweep_loop(
    servo0: Servo,
    servo1: Servo,
    step_deg: float,
    step_delay: float,
    device_index: int,
    i2c_address: int,
) -> None:
    angle = 0.0
    direction = 1
    print(
        f"CH341T USB device index={device_index:08d} | "
        "USB\\VID_1A86&PID_5512&REV_0304 | USB\\VID_1A86&PID_5512"
    )
    print(
        f"Windows device object={EXPECTED_DEVICE_OBJECT} | "
        "status=UNVERIFIED by CH341DLL; selected by CH341 index"
    )
    print(
        f"PCA9685 I2C address=0x{i2c_address:02X} | "
        f"servo channels={servo0.channel},{servo1.channel}"
    )
    print("Running servo sweep loop. Press Ctrl+C to stop.")
    while True:
        servo0.set_angle(angle)
        servo1.set_angle(180.0 - angle)
        print(
            f"I2C 0x{i2c_address:02X} | "
            f"channel {servo0.channel}: {angle:6.1f} deg | "
            f"channel {servo1.channel}: {180.0 - angle:6.1f} deg",
            end="\r",
            flush=True,
        )
        time.sleep(step_delay)

        angle += direction * step_deg
        if angle >= 180.0:
            angle = 180.0
            direction = -1
        elif angle <= 0.0:
            angle = 0.0
            direction = 1


def scan_ch341_devices(dll_path: str | None, address: int, max_index: int) -> None:
    print(f"Scanning CH341 indices 0-{max_index} for PCA9685 at I2C 0x{address:02X}...")
    for index in range(max_index + 1):
        bus = None
        try:
            bus = CH341I2C(device_index=index, dll_path=dll_path)
            pca = PCA9685(bus, address=address)
            detected, mode1, mode2 = pca.verify()
            result = "PASS - PCA9685 response" if detected else "FAIL - no PCA9685 response"
            print(
                f"  CH341 index {index:08d}: {result} "
                f"(MODE1=0x{mode1:02X}, MODE2=0x{mode2:02X})"
            )
        except (OSError, ValueError) as exc:
            print(f"  CH341 index {index:08d}: FAIL - {exc}")
        finally:
            if bus is not None:
                bus.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep two servos on a PCA9685 via CH341T I2C.")
    parser.add_argument(
        "--device-index",
        type=int,
        default=0,
        help="CH341 device index (default 0, shown as 00000000).",
    )
    parser.add_argument("--dll", type=str, default=None, help="Path to CH341DLLA64.DLL / CH341DLL.DLL.")
    parser.add_argument("--address", type=lambda v: int(v, 0), default=0x40, help="PCA9685 I2C address (default 0x40).")
    parser.add_argument("--freq", type=float, default=50.0, help="PWM frequency in Hz (default 50).")
    parser.add_argument("--step-deg", type=float, default=2.0, help="Degrees per step (default 2).")
    parser.add_argument("--step-delay", type=float, default=0.02, help="Delay between steps in seconds (default 0.02).")
    parser.add_argument("--min-us", type=float, default=500.0, help="Servo minimum pulse width in microseconds.")
    parser.add_argument("--max-us", type=float, default=2500.0, help="Servo maximum pulse width in microseconds.")
    parser.add_argument(
        "--scan-max-index",
        type=int,
        default=3,
        help="Highest CH341 index to check before starting (default 3).",
    )
    args = parser.parse_args()

    scan_ch341_devices(args.dll, args.address, args.scan_max_index)
    bus = CH341I2C(device_index=args.device_index, dll_path=args.dll)
    try:
        pca = PCA9685(bus, address=args.address)
        detected, mode1, mode2 = pca.verify()
        print(
            f"Selected CH341 index {args.device_index:08d}: "
            f"MODE1=0x{mode1:02X}, MODE2=0x{mode2:02X}"
        )
        if not detected:
            raise RuntimeError(
                f"Selected CH341 index {args.device_index:08d} did not respond as a PCA9685 "
                f"at I2C address 0x{args.address:02X}; servo sweep not started."
            )
        print("Verification PASS: selected CH341 index and PCA9685 I2C address responded.")
        pca.reset()
        pca.set_pwm_freq(args.freq)

        servo0 = Servo(pca, channel=0, freq_hz=args.freq, min_us=args.min_us, max_us=args.max_us)
        servo1 = Servo(pca, channel=1, freq_hz=args.freq, min_us=args.min_us, max_us=args.max_us)

        try:
            sweep_loop(
                servo0,
                servo1,
                args.step_deg,
                args.step_delay,
                args.device_index,
                args.address,
            )
        except KeyboardInterrupt:
            print("\nStopping.")
    finally:
        bus.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
