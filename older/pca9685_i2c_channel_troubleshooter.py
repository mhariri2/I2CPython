#!/usr/bin/env python3
"""
Interactive CH341T/PCA9685 troubleshooting tool.

The tool tests CH341 adapter indices and PCA9685 I2C addresses. Each candidate
is checked by reading MODE1 and MODE2, then displayed for three seconds. Press
S during that window to select the candidate and configure both servo channels.
Any other key, or no key, advances to the next candidate.
"""

from __future__ import annotations

import argparse
import ctypes
import os
import sys
import time
from dataclasses import dataclass

from older.pca9685_servo_sweep import CH341I2C, PCA9685, Servo

USB_ID = r"USB\VID_1A86&PID_5512"
USB_ID_REVISION = r"USB\VID_1A86&PID_5512&REV_0304"
DEVICE_OBJECT = r"\Device\USBPDO-8"


@dataclass
class Candidate:
    device_index: int
    address: int
    mode1: int
    mode2: int


def key_pressed() -> str | None:
    if os.name == "nt":
        import msvcrt

        if msvcrt.kbhit():
            return msvcrt.getwch().upper()
        return None

    import select

    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        return sys.stdin.read(1).upper()
    return None


def wait_for_selection(seconds: float) -> bool:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        key = key_pressed()
        if key == "S":
            return True
        if key is not None and key not in ("\r", "\n"):
            print(f" ignored key {key!r}")
        time.sleep(0.05)
    return False


def candidate_addresses(first: int, last: int) -> range:
    if not 0 <= first <= last <= 0x7F:
        raise ValueError("I2C address range must be between 0x00 and 0x7F")
    return range(first, last + 1)


def scan_adapter(
    device_index: int,
    dll_path: str | None,
    first_address: int,
    last_address: int,
) -> list[Candidate]:
    bus = CH341I2C(device_index=device_index, dll_path=dll_path)
    candidates: list[Candidate] = []
    try:
        for address in candidate_addresses(first_address, last_address):
            pca = PCA9685(bus, address=address)
            try:
                detected, mode1, mode2 = pca.verify()
            except OSError:
                continue
            if detected:
                candidates.append(Candidate(device_index, address, mode1, mode2))
    finally:
        bus.close()
    return candidates


def configure_selected(candidate: Candidate, dll_path: str | None, freq_hz: float) -> None:
    bus = CH341I2C(device_index=candidate.device_index, dll_path=dll_path)
    try:
        pca = PCA9685(bus, address=candidate.address)
        pca.reset()
        pca.set_pwm_freq(freq_hz)
        servo0 = Servo(pca, channel=0, freq_hz=freq_hz)
        servo1 = Servo(pca, channel=1, freq_hz=freq_hz)
        servo0.set_angle(90.0)
        servo1.set_angle(90.0)
        print(
            f"SELECTED: CH341 index {candidate.device_index:08d}, "
            f"PCA9685 I2C address 0x{candidate.address:02X}, "
            "servo channels 0 and 1"
        )
        print("Configured 50 Hz PWM and moved both channels to 90 degrees.")
        print("Press Ctrl+C to stop this diagnostic.")
        while True:
            time.sleep(1)
    finally:
        bus.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Find and interactively select a CH341/PCA9685 I2C path."
    )
    parser.add_argument("--dll", default=None, help="Path to CH341DLLA64.DLL or CH341DLL.DLL.")
    parser.add_argument("--first-device", type=int, default=0, help="First CH341 index to test.")
    parser.add_argument("--last-device", type=int, default=7, help="Last CH341 index to test.")
    parser.add_argument("--first-address", type=lambda value: int(value, 0), default=0x40)
    parser.add_argument("--last-address", type=lambda value: int(value, 0), default=0x7F)
    parser.add_argument("--wait", type=float, default=3.0, help="Seconds per candidate (default 3).")
    parser.add_argument("--freq", type=float, default=50.0, help="Servo PWM frequency (default 50 Hz).")
    args = parser.parse_args()

    print("CH341T/PCA9685 interactive diagnostic")
    print("Press S during any candidate window to select it; otherwise the next candidate is tried.")
    print("Candidates are read-only until S selects one.")

    for device_index in range(args.first_device, args.last_device + 1):
        try:
            candidates = scan_adapter(
                device_index,
                args.dll,
                args.first_address,
                args.last_address,
            )
        except OSError as exc:
            continue

        if not candidates:
            continue

        for candidate in candidates:
            print(
                "\nDEVICE FOUND\n"
                f"  USB ID:          {USB_ID}\n"
                f"  USB revision:    {USB_ID_REVISION}\n"
                f"  Device object:   {DEVICE_OBJECT} (label only; CH341DLL does not expose this name)\n"
                f"  CH341 index:     {candidate.device_index:08d}\n"
                f"  I2C address:     0x{candidate.address:02X}\n"
                f"  MODE1 register:  0x{candidate.mode1:02X}\n"
                f"  MODE2 register:  0x{candidate.mode2:02X}\n"
                f"  PWM frequency:   {args.freq:.1f} Hz\n"
                "  Servo outputs:   channels 0 and 1 configured (servo presence not detectable over I2C)\n"
                f"  Select with S within {args.wait:.1f} seconds"
            )
            if wait_for_selection(args.wait):
                configure_selected(candidate, args.dll, args.freq)
                return 0
            print("  not selected; moving to the next candidate")

    print("No candidate was selected.")
    return 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nDiagnostic stopped.")
