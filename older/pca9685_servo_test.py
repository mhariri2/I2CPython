#!/usr/bin/env python3
"""Exercise RDS3115MG-class servos on a PCA9685 over CH341T I2C."""

from __future__ import annotations

import argparse
import time

from older.pca9685_servo_sweep import CH341I2C, PCA9685, Servo


def parse_channels(value: str) -> list[int]:
    channels: list[int] = []
    for part in value.split(","):
        channel = int(part.strip(), 0)
        if not 0 <= channel <= 15:
            raise argparse.ArgumentTypeError("servo channels must be between 0 and 15")
        channels.append(channel)
    if not channels:
        raise argparse.ArgumentTypeError("at least one servo channel is required")
    return channels


def pulse_for_angle(servo: Servo, angle: float) -> float:
    bounded_angle = max(0.0, min(180.0, angle))
    return servo.min_us + (servo.max_us - servo.min_us) * (bounded_angle / 180.0)


def move_servos(servos: list[Servo], angle: float, label: str, hold_seconds: float) -> None:
    for servo in servos:
        servo.set_angle(angle)
    pulses = [f"ch{servo.channel}={pulse_for_angle(servo, angle):.0f}us" for servo in servos]
    print(
        f"Move {label}: angle={angle:.1f} deg | channels={[servo.channel for servo in servos]} | "
        + ", ".join(pulses)
    )
    time.sleep(hold_seconds)


def main() -> int:
    parser = argparse.ArgumentParser(description="Test RDS3115MG servo channels on a PCA9685 via CH341T I2C.")
    parser.add_argument("--device-index", type=int, default=0, help="CH341 device index (default 0).")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x40, help="PCA9685 I2C address (default 0x40).")
    parser.add_argument("--dll", default=None, help="Path to CH341DLLA64.DLL or CH341DLL.DLL.")
    parser.add_argument("--freq", type=float, default=50.0, help="Servo PWM frequency in Hz (RDS3115MG default 50).")
    parser.add_argument("--channels", type=parse_channels, default=[0, 1], help="Comma-separated PCA9685 servo channels to test (default 0,1).")
    parser.add_argument("--min-angle", type=float, default=0.0, help="Minimum test angle in degrees.")
    parser.add_argument("--max-angle", type=float, default=180.0, help="Maximum test angle in degrees.")
    parser.add_argument("--center-angle", type=float, default=90.0, help="Center test angle in degrees.")
    parser.add_argument("--hold-seconds", type=float, default=1.0, help="Seconds to hold each angle.")
    parser.add_argument("--cycles", type=int, default=0, help="Number of test cycles before stopping. Use 0 for endless loop.")
    parser.add_argument("--min-us", type=float, default=500.0, help="Servo minimum pulse width in microseconds (RDS3115MG default 500).")
    parser.add_argument("--max-us", type=float, default=2500.0, help="Servo maximum pulse width in microseconds (RDS3115MG default 2500).")
    args = parser.parse_args()

    if args.max_angle < args.min_angle:
        raise ValueError("max-angle must be greater than or equal to min-angle")

    bus = CH341I2C(device_index=args.device_index, dll_path=args.dll)
    try:
        pca = PCA9685(bus, address=args.address)
        detected, mode1, mode2 = pca.verify()
        print(
            f"PCA9685 probe | CH341 index {args.device_index:08d} | I2C 0x{args.address:02X} | "
            f"MODE1=0x{mode1:02X} MODE2=0x{mode2:02X}"
        )
        if not detected:
            raise RuntimeError("PCA9685 did not respond; check power, wiring, and address")

        pca.reset()
        pca.set_pwm_freq(args.freq)
        servos = [
            Servo(
                pca,
                channel=channel,
                freq_hz=args.freq,
                min_us=args.min_us,
                max_us=args.max_us,
            )
            for channel in args.channels
        ]

        print(
            f"Testing servo channels {args.channels} at {args.freq:.1f} Hz. "
            f"RDS3115MG defaults: {args.min_us:.0f}-{args.max_us:.0f} us across {args.min_angle:.1f}-{args.max_angle:.1f} deg. "
            "Press Ctrl+C to stop."
        )

        completed_cycles = 0
        while args.cycles == 0 or completed_cycles < args.cycles:
            move_servos(servos, args.min_angle, "min", args.hold_seconds)
            move_servos(servos, args.max_angle, "max", args.hold_seconds)
            move_servos(servos, args.center_angle, "center", args.hold_seconds)
            completed_cycles += 1

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        bus.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())