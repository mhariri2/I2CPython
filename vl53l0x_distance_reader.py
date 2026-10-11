#!/usr/bin/env python3
"""Continuously read distance from a CJVL53L0XV2 / VL53L0X over CH341T I2C."""

from __future__ import annotations

import argparse
import statistics
import time

from pca9685_servo_sweep import CH341I2C
from vl53l0x_configuration_tester import Sensor


VL53L0X_DEFAULT_ADDRESS = 0x29
SYSRANGE_START = 0x00
SYSTEM_SEQUENCE_CONFIG = 0x01
SYSTEM_INTERRUPT_CLEAR = 0x0B
RESULT_INTERRUPT_STATUS = 0x13
RESULT_RANGE_STATUS = 0x14


class VL53L0X:
    def __init__(self, bus: CH341I2C, address: int = VL53L0X_DEFAULT_ADDRESS) -> None:
        self.bus = bus
        self.address = address

    def write8(self, register: int, value: int) -> None:
        self.bus.write(self.address, bytes([register & 0xFF, value & 0xFF]))

    def read8(self, register: int) -> int:
        return self.bus.write_read(self.address, bytes([register & 0xFF]), 1)[0]

    def read16(self, register: int) -> int:
        data = self.bus.write_read(self.address, bytes([register & 0xFF]), 2)
        return (data[0] << 8) | data[1]

    def verify(self) -> int:
        """Read the model identification register used by VL53L0X modules."""
        return self.read8(0xC0)

    def initialize(self) -> None:
        # This minimal sequence enables the standard single-ranging path.
        self.write8(0x80, 0x01)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)
        self.write8(SYSTEM_SEQUENCE_CONFIG, 0xFF)
        self.write8(SYSRANGE_START, 0x01)
        time.sleep(0.01)
        self.write8(SYSRANGE_START, 0x00)

    def read_distance_mm(self, timeout_s: float = 1.0) -> int:
        self.write8(SYSRANGE_START, 0x01)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.read8(RESULT_INTERRUPT_STATUS)
            if status & 0x07:
                distance = self.read16(RESULT_RANGE_STATUS + 10)
                self.write8(SYSTEM_INTERRUPT_CLEAR, 0x01)
                self.write8(SYSRANGE_START, 0x00)
                return distance
            time.sleep(0.005)
        raise TimeoutError("VL53L0X range measurement timed out")


def main() -> int:
    parser = argparse.ArgumentParser(description="Continuously read CJVL53L0XV2 distance via CH341T I2C.")
    parser.add_argument("--device-index", type=int, default=0, help="CH341 device index (default 0).")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=VL53L0X_DEFAULT_ADDRESS, help="VL53L0X I2C address (default 0x29).")
    parser.add_argument("--dll", default=None, help="Path to CH341DLLA64.DLL or CH341DLL.DLL.")
    parser.add_argument("--interval", type=float, default=0.1, help="Seconds between readings (default 0.1).")
    parser.add_argument("--timeout", type=float, default=1.5, help="Seconds to wait for a reading before retrying initialization.")
    parser.add_argument("--zero-offset-mm", type=float, default=23.0, help="Subtract this offset from each reading and clamp at 0 mm.")
    parser.add_argument("--calibrate-zero", action="store_true", help="Measure a recommended zero offset from the current target position and exit.")
    parser.add_argument("--calibration-samples", type=int, default=15, help="Number of valid samples to average during zero calibration.")
    args = parser.parse_args()

    def read_once(sensor: Sensor, mode: str) -> int:
        return sensor.single_read(timeout_s=args.timeout) if mode == "single" else sensor.continuous_read(timeout_s=args.timeout)

    def apply_zero_offset(distance_mm: int) -> float:
        return max(0.0, distance_mm - args.zero_offset_mm)

    def initialize_sensor(sensor: Sensor) -> tuple[str, int]:
        last_error: Exception | None = None
        for mode in ("single", "continuous"):
            for _ in range(2):
                try:
                    sensor.standard_init()
                    sensor.configure_default_ranging()
                    return mode, read_once(sensor, mode)
                except (OSError, TimeoutError, ValueError) as exc:
                    last_error = exc
                    time.sleep(0.1)
        assert last_error is not None
        raise last_error

    bus = CH341I2C(device_index=args.device_index, dll_path=args.dll)
    try:
        sensor = Sensor(bus, address=args.address)
        model_id = sensor.model_id()
        print(f"VL53L0X found | CH341 index {args.device_index:08d} | I2C address 0x{args.address:02X} | model ID 0x{model_id:02X}")
        mode, distance_mm = initialize_sensor(sensor)

        if args.calibrate_zero:
            samples: list[int] = [distance_mm]
            print(
                "Zero calibration: hold the target at your mechanical zero position. "
                f"Collecting {args.calibration_samples} valid samples using {mode} mode..."
            )
            while len(samples) < args.calibration_samples:
                try:
                    time.sleep(args.interval)
                    samples.append(read_once(sensor, mode))
                except (TimeoutError, ValueError, OSError):
                    mode, distance_mm = initialize_sensor(sensor)
                    if not samples:
                        samples.append(distance_mm)
            recommended_offset = statistics.mean(samples)
            print(f"Recommended zero offset: {recommended_offset:.1f} mm from samples {samples}")
            print(
                "Use it like: "
                f"python .\\vl53l0x_distance_reader.py --zero-offset-mm {recommended_offset:.1f}"
            )
            return 0

        print(f"Reading distance in millimeters using {mode} mode. Press Ctrl+C to stop.")
        while True:
            try:
                adjusted_mm = apply_zero_offset(distance_mm)
                print(
                    f"Distance: raw={distance_mm:4d} mm adjusted={adjusted_mm:6.1f} mm ({adjusted_mm / 10.0:6.1f} cm)",
                    end="\r",
                    flush=True,
                )
                time.sleep(args.interval)
                distance_mm = read_once(sensor, mode)
                continue
            except (TimeoutError, ValueError, OSError) as exc:
                print(f"\nWarning: {exc}; probing sensor again...", flush=True)
                try:
                    mode, distance_mm = initialize_sensor(sensor)
                    print(f"Recovered with {mode} mode.")
                except (TimeoutError, ValueError, OSError) as reinit_exc:
                    print(f"Reinitialization failed: {reinit_exc}")
                    time.sleep(max(args.interval, 0.5))
                    continue
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
