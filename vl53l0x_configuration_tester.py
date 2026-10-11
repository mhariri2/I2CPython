#!/usr/bin/env python3
"""Try VL53L0X software configurations until a valid distance is returned."""

from __future__ import annotations

import argparse
import time

from pca9685_servo_sweep import CH341I2C


DEFAULT_ADDRESS = 0x29
REG_SYSRANGE_START = 0x00
REG_SYSTEM_SEQUENCE_CONFIG = 0x01
REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
REG_SYSTEM_INTERRUPT_CLEAR = 0x0B
REG_RESULT_INTERRUPT_STATUS = 0x13
REG_RESULT_RANGE_STATUS = 0x14
REG_MODEL_ID = 0xC0
REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE = 0x44
REG_GPIO_HV_MUX_ACTIVE_HIGH = 0x84
REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89
REG_MSRC_CONFIG_CONTROL = 0x60
REG_SOFT_RESET_GO2_SOFT_RESET_N = 0xBF

INVALID_RANGE_MM = 8191


class Sensor:
    def __init__(self, bus: CH341I2C, address: int) -> None:
        self.bus = bus
        self.address = address
        self.stop_variable = 0

    def write8(self, register: int, value: int) -> None:
        self.bus.write(self.address, bytes([register & 0xFF, value & 0xFF]))

    def read8(self, register: int) -> int:
        return self.bus.write_read(self.address, bytes([register & 0xFF]), 1)[0]

    def read16(self, register: int) -> int:
        data = self.bus.write_read(self.address, bytes([register & 0xFF]), 2)
        return (data[0] << 8) | data[1]

    def write16(self, register: int, value: int) -> None:
        self.bus.write(self.address, bytes([register & 0xFF, (value >> 8) & 0xFF, value & 0xFF]))

    def model_id(self) -> int:
        return self.read8(REG_MODEL_ID)

    def soft_reset(self) -> None:
        self.write8(REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00)
        time.sleep(0.005)
        self.write8(REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01)
        time.sleep(0.005)

    def set_signal_rate_limit(self, limit_mcps: float) -> None:
        if not 0.0 <= limit_mcps <= 511.99:
            raise ValueError(f"invalid signal rate limit {limit_mcps}")
        self.write16(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE, int(limit_mcps * (1 << 7)))

    def get_spad_info(self) -> tuple[int, bool]:
        self.write8(0x80, 0x01)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0xFF, 0x06)
        self.write8(0x83, self.read8(0x83) | 0x04)
        self.write8(0xFF, 0x07)
        self.write8(0x81, 0x01)
        self.write8(0x80, 0x01)
        self.write8(0x94, 0x6B)
        self.write8(0x83, 0x00)
        deadline = time.monotonic() + 1.0
        while self.read8(0x83) == 0x00:
            if time.monotonic() >= deadline:
                raise TimeoutError("VL53L0X SPAD information timed out")
        self.write8(0x83, 0x01)
        value = self.read8(0x92)
        self.write8(0x81, 0x00)
        self.write8(0xFF, 0x06)
        self.write8(0x83, self.read8(0x83) & ~0x04)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x01)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)
        return value & 0x7F, bool(value & 0x80)

    def write_tuning_settings(self) -> None:
        settings = bytes.fromhex(
            "FF 01 00 00 FF 00 09 00 10 00 11 00 24 01 25 FF 75 00 "
            "FF 01 4E 2C 48 00 30 20 FF 00 30 09 54 00 31 04 32 03 "
            "40 83 46 25 60 00 27 00 50 06 51 00 52 96 56 08 57 30 "
            "61 00 62 00 64 00 65 00 66 A0 FF 01 22 32 47 14 49 FF "
            "4A 00 FF 00 7A 0A 7B 00 78 21 FF 01 23 34 42 00 44 FF "
            "45 26 46 05 40 40 0E 06 20 1A 43 40 FF 00 34 03 35 44 "
            "FF 01 31 04 4B 09 4C 05 4D 04 FF 00 44 00 45 20 47 08 "
            "48 28 67 00 70 04 71 01 72 FE 76 00 77 00 FF 01 0D 01 "
            "FF 00 80 01 01 F8 FF 01 8E 01 00 01 FF 00 80 00"
        )
        for index in range(0, len(settings), 2):
            self.write8(settings[index], settings[index + 1])

    def finish_range_read(self) -> int:
        distance = self.read16(REG_RESULT_RANGE_STATUS + 10)
        raw_status = self.read8(REG_RESULT_RANGE_STATUS)
        self.write8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
        if distance >= INVALID_RANGE_MM:
            raise ValueError(
                f"VL53L0X returned an invalid sample ({distance} mm, range status 0x{raw_status:02X}); "
                "check wiring, target distance, and sensor orientation"
            )
        return distance

    def perform_single_ref_calibration(self, value: int) -> None:
        self.write8(REG_SYSRANGE_START, 0x01 | value)
        deadline = time.monotonic() + 1.0
        while not (self.read8(REG_RESULT_INTERRUPT_STATUS) & 0x07):
            if time.monotonic() >= deadline:
                raise TimeoutError("VL53L0X reference calibration timed out")
        self.write8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self.write8(REG_SYSRANGE_START, 0x00)

    def standard_init(self) -> None:
        """Port the vendor-style VL53L0X reference initialization sequence."""
        self.soft_reset()
        self.write8(
            REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
            self.read8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) | 0x01,
        )
        self.write8(0x88, 0x00)
        self.write8(0x80, 0x01)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.stop_variable = self.read8(0x91)
        self.write8(0x00, 0x01)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)
        self.write8(REG_MSRC_CONFIG_CONTROL, self.read8(REG_MSRC_CONFIG_CONTROL) | 0x12)
        self.set_signal_rate_limit(0.25)
        self.write8(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)
        spad_count, aperture = self.get_spad_info()
        ref_spad_map = list(self.bus.write_read(self.address, bytes([0xB0]), 6))
        self.write8(0xFF, 0x01)
        self.write8(0x4F, 0x00)
        self.write8(0x4E, 0x2C)
        self.write8(0xFF, 0x00)
        self.write8(0xB6, 0xB4)
        first_spad = 12 if aperture else 0
        enabled = 0
        for index in range(48):
            if index < first_spad or enabled == spad_count:
                ref_spad_map[index // 8] &= ~(1 << (index % 8))
            elif ref_spad_map[index // 8] & (1 << (index % 8)):
                enabled += 1
        self.bus.write(self.address, bytes([0xB0, *ref_spad_map]))
        self.write_tuning_settings()
        self.write8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        self.write8(REG_GPIO_HV_MUX_ACTIVE_HIGH, self.read8(REG_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10)
        self.write8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self.write8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self.write8(REG_SYSTEM_SEQUENCE_CONFIG, 0x01)
        self.perform_single_ref_calibration(0x40)
        self.write8(REG_SYSTEM_SEQUENCE_CONFIG, 0x02)
        self.perform_single_ref_calibration(0x00)
        self.write8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8)

    def configure_default_ranging(self) -> None:
        """Use the calibrated reference timing configuration from standard_init."""
        self.write8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8)

    def single_read(self, timeout_s: float) -> int:
        self.write8(0x80, 0x01)
        self.write8(0xFF, 0x01)
        self.write8(0x00, 0x00)
        self.write8(0x91, self.stop_variable)
        self.write8(0x00, 0x01)
        self.write8(0xFF, 0x00)
        self.write8(0x80, 0x00)
        self.write8(REG_SYSRANGE_START, 0x01)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not (self.read8(REG_SYSRANGE_START) & 0x01):
                break
            time.sleep(0.005)
        else:
            raise TimeoutError("VL53L0X did not start single ranging")

        while time.monotonic() < deadline:
            status = self.read8(REG_RESULT_INTERRUPT_STATUS)
            if status & 0x07:
                return self.finish_range_read()
            time.sleep(0.005)
        raise TimeoutError("range interrupt did not assert")

    def continuous_read(self, timeout_s: float) -> int:
        self.write8(REG_SYSRANGE_START, 0x02)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.read8(REG_RESULT_INTERRUPT_STATUS) & 0x07:
                return self.finish_range_read()
            time.sleep(0.005)
        raise TimeoutError("continuous range interrupt did not assert")

    def stop(self) -> None:
        self.write8(REG_SYSRANGE_START, 0x01)
        self.write8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)


def main() -> int:
    parser = argparse.ArgumentParser(description="Try VL53L0X ranging configurations until one works.")
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=DEFAULT_ADDRESS)
    parser.add_argument("--dll", default=None)
    parser.add_argument("--timeout", type=float, default=1.5, help="Timeout per ranging attempt in seconds.")
    parser.add_argument("--repeat", type=int, default=5, help="Valid readings required before success.")
    args = parser.parse_args()

    bus = CH341I2C(device_index=args.device_index, dll_path=args.dll)
    try:
        sensor = Sensor(bus, args.address)
        model = sensor.model_id()
        print(f"VL53L0X software configuration tester | CH341 {args.device_index} | I2C 0x{args.address:02X} | model 0x{model:02X}")
        if model not in (0xEE, 0xCC):
            print("Warning: unexpected model ID; continuing because the device answered I2C.")

        for mode in ("single", "continuous"):
            print(f"TEST {mode}: calibrated VL53L0X reference configuration")
            try:
                sensor.standard_init()
                sensor.configure_default_ranging()
                readings = []
                for _ in range(args.repeat):
                    value = sensor.single_read(args.timeout) if mode == "single" else sensor.continuous_read(args.timeout)
                    readings.append(value)
                print(f"SUCCESS: {readings[-1]} mm | readings={readings}")
                print("Continuing with the successful configuration. Press Ctrl+C to stop.")
                while True:
                    value = sensor.single_read(args.timeout) if mode == "single" else sensor.continuous_read(args.timeout)
                    print(f"Distance: {value:4d} mm ({value / 10.0:6.1f} cm)", end="\r", flush=True)
                    time.sleep(0.1)
            except (OSError, TimeoutError, ValueError) as exc:
                print(f"  failed: {exc}")
                try:
                    sensor.stop()
                except OSError:
                    pass
        print("No tested software configuration produced a valid measurement.")
        return 1
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
