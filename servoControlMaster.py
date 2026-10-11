#!/usr/bin/env python3
"""Drive PCA9685 servos from MPU6050 X/Y angles, while reporting VL53L0X distance and xyz angles."""

from __future__ import annotations

import argparse
import math
import statistics
import time

from older.pca9685_servo_sweep import CH341I2C, PCA9685, Servo
from older.vl53l0x_configuration_tester import Sensor


VL53L0X_DEFAULT_ADDRESS = 0x29
PCA9685_DEFAULT_ADDRESS = 0x40
MPU6050_DEFAULT_ADDRESS = 0x68

MPU6050_WHO_AM_I = 0x75
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_ACCEL_XOUT_H = 0x3B


class MPU6050:
    def __init__(self, bus: CH341I2C, address: int = MPU6050_DEFAULT_ADDRESS) -> None:
        self.bus = bus
        self.address = address
        self.yaw_deg = 0.0

    def write8(self, register: int, value: int) -> None:
        self.bus.write(self.address, bytes([register & 0xFF, value & 0xFF]))

    def read8(self, register: int) -> int:
        return self.bus.write_read(self.address, bytes([register & 0xFF]), 1)[0]

    def read_block(self, register: int, length: int) -> bytes:
        return self.bus.write_read(self.address, bytes([register & 0xFF]), length)

    def verify(self) -> int:
        return self.read8(MPU6050_WHO_AM_I)

    def initialize(self) -> None:
        self.write8(MPU6050_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

    @staticmethod
    def _signed(high: int, low: int) -> int:
        value = (high << 8) | low
        return value - 65536 if value & 0x8000 else value

    def read_angles(self, delta_t: float) -> tuple[float, float, float]:
        block = self.read_block(MPU6050_ACCEL_XOUT_H, 14)
        accel_x = self._signed(block[0], block[1]) / 16384.0
        accel_y = self._signed(block[2], block[3]) / 16384.0
        accel_z = self._signed(block[4], block[5]) / 16384.0
        gyro_z_dps = self._signed(block[12], block[13]) / 131.0

        x_angle = math.degrees(math.atan2(accel_y, accel_z if abs(accel_z) > 1e-6 else 1e-6))
        y_angle = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z)))
        self.yaw_deg += gyro_z_dps * delta_t
        z_angle = self.yaw_deg
        return x_angle, y_angle, z_angle


def pulse_for_angle(servo: Servo, angle: float) -> float:
    bounded_angle = max(0.0, min(180.0, angle))
    return servo.min_us + (servo.max_us - servo.min_us) * (bounded_angle / 180.0)


def distance_to_servo_angle(
    value: float,
    min_value: float,
    max_value: float,
    min_angle: float,
    max_angle: float,
) -> float:
    if max_value <= min_value:
        raise ValueError("maximum source value must be greater than minimum source value")
    clamped_value = min(max(value, min_value), max_value)
    ratio = (clamped_value - min_value) / (max_value - min_value)
    return min_angle + (max_angle - min_angle) * ratio


def apply_deadband(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def smooth_value(previous: float, current: float, alpha: float) -> float:
    return previous + alpha * (current - previous)


def limit_step(previous: float, target: float, max_step: float) -> float:
    if max_step <= 0.0:
        return target
    delta = target - previous
    if delta > max_step:
        return previous + max_step
    if delta < -max_step:
        return previous - max_step
    return target


def triangle_wave_angle(
    elapsed_s: float,
    period_s: float,
    min_angle: float,
    max_angle: float,
    phase_offset: float = 0.0,
) -> float:
    if period_s <= 0.0:
        return min_angle
    span = max_angle - min_angle
    phase = ((elapsed_s + phase_offset) / period_s) % 1.0
    triangle = 1.0 - abs(2.0 * phase - 1.0)
    return min_angle + span * triangle


def main() -> int:
    parser = argparse.ArgumentParser(description="Drive PCA9685 servos from MPU6050 X/Y angles and print xyz plus VL53L0X distance.")
    parser.add_argument("--device-index", type=int, default=0, help="CH341 device index (default 0).")
    parser.add_argument("--vl53l0x-address", type=lambda value: int(value, 0), default=VL53L0X_DEFAULT_ADDRESS, help="VL53L0X I2C address (default 0x29).")
    parser.add_argument("--pca-address", type=lambda value: int(value, 0), default=PCA9685_DEFAULT_ADDRESS, help="PCA9685 I2C address (default 0x40).")
    parser.add_argument("--mpu-address", type=lambda value: int(value, 0), default=MPU6050_DEFAULT_ADDRESS, help="MPU6050 I2C address (default 0x68).")
    parser.add_argument("--dll", default=None, help="Path to CH341DLLA64.DLL or CH341DLL.DLL.")
    parser.add_argument("--freq", type=float, default=50.0, help="Servo PWM frequency in Hz (RDS3115MG default 50).")
    parser.add_argument("--interval", type=float, default=0.05, help="Seconds between updates.")
    parser.add_argument("--timeout", type=float, default=1.5, help="VL53L0X read timeout in seconds.")
    parser.add_argument("--servo-channel", type=int, default=4, help="PCA9685 servo channel driven from MPU X angle (default 4).")
    parser.add_argument("--servo-y-channel", type=int, default=5, help="PCA9685 servo channel driven from MPU Y angle (default 5).")
    parser.add_argument("--neck-channel", type=int, default=8, help="PCA9685 servo channel for Neck test motion (default 8).")
    parser.add_argument("--waist-channel", type=int, default=9, help="PCA9685 servo channel for Waist test motion (default 9).")
    parser.add_argument("--hip-channel", type=int, default=10, help="PCA9685 servo channel for Hip test motion (default 10).")
    parser.add_argument("--left-hand-channel", type=int, default=11, help="PCA9685 servo channel for Left Hand test motion (default 11).")
    parser.add_argument("--right-hand-channel", type=int, default=12, help="PCA9685 servo channel for Right Hand test motion (default 12).")
    parser.add_argument("--min-us", type=float, default=500.0, help="Servo minimum pulse width in microseconds (RDS3115MG default 500).")
    parser.add_argument("--max-us", type=float, default=2500.0, help="Servo maximum pulse width in microseconds (RDS3115MG default 2500).")
    parser.add_argument("--min-angle", type=float, default=0.0, help="Servo angle used at the minimum mapped distance.")
    parser.add_argument("--max-angle", type=float, default=180.0, help="Servo angle used at the maximum mapped distance.")
    parser.add_argument("--test-min-angle", type=float, default=30.0, help="Minimum angle for the Neck/Waist/Hip test sweep.")
    parser.add_argument("--test-max-angle", type=float, default=150.0, help="Maximum angle for the Neck/Waist/Hip test sweep.")
    parser.add_argument("--test-period", type=float, default=3.0, help="Seconds for one full Neck/Waist/Hip test sweep cycle.")
    parser.add_argument("--zero-offset-mm", type=float, default=20.0, help="Subtract this offset from each distance and clamp at 0 mm.")
    parser.add_argument("--mpu-x-min-angle", type=float, default=-90.0, help="MPU X-axis angle mapped to the minimum servo angle.")
    parser.add_argument("--mpu-x-max-angle", type=float, default=90.0, help="MPU X-axis angle mapped to the maximum servo angle.")
    parser.add_argument("--mpu-y-min-angle", type=float, default=-90.0, help="MPU Y-axis angle mapped to the minimum servo angle.")
    parser.add_argument("--mpu-y-max-angle", type=float, default=90.0, help="MPU Y-axis angle mapped to the maximum servo angle.")
    parser.add_argument("--mpu-calibration-samples", type=int, default=20, help="Number of startup samples used to zero the MPU X/Y axes.")
    parser.add_argument("--mpu-deadband-deg", type=float, default=1.5, help="Ignore MPU X/Y motion smaller than this many degrees.")
    parser.add_argument("--mpu-smoothing-alpha", type=float, default=0.45, help="Exponential smoothing factor for MPU X/Y angles between 0 and 1.")
    parser.add_argument("--servo-max-step-deg", type=float, default=10.0, help="Maximum servo angle change per update. Use 0 to disable slew limiting.")
    args = parser.parse_args()

    if args.max_angle < args.min_angle:
        raise ValueError("max-angle must be greater than or equal to min-angle")
    if args.test_max_angle < args.test_min_angle:
        raise ValueError("test-max-angle must be greater than or equal to test-min-angle")
    if args.mpu_x_max_angle <= args.mpu_x_min_angle:
        raise ValueError("mpu-x-max-angle must be greater than mpu-x-min-angle")
    if args.mpu_y_max_angle <= args.mpu_y_min_angle:
        raise ValueError("mpu-y-max-angle must be greater than mpu-y-min-angle")
    if not 0.0 < args.mpu_smoothing_alpha <= 1.0:
        raise ValueError("mpu-smoothing-alpha must be between 0 and 1")
    if args.test_period <= 0.0:
        raise ValueError("test-period must be greater than 0")

    def read_distance(sensor: Sensor, mode: str) -> int:
        return sensor.single_read(args.timeout) if mode == "single" else sensor.continuous_read(args.timeout)

    def calibrate_mpu(mpu: MPU6050) -> tuple[float, float]:
        x_samples: list[float] = []
        y_samples: list[float] = []
        print(f"Calibrating MPU zero from {args.mpu_calibration_samples} samples. Keep the device still...", flush=True)
        calibration_delta_t = max(args.interval, 0.02)
        for _ in range(args.mpu_calibration_samples):
            x_angle, y_angle, _ = mpu.read_angles(calibration_delta_t)
            x_samples.append(x_angle)
            y_samples.append(y_angle)
            time.sleep(calibration_delta_t)
        return statistics.mean(x_samples), statistics.mean(y_samples)

    def reset_vl53l0x(sensor: Sensor) -> None:
        try:
            sensor.stop()
        except OSError:
            pass
        try:
            sensor.soft_reset()
        except OSError:
            pass

    def initialize_vl53l0x(sensor: Sensor, quiet: bool = False) -> tuple[str, int]:
        while True:
            for mode in ("continuous", "single"):
                try:
                    sensor.standard_init()
                    sensor.configure_default_ranging()
                    return mode, read_distance(sensor, mode)
                except (OSError, TimeoutError, ValueError) as exc:
                    reset_vl53l0x(sensor)
                    if not quiet:
                        print(f"VL53L0X init failed in {mode} mode: {exc}; retrying...", flush=True)
                    time.sleep(0.5)

    bus = CH341I2C(device_index=args.device_index, dll_path=args.dll)
    try:
        pca = PCA9685(bus, address=args.pca_address)
        detected, mode1, mode2 = pca.verify()
        print(
            f"PCA9685 probe | CH341 index {args.device_index:08d} | I2C 0x{args.pca_address:02X} | "
            f"MODE1=0x{mode1:02X} MODE2=0x{mode2:02X}"
        )
        if not detected:
            raise RuntimeError("PCA9685 did not respond; check power, wiring, and address")

        pca.reset()
        pca.set_pwm_freq(args.freq)
        servo = Servo(
            pca,
            channel=args.servo_channel,
            freq_hz=args.freq,
            min_us=args.min_us,
            max_us=args.max_us,
        )
        servo_y = Servo(
            pca,
            channel=args.servo_y_channel,
            freq_hz=args.freq,
            min_us=args.min_us,
            max_us=args.max_us,
        )
        neck_servo = Servo(
            pca,
            channel=args.neck_channel,
            freq_hz=args.freq,
            min_us=args.min_us,
            max_us=args.max_us,
        )
        waist_servo = Servo(
            pca,
            channel=args.waist_channel,
            freq_hz=args.freq,
            min_us=args.min_us,
            max_us=args.max_us,
        )
        hip_servo = Servo(
            pca,
            channel=args.hip_channel,
            freq_hz=args.freq,
            min_us=args.min_us,
            max_us=args.max_us,
        )
        left_hand_servo = Servo(
            pca,
            channel=args.left_hand_channel,
            freq_hz=args.freq,
            min_us=args.min_us,
            max_us=args.max_us,
        )
        right_hand_servo = Servo(
            pca,
            channel=args.right_hand_channel,
            freq_hz=args.freq,
            min_us=args.min_us,
            max_us=args.max_us,
        )

        sensor = Sensor(bus, address=args.vl53l0x_address)
        mpu = MPU6050(bus, address=args.mpu_address)
        mpu.initialize()

        mpu_who_am_i = mpu.verify()
        vl53_model = sensor.model_id()
        print(
            f"VL53L0X model=0x{vl53_model:02X} | MPU6050 WHO_AM_I=0x{mpu_who_am_i:02X} | "
            f"servo X channel={args.servo_channel} | servo Y channel={args.servo_y_channel} | "
            f"Neck={args.neck_channel} Waist={args.waist_channel} Hip={args.hip_channel} | "
            f"Left Hand={args.left_hand_channel} Right Hand={args.right_hand_channel}"
        )
        x_zero, y_zero = calibrate_mpu(mpu)
        print(f"MPU zero | x={x_zero:6.1f} deg | y={y_zero:6.1f} deg")
        mode, distance_mm = initialize_vl53l0x(sensor)

        print(
            f"Running master control at {args.freq:.1f} Hz servo PWM and {args.interval:.2f}s update interval. "
            "Press Ctrl+C to stop."
        )

        last_update = time.monotonic()
        motion_start = last_update
        filtered_x_angle = 0.0
        filtered_y_angle = 0.0
        servo_angle = (args.min_angle + args.max_angle) / 2.0
        servo_y_angle = (args.min_angle + args.max_angle) / 2.0
        while True:
            try:
                now = time.monotonic()
                delta_t = max(now - last_update, 1e-3)
                last_update = now

                adjusted_distance_mm = max(0.0, distance_mm - args.zero_offset_mm)
                raw_x_angle, raw_y_angle, z_angle = mpu.read_angles(delta_t)
                x_angle = apply_deadband(raw_x_angle - x_zero, args.mpu_deadband_deg)
                y_angle = apply_deadband(raw_y_angle - y_zero, args.mpu_deadband_deg)
                filtered_x_angle = smooth_value(filtered_x_angle, x_angle, args.mpu_smoothing_alpha)
                filtered_y_angle = smooth_value(filtered_y_angle, y_angle, args.mpu_smoothing_alpha)

                target_servo_angle = distance_to_servo_angle(
                    filtered_x_angle,
                    args.mpu_x_min_angle,
                    args.mpu_x_max_angle,
                    args.min_angle,
                    args.max_angle,
                )
                target_servo_y_angle = distance_to_servo_angle(
                    filtered_y_angle,
                    args.mpu_y_min_angle,
                    args.mpu_y_max_angle,
                    args.min_angle,
                    args.max_angle,
                )
                elapsed_s = now - motion_start
                neck_angle = triangle_wave_angle(
                    elapsed_s,
                    args.test_period,
                    args.test_min_angle,
                    args.test_max_angle,
                    phase_offset=0.0,
                )
                waist_angle = triangle_wave_angle(
                    elapsed_s,
                    args.test_period,
                    args.test_min_angle,
                    args.test_max_angle,
                    phase_offset=args.test_period / 3.0,
                )
                hip_angle = triangle_wave_angle(
                    elapsed_s,
                    args.test_period,
                    args.test_min_angle,
                    args.test_max_angle,
                    phase_offset=(2.0 * args.test_period) / 3.0,
                )
                left_hand_angle = triangle_wave_angle(
                    elapsed_s,
                    args.test_period,
                    args.test_min_angle,
                    args.test_max_angle,
                    phase_offset=args.test_period / 6.0,
                )
                right_hand_angle = triangle_wave_angle(
                    elapsed_s,
                    args.test_period,
                    args.test_min_angle,
                    args.test_max_angle,
                    phase_offset=args.test_period / 2.0,
                )
                servo_angle = limit_step(servo_angle, target_servo_angle, args.servo_max_step_deg)
                servo_y_angle = limit_step(servo_y_angle, target_servo_y_angle, args.servo_max_step_deg)
                servo.set_angle(servo_angle)
                servo_y.set_angle(servo_y_angle)
                neck_servo.set_angle(neck_angle)
                waist_servo.set_angle(waist_angle)
                hip_servo.set_angle(hip_angle)
                left_hand_servo.set_angle(left_hand_angle)
                right_hand_servo.set_angle(right_hand_angle)
                print(
                    f"xyz=({filtered_x_angle:6.1f}, {filtered_y_angle:6.1f}, {z_angle:6.1f}) deg | "
                    f"distance={adjusted_distance_mm:6.1f} mm | servo x={servo_angle:6.1f} deg | servo y={servo_y_angle:6.1f} deg | "
                    f"Neck={neck_angle:6.1f} deg | Waist={waist_angle:6.1f} deg | Hip={hip_angle:6.1f} deg | "
                    f"Left Hand={left_hand_angle:6.1f} deg | Right Hand={right_hand_angle:6.1f} deg",
                    end="\r",
                    flush=True,
                )
                time.sleep(args.interval)
                distance_mm = read_distance(sensor, mode)
            except (OSError, TimeoutError, ValueError):
                reset_vl53l0x(sensor)
                mode, distance_mm = initialize_vl53l0x(sensor, quiet=True)

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        bus.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())