#!/usr/bin/env python3
"""Streamlit dashboard for live USBI2C telemetry and servo control."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

import streamlit as st

from older.pca9685_servo_sweep import CH341I2C, PCA9685, Servo
from older.vl53l0x_configuration_tester import Sensor
from servoControlMaster import (
    MPU6050,
    MPU6050_DEFAULT_ADDRESS,
    PCA9685_DEFAULT_ADDRESS,
    VL53L0X_DEFAULT_ADDRESS,
    apply_deadband,
    distance_to_servo_angle,
    limit_step,
    smooth_value,
    triangle_wave_angle,
)


st.set_page_config(page_title="USBI2C Servo Dashboard", layout="wide")

AUX_SERVO_NAMES = ("Neck", "Waist", "Hip", "Left Hand", "Right Hand")
ALL_SERVO_NAMES = ("Servo X", "Servo Y", *AUX_SERVO_NAMES)
OFFSETS_FILE = Path(__file__).with_name("servo_zero_offsets.json")


def clamp_angle(angle: float) -> float:
    return max(0.0, min(180.0, angle))


def load_servo_offsets() -> dict[str, float]:
    offsets = {name: 0.0 for name in ALL_SERVO_NAMES}
    if not OFFSETS_FILE.exists():
        return offsets
    try:
        data = json.loads(OFFSETS_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return offsets
    for name in ALL_SERVO_NAMES:
        if name in data:
            offsets[name] = float(data[name])
    return offsets


def save_servo_offsets(offsets: dict[str, float]) -> None:
    serialized = {name: float(offsets.get(name, 0.0)) for name in ALL_SERVO_NAMES}
    OFFSETS_FILE.write_text(json.dumps(serialized, indent=2), encoding="utf-8")


@dataclass(frozen=True)
class DashboardConfig:
    device_index: int = 0
    vl53l0x_address: int = VL53L0X_DEFAULT_ADDRESS
    pca_address: int = PCA9685_DEFAULT_ADDRESS
    mpu_address: int = MPU6050_DEFAULT_ADDRESS
    dll_path: str | None = None
    freq_hz: float = 50.0
    interval_s: float = 0.10
    timeout_s: float = 1.5
    min_us: float = 500.0
    max_us: float = 2500.0
    min_angle: float = 0.0
    max_angle: float = 180.0
    zero_offset_mm: float = 20.0
    mpu_x_min_angle: float = -90.0
    mpu_x_max_angle: float = 90.0
    mpu_y_min_angle: float = -90.0
    mpu_y_max_angle: float = 90.0
    mpu_calibration_samples: int = 20
    mpu_deadband_deg: float = 1.5
    mpu_smoothing_alpha: float = 0.45
    servo_max_step_deg: float = 10.0
    test_min_angle: float = 30.0
    test_max_angle: float = 150.0
    test_period_s: float = 3.0
    servo_channel: int = 4
    servo_y_channel: int = 5
    neck_channel: int = 8
    waist_channel: int = 9
    hip_channel: int = 10
    left_hand_channel: int = 11
    right_hand_channel: int = 12


class HardwareController:
    def __init__(self, config: DashboardConfig) -> None:
        self.config = config
        self.bus = CH341I2C(device_index=config.device_index, dll_path=config.dll_path)
        try:
            self.pca = PCA9685(self.bus, address=config.pca_address)
            detected, mode1, mode2 = self.pca.verify()
            if not detected:
                raise RuntimeError(
                    f"PCA9685 did not respond at 0x{config.pca_address:02X} "
                    f"(MODE1=0x{mode1:02X}, MODE2=0x{mode2:02X})"
                )
            self.pca_mode1 = mode1
            self.pca_mode2 = mode2
            self.pca.reset()
            self.pca.set_pwm_freq(config.freq_hz)

            self.servos = {
                "Servo X": Servo(self.pca, channel=config.servo_channel, freq_hz=config.freq_hz, min_us=config.min_us, max_us=config.max_us),
                "Servo Y": Servo(self.pca, channel=config.servo_y_channel, freq_hz=config.freq_hz, min_us=config.min_us, max_us=config.max_us),
                "Neck": Servo(self.pca, channel=config.neck_channel, freq_hz=config.freq_hz, min_us=config.min_us, max_us=config.max_us),
                "Waist": Servo(self.pca, channel=config.waist_channel, freq_hz=config.freq_hz, min_us=config.min_us, max_us=config.max_us),
                "Hip": Servo(self.pca, channel=config.hip_channel, freq_hz=config.freq_hz, min_us=config.min_us, max_us=config.max_us),
                "Left Hand": Servo(self.pca, channel=config.left_hand_channel, freq_hz=config.freq_hz, min_us=config.min_us, max_us=config.max_us),
                "Right Hand": Servo(self.pca, channel=config.right_hand_channel, freq_hz=config.freq_hz, min_us=config.min_us, max_us=config.max_us),
            }
            self.channels = {
                "Servo X": config.servo_channel,
                "Servo Y": config.servo_y_channel,
                "Neck": config.neck_channel,
                "Waist": config.waist_channel,
                "Hip": config.hip_channel,
                "Left Hand": config.left_hand_channel,
                "Right Hand": config.right_hand_channel,
            }
            self.aux_phase_offsets = {
                "Neck": 0.0,
                "Waist": config.test_period_s / 3.0,
                "Hip": (2.0 * config.test_period_s) / 3.0,
                "Left Hand": config.test_period_s / 6.0,
                "Right Hand": config.test_period_s / 2.0,
            }

            self.sensor = Sensor(self.bus, address=config.vl53l0x_address)
            self.mpu = MPU6050(self.bus, address=config.mpu_address)
            self.mpu.initialize()
            self.mpu_who_am_i = self.mpu.verify()
            self.vl53_model = self.sensor.model_id()
            self.x_zero, self.y_zero = self._calibrate_mpu()
            self.mode, self.distance_mm = self._initialize_vl53l0x()
        except Exception:
            self.bus.close()
            raise

        self.servo_offsets = load_servo_offsets()
        self.last_update = time.monotonic()
        self.motion_start = self.last_update
        self.filtered_x_angle = 0.0
        self.filtered_y_angle = 0.0
        self.servo_x_angle = (config.min_angle + config.max_angle) / 2.0
        self.servo_y_angle = (config.min_angle + config.max_angle) / 2.0
        self.logical_servo_angles = {
            "Servo X": self.servo_x_angle,
            "Servo Y": self.servo_y_angle,
            **{name: 90.0 for name in AUX_SERVO_NAMES},
        }
        self.output_servo_angles = {
            name: clamp_angle(self.logical_servo_angles[name] + self.servo_offsets[name])
            for name in ALL_SERVO_NAMES
        }
        self.last_error = ""
        self.paused_at: float | None = None
        self.last_telemetry = {
            "distance_raw_mm": self.distance_mm,
            "distance_adjusted_mm": max(0.0, self.distance_mm - self.config.zero_offset_mm),
            "x_angle": 0.0,
            "y_angle": 0.0,
            "z_angle": 0.0,
            "mode": self.mode,
            "servo_angles": dict(self.output_servo_angles),
            "servo_offsets": dict(self.servo_offsets),
            "auto_aux_motion": True,
            "error": "",
            "run_state": "Paused",
        }

    def close(self) -> None:
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    def _read_distance(self, mode: str) -> int:
        if mode == "single":
            return self.sensor.single_read(self.config.timeout_s)
        return self.sensor.continuous_read(self.config.timeout_s)

    def _calibrate_mpu(self) -> tuple[float, float]:
        x_samples: list[float] = []
        y_samples: list[float] = []
        calibration_delta_t = max(self.config.interval_s, 0.02)
        for _ in range(self.config.mpu_calibration_samples):
            x_angle, y_angle, _ = self.mpu.read_angles(calibration_delta_t)
            x_samples.append(x_angle)
            y_samples.append(y_angle)
            time.sleep(calibration_delta_t)
        return sum(x_samples) / len(x_samples), sum(y_samples) / len(y_samples)

    def _reset_vl53l0x(self) -> None:
        try:
            self.sensor.stop()
        except OSError:
            pass
        try:
            self.sensor.soft_reset()
        except OSError:
            pass

    def _initialize_vl53l0x(self) -> tuple[str, int]:
        while True:
            for mode in ("continuous", "single"):
                try:
                    self.sensor.standard_init()
                    self.sensor.configure_default_ranging()
                    return mode, self._read_distance(mode)
                except (OSError, TimeoutError, ValueError):
                    self._reset_vl53l0x()
                    time.sleep(0.5)

    def zero_all_servos(self) -> dict[str, float]:
        zero_angles: dict[str, float] = {}
        for name in ALL_SERVO_NAMES:
            zero_angles[name] = self.command_servo(name, 0.0)
        self.servo_x_angle = 0.0
        self.servo_y_angle = 0.0
        return zero_angles

    def command_servo(self, name: str, logical_angle: float, offset_override: float | None = None) -> float:
        clamped_logical = clamp_angle(logical_angle)
        servo_offset = self.servo_offsets[name] if offset_override is None else float(offset_override)
        physical_angle = clamp_angle(clamped_logical + servo_offset)
        self.servos[name].set_angle(physical_angle)
        self.logical_servo_angles[name] = clamped_logical
        self.output_servo_angles[name] = physical_angle
        if name == "Servo X":
            self.servo_x_angle = clamped_logical
        elif name == "Servo Y":
            self.servo_y_angle = clamped_logical
        return physical_angle

    def preview_zero_position(self, name: str, offset: float) -> float:
        self.pause()
        return self.command_servo(name, 0.0, offset_override=offset)

    def save_zero_offset(self, name: str, offset: float) -> float:
        self.servo_offsets[name] = float(offset)
        save_servo_offsets(self.servo_offsets)
        return self.preview_zero_position(name, self.servo_offsets[name])

    def _auto_aux_angles(self, elapsed_s: float) -> dict[str, float]:
        return {
            name: triangle_wave_angle(
                elapsed_s,
                self.config.test_period_s,
                self.config.test_min_angle,
                self.config.test_max_angle,
                phase_offset=self.aux_phase_offsets[name],
            )
            for name in AUX_SERVO_NAMES
        }

    def pause(self) -> None:
        if self.paused_at is None:
            self.paused_at = time.monotonic()

    def resume(self) -> None:
        now = time.monotonic()
        if self.paused_at is not None:
            paused_duration = now - self.paused_at
            self.motion_start += paused_duration
            self.paused_at = None
        self.last_update = now

    def snapshot_telemetry(self, auto_aux_motion: bool) -> dict[str, object]:
        telemetry = {
            **self.last_telemetry,
            "distance_adjusted_mm": max(0.0, self.distance_mm - self.config.zero_offset_mm),
            "distance_raw_mm": self.distance_mm,
            "mode": self.mode,
            "servo_angles": dict(self.output_servo_angles),
            "servo_offsets": dict(self.servo_offsets),
            "auto_aux_motion": auto_aux_motion,
            "error": self.last_error,
            "run_state": "Paused",
        }
        self.last_telemetry = telemetry
        return telemetry

    def poll(self, run_motion: bool, auto_aux_motion: bool, manual_angles: dict[str, float], zero_all: bool = False) -> dict[str, object]:
        now = time.monotonic()
        delta_t = max(now - self.last_update, 1e-3)
        self.last_update = now

        if not run_motion and not zero_all:
            self.pause()
            return self.snapshot_telemetry(auto_aux_motion)

        if run_motion:
            self.resume()

        if zero_all:
            self.pause()
            servo_angles = self.zero_all_servos()
            distance_raw_mm = self.distance_mm
            distance_adjusted_mm = max(0.0, distance_raw_mm - self.config.zero_offset_mm)
            telemetry = {
                "distance_raw_mm": distance_raw_mm,
                "distance_adjusted_mm": distance_adjusted_mm,
                "x_angle": 0.0,
                "y_angle": 0.0,
                "z_angle": 0.0,
                "mode": self.mode,
                "servo_angles": servo_angles,
                "servo_offsets": dict(self.servo_offsets),
                "auto_aux_motion": auto_aux_motion,
                "error": "",
                "run_state": "Paused",
            }
            self.last_error = ""
            self.last_telemetry = telemetry
            return telemetry

        try:
            adjusted_distance_mm = max(0.0, self.distance_mm - self.config.zero_offset_mm)
            raw_x_angle, raw_y_angle, z_angle = self.mpu.read_angles(delta_t)
            x_angle = apply_deadband(raw_x_angle - self.x_zero, self.config.mpu_deadband_deg)
            y_angle = apply_deadband(raw_y_angle - self.y_zero, self.config.mpu_deadband_deg)
            self.filtered_x_angle = smooth_value(self.filtered_x_angle, x_angle, self.config.mpu_smoothing_alpha)
            self.filtered_y_angle = smooth_value(self.filtered_y_angle, y_angle, self.config.mpu_smoothing_alpha)

            target_servo_x = distance_to_servo_angle(
                self.filtered_x_angle,
                self.config.mpu_x_min_angle,
                self.config.mpu_x_max_angle,
                self.config.min_angle,
                self.config.max_angle,
            )
            target_servo_y = distance_to_servo_angle(
                self.filtered_y_angle,
                self.config.mpu_y_min_angle,
                self.config.mpu_y_max_angle,
                self.config.min_angle,
                self.config.max_angle,
            )
            self.servo_x_angle = limit_step(self.servo_x_angle, target_servo_x, self.config.servo_max_step_deg)
            self.servo_y_angle = limit_step(self.servo_y_angle, target_servo_y, self.config.servo_max_step_deg)

            self.command_servo("Servo X", self.servo_x_angle)
            self.command_servo("Servo Y", self.servo_y_angle)

            if auto_aux_motion:
                aux_angles = self._auto_aux_angles(now - self.motion_start)
            else:
                aux_angles = {name: clamp_angle(manual_angles[name]) for name in AUX_SERVO_NAMES}

            for name in AUX_SERVO_NAMES:
                self.command_servo(name, aux_angles[name])

            self.distance_mm = self._read_distance(self.mode)
            self.last_error = ""
            telemetry = {
                "distance_raw_mm": self.distance_mm,
                "distance_adjusted_mm": adjusted_distance_mm,
                "x_angle": self.filtered_x_angle,
                "y_angle": self.filtered_y_angle,
                "z_angle": z_angle,
                "mode": self.mode,
                "servo_angles": dict(self.output_servo_angles),
                "servo_offsets": dict(self.servo_offsets),
                "auto_aux_motion": auto_aux_motion,
                "error": "",
                "run_state": "Running",
            }
            self.last_telemetry = telemetry
            return telemetry
        except (OSError, TimeoutError, ValueError) as exc:
            self.last_error = str(exc)
            try:
                self._reset_vl53l0x()
                self.mode, self.distance_mm = self._initialize_vl53l0x()
            except (OSError, TimeoutError, ValueError) as recovery_exc:
                self.last_error = f"{self.last_error}; recovery failed: {recovery_exc}"
            telemetry = {
                "distance_raw_mm": self.distance_mm,
                "distance_adjusted_mm": max(0.0, self.distance_mm - self.config.zero_offset_mm),
                "x_angle": self.filtered_x_angle,
                "y_angle": self.filtered_y_angle,
                "z_angle": 0.0,
                "mode": self.mode,
                "servo_angles": dict(self.output_servo_angles),
                "servo_offsets": dict(self.servo_offsets),
                "auto_aux_motion": auto_aux_motion,
                "error": self.last_error,
                "run_state": "Running" if run_motion else "Paused",
            }
            self.last_telemetry = telemetry
            return telemetry


@st.cache_resource(show_spinner="Connecting to CH341, PCA9685, VL53L0X, and MPU6050...")
def get_controller(config: DashboardConfig) -> HardwareController:
    return HardwareController(config)


def initialize_state() -> None:
    if "auto_aux_motion" not in st.session_state:
        st.session_state.auto_aux_motion = True
    if "run_motion" not in st.session_state:
        st.session_state.run_motion = True
    if "zero_all_requested" not in st.session_state:
        st.session_state.zero_all_requested = False
    if "preview_zero_requested" not in st.session_state:
        st.session_state.preview_zero_requested = False
    if "save_zero_requested" not in st.session_state:
        st.session_state.save_zero_requested = False
    if "selected_servo" not in st.session_state:
        st.session_state.selected_servo = ALL_SERVO_NAMES[0]
    for servo_name in AUX_SERVO_NAMES:
        key = servo_name.lower().replace(" ", "_")
        session_key = f"manual_{key}"
        if session_key not in st.session_state:
            st.session_state[session_key] = 90.0
    for servo_name in ALL_SERVO_NAMES:
        offset_key = f"offset_{servo_name.lower().replace(' ', '_')}"
        if offset_key not in st.session_state:
            st.session_state[offset_key] = 0.0


def manual_aux_angles() -> dict[str, float]:
    return {
        servo_name: float(st.session_state[f"manual_{servo_name.lower().replace(' ', '_')}"])
        for servo_name in AUX_SERVO_NAMES
    }


def build_config() -> DashboardConfig:
    with st.sidebar:
        st.header("Hardware")
        device_index = int(st.number_input("CH341 device index", min_value=0, value=0, step=1))
        dll_value = st.text_input("CH341 DLL path", value="")
        st.header("Timing")
        interval_s = float(st.slider("Refresh interval (s)", min_value=0.05, max_value=0.50, value=0.10, step=0.01))
        timeout_s = float(st.slider("VL53L0X timeout (s)", min_value=0.5, max_value=3.0, value=1.5, step=0.1))
        st.header("Auto Sweep")
        test_min_angle = float(st.slider("Sweep minimum angle", min_value=0.0, max_value=180.0, value=30.0, step=1.0))
        test_max_angle = float(st.slider("Sweep maximum angle", min_value=0.0, max_value=180.0, value=150.0, step=1.0))
        test_period_s = float(st.slider("Sweep period (s)", min_value=1.0, max_value=10.0, value=3.0, step=0.1))
        st.header("Actions")
        reconnect = st.button("Reconnect Hardware", use_container_width=True)

    if reconnect:
        cached_controller = st.session_state.get("controller_ref")
        if cached_controller is not None:
            cached_controller.close()
        get_controller.clear()
        st.session_state.pop("controller_ref", None)
        st.rerun()

    dll_path = dll_value.strip() or None
    return DashboardConfig(
        device_index=device_index,
        dll_path=dll_path,
        interval_s=interval_s,
        timeout_s=timeout_s,
        test_min_angle=test_min_angle,
        test_max_angle=test_max_angle,
        test_period_s=test_period_s,
    )


def sync_offset_state(controller: HardwareController) -> None:
    for servo_name in ALL_SERVO_NAMES:
        offset_key = f"offset_{servo_name.lower().replace(' ', '_')}"
        if offset_key not in st.session_state or st.session_state[offset_key] == 0.0:
            st.session_state[offset_key] = float(controller.servo_offsets[servo_name])


def selected_offset_value() -> float:
    selected_servo = st.session_state.selected_servo
    offset_key = f"offset_{selected_servo.lower().replace(' ', '_')}"
    return float(st.session_state[offset_key])


def render_header(controller: HardwareController, telemetry: dict[str, object]) -> None:
    probe_left, probe_middle, probe_right = st.columns(3)
    with probe_left:
        st.metric("VL53L0X", f"0x{controller.vl53_model:02X}")
        st.caption(f"Mode: {telemetry['mode']}")
    with probe_middle:
        st.metric("MPU6050", f"0x{controller.mpu_who_am_i:02X}")
        st.caption(f"X zero {controller.x_zero:.1f} deg | Y zero {controller.y_zero:.1f} deg")
    with probe_right:
        st.metric("PCA9685", f"0x{controller.config.pca_address:02X}")
        st.caption(f"MODE1 0x{controller.pca_mode1:02X} | MODE2 0x{controller.pca_mode2:02X}")


def render_telemetry(telemetry: dict[str, object]) -> None:
    top = st.columns(4)
    top[0].metric("Distance Raw", f"{float(telemetry['distance_raw_mm']):.1f} mm")
    top[1].metric("Distance Adjusted", f"{float(telemetry['distance_adjusted_mm']):.1f} mm")
    top[2].metric("MPU X", f"{float(telemetry['x_angle']):.1f} deg")
    top[3].metric("MPU Y", f"{float(telemetry['y_angle']):.1f} deg")
    bottom = st.columns(3)
    bottom[0].metric("MPU Z", f"{float(telemetry['z_angle']):.1f} deg")
    bottom[1].metric("Aux Motion", "Auto" if telemetry["auto_aux_motion"] else "Manual")
    bottom[2].metric("State", str(telemetry["run_state"]))


def render_servo_status(controller: HardwareController, telemetry: dict[str, object]) -> None:
    servo_angles = telemetry["servo_angles"]
    servo_offsets = telemetry["servo_offsets"]
    columns = st.columns(7)
    for index, name in enumerate(ALL_SERVO_NAMES):
        channel = controller.channels[name]
        angle = float(servo_angles[name])
        with columns[index]:
            st.metric(name, f"{angle:.1f} deg")
            st.caption(f"Channel {channel} | Offset {float(servo_offsets[name]):+.1f} deg")


def render_aux_controls() -> None:
    st.subheader("Channels 8-12")
    auto_aux_motion = st.toggle(
        "Auto movement for Neck, Waist, Hip, Left Hand, and Right Hand",
        key="auto_aux_motion",
    )
    if auto_aux_motion:
        st.caption("Auto sweep is running. Disable it to send fixed manual angles.")
    else:
        st.caption("Manual override is active for channels 8-12.")

    control_columns = st.columns(len(AUX_SERVO_NAMES))
    for column, servo_name in zip(control_columns, AUX_SERVO_NAMES):
        session_key = f"manual_{servo_name.lower().replace(' ', '_')}"
        with column:
            st.slider(
                servo_name,
                min_value=0.0,
                max_value=180.0,
                step=1.0,
                key=session_key,
                disabled=auto_aux_motion,
            )


def render_zero_calibration(controller: HardwareController) -> None:
    st.subheader("Zero Calibration")
    st.caption("Pause the system, select a servo, preview an offset-adjusted zero, then save that offset as the new zero baseline.")
    select_column, offset_column, preview_column, save_column = st.columns((2, 2, 1, 1))
    with select_column:
        st.selectbox("Axis / Servo", ALL_SERVO_NAMES, key="selected_servo")
    with offset_column:
        selected_servo = st.session_state.selected_servo
        offset_key = f"offset_{selected_servo.lower().replace(' ', '_')}"
        st.number_input(
            "Zero offset (deg)",
            min_value=-90.0,
            max_value=90.0,
            step=1.0,
            key=offset_key,
            help="This offset is added to the logical servo angle before writing to the PCA9685.",
        )
    with preview_column:
        if st.button("Preview Zero", use_container_width=True):
            st.session_state.run_motion = False
            st.session_state.preview_zero_requested = True
            st.rerun()
    with save_column:
        if st.button("Save Zero", use_container_width=True):
            st.session_state.run_motion = False
            st.session_state.save_zero_requested = True
            st.rerun()


def main() -> None:
    initialize_state()
    config = build_config()

    st.title("USBI2C Servo Dashboard")
    st.caption("Live VL53L0X, MPU6050, and PCA9685 view with manual override for channels 8-12.")

    run_column, pause_column, zero_button_column, spacer_column = st.columns((1, 1, 1, 2))
    with run_column:
        if st.button("Run", use_container_width=True):
            st.session_state.run_motion = True
            st.rerun()
    with pause_column:
        if st.button("Pause", use_container_width=True):
            st.session_state.run_motion = False
            st.rerun()
    with zero_button_column:
        if st.button("Set All Servos to 0", use_container_width=True):
            st.session_state.run_motion = False
            st.session_state.auto_aux_motion = False
            for servo_name in AUX_SERVO_NAMES:
                st.session_state[f"manual_{servo_name.lower().replace(' ', '_')}"] = 0.0
            st.session_state.zero_all_requested = True
            st.rerun()
    st.caption("Pause freezes the motion loop. Use Set All Servos to 0 while paused for tool calibration, then press Run to resume.")

    try:
        controller = get_controller(config)
        st.session_state.controller_ref = controller
    except Exception as exc:
        st.error(f"Hardware initialization failed: {exc}")
        st.stop()

    sync_offset_state(controller)

    calibration_message = ""
    if st.session_state.preview_zero_requested:
        selected_servo = st.session_state.selected_servo
        preview_angle = controller.preview_zero_position(selected_servo, selected_offset_value())
        calibration_message = f"Previewed {selected_servo} at logical zero with offset {selected_offset_value():+.1f} deg. Commanded output {preview_angle:.1f} deg."
        st.session_state.preview_zero_requested = False
    elif st.session_state.save_zero_requested:
        selected_servo = st.session_state.selected_servo
        saved_angle = controller.save_zero_offset(selected_servo, selected_offset_value())
        calibration_message = f"Saved zero offset for {selected_servo}: {selected_offset_value():+.1f} deg. Commanded output {saved_angle:.1f} deg."
        st.session_state.save_zero_requested = False

    telemetry = controller.poll(
        run_motion=bool(st.session_state.run_motion),
        auto_aux_motion=bool(st.session_state.auto_aux_motion),
        manual_angles=manual_aux_angles(),
        zero_all=bool(st.session_state.zero_all_requested),
    )
    st.session_state.zero_all_requested = False

    render_header(controller, telemetry)
    st.divider()
    render_telemetry(telemetry)
    st.divider()
    render_servo_status(controller, telemetry)
    st.divider()
    render_aux_controls()
    st.divider()
    render_zero_calibration(controller)

    if calibration_message:
        st.success(calibration_message)

    if telemetry["error"]:
        st.warning(f"Last hardware warning: {telemetry['error']}")

    if st.session_state.run_motion:
        time.sleep(config.interval_s)
        st.rerun()


if __name__ == "__main__":
    main()