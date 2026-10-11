from __future__ import annotations

import ctypes
import json
import platform
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


if sys.platform != "win32":
    raise SystemExit("This script currently supports Windows only because it uses CH341DLL.DLL.")


try:
    import msvcrt
except ImportError as exc:
    raise SystemExit("msvcrt is required on Windows for the timeout confirmation prompt.") from exc


HELLO_TEXT = "HELLO WORLD"
TARGET_INSTANCE_ID = r"USB\VID_1A86&PID_5512\5&BA996E2&0&7"
CH341_INSTANCE_PREFIX = r"USB\VID_1A86&PID_5512"
TARGET_PANEL = "w128-240240-RGB-7"


FONT_5X7: dict[str, list[int]] = {
    " ": [0x00, 0x00, 0x00, 0x00, 0x00],
    "D": [0x7F, 0x41, 0x41, 0x22, 0x1C],
    "E": [0x7F, 0x49, 0x49, 0x49, 0x41],
    "H": [0x7F, 0x08, 0x08, 0x08, 0x7F],
    "L": [0x7F, 0x40, 0x40, 0x40, 0x40],
    "O": [0x3E, 0x41, 0x41, 0x41, 0x3E],
    "R": [0x7F, 0x09, 0x19, 0x29, 0x46],
    "W": [0x7F, 0x20, 0x18, 0x20, 0x7F],
}


STREAM_MODES: list[tuple[str, int]] = [
    ("20kHz", 0x00),
    ("100kHz", 0x01),
    ("400kHz", 0x02),
    ("750kHz", 0x03),
    ("20kHz-alt", 0x80),
    ("100kHz-alt", 0x81),
    ("400kHz-alt", 0x82),
    ("750kHz-alt", 0x83),
]


@dataclass(frozen=True)
class TargetPanelProfile:
    name: str
    allow_i2c_display_attempts: bool
    notes: str


TARGET_PANEL_PROFILES: dict[str, TargetPanelProfile] = {
    "w128-240240-RGB-7": TargetPanelProfile(
        name="w128-240240-RGB-7",
        allow_i2c_display_attempts=False,
        notes=(
            "This 240x240 RGB class of panel is typically a TFT driven over SPI, even when pins are labeled "
            "SCL/SDA. SDA/SCL alone is not enough to render text; a real init usually also needs CS, DC, "
            "and RESET. White screen with dark lines is not a successful match."
        ),
    ),
    "generic-i2c": TargetPanelProfile(
        name="generic-i2c",
        allow_i2c_display_attempts=True,
        notes="Generic fallback profile for unknown I2C-capable displays.",
    ),
}


def chunked(data: Sequence[int], size: int) -> Iterable[Sequence[int]]:
    for start in range(0, len(data), size):
        yield data[start : start + size]


def wait_for_success(timeout_seconds: float = 3.0) -> bool:
    print(f"  Waiting {timeout_seconds:.0f}s. Press 's' only if 'HELLO WORLD' is clearly readable.")
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if msvcrt.kbhit():
            key = msvcrt.getwch()
            if key.lower() == "s":
                return True
        time.sleep(0.05)
    return False


def render_text_to_pages(text: str, width: int, pages: int) -> list[int]:
    buffer = [0x00] * (width * pages)
    x = 0
    for char in text:
        glyph = FONT_5X7.get(char, FONT_5X7[" "])
        if x + 6 >= width:
            break
        for column, bits in enumerate(glyph):
            buffer[x + column] = bits
        x += 6
    return buffer


@dataclass(frozen=True)
class PnpDeviceInfo:
    instance_id: str
    status: str
    present: bool
    friendly_name: str


def _run_powershell_json(command: str) -> object:
    completed = subprocess.run(
        [
            "powershell.exe",
            "-NoProfile",
            "-Command",
            command,
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        stderr = completed.stderr.strip() or completed.stdout.strip() or "PowerShell command failed"
        raise OSError(stderr)

    output = completed.stdout.strip()
    if not output:
        return None
    return json.loads(output)


def get_pnp_device(instance_id: str) -> PnpDeviceInfo | None:
    escaped = instance_id.replace("'", "''")
    raw = _run_powershell_json(
        "Get-PnpDevice -InstanceId '{}' -ErrorAction SilentlyContinue | "
        "Select-Object InstanceId,Status,Present,FriendlyName | ConvertTo-Json -Compress".format(escaped)
    )
    if raw is None:
        return None
    if isinstance(raw, list):
        if not raw:
            return None
        raw = raw[0]
    return PnpDeviceInfo(
        instance_id=raw.get("InstanceId", ""),
        status=raw.get("Status", "Unknown"),
        present=bool(raw.get("Present", False)),
        friendly_name=raw.get("FriendlyName") or "Unknown",
    )


def get_present_ch341_devices() -> list[PnpDeviceInfo]:
    prefix = CH341_INSTANCE_PREFIX.replace("'", "''")
    raw = _run_powershell_json(
        "Get-PnpDevice | "
        "Where-Object { $_.Present -and $_.InstanceId -like '" + prefix + "*' } | "
        "Select-Object InstanceId,Status,Present,FriendlyName | ConvertTo-Json -Compress"
    )
    if raw is None:
        return []
    if not isinstance(raw, list):
        raw = [raw]
    return [
        PnpDeviceInfo(
            instance_id=item.get("InstanceId", ""),
            status=item.get("Status", "Unknown"),
            present=bool(item.get("Present", False)),
            friendly_name=item.get("FriendlyName") or "Unknown",
        )
        for item in raw
    ]


def verify_target_ch341_device(target_instance_id: str) -> PnpDeviceInfo:
    target = get_pnp_device(target_instance_id)
    if target is None:
        raise SystemExit(f"Target CH341 device not found in Plug and Play: {target_instance_id}")
    if not target.present:
        raise SystemExit(f"Target CH341 device is not present: {target_instance_id}")
    if target.status.upper() != "OK":
        raise SystemExit(
            f"Target CH341 device is present but not ready. Status={target.status}, FriendlyName={target.friendly_name}"
        )

    present_devices = get_present_ch341_devices()
    if len(present_devices) > 1:
        device_list = ", ".join(device.instance_id for device in present_devices)
        raise SystemExit(
            "More than one present CH341 USB device was found. "
            f"This script currently assumes CH341 index 0 maps to the target instance. Present devices: {device_list}"
        )
    if len(present_devices) == 1 and present_devices[0].instance_id != target_instance_id:
        raise SystemExit(
            "The only present CH341 device does not match the configured target instance. "
            f"Expected {target_instance_id}, found {present_devices[0].instance_id}"
        )
    return target


class Ch341Dll:
    def __init__(self) -> None:
        candidates = self._candidate_dlls()
        if not candidates:
            raise FileNotFoundError(
                "Could not find CH341DLL.DLL. Place it next to the script or install the WCH driver package."
            )
        load_errors: list[str] = []
        self.dll = None
        for dll_path in candidates:
            try:
                self.dll = ctypes.WinDLL(str(dll_path))
                break
            except OSError as exc:
                load_errors.append(f"{dll_path}: {exc}")
        if self.dll is None:
            python_bits = platform.architecture()[0]
            searched = "\n".join(f"  - {path}" for path in candidates)
            details = "\n".join(f"  - {error}" for error in load_errors)
            raise OSError(
                "Failed to load any CH341 DLL that matches this Python process.\n"
                f"Python architecture: {python_bits}\n"
                "Searched candidates:\n"
                f"{searched}\n"
                "Load errors:\n"
                f"{details}\n"
                "Install the matching WCH driver package or run Python with the same architecture as the available CH341 DLL."
            )

        self.dll.CH341OpenDevice.argtypes = [ctypes.c_ulong]
        self.dll.CH341OpenDevice.restype = ctypes.c_long
        self.dll.CH341CloseDevice.argtypes = [ctypes.c_ulong]
        self.dll.CH341CloseDevice.restype = ctypes.c_bool
        self.dll.CH341SetStream.argtypes = [ctypes.c_ulong, ctypes.c_ulong]
        self.dll.CH341SetStream.restype = ctypes.c_bool
        self.dll.CH341StreamI2C.argtypes = [
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ubyte),
        ]
        self.dll.CH341StreamI2C.restype = ctypes.c_bool

    @staticmethod
    def _candidate_dlls() -> list[Path]:
        is_64bit_python = ctypes.sizeof(ctypes.c_void_p) == 8
        workspace_paths = [
            Path.cwd() / "CH341DLLA64.DLL",
            Path.cwd() / "CH341DLL.DLL",
            Path.cwd() / "CH341PAR.DLL",
        ]
        system64_paths = [
            Path("C:/Windows/System32/CH341DLLA64.DLL"),
            Path("C:/Windows/System32/CH341DLL.DLL"),
        ]
        system32_paths = [
            Path("C:/Windows/SysWOW64/CH341DLL.DLL"),
            Path("C:/Windows/SysWOW64/CH341DLLA64.DLL"),
        ]
        search_paths = workspace_paths + (system64_paths + system32_paths if is_64bit_python else system32_paths + system64_paths)
        return [path for path in search_paths if path.exists()]

    def open(self, index: int = 0) -> None:
        result = self.dll.CH341OpenDevice(index)
        if result == -1:
            raise OSError(f"CH341OpenDevice({index}) failed.")

    def close(self, index: int = 0) -> None:
        self.dll.CH341CloseDevice(index)

    def set_stream(self, index: int, mode: int) -> None:
        if not self.dll.CH341SetStream(index, mode):
            raise OSError(f"CH341SetStream({index}, 0x{mode:02X}) failed.")

    def stream_i2c(self, index: int, write_data: Sequence[int], read_len: int = 0) -> bytes:
        write_buffer = (ctypes.c_ubyte * len(write_data))(*write_data)
        read_buffer = (ctypes.c_ubyte * max(read_len, 1))()
        ok = self.dll.CH341StreamI2C(index, len(write_data), write_buffer, read_len, read_buffer)
        if not ok:
            raise OSError("CH341StreamI2C failed.")
        return bytes(read_buffer[:read_len])


class Ch341I2cBus:
    def __init__(self, dll: Ch341Dll, index: int = 0) -> None:
        self.dll = dll
        self.index = index

    def set_speed(self, mode: int) -> None:
        self.dll.set_stream(self.index, mode)

    def probe(self, address: int) -> bool:
        try:
            self.dll.stream_i2c(self.index, [(address << 1) & 0xFE])
            return True
        except OSError:
            return False

    def write(self, address: int, payload: Sequence[int]) -> None:
        frame = [(address << 1) & 0xFE, *payload]
        self.dll.stream_i2c(self.index, frame)


class I2cDisplayAttempt:
    name = "base"
    addresses: Sequence[int] = ()

    def supports_address(self, address: int) -> bool:
        return address in self.addresses

    def write_hello(self, bus: Ch341I2cBus, address: int) -> None:
        raise NotImplementedError


class Ssd1306Attempt(I2cDisplayAttempt):
    def __init__(self, height: int) -> None:
        self.height = height
        self.name = f"SSD1306-{height}"
        self.addresses = (0x3C, 0x3D)

    def _command(self, bus: Ch341I2cBus, address: int, *values: int) -> None:
        bus.write(address, [0x00, *values])

    def _data(self, bus: Ch341I2cBus, address: int, values: Sequence[int]) -> None:
        for chunk in chunked(values, 16):
            bus.write(address, [0x40, *chunk])

    def write_hello(self, bus: Ch341I2cBus, address: int) -> None:
        multiplex = 0x1F if self.height == 32 else 0x3F
        compins = 0x02 if self.height == 32 else 0x12
        pages = self.height // 8
        image = render_text_to_pages(HELLO_TEXT, 128, pages)
        init = [
            0xAE,
            0x20,
            0x00,
            0x40,
            0xA1,
            0xC8,
            0x81,
            0xCF,
            0xA6,
            0xA8,
            multiplex,
            0xD3,
            0x00,
            0xD5,
            0x80,
            0xD9,
            0xF1,
            0xDA,
            compins,
            0xDB,
            0x40,
            0x8D,
            0x14,
            0xAF,
        ]
        self._command(bus, address, *init)
        self._command(bus, address, 0x21, 0x00, 0x7F, 0x22, 0x00, pages - 1)
        self._data(bus, address, image)


class Sh1106Attempt(I2cDisplayAttempt):
    name = "SH1106-64"
    addresses = (0x3C, 0x3D)

    def _command(self, bus: Ch341I2cBus, address: int, *values: int) -> None:
        bus.write(address, [0x00, *values])

    def _data(self, bus: Ch341I2cBus, address: int, values: Sequence[int]) -> None:
        for chunk in chunked(values, 16):
            bus.write(address, [0x40, *chunk])

    def write_hello(self, bus: Ch341I2cBus, address: int) -> None:
        pages = 8
        image = render_text_to_pages(HELLO_TEXT, 128, pages)
        init = [
            0xAE,
            0xD5,
            0x80,
            0xA8,
            0x3F,
            0xD3,
            0x00,
            0x40,
            0xAD,
            0x8B,
            0xA1,
            0xC8,
            0xDA,
            0x12,
            0x81,
            0x80,
            0xD9,
            0x22,
            0xDB,
            0x20,
            0xA4,
            0xA6,
            0xAF,
        ]
        self._command(bus, address, *init)
        for page in range(pages):
            self._command(bus, address, 0xB0 + page, 0x02, 0x10)
            row = image[page * 128 : (page + 1) * 128]
            self._data(bus, address, row)


class St7032Attempt(I2cDisplayAttempt):
    name = "ST7032-text"
    addresses = (0x3E, 0x3F)

    def _command(self, bus: Ch341I2cBus, address: int, value: int) -> None:
        bus.write(address, [0x00, value])

    def _text(self, bus: Ch341I2cBus, address: int, text: str) -> None:
        bus.write(address, [0x40, *text.encode("ascii", errors="replace")])

    def write_hello(self, bus: Ch341I2cBus, address: int) -> None:
        init = [0x38, 0x39, 0x14, 0x78, 0x5E, 0x6D, 0x0C, 0x01, 0x06]
        for command in init:
            self._command(bus, address, command)
            time.sleep(0.01)
        self._text(bus, address, HELLO_TEXT[:16])


class Hd44780Pcf8574Attempt(I2cDisplayAttempt):
    name = "HD44780-PCF8574"
    addresses = tuple(range(0x20, 0x28)) + tuple(range(0x38, 0x40))

    EN = 0x04
    RW = 0x02
    RS = 0x01
    BL = 0x08

    def _expander_write(self, bus: Ch341I2cBus, address: int, value: int) -> None:
        bus.write(address, [value | self.BL])

    def _pulse(self, bus: Ch341I2cBus, address: int, value: int) -> None:
        self._expander_write(bus, address, value | self.EN)
        time.sleep(0.001)
        self._expander_write(bus, address, value & ~self.EN)
        time.sleep(0.001)

    def _write4(self, bus: Ch341I2cBus, address: int, nibble: int, mode: int) -> None:
        value = (nibble & 0xF0) | mode
        self._expander_write(bus, address, value)
        self._pulse(bus, address, value)

    def _send(self, bus: Ch341I2cBus, address: int, value: int, mode: int) -> None:
        self._write4(bus, address, value & 0xF0, mode)
        self._write4(bus, address, (value << 4) & 0xF0, mode)

    def _command(self, bus: Ch341I2cBus, address: int, value: int) -> None:
        self._send(bus, address, value, 0)

    def _data(self, bus: Ch341I2cBus, address: int, value: int) -> None:
        self._send(bus, address, value, self.RS)

    def write_hello(self, bus: Ch341I2cBus, address: int) -> None:
        time.sleep(0.05)
        self._write4(bus, address, 0x30, 0)
        time.sleep(0.005)
        self._write4(bus, address, 0x30, 0)
        time.sleep(0.005)
        self._write4(bus, address, 0x30, 0)
        time.sleep(0.005)
        self._write4(bus, address, 0x20, 0)
        time.sleep(0.005)
        for command in (0x28, 0x0C, 0x01, 0x06, 0x80):
            self._command(bus, address, command)
            time.sleep(0.002)
        for char in HELLO_TEXT:
            self._data(bus, address, ord(char))


@dataclass(frozen=True)
class AttemptResult:
    controller: str
    address: int
    speed_name: str
    success: bool
    error: str | None = None


@dataclass(frozen=True)
class SpeedScanResult:
    speed_name: str
    stream_mode: int
    ready: bool
    responding_addresses: tuple[int, ...]
    error: str | None = None


def build_attempts() -> list[I2cDisplayAttempt]:
    return [
        Ssd1306Attempt(height=64),
        Ssd1306Attempt(height=32),
        Sh1106Attempt(),
        St7032Attempt(),
        Hd44780Pcf8574Attempt(),
    ]


def get_target_panel_profile() -> TargetPanelProfile:
    return TARGET_PANEL_PROFILES.get(
        TARGET_PANEL,
        TargetPanelProfile(
            name=TARGET_PANEL,
            allow_i2c_display_attempts=True,
            notes="Unknown panel profile; falling back to generic I2C probing.",
        ),
    )


def scan_i2c_bus(bus: Ch341I2cBus) -> list[SpeedScanResult]:
    scan_results: list[SpeedScanResult] = []
    for speed_name, stream_mode in STREAM_MODES:
        try:
            bus.set_speed(stream_mode)
        except OSError as exc:
            scan_results.append(SpeedScanResult(speed_name, stream_mode, False, (), str(exc)))
            continue

        responders = [address for address in range(0x03, 0x78) if bus.probe(address)]
        scan_results.append(SpeedScanResult(speed_name, stream_mode, True, tuple(responders)))
    return scan_results


def print_scan_summary(scan_results: Sequence[SpeedScanResult]) -> None:
    print("CH341/I2C preflight summary:")
    for result in scan_results:
        if not result.ready:
            print(f"  {result.speed_name} (0x{result.stream_mode:02X}): unavailable - {result.error}")
            continue
        if result.responding_addresses:
            addresses = ", ".join(f"0x{address:02X}" for address in result.responding_addresses)
            print(f"  {result.speed_name} (0x{result.stream_mode:02X}): ready, responders = {addresses}")
        else:
            print(f"  {result.speed_name} (0x{result.stream_mode:02X}): ready, no I2C ACKs detected")


def group_attempts_by_scan(
    attempts: Sequence[I2cDisplayAttempt], scan_results: Sequence[SpeedScanResult]
) -> list[tuple[SpeedScanResult, list[tuple[I2cDisplayAttempt, int]]]]:
    grouped: list[tuple[SpeedScanResult, list[tuple[I2cDisplayAttempt, int]]]] = []
    for scan_result in scan_results:
        if not scan_result.ready or not scan_result.responding_addresses:
            continue

        matches: list[tuple[I2cDisplayAttempt, int]] = []
        for address in scan_result.responding_addresses:
            for attempt in attempts:
                if attempt.supports_address(address):
                    matches.append((attempt, address))
        if matches:
            grouped.append((scan_result, matches))
    return grouped


def run() -> int:
    dll = Ch341Dll()
    bus = Ch341I2cBus(dll)
    target_panel = get_target_panel_profile()
    attempts = build_attempts()
    results: list[AttemptResult] = []

    print("CH341 display probe starting.")
    print(f"Target panel profile: {target_panel.name}")
    print("This only tests I2C-capable display paths reachable over SDA/SCL.")
    print(target_panel.notes)
    print()

    try:
        print(f"Checking target CH341 availability for {TARGET_INSTANCE_ID}...")
        target_device = verify_target_ch341_device(TARGET_INSTANCE_ID)
        print(f"Target device ready: {target_device.friendly_name} [{target_device.instance_id}]")

        print("Opening CH341 device 0...")
        dll.open(0)
        print("CH341 device 0 opened successfully.")

        print("Scanning I2C bus before display attempts...")
        scan_results = scan_i2c_bus(bus)
        print_scan_summary(scan_results)

        ready_speeds = [result for result in scan_results if result.ready]
        if not ready_speeds:
            print("No usable CH341 I2C stream modes were available.")
            return 1

        if not target_panel.allow_i2c_display_attempts:
            print()
            print("The target panel profile does not support I2C text-render attempts over SDA/SCL alone.")
            print("Stopping here to avoid false positives such as white screens or black line patterns.")
            return 1

        candidate_groups = group_attempts_by_scan(attempts, scan_results)
        if not candidate_groups:
            print()
            print("No discovered I2C address matches the built-in display/controller profiles.")
            print("Stopping before the display loop because there is nothing plausible to try.")
            return 1

        print()
        print("Starting display loop from discovered devices only.")
        for scan_result, matches in candidate_groups:
            bus.set_speed(scan_result.stream_mode)
            print(f"Using CH341 stream mode {scan_result.speed_name} (0x{scan_result.stream_mode:02X})")
            for attempt, address in matches:
                label = f"{attempt.name} @ 0x{address:02X}"
                print(f"Trying {label}")
                try:
                    attempt.write_hello(bus, address)
                except Exception as exc:
                    print(f"  Write failed: {exc}")
                    results.append(AttemptResult(attempt.name, address, scan_result.speed_name, False, str(exc)))
                    continue

                if wait_for_success(3.0):
                    print(f"Confirmed working configuration: {label} at {scan_result.speed_name}")
                    results.append(AttemptResult(attempt.name, address, scan_result.speed_name, True))
                    return 0

                print("  No confirmation, moving on")
                results.append(AttemptResult(attempt.name, address, scan_result.speed_name, False, "Timed out"))
        print()
        print("No tested configuration was confirmed.")
        if results:
            print("Summary of attempted controller/address matches:")
            for result in results:
                print(
                    f"  {result.controller} @ 0x{result.address:02X} on {result.speed_name}: "
                    f"{'success' if result.success else result.error}"
                )
        return 1
    finally:
        dll.close(0)


if __name__ == "__main__":
    raise SystemExit(run())