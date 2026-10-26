"""Simulated smbus2 subset (SMBus, i2c_msg) for machines without /dev/i2c (e.g. Windows).

Emulates an MPU9250 + AK8963 well enough to exercise the scripts.
"""
import math
import struct
import time


class _Msg:
    def __init__(self, addr, data, is_read):
        self.addr = addr
        self.data = list(data)
        self.is_read = is_read

    def __iter__(self):
        return iter(self.data)

    def __len__(self):
        return len(self.data)


class i2c_msg:
    @staticmethod
    def write(addr, data):
        return _Msg(addr, data, False)

    @staticmethod
    def read(addr, n):
        return _Msg(addr, [0] * n, True)


class SMBus:
    def __init__(self, bus=0):
        self.bus = bus
        self._t0 = time.time()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def close(self):
        pass

    def i2c_rdwr(self, *msgs):
        reg = None
        for m in msgs:
            if not m.is_read:
                reg = m.data[0] if m.data else None
            else:
                m.data[:] = self._read(m.addr, reg, len(m.data))

    def _read(self, addr, reg, n):
        t = time.time() - self._t0
        if addr == 0x68 and reg == 0x75:
            return [0x71]
        if addr == 0x68 and reg == 0x3B:
            ax, ay, az = int(800 * math.sin(t)), int(800 * math.cos(t)), 16384
            gx, gy, gz = int(500 * math.sin(t * 2)), 0, int(300 * math.cos(t))
            return list(struct.pack(">hhhhhhh", ax, ay, az, 0, gx, gy, gz))[:n]
        if addr == 0x0C:
            if reg == 0x00:
                return [0x48]
            if reg == 0x02:
                return [0x01]
            if reg == 0x03:
                x, y, z = int(300 * math.cos(t / 3)), int(300 * math.sin(t / 3)), -200
                return list(struct.pack("<hhh", x, y, z)) + [0]
            if reg == 0x10:
                return [128, 128, 128]
        return [0] * n
