#!/usr/bin/env python3
"""Read accel, gyro, temp and magnetometer from an MPU9250 via a CH341 USB-I2C adapter.

Uses only raw i2c_rdwr transfers (the CH341 driver supports basic I2C only,
max 26 data bytes per message).

Install:  sudo apt install python3-smbus2     (or: pip install smbus2)
Run:      python3 mpu9250_read.py [bus_number]
"""
import glob
import struct
import sys
import time

from smbus2 import SMBus, i2c_msg

MPU_ADDR = 0x68          # 0x69 if AD0 is high
AK_ADDR = 0x0C           # AK8963 magnetometer (via bypass)

ACCEL_LSB_PER_G = 16384.0     # +/-2 g
GYRO_LSB_PER_DPS = 131.0      # +/-250 dps
MAG_UT_PER_LSB = 0.15         # 16-bit mode


def find_ch341_bus():
    for path in glob.glob("/sys/class/i2c-dev/i2c-*/name"):
        with open(path) as f:
            if "CH341" in f.read():
                return int(path.split("/")[-2].replace("i2c-", ""))
    raise SystemExit("CH341 I2C bus not found. Check `i2cdetect -l`.")


def wr(bus, addr, reg, val):
    bus.i2c_rdwr(i2c_msg.write(addr, [reg, val]))


def rd(bus, addr, reg, n=1):
    w = i2c_msg.write(addr, [reg])
    r = i2c_msg.read(addr, n)
    bus.i2c_rdwr(w, r)
    return bytes(list(r))


def init_mpu(bus):
    who = rd(bus, MPU_ADDR, 0x75)[0]
    print(f"WHO_AM_I = 0x{who:02X} (0x71/0x73 = MPU9250/9255, 0x70 = MPU6500, no magnetometer)")
    wr(bus, MPU_ADDR, 0x6B, 0x80)      # reset
    time.sleep(0.1)
    wr(bus, MPU_ADDR, 0x6B, 0x01)      # wake, PLL clock
    time.sleep(0.05)
    wr(bus, MPU_ADDR, 0x1B, 0x00)      # gyro +/-250 dps
    wr(bus, MPU_ADDR, 0x1C, 0x00)      # accel +/-2 g
    wr(bus, MPU_ADDR, 0x37, 0x02)      # I2C bypass: expose AK8963 on main bus
    time.sleep(0.01)
    return who


def init_mag(bus):
    try:
        wia = rd(bus, AK_ADDR, 0x00)[0]
    except OSError:
        print("Magnetometer not responding (MPU6500/6050 or bypass failed).")
        return None
    if wia != 0x48:
        print(f"Unexpected AK8963 WIA 0x{wia:02X}")
        return None
    wr(bus, AK_ADDR, 0x0A, 0x00)       # power down
    time.sleep(0.01)
    wr(bus, AK_ADDR, 0x0A, 0x0F)       # fuse ROM access
    time.sleep(0.01)
    asa = rd(bus, AK_ADDR, 0x10, 3)
    adj = [((a - 128) / 256.0) + 1.0 for a in asa]
    wr(bus, AK_ADDR, 0x0A, 0x00)
    time.sleep(0.01)
    wr(bus, AK_ADDR, 0x0A, 0x16)       # 16-bit, continuous mode 2 (100 Hz)
    time.sleep(0.01)
    return adj


def read_mpu(bus):
    d = rd(bus, MPU_ADDR, 0x3B, 14)
    ax, ay, az, t, gx, gy, gz = struct.unpack(">hhhhhhh", d)
    return (
        (ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G),
        (gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS),
        t / 333.87 + 21.0,
    )


def read_mag(bus, adj):
    if rd(bus, AK_ADDR, 0x02)[0] & 0x01 == 0:     # ST1 DRDY
        return None
    d = rd(bus, AK_ADDR, 0x03, 7)                 # HXL..HZH, ST2
    x, y, z = struct.unpack("<hhh", d[:6])
    if d[6] & 0x08:                               # overflow
        return None
    return tuple(v * a * MAG_UT_PER_LSB for v, a in zip((x, y, z), adj))


def main():
    busnum = int(sys.argv[1]) if len(sys.argv) > 1 else find_ch341_bus()
    print(f"Using /dev/i2c-{busnum}")
    with SMBus(busnum) as bus:
        init_mpu(bus)
        adj = init_mag(bus)
        mag = None
        try:
            while True:
                accel, gyro, temp = read_mpu(bus)
                if adj:
                    m = read_mag(bus, adj)
                    mag = m or mag
                line = (
                    f"A[g] {accel[0]:+.3f} {accel[1]:+.3f} {accel[2]:+.3f} | "
                    f"G[dps] {gyro[0]:+8.2f} {gyro[1]:+8.2f} {gyro[2]:+8.2f} | "
                    f"T {temp:5.1f}C"
                )
                if mag:
                    line += f" | M[uT] {mag[0]:+7.1f} {mag[1]:+7.1f} {mag[2]:+7.1f}"
                print(line)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()