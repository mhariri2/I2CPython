#!/usr/bin/env python3
"""Live MPU9250 dashboard (CH341 USB-I2C) with magnetometer heading.

Install:  pip install streamlit pandas smbus2
Run:      streamlit run mpu9250_streamlit.py
"""
import glob
import math
import struct
import time
from collections import deque

import pandas as pd
import streamlit as st
from smbus2 import SMBus, i2c_msg

MPU_ADDR = 0x68          # 0x69 if AD0 is high
AK_ADDR = 0x0C
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0
MAG_UT_PER_LSB = 0.15
HISTORY = 200
DIRS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


# ---------- low-level I2C ----------
def find_ch341_bus():
    for path in glob.glob("/sys/class/i2c-dev/i2c-*/name"):
        with open(path) as f:
            if "CH341" in f.read():
                return int(path.split("/")[-2].replace("i2c-", ""))
    raise RuntimeError("CH341 I2C bus not found. Check `i2cdetect -l`.")


def wr(bus, addr, reg, val):
    bus.i2c_rdwr(i2c_msg.write(addr, [reg, val]))


def rd(bus, addr, reg, n=1):
    w = i2c_msg.write(addr, [reg])
    r = i2c_msg.read(addr, n)
    bus.i2c_rdwr(w, r)
    return bytes(list(r))


def init_mpu(bus):
    wr(bus, MPU_ADDR, 0x6B, 0x80)
    time.sleep(0.1)
    wr(bus, MPU_ADDR, 0x6B, 0x01)
    time.sleep(0.05)
    wr(bus, MPU_ADDR, 0x1B, 0x00)
    wr(bus, MPU_ADDR, 0x1C, 0x00)
    wr(bus, MPU_ADDR, 0x37, 0x02)   # bypass -> AK8963 visible
    time.sleep(0.01)


def init_mag(bus):
    try:
        if rd(bus, AK_ADDR, 0x00)[0] != 0x48:
            return None
    except OSError:
        return None
    wr(bus, AK_ADDR, 0x0A, 0x00)
    time.sleep(0.01)
    wr(bus, AK_ADDR, 0x0A, 0x0F)
    time.sleep(0.01)
    asa = rd(bus, AK_ADDR, 0x10, 3)
    adj = [((a - 128) / 256.0) + 1.0 for a in asa]
    wr(bus, AK_ADDR, 0x0A, 0x00)
    time.sleep(0.01)
    wr(bus, AK_ADDR, 0x0A, 0x16)
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
    if rd(bus, AK_ADDR, 0x02)[0] & 0x01 == 0:
        return None
    d = rd(bus, AK_ADDR, 0x03, 7)
    x, y, z = struct.unpack("<hhh", d[:6])
    if d[6] & 0x08:
        return None
    return tuple(v * a * MAG_UT_PER_LSB for v, a in zip((x, y, z), adj))


@st.cache_resource
def get_sensor():
    busnum = find_ch341_bus()
    bus = SMBus(busnum)
    init_mpu(bus)
    return bus, init_mag(bus), busnum


# ---------- state ----------
ss = st.session_state
ss.setdefault("hist", deque(maxlen=HISTORY))
ss.setdefault("mag", None)
ss.setdefault("offset", (0.0, 0.0, 0.0))
ss.setdefault("mn", [1e9] * 3)
ss.setdefault("mx", [-1e9] * 3)
ss.setdefault("yaw", 0.0)
ss.setdefault("last_t", None)
ss.setdefault("gbz", 0.0)
ss.setdefault("biasing", False)
ss.setdefault("bias_samples", [])

# ---------- UI ----------
st.set_page_config(page_title="MPU9250 Live", layout="wide")
st.title("MPU9250 live dashboard")

with st.sidebar:
    st.header("Magnetometer calibration")
    st.caption("Tick, slowly rotate the sensor through all orientations "
               "(figure-8), then untick to apply hard-iron offsets.")
    calibrating = st.checkbox("Calibrating", key="calibrating")
    if ss.get("was_cal") and not calibrating and ss.mx[0] > ss.mn[0]:
        ss.offset = tuple((a + b) / 2 for a, b in zip(ss.mn, ss.mx))
    ss.was_cal = calibrating
    if st.button("Reset calibration"):
        ss.offset = (0.0, 0.0, 0.0)
        ss.mn, ss.mx = [1e9] * 3, [-1e9] * 3
    st.write("Offsets (µT):", [round(v, 1) for v in ss.offset])
    heading_offset = st.number_input("Heading offset / declination (°)", value=0.0, step=0.5)
    st.header("Gyro yaw (no-magnetometer fallback)")
    if st.button("Zero yaw"):
        ss.yaw = 0.0
    if st.button("Calibrate gyro bias (hold still)"):
        ss.bias_samples, ss.biasing = [], True
    st.write("Z bias (°/s):", round(ss.gbz, 3))

try:
    bus, adj, busnum = get_sensor()
except Exception as e:
    st.error(f"Sensor init failed: {e}")
    st.stop()
st.caption(f"/dev/i2c-{busnum}" + ("" if adj else "  |  no magnetometer detected"))


@st.fragment(run_every=0.2)
def live():
    try:
        accel, gyro, temp = read_mpu(bus)
        m = read_mag(bus, adj) if adj else None
    except OSError as e:
        st.error(f"I2C error: {e}. Replug and rerun (cache_resource: press C > Clear cache).")
        return
    if m:
        ss.mag = m
        if ss.get("calibrating"):
            ss.mn = [min(a, b) for a, b in zip(ss.mn, m)]
            ss.mx = [max(a, b) for a, b in zip(ss.mx, m)]

    now = time.monotonic()
    dt = 0 if ss.last_t is None else min(now - ss.last_t, 1.0)
    ss.last_t = now
    if ss.biasing:
        ss.bias_samples.append(gyro[2])
        if len(ss.bias_samples) >= 40:
            ss.gbz = sum(ss.bias_samples) / len(ss.bias_samples)
            ss.biasing = False
    else:
        gz = gyro[2] - ss.gbz
        if abs(gz) > 0.2:                       # deadband against noise drift
            ss.yaw = (ss.yaw + gz * dt) % 360

    mag = ss.mag
    row = {"ax": accel[0], "ay": accel[1], "az": accel[2],
           "gx": gyro[0], "gy": gyro[1], "gz": gyro[2]}
    if mag:
        cx, cy, cz = (v - o for v, o in zip(mag, ss.offset))
        row.update(mx=cx, my=cy, mz=cz)
    ss.hist.append(row)

    c1, c2, c3, c4, c5 = st.columns(5)
    c1.metric("Accel (g)", f"{accel[0]:+.2f} {accel[1]:+.2f} {accel[2]:+.2f}")
    c2.metric("Gyro (°/s)", f"{gyro[0]:+.1f} {gyro[1]:+.1f} {gyro[2]:+.1f}")
    c3.metric("Temp (°C)", f"{temp:.1f}")
    if mag:
        c4.metric("Mag (µT)", f"{cx:+.1f} {cy:+.1f} {cz:+.1f}")
        heading = (math.degrees(math.atan2(cy, cx)) + heading_offset) % 360
        c5.metric("Heading", f"{heading:.0f}° {DIRS[round(heading / 45) % 8]}")
    else:
        c4.metric("Mag (µT)", "n/a")
        c5.metric("Yaw (relative, gyro)", f"{ss.yaw:.0f}°")

    df = pd.DataFrame(ss.hist)
    g1, g2, g3 = st.columns(3)
    g1.caption("Accelerometer (g)")
    g1.line_chart(df[["ax", "ay", "az"]], height=220)
    g2.caption("Gyroscope (°/s)")
    g2.line_chart(df[["gx", "gy", "gz"]], height=220)
    if mag and "mx" in df:
        g3.caption("Magnetometer (µT)")
        g3.line_chart(df[["mx", "my", "mz"]].dropna(), height=220)
    if mag:
        st.caption("Heading is valid with the board held level (z up). "
                   "It is magnetic north; set the sidebar offset for local declination.")
    else:
        st.caption("No magnetometer (chip is an MPU6500). Yaw is integrated from the Z gyro: "
                   "relative to where you zeroed it, and it drifts over time.")


live()
