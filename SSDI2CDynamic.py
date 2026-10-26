import ctypes
import time
import math
from ctypes import c_uint, c_void_p, c_bool, c_ubyte
from PIL import Image, ImageDraw, ImageFont


# ============================================================
# CONFIGURATION
# ============================================================

CH341_INDEX = 0
SSD1306_ADDR = 0x3C

WIDTH = 128
HEIGHT = 64

# Change screens every N seconds
SCREEN_TIME = 4

# CH341 I2C transfer chunk
CHUNK_SIZE = 24


# ============================================================
# LOAD CH341 DLL
# ============================================================

try:
    ch341 = ctypes.WinDLL("CH341DLLA64.DLL")
except OSError:
    try:
        ch341 = ctypes.WinDLL("CH341DLL.DLL")
    except OSError:
        raise RuntimeError(
            "Could not find CH341DLLA64.DLL or CH341DLL.DLL"
        )


# ============================================================
# CH341 DLL DEFINITIONS
# ============================================================

ch341.CH341OpenDevice.argtypes = [c_uint]
ch341.CH341OpenDevice.restype = c_void_p

ch341.CH341CloseDevice.argtypes = [c_uint]
ch341.CH341CloseDevice.restype = None

ch341.CH341SetStream.argtypes = [c_uint, c_uint]
ch341.CH341SetStream.restype = c_bool

ch341.CH341StreamI2C.argtypes = [
    c_uint,
    c_uint,
    c_void_p,
    c_uint,
    c_void_p
]

ch341.CH341StreamI2C.restype = c_bool


# ============================================================
# CH341
# ============================================================

def ch341_open():

    handle = ch341.CH341OpenDevice(CH341_INDEX)

    if not handle or handle == c_void_p(-1).value:
        raise RuntimeError("Could not open CH341 device.")

    # 400 kHz I2C
    if not ch341.CH341SetStream(CH341_INDEX, 2):
        raise RuntimeError("Could not configure CH341.")


def ch341_close():
    ch341.CH341CloseDevice(CH341_INDEX)


def i2c_write(data):

    data = bytes(data)

    # Convert 7-bit address to 8-bit address
    buffer_data = bytes([SSD1306_ADDR << 1]) + data

    buffer = (c_ubyte * len(buffer_data))(*buffer_data)

    result = ch341.CH341StreamI2C(
        CH341_INDEX,
        len(buffer_data),
        ctypes.cast(buffer, c_void_p),
        0,
        None
    )

    if not result:
        raise RuntimeError("I2C write failed.")


# ============================================================
# SSD1306
# ============================================================

def ssd1306_command(command):
    i2c_write([0x00, command])


def ssd1306_commands(commands):

    for command in commands:
        ssd1306_command(command)


def ssd1306_init():

    commands = [
        0xAE,

        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,

        0x8D, 0x14,

        0x20, 0x00,

        0xA1,
        0xC8,

        0xDA, 0x12,

        0x81, 0x7F,

        0xD9, 0xF1,

        0xDB, 0x40,

        0xA4,
        0xA6,

        0xAF
    ]

    ssd1306_commands(commands)


def ssd1306_display(image):

    # Horizontal addressing mode
    ssd1306_commands([
        0x21, 0x00, 0x7F,
        0x22, 0x00, 0x07
    ])

    for page in range(8):

        page_data = []

        for x in range(WIDTH):

            byte = 0

            for bit in range(8):

                y = page * 8 + bit

                if image.getpixel((x, y)):
                    byte |= (1 << bit)

            page_data.append(byte)

        for pos in range(0, WIDTH, CHUNK_SIZE):

            chunk = page_data[pos:pos + CHUNK_SIZE]

            i2c_write([0x40] + chunk)


# ============================================================
# FONTS
# ============================================================

def get_font(size):

    paths = [
        r"C:\Windows\Fonts\arial.ttf",
        r"C:\Windows\Fonts\consola.ttf",
        r"C:\Windows\Fonts\segoeui.ttf"
    ]

    for path in paths:

        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass

    return ImageFont.load_default()


font_small = get_font(10)
font_medium = get_font(14)
font_large = get_font(18)


# ============================================================
# SCREEN 1 - CLOCK
# ============================================================

def screen_clock():

    image = Image.new("1", (WIDTH, HEIGHT), 0)
    draw = ImageDraw.Draw(image)

    draw.text(
        (2, 2),
        "SmartQuant",
        font=font_small,
        fill=1
    )

    draw.line(
        (0, 14, 127, 14),
        fill=1
    )

    current_time = time.strftime("%I:%M:%S")

    draw.text(
        (8, 20),
        current_time,
        font=font_large,
        fill=1
    )

    ampm = time.strftime("%p")

    draw.text(
        (92, 28),
        ampm,
        font=font_small,
        fill=1
    )

    draw.text(
        (25, 48),
        time.strftime("%a %b %d"),
        font=font_small,
        fill=1
    )

    return image


# ============================================================
# SCREEN 2 - BOUNCING BALL
# ============================================================

def screen_ball(frame):

    image = Image.new("1", (WIDTH, HEIGHT), 0)
    draw = ImageDraw.Draw(image)

    draw.text(
        (2, 2),
        "Animation",
        font=font_small,
        fill=1
    )

    # Ball position
    x = int(64 + 45 * math.sin(frame * 0.15))

    y = int(40 + 12 * math.sin(frame * 0.30))

    radius = 5

    draw.ellipse(
        (
            x - radius,
            y - radius,
            x + radius,
            y + radius
        ),
        outline=1,
        fill=1
    )

    # Ground
    draw.line(
        (5, 55, 122, 55),
        fill=1
    )

    return image


# ============================================================
# SCREEN 3 - GRAPH / GAUGE
# ============================================================

def screen_graph(frame):

    image = Image.new("1", (WIDTH, HEIGHT), 0)
    draw = ImageDraw.Draw(image)

    draw.text(
        (2, 0),
        "Market Signal",
        font=font_small,
        fill=1
    )

    # Border
    draw.rectangle(
        (2, 15, 125, 61),
        outline=1
    )

    # Generate animated graph
    points = []

    for x in range(5, 123):

        value = (
            38
            + 10 * math.sin(x * 0.12 + frame * 0.08)
            + 4 * math.sin(x * 0.31)
        )

        points.append((x, int(value)))

    draw.line(
        points,
        fill=1,
        width=1
    )

    return image


# ============================================================
# SCREEN 4 - SMARTQUANT LOGO
# ============================================================

def screen_logo():

    image = Image.new("1", (WIDTH, HEIGHT), 0)
    draw = ImageDraw.Draw(image)

    # Simple SQ logo

    draw.rectangle(
        (12, 8, 55, 51),
        outline=1,
        width=2
    )

    draw.ellipse(
        (20, 16, 47, 43),
        outline=1,
        width=2
    )

    draw.line(
        (40, 36, 52, 48),
        fill=1,
        width=2
    )

    draw.text(
        (65, 18),
        "SMART",
        font=font_small,
        fill=1
    )

    draw.text(
        (65, 32),
        "QUANT",
        font=font_small,
        fill=1
    )

    return image


# ============================================================
# SCREEN 5 - SYSTEM INFO
# ============================================================

def screen_info():

    image = Image.new("1", (WIDTH, HEIGHT), 0)
    draw = ImageDraw.Draw(image)

    draw.text(
        (2, 2),
        "SYSTEM STATUS",
        font=font_small,
        fill=1
    )

    draw.line(
        (0, 14, 127, 14),
        fill=1
    )

    draw.text(
        (4, 19),
        "I2C: 400 kHz",
        font=font_small,
        fill=1
    )

    draw.text(
        (4, 31),
        "OLED: 128 x 64",
        font=font_small,
        fill=1
    )

    draw.text(
        (4, 43),
        "CH341: OK",
        font=font_small,
        fill=1
    )

    return image


# ============================================================
# MAIN
# ============================================================

def main():

    ch341_open()

    print("CH341 connected")

    ssd1306_init()

    print("SSD1306 initialized")

    frame = 0

    try:

        while True:

            # -----------------------------------------------
            # Screen 1 - Clock
            # -----------------------------------------------

            start = time.time()

            while time.time() - start < SCREEN_TIME:

                image = screen_clock()

                ssd1306_display(image)

                time.sleep(1)


            # -----------------------------------------------
            # Screen 2 - Animated ball
            # -----------------------------------------------

            start = time.time()

            while time.time() - start < SCREEN_TIME:

                image = screen_ball(frame)

                ssd1306_display(image)

                frame += 1

                time.sleep(0.08)


            # -----------------------------------------------
            # Screen 3 - Graph
            # -----------------------------------------------

            start = time.time()

            while time.time() - start < SCREEN_TIME:

                image = screen_graph(frame)

                ssd1306_display(image)

                frame += 1

                time.sleep(0.15)


            # -----------------------------------------------
            # Screen 4 - Logo
            # -----------------------------------------------

            image = screen_logo()

            ssd1306_display(image)

            time.sleep(SCREEN_TIME)


            # -----------------------------------------------
            # Screen 5 - System information
            # -----------------------------------------------

            image = screen_info()

            ssd1306_display(image)

            time.sleep(SCREEN_TIME)


    except KeyboardInterrupt:

        print("\nStopping...")


    finally:

        try:
            ssd1306_command(0xAE)
        except:
            pass

        ch341_close()

        print("CH341 closed")


if __name__ == "__main__":
    main()