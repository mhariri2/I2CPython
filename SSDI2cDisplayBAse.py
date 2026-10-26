import ctypes
import time
from ctypes import c_uint, c_void_p, c_bool, POINTER, c_ubyte
from PIL import Image, ImageDraw, ImageFont


# ============================================================
# Configuration
# ============================================================

CH341_INDEX = 0

# Most SSD1306 I2C modules use 0x3C.
# Try 0x3D if yours does not respond.
SSD1306_ADDR = 0x3C

WIDTH = 128
HEIGHT = 64

# Number of bytes that can safely be sent in one CH341
# I2C transaction.
CHUNK_SIZE = 24

# Font
FONT_SIZE = 10

# Text to display
LINE1 = "SmartQuant"
LINE2 = "CH341T + SSD1306"
LINE3 = "System Monitor"


# ============================================================
# Load CH341 DLL
# ============================================================

try:
    ch341 = ctypes.WinDLL("CH341DLLA64.DLL")
except OSError:
    try:
        ch341 = ctypes.WinDLL("CH341DLL.DLL")
    except OSError:
        raise RuntimeError(
            "Could not find CH341DLLA64.DLL or CH341DLL.DLL.\n"
            "Place the appropriate DLL in this program's directory."
        )


# ============================================================
# Define CH341 DLL functions
# ============================================================

ch341.CH341OpenDevice.argtypes = [c_uint]
ch341.CH341OpenDevice.restype = c_void_p

ch341.CH341CloseDevice.argtypes = [c_uint]
ch341.CH341CloseDevice.restype = None

ch341.CH341SetStream.argtypes = [c_uint, c_uint]
ch341.CH341SetStream.restype = c_bool

ch341.CH341StreamI2C.argtypes = [
    c_uint,       # device index
    c_uint,       # write length
    c_void_p,     # write buffer
    c_uint,       # read length
    c_void_p      # read buffer
]
ch341.CH341StreamI2C.restype = c_bool


# ============================================================
# CH341 I2C functions
# ============================================================

def ch341_open():
    handle = ch341.CH341OpenDevice(CH341_INDEX)

    if not handle or handle == c_void_p(-1).value:
        raise RuntimeError("Could not open CH341 device.")

    # I2C speed:
    # 0 = 20 kHz
    # 1 = 100 kHz
    # 2 = 400 kHz
    # 3 = 750 kHz
    #
    # 400 kHz is a good choice for an SSD1306.
    if not ch341.CH341SetStream(CH341_INDEX, 2):
        raise RuntimeError("Could not configure CH341 I2C speed.")

    print("CH341 opened successfully.")


def ch341_close():
    ch341.CH341CloseDevice(CH341_INDEX)
    print("CH341 closed.")


def i2c_write(data):
    """
    Write raw bytes to the SSD1306.

    CH341StreamI2C expects the first byte to contain the
    8-bit I2C address (7-bit address shifted left by 1).
    """

    data = bytes(data)

    # SSD1306 address + data
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
        raise RuntimeError("CH341 I2C write failed.")


# ============================================================
# SSD1306 functions
# ============================================================

def ssd1306_command(command):
    """
    Send one command to the SSD1306.
    0x00 = command control byte.
    """

    i2c_write([0x00, command])


def ssd1306_commands(commands):
    """
    Send multiple SSD1306 commands.
    """

    for command in commands:
        ssd1306_command(command)


def ssd1306_init():

    # SSD1306 initialization for 128x64
    commands = [
        0xAE,       # Display OFF

        0xD5, 0x80, # Display clock divide ratio
        0xA8, 0x3F, # Multiplex ratio = 64
        0xD3, 0x00, # Display offset
        0x40,       # Start line = 0

        0x8D, 0x14, # Charge pump ON

        0x20, 0x00, # Horizontal addressing mode

        0xA1,       # Segment remap
        0xC8,       # COM scan direction

        0xDA, 0x12, # COM pins configuration

        0x81, 0x7F, # Contrast

        0xD9, 0xF1, # Pre-charge

        0xDB, 0x40, # VCOMH

        0xA4,       # Resume RAM display
        0xA6,       # Normal display

        0xAF        # Display ON
    ]

    ssd1306_commands(commands)


def ssd1306_clear():

    # Set column address
    ssd1306_commands([
        0x21,  # Column address
        0x00,
        0x7F
    ])

    # Set page address
    ssd1306_commands([
        0x22,  # Page address
        0x00,
        0x07
    ])

    # Each page contains 128 bytes
    blank = bytes([0] * 128)

    for page in range(8):
        # Page must be selected
        ssd1306_command(0xB0 + page)

        for pos in range(0, 128, CHUNK_SIZE):
            chunk = blank[pos:pos + CHUNK_SIZE]

            # 0x40 = display RAM data
            i2c_write([0x40] + list(chunk))


def ssd1306_display(image):

    """
    Send a 128x64 1-bit Pillow image to SSD1306.
    """

    # SSD1306 has 8 pages of 8 pixels each.
    for page in range(8):

        ssd1306_command(0xB0 + page)

        # Start at column 0
        ssd1306_command(0x00)
        ssd1306_command(0x10)

        page_data = []

        for x in range(WIDTH):

            byte = 0

            for bit in range(8):

                y = page * 8 + bit

                if image.getpixel((x, y)):
                    byte |= (1 << bit)

            page_data.append(byte)

        # Send page in small chunks
        for pos in range(0, WIDTH, CHUNK_SIZE):

            chunk = page_data[pos:pos + CHUNK_SIZE]

            # 0x40 = display RAM data
            i2c_write([0x40] + chunk)


# ============================================================
# Text rendering
# ============================================================

def get_font(size=10):

    # Windows usually has Arial here.
    font_paths = [
        r"C:\Windows\Fonts\arial.ttf",
        r"C:\Windows\Fonts\consola.ttf",
        r"C:\Windows\Fonts\segoeui.ttf"
    ]

    for path in font_paths:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass

    # Pillow fallback
    return ImageFont.load_default()


font = get_font(FONT_SIZE)


def create_screen():

    image = Image.new("1", (WIDTH, HEIGHT), 0)

    draw = ImageDraw.Draw(image)

    # Static information
    draw.text(
        (0, 0),
        LINE1,
        font=font,
        fill=1
    )

    draw.text(
        (0, 14),
        LINE2,
        font=font,
        fill=1
    )

    draw.text(
        (0, 28),
        LINE3,
        font=font,
        fill=1
    )

    # Current time
    current_time = time.strftime("%I:%M:%S %p")

    draw.text(
        (0, 45),
        current_time,
        font=font,
        fill=1
    )

    return image


# ============================================================
# Main program
# ============================================================

def main():

    try:

        ch341_open()

        print("Initializing SSD1306...")
        ssd1306_init()

        print("Clearing display...")
        ssd1306_clear()

        print("Starting clock... Press Ctrl+C to stop.")

        while True:

            start = time.time()

            image = create_screen()

            ssd1306_display(image)

            # Maintain approximately one-second updates
            elapsed = time.time() - start

            time.sleep(max(0.01, 1.0 - elapsed))

    except KeyboardInterrupt:

        print("\nStopping...")

    finally:

        try:
            # Turn display off before exiting
            ssd1306_command(0xAE)
        except:
            pass

        ch341_close()


if __name__ == "__main__":
    main()