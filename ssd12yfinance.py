import ctypes
import time
import yfinance as yf

from ctypes import c_uint, c_void_p, c_bool, c_ubyte
from PIL import Image, ImageDraw, ImageFont


# ============================================================
# CONFIGURATION
# ============================================================

CH341_INDEX = 0
SSD1306_ADDR = 0x3C

WIDTH = 128
HEIGHT = 64

STOCKS = [
    "AAPL",
    "MSFT",
    "NVDA",
    "GOOG",
    "AMZN",
    "META",
    "TSLA"
]

# Seconds each stock stays on screen
DISPLAY_SECONDS = 5

# Refresh Yahoo Finance data every 5 minutes
DATA_REFRESH_SECONDS = 300

# CH341 transfer size
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
# CH341 I2C
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

    # Convert 7-bit I2C address to 8-bit
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

    # Set full display range
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

        # Send display data in chunks
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


font_tiny = get_font(8)
font_small = get_font(9)
font_medium = get_font(11)
font_large = get_font(14)


# ============================================================
# YAHOO FINANCE
# ============================================================

def get_stock_data():

    print("Downloading 30-day Yahoo Finance data...")

    data = {}

    try:

        df = yf.download(
            STOCKS,
            period="30d",
            interval="1d",
            auto_adjust=True,
            progress=False,
            group_by="ticker",
            threads=True
        )

        for ticker in STOCKS:

            try:

                prices = df[ticker]["Close"].dropna()

                if len(prices) < 2:
                    print(f"{ticker}: not enough data")
                    continue

                prices = prices.tolist()

                current = prices[-1]
                first = prices[0]

                change = current - first

                change_pct = (
                    change / first
                ) * 100

                data[ticker] = {
                    "prices": prices,
                    "current": current,
                    "change": change,
                    "change_pct": change_pct
                }

                print(
                    f"{ticker}: "
                    f"${current:.2f} "
                    f"{change_pct:+.2f}% "
                    f"({len(prices)} days)"
                )

            except Exception as e:

                print(f"{ticker}: {e}")

    except Exception as e:

        print("Yahoo Finance error:", e)

    return data


# ============================================================
# CREATE 30-DAY STOCK CHART
# ============================================================

def create_stock_screen(ticker, stock):

    image = Image.new(
        "1",
        (WIDTH, HEIGHT),
        0
    )

    draw = ImageDraw.Draw(image)

    prices = stock["prices"]
    current = stock["current"]
    change_pct = stock["change_pct"]

    # ========================================================
    # HEADER
    # ========================================================

    # Ticker
    draw.text(
        (1, 0),
        ticker,
        font=font_medium,
        fill=1
    )

    # Current price
    price_text = f"${current:.2f}"

    draw.text(
        (38, 1),
        price_text,
        font=font_small,
        fill=1
    )

    # 30-day percentage
    change_text = f"{change_pct:+.1f}%"

    # Put percentage on right
    bbox = draw.textbbox(
        (0, 0),
        change_text,
        font=font_small
    )

    text_width = bbox[2] - bbox[0]

    draw.text(
        (127 - text_width, 1),
        change_text,
        font=font_small,
        fill=1
    )

    # Separator
    draw.line(
        (0, 12, 127, 12),
        fill=1
    )

    # ========================================================
    # LARGE CHART
    # ========================================================

    # Almost the entire rest of the display
    chart_left = 1
    chart_right = 126

    chart_top = 15
    chart_bottom = 61

    # Border
    draw.rectangle(
        (
            chart_left,
            chart_top,
            chart_right,
            chart_bottom
        ),
        outline=1
    )

    # --------------------------------------------------------
    # Normalize prices
    # --------------------------------------------------------

    min_price = min(prices)
    max_price = max(prices)

    if max_price == min_price:

        max_price = min_price + 1

    chart_width = (
        chart_right
        - chart_left
        - 4
    )

    chart_height = (
        chart_bottom
        - chart_top
        - 4
    )

    points = []

    # --------------------------------------------------------
    # Create 30-day line
    # --------------------------------------------------------

    for i, price in enumerate(prices):

        if len(prices) > 1:

            x = (
                chart_left
                + 2
                + int(
                    i
                    * chart_width
                    / (len(prices) - 1)
                )
            )

        else:

            x = chart_left + 2

        normalized = (
            price - min_price
        ) / (
            max_price - min_price
        )

        y = (
            chart_bottom
            - 2
            - int(
                normalized
                * chart_height
            )
        )

        points.append((x, y))

    # --------------------------------------------------------
    # Draw the chart
    # --------------------------------------------------------

    if len(points) >= 2:

        draw.line(
            points,
            fill=1,
            width=1
        )

    # --------------------------------------------------------
    # Highlight latest price
    # --------------------------------------------------------

    if points:

        x, y = points[-1]

        draw.rectangle(
            (
                x - 1,
                y - 1,
                x + 1,
                y + 1
            ),
            fill=1
        )

    # --------------------------------------------------------
    # Show high / low
    # --------------------------------------------------------

    low_text = f"{min_price:.0f}"
    high_text = f"{max_price:.0f}"

    draw.text(
        (3, chart_bottom - 8),
        low_text,
        font=font_tiny,
        fill=1
    )

    # Only display high if it doesn't interfere too badly
    draw.text(
        (3, chart_top + 1),
        high_text,
        font=font_tiny,
        fill=1
    )

    return image


# ============================================================
# ERROR SCREEN
# ============================================================

def error_screen():

    image = Image.new(
        "1",
        (WIDTH, HEIGHT),
        0
    )

    draw = ImageDraw.Draw(image)

    draw.text(
        (20, 8),
        "Yahoo Finance",
        font=font_medium,
        fill=1
    )

    draw.text(
        (35, 27),
        "No data",
        font=font_medium,
        fill=1
    )

    draw.text(
        (25, 46),
        "Retrying...",
        font=font_small,
        fill=1
    )

    return image


# ============================================================
# MAIN
# ============================================================

def main():

    print("Starting CH341T + SSD1306...")

    ch341_open()

    print("CH341 connected")

    ssd1306_init()

    print("SSD1306 initialized")

    # Initial Yahoo download
    stock_data = get_stock_data()

    last_data_update = time.time()

    stock_index = 0

    try:

        while True:

            # =================================================
            # REFRESH DATA
            # =================================================

            if (
                time.time() - last_data_update
                >= DATA_REFRESH_SECONDS
            ):

                new_data = get_stock_data()

                if new_data:
                    stock_data = new_data

                last_data_update = time.time()

            # =================================================
            # NO DATA
            # =================================================

            if not stock_data:

                image = error_screen()

                ssd1306_display(image)

                time.sleep(10)

                continue

            # =================================================
            # NEXT STOCK
            # =================================================

            ticker = STOCKS[
                stock_index % len(STOCKS)
            ]

            stock_index += 1

            # Skip unavailable stocks
            if ticker not in stock_data:
                continue

            stock = stock_data[ticker]

            # =================================================
            # DISPLAY
            # =================================================

            print(
                f"Displaying {ticker}: "
                f"${stock['current']:.2f} "
                f"{stock['change_pct']:+.2f}%"
            )

            image = create_stock_screen(
                ticker,
                stock
            )

            ssd1306_display(image)

            # =================================================
            # WAIT
            # =================================================

            time.sleep(DISPLAY_SECONDS)

    except KeyboardInterrupt:

        print("\nStopping...")

    finally:

        try:
            ssd1306_command(0xAE)
        except:
            pass

        ch341_close()

        print("CH341 closed")


# ============================================================
# START
# ============================================================

if __name__ == "__main__":
    main()