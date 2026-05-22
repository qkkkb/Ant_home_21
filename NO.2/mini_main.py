import gc
import sensor
import time
from machine import UART
from pyb import LED


# ================= Camera =================
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False, exposure_us=2500)
try:
    sensor.set_auto_whitebal(False)
except Exception:
    pass
sensor.skip_frames(time=1500)
clock = time.clock()


# ================= UART =================
uart = UART(12, 9600)
uart.init(9600, timeout_char=1000)

SEARCH_TRIGGER = b"SEARCH\n"
COARSE_TRIGGER = b"COARSE\n"
FINE_TRIGGER = b"FINE\n"
IDLE_TRIGGER = b"IDLE\n"

UART_FRAME_HEAD = 0xFF
NO_TARGET_MARKER = 0xFE
ERROR_OFFSET = 120
ERROR_LIMIT = 240
ERROR_SCALE = 2.0


# ================= IR follow config =================
FRAME_W = 320
FRAME_H = 240
IMG_CENTER_X = FRAME_W // 2

IR_THRESHOLDS = [(215, 255)]
MIN_PIXELS = 4
MIN_AREA = 4
MAX_LAMP_W = 28
MAX_LAMP_H = 28
MAX_PAIR_DY = 32
MIN_PAIR_DX = 8
MAX_PAIR_DX = 180
TARGET_PAIR_DX = 48
TARGET_CENTER_X_OFFSET = 0

EMA_ALPHA_NUM = 3
EMA_ALPHA_DEN = 4
SEND_NO_TARGET_WHEN_EMPTY = True
PRINT_DEBUG = False
DRAW_DEBUG = False


red = LED(1)
green = LED(2)
blue = LED(3)
white = LED(4)

detect_mode = "TRACK"
uart_rx_buf = bytearray()
ema_err_x = None
ema_err_y = None
frame_count = 0


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def encode_error(value):
    value = clamp(value, -ERROR_LIMIT, ERROR_LIMIT)
    return clamp(int(value / ERROR_SCALE) + ERROR_OFFSET, 0, ERROR_OFFSET * 2)


def send_error(err_x, err_y):
    uart.write(bytearray([
        UART_FRAME_HEAD,
        encode_error(err_x),
        encode_error(err_y),
    ]))


def send_no_target():
    uart.write(bytearray([UART_FRAME_HEAD, NO_TARGET_MARKER, NO_TARGET_MARKER]))


def set_mode(mode):
    global detect_mode, ema_err_x, ema_err_y
    detect_mode = mode
    ema_err_x = None
    ema_err_y = None


def poll_uart_mode():
    global uart_rx_buf

    if not uart.any():
        return
    data = uart.read()
    if not data:
        return
    uart_rx_buf += data
    if SEARCH_TRIGGER in uart_rx_buf:
        set_mode("TRACK")
        uart_rx_buf = bytearray()
    elif COARSE_TRIGGER in uart_rx_buf:
        set_mode("TRACK")
        uart_rx_buf = bytearray()
    elif FINE_TRIGGER in uart_rx_buf:
        set_mode("TRACK")
        uart_rx_buf = bytearray()
    elif IDLE_TRIGGER in uart_rx_buf:
        set_mode("IDLE")
        uart_rx_buf = bytearray()
    elif len(uart_rx_buf) > 20:
        uart_rx_buf = bytearray()


def blob_ok(blob):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False
    if w > MAX_LAMP_W or h > MAX_LAMP_H:
        return False
    return True


def better_pair(best, cand):
    if best is None:
        return True
    # Prefer brighter/larger pairs, then pairs closer to the expected span.
    if cand[0] > best[0]:
        return True
    if cand[0] == best[0] and cand[1] < best[1]:
        return True
    return False


def find_lamp_pair(img):
    blobs = img.find_blobs(
        IR_THRESHOLDS,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=True,
    )
    best = None
    count = len(blobs)
    for i in range(count):
        b0 = blobs[i]
        if not blob_ok(b0):
            continue
        for j in range(i + 1, count):
            b1 = blobs[j]
            if not blob_ok(b1):
                continue
            dx = abs(b0.cx() - b1.cx())
            dy = abs(b0.cy() - b1.cy())
            if dx < MIN_PAIR_DX or dx > MAX_PAIR_DX or dy > MAX_PAIR_DY:
                continue
            score = b0.pixels() + b1.pixels()
            span_err = abs(dx - TARGET_PAIR_DX)
            cand = (score, span_err, b0, b1, dx)
            if better_pair(best, cand):
                best = cand
    if best is None:
        return None
    return best[2], best[3], best[4]


def update_ema(err_x, err_y):
    global ema_err_x, ema_err_y

    if ema_err_x is None:
        ema_err_x = err_x
        ema_err_y = err_y
    else:
        ema_err_x = (
            EMA_ALPHA_NUM * err_x
            + (EMA_ALPHA_DEN - EMA_ALPHA_NUM) * ema_err_x
        ) // EMA_ALPHA_DEN
        ema_err_y = (
            EMA_ALPHA_NUM * err_y
            + (EMA_ALPHA_DEN - EMA_ALPHA_NUM) * ema_err_y
        ) // EMA_ALPHA_DEN
    return ema_err_x, ema_err_y


def process_frame(img):
    pair = find_lamp_pair(img)
    if pair is None:
        if SEND_NO_TARGET_WHEN_EMPTY:
            send_no_target()
        if PRINT_DEBUG:
            print("IR MISS")
        return

    b0, b1, span_x = pair
    center_x = (b0.cx() + b1.cx()) // 2
    err_x = center_x - (IMG_CENTER_X + TARGET_CENTER_X_OFFSET)
    err_y = TARGET_PAIR_DX - span_x
    err_x, err_y = update_ema(int(err_x), int(err_y))
    send_error(err_x, err_y)

    if DRAW_DEBUG:
        img.draw_rectangle(b0.rect(), color=255)
        img.draw_rectangle(b1.rect(), color=255)
        img.draw_cross(center_x, (b0.cy() + b1.cy()) // 2, color=255)
    if PRINT_DEBUG:
        print("IR err=(%d,%d) span=%d" % (err_x, err_y, span_x))


while True:
    clock.tick()
    poll_uart_mode()
    green.on()

    img = sensor.snapshot()
    if detect_mode == "IDLE":
        send_no_target()
    else:
        process_frame(img)

    frame_count += 1
    if (frame_count & 0x0F) == 0:
        gc.collect()
