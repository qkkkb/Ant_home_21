import gc
import sensor
from machine import UART
from pyb import LED


EXPOSURE_US = 1200


# ================= Camera =================
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
try:
    sensor.set_auto_whitebal(False)
except Exception:
    pass
sensor.skip_frames(time=800)


# ================= UART =================
uart = UART(12, 9600)
uart.init(9600, timeout_char=1000)

TRACK_TRIGGER = b"TRACK\n"
IDLE_TRIGGER = b"IDLE\n"

UART_FRAME_HEAD = 0xFF
NO_TARGET_MARKER = 0xFE
ERROR_OFFSET = 120
ERROR_LIMIT = 240
ERROR_SCALE = 2.0

uart_packet = bytearray([UART_FRAME_HEAD, ERROR_OFFSET, ERROR_OFFSET, ERROR_OFFSET])
no_target_packet = bytearray([UART_FRAME_HEAD, NO_TARGET_MARKER, NO_TARGET_MARKER])


# ================= IR follow config =================
FRAME_W = 160
FRAME_H = 120
IMG_CENTER_X = FRAME_W // 2

IR_THRESHOLDS = [(215, 255)]
MIN_PIXELS = 3
MIN_AREA = 3
MAX_LAMP_W = 14
MAX_LAMP_H = 14
MAX_PAIR_DY = 16
MIN_PAIR_DX = 4
MAX_PAIR_DX = 90
TARGET_PAIR_DX = 24
TARGET_PAIR_DY = -12
TARGET_CENTER_X_OFFSET = 12
ERROR_OUTPUT_SCALE = 2
PAIR_SPAN_WEIGHT = 3
PAIR_ANGLE_WEIGHT = 2

MAX_PAIR_BLOBS = 10
ROI_PAD_X = 36
ROI_PAD_Y = 24
ROI_MISS_RESET = 2

EMA_ALPHA_NUM = 3
EMA_ALPHA_DEN = 4
SEND_NO_TARGET_WHEN_EMPTY = True
DRAW_DEBUG = True
DRAW_ROI_DEBUG = False
CALIB_LOG_ENABLE = True
CALIB_LOG_INTERVAL = 5
CALIB_LOG_BLOBS = True
GC_FRAME_MASK = 0x3F


MODE_TRACK = "TRACK"
MODE_IDLE = "IDLE"

green = LED(2)
green.on()

detect_mode = MODE_TRACK
uart_rx_buf = bytearray()
ema_err_x = None
ema_err_y = None
ema_err_angle = None
track_roi = None
roi_miss_count = 0
frame_count = 0
blob_pool = [None] * MAX_PAIR_BLOBS
blob_score = [0] * MAX_PAIR_BLOBS


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def encode_error(value):
    value = clamp(value, -ERROR_LIMIT, ERROR_LIMIT)
    return clamp(int(value / ERROR_SCALE) + ERROR_OFFSET, 0, ERROR_OFFSET * 2)


def send_error(err_x, err_y, err_angle):
    uart_packet[1] = encode_error(err_x)
    uart_packet[2] = encode_error(err_y)
    uart_packet[3] = encode_error(err_angle)
    uart.write(uart_packet)


def send_no_target():
    uart.write(no_target_packet)


def set_mode(mode):
    global detect_mode, ema_err_x, ema_err_y, ema_err_angle
    global track_roi, roi_miss_count
    detect_mode = mode
    ema_err_x = None
    ema_err_y = None
    ema_err_angle = None
    track_roi = None
    roi_miss_count = 0


def poll_uart_mode():
    global uart_rx_buf

    if not uart.any():
        return
    data = uart.read()
    if not data:
        return
    uart_rx_buf += data
    if TRACK_TRIGGER in uart_rx_buf:
        set_mode(MODE_TRACK)
        uart_rx_buf = bytearray()
    elif IDLE_TRIGGER in uart_rx_buf:
        set_mode(MODE_IDLE)
        uart_rx_buf = bytearray()
    elif len(uart_rx_buf) > 16:
        uart_rx_buf = bytearray()


def blob_ok(blob):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False
    if w > MAX_LAMP_W or h > MAX_LAMP_H:
        return False
    return True


def collect_candidate_blobs(blobs):
    count = 0
    for blob in blobs:
        if not blob_ok(blob):
            continue
        pixels = blob.pixels()
        if count < MAX_PAIR_BLOBS:
            blob_pool[count] = blob
            blob_score[count] = pixels
            count += 1
        else:
            min_idx = 0
            min_score = blob_score[0]
            for idx in range(1, MAX_PAIR_BLOBS):
                if blob_score[idx] < min_score:
                    min_idx = idx
                    min_score = blob_score[idx]
            if pixels > min_score:
                blob_pool[min_idx] = blob
                blob_score[min_idx] = pixels
    return count


def pair_signed_dy(b0, b1):
    if b0.cx() >= b1.cx():
        return b0.cy() - b1.cy()
    return b1.cy() - b0.cy()


def pair_angle_error_num(pair_dy, span_x):
    return pair_dy * TARGET_PAIR_DX - TARGET_PAIR_DY * span_x


def pair_angle_error(pair_dy, span_x):
    err = pair_angle_error_num(pair_dy, span_x)
    if err < 0:
        err = -err
    return err // TARGET_PAIR_DX


def find_lamp_pair(img, roi):
    if roi is None:
        blobs = img.find_blobs(
            IR_THRESHOLDS,
            pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_AREA,
            merge=True,
        )
    else:
        blobs = img.find_blobs(
            IR_THRESHOLDS,
            roi=roi,
            pixels_threshold=MIN_PIXELS,
            area_threshold=MIN_AREA,
            merge=True,
        )

    count = collect_candidate_blobs(blobs)
    best_b0 = None
    best_b1 = None
    best_dx = 0
    best_score = -1
    best_cost = 10000

    for i in range(count - 1):
        b0 = blob_pool[i]
        for j in range(i + 1, count):
            b1 = blob_pool[j]
            dx = b0.cx() - b1.cx()
            if dx < 0:
                dx = -dx
            dy = b0.cy() - b1.cy()
            if dy < 0:
                dy = -dy
            if dx < MIN_PAIR_DX or dx > MAX_PAIR_DX or dy > MAX_PAIR_DY:
                continue
            score = b0.pixels() + b1.pixels()
            span_err = dx - TARGET_PAIR_DX
            if span_err < 0:
                span_err = -span_err
            angle_err = pair_angle_error(pair_signed_dy(b0, b1), dx)
            cost = span_err * PAIR_SPAN_WEIGHT + angle_err * PAIR_ANGLE_WEIGHT
            if cost < best_cost or (cost == best_cost and score > best_score):
                best_b0 = b0
                best_b1 = b1
                best_dx = dx
                best_score = score
                best_cost = cost

    if best_b0 is None:
        return None
    return best_b0, best_b1, best_dx


def make_track_roi(b0, b1):
    x0 = b0.x()
    y0 = b0.y()
    x1 = b0.x() + b0.w()
    y1 = b0.y() + b0.h()

    if b1.x() < x0:
        x0 = b1.x()
    if b1.y() < y0:
        y0 = b1.y()
    if b1.x() + b1.w() > x1:
        x1 = b1.x() + b1.w()
    if b1.y() + b1.h() > y1:
        y1 = b1.y() + b1.h()

    x0 = clamp(x0 - ROI_PAD_X, 0, FRAME_W - 1)
    y0 = clamp(y0 - ROI_PAD_Y, 0, FRAME_H - 1)
    x1 = clamp(x1 + ROI_PAD_X, x0 + 1, FRAME_W)
    y1 = clamp(y1 + ROI_PAD_Y, y0 + 1, FRAME_H)
    return (x0, y0, x1 - x0, y1 - y0)


def update_ema(err_x, err_y, err_angle):
    global ema_err_x, ema_err_y, ema_err_angle

    if ema_err_x is None:
        ema_err_x = err_x
        ema_err_y = err_y
        ema_err_angle = err_angle
    else:
        ema_err_x = (
            EMA_ALPHA_NUM * err_x
            + (EMA_ALPHA_DEN - EMA_ALPHA_NUM) * ema_err_x
        ) // EMA_ALPHA_DEN
        ema_err_y = (
            EMA_ALPHA_NUM * err_y
            + (EMA_ALPHA_DEN - EMA_ALPHA_NUM) * ema_err_y
        ) // EMA_ALPHA_DEN
        ema_err_angle = (
            EMA_ALPHA_NUM * err_angle
            + (EMA_ALPHA_DEN - EMA_ALPHA_NUM) * ema_err_angle
        ) // EMA_ALPHA_DEN
    return ema_err_x, ema_err_y, ema_err_angle


def should_calib_log():
    if not CALIB_LOG_ENABLE:
        return False
    return (frame_count % CALIB_LOG_INTERVAL) == 0


def print_calib_blob(name, blob):
    print(
        "IR CALIB %s x=%d y=%d w=%d h=%d cx=%d cy=%d pixels=%d"
        % (
            name,
            blob.x(),
            blob.y(),
            blob.w(),
            blob.h(),
            blob.cx(),
            blob.cy(),
            blob.pixels(),
        )
    )


def process_frame(img):
    global track_roi, roi_miss_count

    pair = find_lamp_pair(img, track_roi)
    if pair is None:
        roi_miss_count += 1
        if roi_miss_count >= ROI_MISS_RESET:
            track_roi = None
        if SEND_NO_TARGET_WHEN_EMPTY:
            send_no_target()
        if should_calib_log():
            print(
                "IR CALIB MISS roi=%s threshold=%s exposure_us=%d target_dx=%d"
                % (
                    track_roi,
                    IR_THRESHOLDS,
                    EXPOSURE_US,
                    TARGET_PAIR_DX,
                )
            )
        return

    b0, b1, span_x = pair
    roi_miss_count = 0
    track_roi = make_track_roi(b0, b1)

    center_x = (b0.cx() + b1.cx()) // 2
    center_y = (b0.cy() + b1.cy()) // 2
    pair_dy = pair_signed_dy(b0, b1)
    err_x = (center_x - (IMG_CENTER_X + TARGET_CENTER_X_OFFSET)) * ERROR_OUTPUT_SCALE
    err_y = (TARGET_PAIR_DX - span_x) * ERROR_OUTPUT_SCALE
    err_angle = (pair_angle_error_num(pair_dy, span_x) * ERROR_OUTPUT_SCALE) // TARGET_PAIR_DX
    err_x, err_y, err_angle = update_ema(int(err_x), int(err_y), int(err_angle))
    send_error(err_x, err_y, err_angle)

    if DRAW_DEBUG:
        img.draw_rectangle(b0.rect(), color=255)
        img.draw_rectangle(b1.rect(), color=255)
        img.draw_cross(center_x, center_y, color=255)
        if DRAW_ROI_DEBUG and track_roi is not None:
            img.draw_rectangle(track_roi, color=255)
    if should_calib_log():
        print(
            "IR CALIB HIT center=(%d,%d) span_x=%d pair_dy=%d err=(%d,%d,%d) "
            "target_center_x=%d roi=%s threshold=%s exposure_us=%d target_dx=%d target_dy=%d"
            % (
                center_x,
                center_y,
                span_x,
                pair_dy,
                err_x,
                err_y,
                err_angle,
                IMG_CENTER_X + TARGET_CENTER_X_OFFSET,
                track_roi,
                IR_THRESHOLDS,
                EXPOSURE_US,
                TARGET_PAIR_DX,
                TARGET_PAIR_DY,
            )
        )
        if CALIB_LOG_BLOBS:
            print_calib_blob("L0", b0)
            print_calib_blob("L1", b1)


while True:
    poll_uart_mode()

    img = sensor.snapshot()
    if detect_mode == MODE_IDLE:
        send_no_target()
    else:
        process_frame(img)

    frame_count += 1
    if (frame_count & GC_FRAME_MASK) == 0:
        gc.collect()
