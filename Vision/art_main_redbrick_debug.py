import seekfree, pyb
import sensor, image, time, tf, gc
from pyb import LED
from machine import UART


# Red-brick / red-sandbag separation debug script.
# This file is intentionally standalone and does not modify art_main.py.
# Defaults are safe for bench/IDE debugging: draw and print are enabled,
# but UART motion/target packets are disabled unless explicitly enabled.


# ================= Hardware =================
red = LED(1)
green = LED(2)
blue = LED(3)
white = LED(4)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240, scaled to 192x192 below.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True, exposure_us=150)
sensor.skip_frames(time=3000)
clock = time.clock()


# ================= Model =================
detect_model = "/sd/detect.tflite"
net = tf.load(detect_model)


# ================= UART =================
uart = UART(12, 9600)
uart.init(9600, timeout_char=1000)

SEARCH_TRIGGER = b"SEARCH\n"
COARSE_TRIGGER = b"COARSE\n"
FINE_TRIGGER = b"FINE\n"
CLASSIFY_TRIGGER = b"CLASSIFY\n"
LINE_TRIGGER = b"LINE\n"
IDLE_TRIGGER = b"IDLE\n"


# ================= Runtime switches =================
DETECT_SCORE_TH = 0.70

# Debug switches requested by the plan.
RED_BRICK_DEBUG_LOG = True
RED_BRICK_DEBUG_DRAW = True
RED_BRICK_UART_SEND = False

# Keep all normal navigation UART disabled in this debug file by default.
# Set this True only when you intentionally want this script to drive the host
# exactly like the production art_main.py target stream.
DEBUG_TARGET_UART_SEND = False

SEND_NEUTRAL_WHEN_EMPTY = True
PRINT_FPS = False
PRINT_EVENT = False
PRINT_VERBOSE = False
RED_BRICK_LOG_EVERY = 4

ERROR_OFFSET = 120
ERROR_LIMIT = 240
ERROR_SCALE = 1.6
UART_FRAME_HEAD = 0xFF
RED_BRICK_PACKET_TAG = 0xFB
LINE_PACKET_TAG = 0xFC
CLASSIFY_PACKET_TAG = 0xFD
NO_TARGET_MARKER = 0xFE

RED_BRICK_NONE = 0
RED_BRICK_LEFT = 1
RED_BRICK_CENTER = 2
RED_BRICK_RIGHT = 3

FRAME_W = 320
FRAME_H = 240
WORK_W = 192
WORK_H = 192
Ema_Alpha = 0.75


# ================= Red-brick debug thresholds =================
# LAB thresholds for red objects. Tune these first if lighting changes.
# The range is intentionally broad because red brick and red sandbag are close.
RED_THRESHOLDS = [
    (12, 100, 18, 127, -5, 127),
]

RED_ROI_X = 0
RED_ROI_Y = int(WORK_H * 0.08)
RED_ROI_W = WORK_W
RED_ROI_H = WORK_H - RED_ROI_Y
RED_MIN_PIXELS = 30
RED_MIN_AREA = 40
RED_MERGE_MARGIN = 3

# Blob geometry: keep this permissive for the first field test.
RED_BLOB_MIN_W = 4
RED_BLOB_MIN_H = 5
RED_BLOB_MAX_W = int(WORK_W * 0.70)
RED_BLOB_MAX_H = int(WORK_H * 0.70)
RED_BLOB_MIN_FILL100 = 18
RED_BRICK_ASPECT_MIN100 = 30     # w / h >= 0.30
RED_BRICK_ASPECT_MAX100 = 380    # w / h <= 3.80
RED_BRICK_MAX_AREA = int(WORK_W * WORK_H * 0.22)
RED_BRICK_CONFIRM_FRAMES = 2

# Model/frame relationship.
RED_MODEL_EDGE_MARGIN = 5
RED_MODEL_SMALL_AREA = int(WORK_W * WORK_H * 0.045)
RED_MODEL_MIN_RED_COVER100 = 25
RED_BLOB_MIN_MODEL_COVER100 = 45

# Full red-sandbag body protection. A complete large red target should not be
# filtered as a brick even though it is red.
REDBAG_PROTECT_MIN_AREA = int(WORK_W * WORK_H * 0.060)
REDBAG_PROTECT_ASPECT_MIN100 = 45
REDBAG_PROTECT_ASPECT_MAX100 = 520
REDBAG_PROTECT_RED_COVER100 = 18
REDBAG_PROTECT_BLOB_COVER100 = 35

# Collision corridor only affects debug obstacle code, not target filtering.
RED_BRICK_CENTER_HALF_W = int(WORK_W * 0.16)


# Red blob tuple indexes.
RB_X1 = 0
RB_Y1 = 1
RB_X2 = 2
RB_Y2 = 3
RB_W = 4
RB_H = 5
RB_PIXELS = 6
RB_AREA = 7
RB_ASPECT100 = 8
RB_FILL100 = 9
RB_EDGE = 10
RB_BRICKLIKE = 11

# Model tuple indexes.
M_X1 = 0
M_Y1 = 1
M_X2 = 2
M_Y2 = 3
M_W = 4
M_H = 5
M_AREA = 6
M_ASPECT100 = 7
M_LABEL = 8
M_SCORE = 9
M_EDGE = 10


# ================= Inverse Perspective =================
IPM_ENABLE = True
IPM_MATRIX_IS_IMAGE_TO_BEV = True
IPM_MATRIX_DST_W = WORK_W
IPM_MATRIX_DST_H = WORK_H
IPM_MATRIX = [
    [-7.67101775, -10.40253246, 866.37487917],
    [0.00000000, -25.70383070, 1156.67238166],
    [0.00015928, -0.10797605, 1.00000000],
]

MM_PER_PIX_X = 3.134328
MM_PER_PIX_Y = 3.093750
BEV_CENTER_X = IPM_MATRIX_DST_W // 2
BEV_CENTER_Y = IPM_MATRIX_DST_H // 2
BEV_TARGET_Y = 155
BEV_FLIP_X = True
BEV_FLIP_Y = False


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def rect_area(x1, y1, x2, y2):
    if x2 <= x1 or y2 <= y1:
        return 0
    return (x2 - x1) * (y2 - y1)


def rect_intersection_area(a, b):
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    return rect_area(x1, y1, x2, y2)


def rect_touches_edge(x1, y1, x2, y2, margin):
    return (
        x1 <= margin
        or y1 <= margin
        or x2 >= WORK_W - 1 - margin
        or y2 >= WORK_H - 1 - margin
    )


def invert_3x3(mtx):
    a = mtx[0][0]
    b = mtx[0][1]
    c = mtx[0][2]
    d = mtx[1][0]
    e = mtx[1][1]
    f = mtx[1][2]
    g = mtx[2][0]
    h = mtx[2][1]
    i = mtx[2][2]
    det = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(det) < 1e-6:
        return None
    inv_det = 1.0 / det
    return [
        [
            (e * i - f * h) * inv_det,
            (c * h - b * i) * inv_det,
            (b * f - c * e) * inv_det,
        ],
        [
            (f * g - d * i) * inv_det,
            (a * i - c * g) * inv_det,
            (c * d - a * f) * inv_det,
        ],
        [
            (d * h - e * g) * inv_det,
            (b * g - a * h) * inv_det,
            (a * e - b * d) * inv_det,
        ],
    ]


def project_point(mtx, x, y):
    den = mtx[2][0] * x + mtx[2][1] * y + mtx[2][2]
    if abs(den) < 1e-6:
        return None, None
    out_x = (mtx[0][0] * x + mtx[0][1] * y + mtx[0][2]) / den
    out_y = (mtx[1][0] * x + mtx[1][1] * y + mtx[1][2]) / den
    return out_x, out_y


def ipm_transform(px, py):
    if not IPM_ENABLE:
        return float(px), float(py)
    if IPM_MATRIX_IS_IMAGE_TO_BEV:
        mtx = IPM_MATRIX
    else:
        mtx = invert_3x3(IPM_MATRIX)
        if mtx is None:
            return float(px), float(py)
    bev_x, bev_y = project_point(mtx, px, py)
    if bev_x is None:
        return float(px), float(py)
    return bev_x, bev_y


def normalize_bev_point(bev_x, bev_y):
    if BEV_FLIP_X:
        bev_x = (IPM_MATRIX_DST_W - 1) - bev_x
    if BEV_FLIP_Y:
        bev_y = (IPM_MATRIX_DST_H - 1) - bev_y
    return bev_x, bev_y


def bev_to_mm(bev_x, bev_y):
    dx_mm = (bev_x - BEV_CENTER_X) * MM_PER_PIX_X
    dy_mm = (bev_y - BEV_CENTER_Y) * MM_PER_PIX_Y
    return dx_mm, dy_mm


def encode_error(value):
    value = clamp(value, -ERROR_LIMIT, ERROR_LIMIT)
    return clamp(int(value / ERROR_SCALE) + ERROR_OFFSET, 0, ERROR_OFFSET * 2)


def send_error(error_x, error_y):
    send_x = encode_error(error_x)
    send_y = encode_error(error_y)
    if DEBUG_TARGET_UART_SEND:
        uart.write(bytearray([UART_FRAME_HEAD, send_x, send_y]))
    return send_x, send_y


def send_no_target():
    if DEBUG_TARGET_UART_SEND:
        uart.write(bytearray([UART_FRAME_HEAD, NO_TARGET_MARKER, NO_TARGET_MARKER]))


def send_red_brick_state(code):
    if RED_BRICK_UART_SEND:
        uart.write(bytearray([UART_FRAME_HEAD, RED_BRICK_PACKET_TAG, code & 0xFF]))


def should_debug_log(frame_count):
    if not RED_BRICK_DEBUG_LOG:
        return False
    if RED_BRICK_LOG_EVERY <= 1:
        return True
    return (frame_count % RED_BRICK_LOG_EVERY) == 0


def collect_model_candidates(img):
    img_w = img.width()
    img_h = img.height()
    models = []
    for obj in tf.detect(net, img):
        x1, y1, x2, y2, label, score = obj
        if score <= DETECT_SCORE_TH:
            continue
        x1 = clamp(int(x1 * img_w), 0, img_w - 1)
        y1 = clamp(int(y1 * img_h), 0, img_h - 1)
        x2 = clamp(int(x2 * img_w), 0, img_w - 1)
        y2 = clamp(int(y2 * img_h), 0, img_h - 1)
        if x2 <= x1 or y2 <= y1:
            continue
        w = x2 - x1
        h = y2 - y1
        area = w * h
        aspect100 = (w * 100) // max(h, 1)
        edge = 1 if rect_touches_edge(x1, y1, x2, y2, RED_MODEL_EDGE_MARGIN) else 0
        models.append((x1, y1, x2, y2, w, h, area, aspect100, label, score, edge))
    return models


def collect_red_blobs(img, log_this_frame):
    blobs = []
    roi = (RED_ROI_X, RED_ROI_Y, RED_ROI_W, RED_ROI_H)
    for blob in img.find_blobs(
        RED_THRESHOLDS,
        roi=roi,
        pixels_threshold=RED_MIN_PIXELS,
        area_threshold=RED_MIN_AREA,
        merge=True,
        margin=RED_MERGE_MARGIN,
    ):
        x1 = blob.x()
        y1 = blob.y()
        w = blob.w()
        h = blob.h()
        x2 = x1 + w
        y2 = y1 + h
        if w < RED_BLOB_MIN_W or h < RED_BLOB_MIN_H:
            continue
        if w > RED_BLOB_MAX_W or h > RED_BLOB_MAX_H:
            continue
        area = w * h
        pixels = blob.pixels()
        if area <= 0:
            continue
        aspect100 = (w * 100) // max(h, 1)
        fill100 = (pixels * 100) // area
        edge = 1 if rect_touches_edge(x1, y1, x2, y2, RED_MODEL_EDGE_MARGIN) else 0
        bricklike = 0
        if (
            fill100 >= RED_BLOB_MIN_FILL100
            and area <= RED_BRICK_MAX_AREA
            and aspect100 >= RED_BRICK_ASPECT_MIN100
            and aspect100 <= RED_BRICK_ASPECT_MAX100
        ):
            bricklike = 1
        item = (x1, y1, x2, y2, w, h, pixels, area, aspect100, fill100, edge, bricklike)
        blobs.append(item)
        if log_this_frame:
            print(
                "RB blob x=%d y=%d w=%d h=%d aspect100=%d fill100=%d area=%d edge=%d bricklike=%d"
                % (x1, y1, w, h, aspect100, fill100, area, edge, bricklike)
            )
    return blobs


def best_red_overlap_for_model(model, red_blobs):
    best_blob = None
    best_overlap = 0
    best_model_cover100 = 0
    best_blob_cover100 = 0
    model_rect = (model[M_X1], model[M_Y1], model[M_X2], model[M_Y2])
    for rb in red_blobs:
        rb_rect = (rb[RB_X1], rb[RB_Y1], rb[RB_X2], rb[RB_Y2])
        overlap = rect_intersection_area(model_rect, rb_rect)
        if overlap > best_overlap:
            best_overlap = overlap
            best_blob = rb
            best_model_cover100 = (overlap * 100) // max(model[M_AREA], 1)
            best_blob_cover100 = (overlap * 100) // max(rb[RB_AREA], 1)
    return best_blob, best_overlap, best_model_cover100, best_blob_cover100


def model_protects_redbag(model, red_blobs):
    rb, overlap, model_cover100, blob_cover100 = best_red_overlap_for_model(model, red_blobs)
    if rb is None or overlap <= 0:
        return False, None, model_cover100, blob_cover100
    if model[M_EDGE]:
        return False, rb, model_cover100, blob_cover100
    if model[M_AREA] < REDBAG_PROTECT_MIN_AREA:
        return False, rb, model_cover100, blob_cover100
    if (
        model[M_ASPECT100] < REDBAG_PROTECT_ASPECT_MIN100
        or model[M_ASPECT100] > REDBAG_PROTECT_ASPECT_MAX100
    ):
        return False, rb, model_cover100, blob_cover100
    if (
        model_cover100 >= REDBAG_PROTECT_RED_COVER100
        or blob_cover100 >= REDBAG_PROTECT_BLOB_COVER100
    ):
        return True, rb, model_cover100, blob_cover100
    return False, rb, model_cover100, blob_cover100


def model_is_suspected_brick(model, red_blobs):
    protected, rb, model_cover100, blob_cover100 = model_protects_redbag(model, red_blobs)
    if protected:
        return False, "redbag_protect", rb, model_cover100, blob_cover100
    if rb is None:
        return False, "no_red_overlap", None, model_cover100, blob_cover100
    if not rb[RB_BRICKLIKE]:
        return False, "red_not_bricklike", rb, model_cover100, blob_cover100
    weak_model = (
        model[M_EDGE]
        or model[M_AREA] <= RED_MODEL_SMALL_AREA
        or rb[RB_EDGE]
    )
    enough_overlap = (
        model_cover100 >= RED_MODEL_MIN_RED_COVER100
        or blob_cover100 >= RED_BLOB_MIN_MODEL_COVER100
    )
    if weak_model and enough_overlap:
        if model[M_EDGE]:
            return True, "edge_redbrick", rb, model_cover100, blob_cover100
        if model[M_AREA] <= RED_MODEL_SMALL_AREA:
            return True, "small_redbrick", rb, model_cover100, blob_cover100
        return True, "blob_edge_redbrick", rb, model_cover100, blob_cover100
    return False, "weak_or_overlap_not_enough", rb, model_cover100, blob_cover100


def draw_red_blob(img, rb, confirmed):
    if not RED_BRICK_DEBUG_DRAW:
        return
    color = (255, 0, 0)
    label = "BRICK_BLOB"
    if confirmed:
        color = (255, 0, 255)
        label = "BRICK_OK"
    img.draw_rectangle(rb[RB_X1], rb[RB_Y1], rb[RB_W], rb[RB_H], color=color, thickness=2)
    img.draw_string(rb[RB_X1], max(0, rb[RB_Y1] - 10), label, color=color, scale=1)


def draw_model(img, model, filtered, protected, target):
    if not RED_BRICK_DEBUG_DRAW:
        return
    if target:
        color = (0, 255, 0)
        label = "TARGET"
    elif filtered:
        color = (255, 255, 0)
        label = "FILTERED_MODEL"
    elif protected:
        color = (0, 255, 255)
        label = "REDBAG_PROTECT"
    else:
        color = (128, 128, 128)
        label = "MODEL"
    img.draw_rectangle(
        model[M_X1],
        model[M_Y1],
        model[M_W],
        model[M_H],
        color=color,
        thickness=2,
    )
    img.draw_string(model[M_X1], max(0, model[M_Y1] - 10), label, color=color, scale=1)


def calc_red_brick_code(red_blobs):
    best = None
    for rb in red_blobs:
        if not rb[RB_BRICKLIKE]:
            continue
        if best is None or rb[RB_PIXELS] > best[RB_PIXELS]:
            best = rb
    if best is None:
        return RED_BRICK_NONE, None
    center_x = (best[RB_X1] + best[RB_X2]) // 2
    if center_x < (WORK_W // 2 - RED_BRICK_CENTER_HALF_W):
        return RED_BRICK_LEFT, best
    if center_x > (WORK_W // 2 + RED_BRICK_CENTER_HALF_W):
        return RED_BRICK_RIGHT, best
    return RED_BRICK_CENTER, best


def select_target_with_redbrick_filter(img, models, red_blobs, log_this_frame):
    best = None
    filtered_count = 0
    protected_count = 0
    for model in models:
        filtered, reason, rb, model_cover100, blob_cover100 = model_is_suspected_brick(model, red_blobs)
        protected = reason == "redbag_protect"
        if filtered:
            filtered_count += 1
        if protected:
            protected_count += 1
        if log_this_frame:
            print(
                "MODEL label=%s score=%.2f x=%d y=%d w=%d h=%d edge=%d "
                "suspect_brick=%d filtered=%d protect=%d red_model100=%d red_blob100=%d reason=%s"
                % (
                    str(model[M_LABEL]),
                    model[M_SCORE],
                    model[M_X1],
                    model[M_Y1],
                    model[M_W],
                    model[M_H],
                    model[M_EDGE],
                    1 if filtered else 0,
                    1 if filtered else 0,
                    1 if protected else 0,
                    model_cover100,
                    blob_cover100,
                    reason,
                )
            )
        if filtered:
            draw_model(img, model, True, False, False)
            continue
        if (best is None) or (model[M_Y2] > best[M_Y2]) or (
            model[M_Y2] == best[M_Y2] and model[M_SCORE] > best[M_SCORE]
        ):
            best = model
        draw_model(img, model, False, protected, False)
    if best is not None:
        draw_model(img, best, False, False, True)
    if log_this_frame:
        if best is None:
            reason = "filtered_by_redbrick" if filtered_count else "no_model"
            print("TARGET selected=0 reason=%s filtered=%d protected=%d" %
                  (reason, filtered_count, protected_count))
        else:
            print(
                "TARGET selected=1 x=%d y=%d w=%d h=%d filtered=%d protected=%d"
                % (best[M_X1], best[M_Y1], best[M_W], best[M_H], filtered_count, protected_count)
            )
    return best, filtered_count, protected_count


def print_state_log(mode, event, score=None, label=None, error_x=None, error_y=None,
                    send_x=None, send_y=None, verbose=False):
    if verbose:
        if not PRINT_VERBOSE:
            return
    elif not PRINT_EVENT:
        return
    msg = "%s %s" % (mode, event)
    if score is not None:
        msg += " s=%.2f" % score
    if label is not None:
        msg += " label=%s" % str(label)
    if error_x is not None and error_y is not None:
        msg += " err=(%d,%d)" % (error_x, error_y)
    if send_x is not None and send_y is not None:
        msg += " send=(%d,%d)" % (send_x, send_y)
    print(msg)


def set_detect_mode(new_mode, event, clear_state=True):
    global detect_mode, frozen_error, ema_bev_x, ema_bev_y
    global coarse_frame_in_interval, coarse_frame_count
    detect_mode = new_mode
    if clear_state:
        frozen_error = None
        ema_bev_x = None
        ema_bev_y = None
        coarse_frame_in_interval = 0
        coarse_frame_count = 0
    print_state_log("UART", event)


print("RED_BRICK_DEBUG start draw=%d log=%d rb_uart=%d target_uart=%d" %
      (
          1 if RED_BRICK_DEBUG_DRAW else 0,
          1 if RED_BRICK_DEBUG_LOG else 0,
          1 if RED_BRICK_UART_SEND else 0,
          1 if DEBUG_TARGET_UART_SEND else 0,
      ))

detect_mode = "SEARCH"
frozen_error = None
ema_bev_x = None
ema_bev_y = None
coarse_frame_in_interval = 0
coarse_frame_count = 0
uart_rx_buf = bytearray()
frame_count = 0
red_brick_confirm_count = 0
red_brick_last_code = RED_BRICK_NONE


while True:
    clock.tick()
    green.on()

    if uart.any():
        uart_data = uart.read()
        if uart_data:
            uart_rx_buf += uart_data
            if SEARCH_TRIGGER in uart_rx_buf:
                set_detect_mode("SEARCH", "TRIGGER->SEARCH")
                uart_rx_buf = bytearray()
            elif COARSE_TRIGGER in uart_rx_buf:
                set_detect_mode("COARSE", "TRIGGER->COARSE")
                uart_rx_buf = bytearray()
            elif FINE_TRIGGER in uart_rx_buf:
                set_detect_mode("FINE", "TRIGGER->FINE")
                uart_rx_buf = bytearray()
            elif CLASSIFY_TRIGGER in uart_rx_buf:
                set_detect_mode("CLASSIFY", "TRIGGER->CLASSIFY")
                uart_rx_buf = bytearray()
            elif LINE_TRIGGER in uart_rx_buf:
                set_detect_mode("LINE", "TRIGGER->LINE")
                uart_rx_buf = bytearray()
            elif IDLE_TRIGGER in uart_rx_buf:
                set_detect_mode("IDLE", "TRIGGER->IDLE")
                uart_rx_buf = bytearray()
            elif len(uart_rx_buf) > 20:
                uart_rx_buf = bytearray()

    img = sensor.snapshot().scale(
        x_scale=WORK_W / float(FRAME_W),
        y_scale=WORK_H / float(FRAME_H),
    )
    log_this_frame = should_debug_log(frame_count)

    if detect_mode == "IDLE":
        if SEND_NEUTRAL_WHEN_EMPTY:
            send_no_target()
        frame_count += 1
        continue

    red_blobs = collect_red_blobs(img, log_this_frame)
    raw_code, raw_code_blob = calc_red_brick_code(red_blobs)
    if raw_code == RED_BRICK_NONE:
        red_brick_confirm_count = 0
        red_brick_last_code = RED_BRICK_NONE
        confirmed_code = RED_BRICK_NONE
    else:
        if raw_code == red_brick_last_code:
            if red_brick_confirm_count < RED_BRICK_CONFIRM_FRAMES:
                red_brick_confirm_count += 1
        else:
            red_brick_last_code = raw_code
            red_brick_confirm_count = 1
        confirmed_code = raw_code if red_brick_confirm_count >= RED_BRICK_CONFIRM_FRAMES else RED_BRICK_NONE

    if raw_code_blob is not None:
        draw_red_blob(img, raw_code_blob, confirmed_code != RED_BRICK_NONE)

    send_red_brick_state(confirmed_code)
    if log_this_frame:
        print(
            "RB state raw=%d confirmed=%d count=%d"
            % (raw_code, confirmed_code, red_brick_confirm_count)
        )

    models = collect_model_candidates(img)
    target, filtered_count, protected_count = select_target_with_redbrick_filter(
        img,
        models,
        red_blobs,
        log_this_frame,
    )

    if target is None:
        frozen_error = None
        if SEND_NEUTRAL_WHEN_EMPTY:
            send_no_target()
        print_state_log(detect_mode, "NO_TARGET", verbose=True)
    else:
        mid_x = (target[M_X1] + target[M_X2]) // 2
        bottom_y = target[M_Y2]
        bev_x, bev_y = ipm_transform(mid_x, bottom_y)
        bev_x, bev_y = normalize_bev_point(bev_x, bev_y)

        if detect_mode == "SEARCH" or detect_mode == "COARSE":
            if ema_bev_x is None:
                ema_bev_x = bev_x
                ema_bev_y = bev_y
            else:
                ema_bev_x = Ema_Alpha * bev_x + (1.0 - Ema_Alpha) * ema_bev_x
                ema_bev_y = Ema_Alpha * bev_y + (1.0 - Ema_Alpha) * ema_bev_y
            out_bev_x = ema_bev_x
            out_bev_y = ema_bev_y
        else:
            out_bev_x = bev_x
            out_bev_y = bev_y

        error_x = int(out_bev_x) - BEV_CENTER_X
        error_y = BEV_TARGET_Y - int(out_bev_y)
        frozen_error = (error_x, error_y)
        dx_mm, dy_mm = bev_to_mm(out_bev_x, out_bev_y)
        send_x, send_y = send_error(error_x, error_y)
        if log_this_frame:
            print(
                "TRACK mode=%s label=%s score=%.2f px=(%d,%d) bev=(%.1f,%.1f) "
                "mm=(%.0f,%.0f) err=(%d,%d) send=(%d,%d)"
                % (
                    detect_mode,
                    str(target[M_LABEL]),
                    target[M_SCORE],
                    mid_x,
                    bottom_y,
                    out_bev_x,
                    out_bev_y,
                    dx_mm,
                    dy_mm,
                    error_x,
                    error_y,
                    send_x,
                    send_y,
                )
            )

    frame_count += 1
    coarse_frame_count += 1
    if PRINT_FPS and (frame_count & 0x07) == 0:
        print(clock.fps())
    if (frame_count & 0x07) == 0:
        gc.collect()
