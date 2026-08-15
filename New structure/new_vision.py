import seekfree, pyb
import sensor, image, time, tf, gc
from pyb import LED #导入LED CLASSIFY_SCORE_TH = 0.45
from machine import UART
#test
red = LED(1)    # 定义一个LED1   红灯
green = LED(2)
blue = LED(3)
white = LED(4)
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565.
sensor.set_framesize(sensor.QVGA)      # 320x240, scaled to 192x192 in code
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True, exposure_us = 150)
sensor.skip_frames(time = 3000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.


# ================= Model =================
detect_model = "/sd/detect.tflite"
classify_model = "/sd/classify.tflite"
net = tf.load(detect_model)
classify_net = None
try:
    classify_net = tf.load(classify_model, load_to_fb = True)
except Exception as ex:
    print("CLASSIFY LOAD FAIL:", ex)


# ================= UART =================
uart = UART(12, 9600)
uart.init(9600, timeout_char = 1000)
SEARCH_TRIGGER = b"SEARCH\n"
COARSE_TRIGGER = b"COARSE\n"
FINE_TRIGGER = b"FINE\n"
CLASSIFY_TRIGGER = b"CLASSIFY\n"
LINE_TRIGGER = b"LINE\n"
IDLE_TRIGGER = b"IDLE\n"


# ================= Runtime =================
DETECT_SCORE_TH = 0.70
SEND_NEUTRAL_WHEN_EMPTY = True
DEBUG_DRAW_BOX = False
PRINT_FPS = False
PRINT_EVENT = False
PRINT_VERBOSE = False

ERROR_OFFSET = 120
ERROR_LIMIT = 240
ERROR_SCALE = 1.6
UART_FRAME_HEAD = 0xFF
LINE_PACKET_TAG = 0xFC
CLASSIFY_PACKET_TAG = 0xFD
CLASSIFY_DIR_RIGHT = 1
CLASSIFY_DIR_UP = 2
CLASSIFY_DIR_LEFT = 3
LINE_STATE_NONE = 0
LINE_STATE_CROSSED = 1
CLASSIFY_SCORE_TH = 0.80
CLASSIFY_LABELS = [
    "ball",
    "redbag",
    "bluebag",
    "brownbear",
    "whitebear",
]
NO_TARGET_MARKER = 0xFE   # 254，超出 encode_error 范围 [0,240]，永不冲突

FRAME_W = 320
FRAME_H = 240
WORK_W  = 192
WORK_H  = 192
Ema_Alpha = 0.75
YELLOW_LINE_THRESHOLDS = [(60, 100, -128, 2, 22, 127)]
LINE_ROI_Y = WORK_H // 2
LINE_MIN_PIXELS = 77
LINE_MIN_AREA = 77
LINE_MIN_BOTTOM = int(WORK_H * 0.70)
LINE_SIDE_MIN_WIDTH = int(WORK_W * 0.16)
LINE_SIDE_MIN_ASPECT = 1.5
LINE_SIDE_MIN_FILL = 0.35
LINE_BALL_SLANT_MIN_WIDTH = int(WORK_W * 0.12)
LINE_BALL_SLANT_MIN_FILL = 0.18
LINE_BALL_SLANT_MIN_ELONGATION = 0.68
LINE_BALL_SLANT_MIN_BOTTOM = (WORK_H * 78 + 99) // 100
LINE_BALL_EDGE_MARGIN = 2
LINE_CENTER_MASK_W = int(WORK_W * 0.42)
LINE_CONFIRM_FRAMES = 1
LINE_BALL_CONFIRM_FRAMES = 1
LINE_WHITEBEAR_CONFIRM_FRAMES = 2
FIRST_PUSH_YELLOW_MIN_PIXELS = 30
FIRST_PUSH_YELLOW_MIN_AREA = 30
FIRST_PUSH_YELLOW_MIN_COVER100 = 15

# ================= Red brick filtering =================
# Production path: no draw, no log. These thresholds are copied from the
# validated debug script and only protect target selection from red-brick
# false model hits.
RED_THRESHOLDS = [(12, 100, 18, 127, -5, 127)]
RED_ROI_X = 0
RED_ROI_Y = int(WORK_H * 0.08)
RED_ROI_W = WORK_W
RED_ROI_H = WORK_H - RED_ROI_Y
RED_MIN_PIXELS = 30
RED_MIN_AREA = 40
RED_MERGE_MARGIN = 3
RED_BLOB_MAX_COUNT = 6
RED_BLOB_MIN_W = 4
RED_BLOB_MIN_H = 5
RED_BLOB_MAX_W = int(WORK_W * 0.70)
RED_BLOB_MAX_H = int(WORK_H * 0.70)
RED_BLOB_MIN_FILL100 = 18
RED_BRICK_ASPECT_MIN100 = 30
RED_BRICK_ASPECT_MAX100 = 380
RED_BRICK_MAX_AREA = int(WORK_W * WORK_H * 0.42)
RED_MODEL_EDGE_MARGIN = 5
RED_MODEL_SMALL_AREA = int(WORK_W * WORK_H * 0.045)
RED_MODEL_MIN_RED_COVER100 = 25
RED_BLOB_MIN_MODEL_COVER100 = 45
COMPACT_TARGET_PROTECT_SCORE = 0.85
COMPACT_TARGET_PROTECT_MIN_AREA = 120
COMPACT_TARGET_PROTECT_MODEL_COVER100 = 60
COMPACT_TARGET_PROTECT_BLOB_COVER100 = 70
COMPACT_TARGET_PROTECT_ASPECT_MIN100 = 45
COMPACT_TARGET_PROTECT_ASPECT_MAX100 = 220
REDBAG_PROTECT_MIN_AREA = int(WORK_W * WORK_H * 0.060)
REDBAG_PROTECT_ASPECT_MIN100 = 45
REDBAG_PROTECT_ASPECT_MAX100 = 520
REDBAG_PROTECT_RED_COVER100 = 18
REDBAG_PROTECT_BLOB_COVER100 = 35
BOTTOM_REDBAG_PROTECT_SCORE = 0.75
BOTTOM_REDBAG_PROTECT_MIN_AREA = (WORK_W * WORK_H * 25 + 999) // 1000
BOTTOM_REDBAG_PROTECT_MIN_H = (WORK_H * 14 + 99) // 100
BOTTOM_REDBAG_PROTECT_ASPECT_MIN100 = 30
BOTTOM_REDBAG_PROTECT_ASPECT_MAX100 = 420
BOTTOM_REDBAG_PROTECT_MODEL_COVER100 = 35
BOTTOM_REDBAG_PROTECT_BLOB_COVER100 = 35
RED_TARGET_OWNED_BLOB_COVER100 = 55
RED_TARGET_OWNED_MODEL_COVER100 = 20
RED_BRICK_CENTER_HALF_W = int(WORK_W * 0.16)
RED_BRICK_SCORE_Y_WEIGHT = 2
RED_BRICK_SCORE_PIXELS_WEIGHT = 1
RED_BRICK_CONFIRM_FRAMES = 2
RED_BRICK_NONE = 0
RED_BRICK_LEFT = 1
RED_BRICK_CENTER = 2
RED_BRICK_RIGHT = 3

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

M_X1 = 0
M_Y1 = 1
M_X2 = 2
M_Y2 = 3
M_LABEL = 4
M_SCORE = 5


# ================= Inverse Perspective =================
# Replace this matrix with your calibration result.
# Default assumption: image coordinates -> bird-view coordinates.
IPM_ENABLE = True
IPM_MATRIX_IS_IMAGE_TO_BEV = True

# If calibration was done on another image size, update these values too.
IPM_MATRIX_DST_W = WORK_W
IPM_MATRIX_DST_H = WORK_H

# Calibrated after camera angle update at 192x192 work size.
IPM_MATRIX = [
    [7.10786812, 8.93241664, -626.42158003],
    [-0.66566980, 23.29844310, -738.89348118],
    [-0.00364567, 0.09582190, 1.00000000],
]

# BEV 像素 -> 物理距离换算 (mm)
# 标定：594mm x 210mm 纸，_bev_w=84, BEV 240x240
MM_PER_PIX_X = 3.134328
MM_PER_PIX_Y = 3.093750
BEV_CENTER_X = (IPM_MATRIX_DST_W // 2) + 8   # 96
BEV_CENTER_Y = IPM_MATRIX_DST_H // 2   # 96
BEV_TARGET_Y = 175    # pre-push target line in BEV
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
            (b * f - c * e) * inv_det
        ],
        [
            (f * g - d * i) * inv_det,
            (a * i - c * g) * inv_det,
            (c * d - a * f) * inv_det
        ],
        [
            (d * h - e * g) * inv_det,
            (b * g - a * h) * inv_det,
            (a * e - b * d) * inv_det
        ]
    ]


def project_point(mtx, x, y):
    den = mtx[2][0] * x + mtx[2][1] * y + mtx[2][2]
    if abs(den) < 1e-6:
        return None, None

    out_x = (mtx[0][0] * x + mtx[0][1] * y + mtx[0][2]) / den
    out_y = (mtx[1][0] * x + mtx[1][1] * y + mtx[1][2]) / den
    return out_x, out_y


def ipm_transform(px, py):
    """将图像像素坐标通过 IPM 矩阵映射到 BEV 坐标。"""
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
    """BEV 像素 -> 相对 BEV 中心的物理偏移 (mm)。"""
    dx_mm = (bev_x - BEV_CENTER_X) * MM_PER_PIX_X
    dy_mm = (bev_y - BEV_CENTER_Y) * MM_PER_PIX_Y
    return dx_mm, dy_mm


def encode_error(value):
    value = clamp(value, -ERROR_LIMIT, ERROR_LIMIT)
    return clamp(int(value / ERROR_SCALE) + ERROR_OFFSET, 0, ERROR_OFFSET * 2)


def send_error(error_x, error_y):
    send_x = encode_error(error_x)
    send_y = encode_error(error_y)
    uart.write(bytearray([UART_FRAME_HEAD, send_x, send_y]))
    return send_x, send_y


def send_no_target():
    uart.write(bytearray([UART_FRAME_HEAD, NO_TARGET_MARKER, NO_TARGET_MARKER]))


def send_classify_dir(dir_code):
    uart.write(bytearray([UART_FRAME_HEAD, CLASSIFY_PACKET_TAG, dir_code & 0xFF]))


def send_line_state(line_state):
    uart.write(bytearray([UART_FRAME_HEAD, LINE_PACKET_TAG, line_state & 0xFF]))


def classify_dir_name(dir_code):
    if dir_code == CLASSIFY_DIR_RIGHT:
        return "RIGHT"
    if dir_code == CLASSIFY_DIR_UP:
        return "UP"
    if dir_code == CLASSIFY_DIR_LEFT:
        return "LEFT"
    return "NONE"


def collect_red_blobs(img):
    blobs = []
    for blob in img.find_blobs(
        RED_THRESHOLDS,
        roi = (RED_ROI_X, RED_ROI_Y, RED_ROI_W, RED_ROI_H),
        pixels_threshold = RED_MIN_PIXELS,
        area_threshold = RED_MIN_AREA,
        merge = True,
        margin = RED_MERGE_MARGIN,
    ):
        x1 = blob.x()
        y1 = blob.y()
        w = blob.w()
        h = blob.h()
        if w < RED_BLOB_MIN_W or h < RED_BLOB_MIN_H:
            continue
        if w > RED_BLOB_MAX_W or h > RED_BLOB_MAX_H:
            continue
        area = w * h
        if area <= 0:
            continue
        pixels = blob.pixels()
        aspect100 = (w * 100) // max(h, 1)
        fill100 = (pixels * 100) // area
        edge = 1 if rect_touches_edge(x1, y1, x1 + w, y1 + h, RED_MODEL_EDGE_MARGIN) else 0
        bricklike = 0
        if (
            fill100 >= RED_BLOB_MIN_FILL100
            and area <= RED_BRICK_MAX_AREA
            and aspect100 >= RED_BRICK_ASPECT_MIN100
            and aspect100 <= RED_BRICK_ASPECT_MAX100
        ):
            bricklike = 1
        item = (x1, y1, x1 + w, y1 + h, w, h, pixels, area, aspect100, fill100, edge, bricklike)
        if len(blobs) < RED_BLOB_MAX_COUNT:
            blobs.append(item)
        else:
            min_idx = 0
            min_pixels = blobs[0][RB_PIXELS]
            for idx in range(1, RED_BLOB_MAX_COUNT):
                if blobs[idx][RB_PIXELS] < min_pixels:
                    min_idx = idx
                    min_pixels = blobs[idx][RB_PIXELS]
            if pixels > min_pixels:
                blobs[min_idx] = item
    return blobs


def model_area(model):
    return rect_area(model[M_X1], model[M_Y1], model[M_X2], model[M_Y2])


def model_aspect100(model):
    return ((model[M_X2] - model[M_X1]) * 100) // max(model[M_Y2] - model[M_Y1], 1)


def model_touches_edge(model):
    return rect_touches_edge(
        model[M_X1],
        model[M_Y1],
        model[M_X2],
        model[M_Y2],
        RED_MODEL_EDGE_MARGIN,
    )


def model_touches_bottom(model):
    return model[M_Y2] >= WORK_H - 1 - RED_MODEL_EDGE_MARGIN


def red_blob_touches_bottom(rb):
    return rb[RB_Y2] >= WORK_H - 1 - RED_MODEL_EDGE_MARGIN


def best_red_overlap_for_model(model, red_blobs):
    best_blob = None
    best_overlap = 0
    best_model_cover100 = 0
    best_blob_cover100 = 0
    model_rect = (model[M_X1], model[M_Y1], model[M_X2], model[M_Y2])
    area = model_area(model)
    for rb in red_blobs:
        rb_rect = (rb[RB_X1], rb[RB_Y1], rb[RB_X2], rb[RB_Y2])
        overlap = rect_intersection_area(model_rect, rb_rect)
        if overlap > best_overlap:
            best_overlap = overlap
            best_blob = rb
            best_model_cover100 = (overlap * 100) // max(area, 1)
            best_blob_cover100 = (overlap * 100) // max(rb[RB_AREA], 1)
    return best_blob, best_overlap, best_model_cover100, best_blob_cover100


def bottom_partial_model_protects_redbag(model, rb, model_cover100, blob_cover100):
    if rb is None:
        return False
    if not model_touches_bottom(model) and not red_blob_touches_bottom(rb):
        return False
    if model[M_SCORE] < BOTTOM_REDBAG_PROTECT_SCORE:
        return False
    area = model_area(model)
    if area < BOTTOM_REDBAG_PROTECT_MIN_AREA:
        return False
    height = model[M_Y2] - model[M_Y1]
    if height < BOTTOM_REDBAG_PROTECT_MIN_H:
        return False
    aspect100 = model_aspect100(model)
    if (
        aspect100 < BOTTOM_REDBAG_PROTECT_ASPECT_MIN100
        or aspect100 > BOTTOM_REDBAG_PROTECT_ASPECT_MAX100
    ):
        return False
    return (
        model_cover100 >= BOTTOM_REDBAG_PROTECT_MODEL_COVER100
        or blob_cover100 >= BOTTOM_REDBAG_PROTECT_BLOB_COVER100
    )


def model_protects_redbag(model, red_blobs):
    rb, overlap, model_cover100, blob_cover100 = best_red_overlap_for_model(model, red_blobs)
    if rb is None or overlap <= 0:
        return False, None, model_cover100, blob_cover100
    if bottom_partial_model_protects_redbag(model, rb, model_cover100, blob_cover100):
        return True, rb, model_cover100, blob_cover100
    if model_touches_edge(model):
        return False, rb, model_cover100, blob_cover100
    if model_area(model) < REDBAG_PROTECT_MIN_AREA:
        return False, rb, model_cover100, blob_cover100
    aspect100 = model_aspect100(model)
    if aspect100 < REDBAG_PROTECT_ASPECT_MIN100 or aspect100 > REDBAG_PROTECT_ASPECT_MAX100:
        return False, rb, model_cover100, blob_cover100
    if model_cover100 >= REDBAG_PROTECT_RED_COVER100 or blob_cover100 >= REDBAG_PROTECT_BLOB_COVER100:
        return True, rb, model_cover100, blob_cover100
    return False, rb, model_cover100, blob_cover100


def compact_model_target_protects(model, rb, model_cover100, blob_cover100):
    if rb is None:
        return False
    if model_touches_edge(model) or rb[RB_EDGE]:
        return False
    if model[M_SCORE] < COMPACT_TARGET_PROTECT_SCORE:
        return False
    area = model_area(model)
    if area < COMPACT_TARGET_PROTECT_MIN_AREA or area > RED_MODEL_SMALL_AREA:
        return False
    aspect100 = model_aspect100(model)
    if (
        aspect100 < COMPACT_TARGET_PROTECT_ASPECT_MIN100
        or aspect100 > COMPACT_TARGET_PROTECT_ASPECT_MAX100
    ):
        return False
    return (
        model_cover100 >= COMPACT_TARGET_PROTECT_MODEL_COVER100
        and blob_cover100 >= COMPACT_TARGET_PROTECT_BLOB_COVER100
    )


def model_is_suspected_brick(model, red_blobs):
    protected, rb, model_cover100, blob_cover100 = model_protects_redbag(model, red_blobs)
    if protected:
        return False
    if rb is None:
        return False
    if compact_model_target_protects(model, rb, model_cover100, blob_cover100):
        return False
    if not rb[RB_BRICKLIKE]:
        return False
    weak_model = model_touches_edge(model) or rb[RB_EDGE]
    enough_overlap = (
        model_cover100 >= RED_MODEL_MIN_RED_COVER100
        or blob_cover100 >= RED_BLOB_MIN_MODEL_COVER100
    )
    return weak_model and enough_overlap


def red_blob_is_target_owned(rb, target):
    if target is None:
        return False
    rb_rect = (rb[RB_X1], rb[RB_Y1], rb[RB_X2], rb[RB_Y2])
    target_rect = (target[M_X1], target[M_Y1], target[M_X2], target[M_Y2])
    overlap = rect_intersection_area(rb_rect, target_rect)
    if overlap <= 0:
        return False
    blob_cover100 = (overlap * 100) // max(rb[RB_AREA], 1)
    target_cover100 = (overlap * 100) // max(model_area(target), 1)
    return (
        blob_cover100 >= RED_TARGET_OWNED_BLOB_COVER100
        or target_cover100 >= RED_TARGET_OWNED_MODEL_COVER100
    )


def calc_red_brick_code(red_blobs, target):
    best = None
    best_score = -1
    for rb in red_blobs:
        if not rb[RB_BRICKLIKE]:
            continue
        if red_blob_is_target_owned(rb, target):
            continue
        score = rb[RB_PIXELS] * RED_BRICK_SCORE_PIXELS_WEIGHT + rb[RB_Y2] * RED_BRICK_SCORE_Y_WEIGHT
        if best is None or score > best_score:
            best = rb
            best_score = score
    if best is None:
        return RED_BRICK_NONE, None
    center_x = (best[RB_X1] + best[RB_X2]) // 2
    if center_x < (WORK_W // 2 - RED_BRICK_CENTER_HALF_W):
        return RED_BRICK_LEFT, best
    if center_x > (WORK_W // 2 + RED_BRICK_CENTER_HALF_W):
        return RED_BRICK_RIGHT, best
    return RED_BRICK_CENTER, best


def calc_red_brick_error(rb):
    if rb is None:
        return 0, 0
    mid_x = (rb[RB_X1] + rb[RB_X2]) // 2
    bottom_y = rb[RB_Y2]
    bev_x, bev_y = ipm_transform(mid_x, bottom_y)
    bev_x, bev_y = normalize_bev_point(bev_x, bev_y)
    return int(bev_x) - BEV_CENTER_X, BEV_TARGET_Y - int(bev_y)


def clear_red_brick_state():
    global red_brick_code, red_brick_error_x, red_brick_error_y
    global red_brick_confirm_count, red_brick_last_code

    red_brick_code = RED_BRICK_NONE
    red_brick_error_x = 0
    red_brick_error_y = 0
    red_brick_confirm_count = 0
    red_brick_last_code = RED_BRICK_NONE


def update_red_brick_state(red_blobs, target):
    global red_brick_code, red_brick_error_x, red_brick_error_y
    global red_brick_confirm_count, red_brick_last_code

    raw_code, rb = calc_red_brick_code(red_blobs, target)
    if raw_code == RED_BRICK_NONE:
        clear_red_brick_state()
        return

    if raw_code == red_brick_last_code:
        if red_brick_confirm_count < RED_BRICK_CONFIRM_FRAMES:
            red_brick_confirm_count += 1
    else:
        red_brick_last_code = raw_code
        red_brick_confirm_count = 1

    if red_brick_confirm_count >= RED_BRICK_CONFIRM_FRAMES:
        red_brick_code = raw_code
        red_brick_error_x, red_brick_error_y = calc_red_brick_error(rb)
    else:
        red_brick_code = RED_BRICK_NONE
        red_brick_error_x = 0
        red_brick_error_y = 0


def model_is_yellow_interference(img, model):
    x1 = model[M_X1]
    y1 = model[M_Y1]
    w = model[M_X2] - x1
    h = model[M_Y2] - y1
    area = w * h
    if area <= 0:
        return False

    yellow_pixels = 0
    for blob in img.find_blobs(
        YELLOW_LINE_THRESHOLDS,
        roi = (x1, y1, w, h),
        pixels_threshold = FIRST_PUSH_YELLOW_MIN_PIXELS,
        area_threshold = FIRST_PUSH_YELLOW_MIN_AREA,
        merge = True,
    ):
        yellow_pixels += blob.pixels()
        if yellow_pixels * 100 >= area * FIRST_PUSH_YELLOW_MIN_COVER100:
            return True
    return False


def find_nearest_target(img, use_redbrick_filter = True):
    img_w = img.width()
    img_h = img.height()
    best = None
    red_blobs = None
    if use_redbrick_filter:
        red_blobs = collect_red_blobs(img)

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

        target = (x1, y1, x2, y2, label, score)
        if yellow_model_filter_enabled and model_is_yellow_interference(img, target):
            if PRINT_VERBOSE:
                print_state_log(
                    "FILTER", "YELLOW_REJECT", coarse_frame_count, freeze_count,
                    score = score, label = label,
                    mid_x = (x1 + x2) // 2, bottom_y = y2,
                    verbose = True,
                )
            continue
        if use_redbrick_filter and model_is_suspected_brick(target, red_blobs):
            continue

        if (best is None) or (y2 > best[3]) or ((y2 == best[3]) and (score > best[5])):
            best = target

    if use_redbrick_filter:
        update_red_brick_state(red_blobs, best)
    else:
        clear_red_brick_state()
    return best


def classify_target(img, target):
    if classify_net is None:
        return None, None, 0.0

    x1, y1, x2, y2, _, _ = target
    w = x2 - x1
    h = y2 - y1
    if w < 8 or h < 8:
        return None, None, 0.0

    pad_x = w // 8
    pad_y = h // 8
    #rx1 = clamp(x1 - pad_x, 0, img.width() - 1)
    #ry1 = clamp(y1 - pad_y, 0, img.height() - 1)
    #rx2 = clamp(x2 + pad_x, 0, img.width() - 1)
    #ry2 = clamp(y2 + pad_y, 0, img.height() - 1)
    rx1 = clamp(x1 , 0, img.width() - 1)
    ry1 = clamp(y1 , 0, img.height() - 1)
    rx2 = clamp(x2 , 0, img.width() - 1)
    ry2 = clamp(y2 , 0, img.height() - 1)

    rw = rx2 - rx1
    rh = ry2 - ry1
    if rw < 4 or rh < 4:
        return None, None, 0.0

    roi_img = img.copy(roi = (rx1, ry1, rw, rh))
    best_label = None
    best_score = 0.0
    for obj in tf.classify(
        classify_net,
        roi_img,
        min_scale = 1,
        scale_mul = 0.2,
        x_overlap = 0.2,
        y_overlap = 0.2,
        scale = 1,
        offset = 1,
    ):
        outputs = obj.output()
        best_idx = 0
        for idx in range(1, len(outputs)):
            if outputs[idx] > outputs[best_idx]:
                best_idx = idx
        best_label = CLASSIFY_LABELS[best_idx]
        best_score = outputs[best_idx]
        break

    if best_label is None or best_score < CLASSIFY_SCORE_TH:
        return None, best_label, best_score

    if best_label == "ball":
        return CLASSIFY_DIR_UP, best_label, best_score
    if best_label in ("brownbear", "whitebear"):
        return CLASSIFY_DIR_RIGHT, best_label, best_score
    if best_label in ("redbag", "bluebag"):
        return CLASSIFY_DIR_LEFT, best_label, best_score
    return None, best_label, best_score


def detect_yellow_line(img, allow_ball_slant = False):
    roi_h = img.height() - LINE_ROI_Y
    if roi_h <= 0:
        return False, None

    side_w = (img.width() - LINE_CENTER_MASK_W) // 2
    if side_w <= 0:
        return False, None

    best_blob = None
    right_x = img.width() - side_w
    for roi_x in (0, right_x):
        for blob in img.find_blobs(
            YELLOW_LINE_THRESHOLDS,
            roi = (roi_x, LINE_ROI_Y, side_w, roi_h),
            pixels_threshold = LINE_MIN_PIXELS,
            area_threshold = LINE_MIN_AREA,
            merge = True,
        ):
            w = blob.w()
            h = blob.h()
            bottom = blob.y() + h
            aspect = float(w) / max(h, 1)
            fill = float(blob.pixels()) / max(w * h, 1)
            normal_shape_ok = (
                w >= LINE_SIDE_MIN_WIDTH
                and aspect >= LINE_SIDE_MIN_ASPECT
                and fill >= LINE_SIDE_MIN_FILL
            )
            if allow_ball_slant:
                if roi_x == 0:
                    outer_edge_ok = blob.x() <= LINE_BALL_EDGE_MARGIN
                else:
                    outer_edge_ok = (blob.x() + w) >= (img.width() - LINE_BALL_EDGE_MARGIN)
                slant_shape_ok = (
                    w >= LINE_BALL_SLANT_MIN_WIDTH
                    and fill >= LINE_BALL_SLANT_MIN_FILL
                    and blob.elongation() >= LINE_BALL_SLANT_MIN_ELONGATION
                )
                shape_ok = (
                    outer_edge_ok
                    and bottom >= LINE_BALL_SLANT_MIN_BOTTOM
                    and (normal_shape_ok or slant_shape_ok)
                )
            else:
                shape_ok = normal_shape_ok and bottom >= LINE_MIN_BOTTOM
            if shape_ok:
                if (best_blob is None) or (blob.pixels() > best_blob.pixels()):
                    best_blob = blob

    if best_blob is not None:
        return True, best_blob.rect()

    return False, None


def draw_target_overlay(img, x1, y1, x2, y2):
    img.draw_rectangle(x1, y1, x2 - x1, y2 - y1, color = (0, 255, 0), thickness = 2)


def print_state_cfg():
    if not (PRINT_EVENT or PRINT_VERBOSE):
        return
    print(
        "STATE CFG host_mode=1 ema_alpha=%.2f bev_target_y=%d mm_per_px=(%.3f,%.3f) uart=[SEARCH/COARSE/FINE/LINE/IDLE]" %
        (
            Ema_Alpha,
            BEV_TARGET_Y,
            MM_PER_PIX_X,
            MM_PER_PIX_Y,
        )
    )


def print_state_log(mode, event, coarse_frame_count, freeze_count,
                    score = None, label = None,
                    mid_x = None, bottom_y = None,
                    bev_x = None, bev_y = None,
                    dx_mm = None, dy_mm = None,
                    error_x = None, error_y = None,
                    send_x = None, send_y = None,
                    verbose = False):
    if verbose:
        if not PRINT_VERBOSE:
            return
    elif not PRINT_EVENT:
        return

    msg = "%s %s cf=%d fr=%d" % (mode, event, coarse_frame_count, freeze_count)
    if score is not None:
        msg += " s=%.2f" % score
    if label is not None:
        msg += " cls=%s" % str(label)
    if (mid_x is not None) and (bottom_y is not None):
        msg += " px=(%d,%d)" % (mid_x, bottom_y)
    if (bev_x is not None) and (bev_y is not None):
        msg += " bev=(%.1f,%.1f)" % (bev_x, bev_y)
    if (dx_mm is not None) and (dy_mm is not None):
        msg += " mm=(%.0f,%.0f)" % (dx_mm, dy_mm)
    if (error_x is not None) and (error_y is not None):
        msg += " err=(%d,%d)" % (error_x, error_y)
    if (send_x is not None) and (send_y is not None):
        msg += " send=(%d,%d)" % (send_x, send_y)
    print(msg)


print("IPM_ENABLE=%s IS_IMG_TO_BEV=%s DST=%dx%d" %
      (IPM_ENABLE, IPM_MATRIX_IS_IMAGE_TO_BEV, IPM_MATRIX_DST_W, IPM_MATRIX_DST_H))
print_state_cfg()
detect_mode = "SEARCH"
frozen_error = None
frozen_overlay = None
ema_bev_x = None
ema_bev_y = None
coarse_frame_in_interval = 0
freeze_count = 0
coarse_frame_count = 0
uart_rx_buf = bytearray()
frame_count = 0
classify_sent = False
line_confirm_count = 0
line_ball_slant_state = 0
whitebear_line_confirm_enabled = False
red_brick_code = RED_BRICK_NONE
red_brick_error_x = 0
red_brick_error_y = 0
red_brick_confirm_count = 0
red_brick_last_code = RED_BRICK_NONE
yellow_model_filter_enabled = True


def set_detect_mode(new_mode, event, clear_state = True):
    global detect_mode, frozen_error, frozen_overlay, ema_bev_x, ema_bev_y
    global coarse_frame_in_interval, freeze_count, coarse_frame_count, classify_sent
    global line_confirm_count, line_ball_slant_state
    global whitebear_line_confirm_enabled
    global red_brick_code, red_brick_error_x, red_brick_error_y
    global red_brick_confirm_count, red_brick_last_code
    global yellow_model_filter_enabled

    old_mode = detect_mode
    detect_mode = new_mode
    if clear_state:
        frozen_error = None
        if DEBUG_DRAW_BOX:
            frozen_overlay = None
        ema_bev_x = None
        ema_bev_y = None
        coarse_frame_in_interval = 0
        freeze_count = 0
        coarse_frame_count = 0
        classify_sent = False
        line_confirm_count = 0
        red_brick_code = RED_BRICK_NONE
        red_brick_error_x = 0
        red_brick_error_y = 0
        red_brick_confirm_count = 0
        red_brick_last_code = RED_BRICK_NONE
    if old_mode == "LINE" and new_mode != "LINE":
        whitebear_line_confirm_enabled = False
    if new_mode in ("SEARCH", "COARSE", "CLASSIFY"):
        line_ball_slant_state = 0
        if new_mode == "CLASSIFY":
            whitebear_line_confirm_enabled = False
    elif new_mode == "LINE":
        yellow_model_filter_enabled = False
        if line_ball_slant_state == 1:
            line_ball_slant_state = 2
        elif old_mode != "LINE":
            line_ball_slant_state = 0
    elif old_mode == "LINE" and line_ball_slant_state == 2:
        line_ball_slant_state = 0
    print_state_log("UART", event, coarse_frame_count, freeze_count)


while True:
    clock.tick()
    green.on()        # 打开红灯
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

    img = sensor.snapshot().scale(x_scale = WORK_W / float(FRAME_W),
                                  y_scale = WORK_H / float(FRAME_H))

    if detect_mode == "IDLE":
        if SEND_NEUTRAL_WHEN_EMPTY:
            send_no_target()
    elif detect_mode == "CLASSIFY":
        if not classify_sent:
            target = find_nearest_target(img, False)
            if target is None:
                print_state_log("CLASSIFY", "NO_TARGET", coarse_frame_count, freeze_count, verbose = True)
            else:
                x1, y1, x2, y2, _, _ = target
                dir_code, class_label, class_score = classify_target(img, target)
                if DEBUG_DRAW_BOX:
                    draw_target_overlay(img, x1, y1, x2, y2)
                if dir_code is None:
                    print_state_log(
                        "CLASSIFY", "UNSURE", coarse_frame_count, freeze_count,
                        score = class_score, label = class_label, verbose = True
                    )
                else:
                    send_classify_dir(dir_code)
                    bev_x, bev_y = ipm_transform((x1 + x2) // 2, y2)
                    bev_x, bev_y = normalize_bev_point(bev_x, bev_y)
                    send_error(int(bev_x) - BEV_CENTER_X, BEV_TARGET_Y - int(bev_y))
                    line_ball_slant_state = 1 if dir_code == CLASSIFY_DIR_UP else 0
                    whitebear_line_confirm_enabled = class_label == "whitebear"
                    classify_sent = True
                    print_state_log(
                        "CLASSIFY", classify_dir_name(dir_code), coarse_frame_count, freeze_count,
                        score = class_score, label = class_label, verbose = True
                    )
    elif detect_mode == "LINE":
        allow_ball_slant = line_ball_slant_state == 2
        if whitebear_line_confirm_enabled:
            required_frames = LINE_WHITEBEAR_CONFIRM_FRAMES
        elif allow_ball_slant:
            required_frames = LINE_BALL_CONFIRM_FRAMES
        else:
            required_frames = LINE_CONFIRM_FRAMES
        line_crossed, line_rect = detect_yellow_line(img, allow_ball_slant)
        if line_crossed:
            if line_confirm_count < required_frames:
                line_confirm_count += 1
        else:
            line_confirm_count = 0
        if line_confirm_count >= required_frames:
            send_line_state(LINE_STATE_CROSSED)
            print_state_log("LINE", "CROSSED", coarse_frame_count, freeze_count)
        else:
            send_line_state(LINE_STATE_NONE)
            print_state_log("LINE", "SEARCH", coarse_frame_count, freeze_count, verbose = True)
        if DEBUG_DRAW_BOX and line_rect is not None:
            img.draw_rectangle(line_rect, color = (255, 255, 0), thickness = 2)
    elif detect_mode == "SEARCH" or detect_mode == "COARSE":
        mode_name = detect_mode
        coarse_frame_count += 1

        # 自适应采样间隔：远距离快采样，近距离慢采样
        if frozen_error is None:
            sample_interval = 0
        elif abs(frozen_error[1]) > 140:
            sample_interval = 2
        else:
            sample_interval = 1

        need_detect = False
        if frozen_error is None:
            need_detect = True
        else:
            coarse_frame_in_interval += 1
            if coarse_frame_in_interval >= sample_interval:
                need_detect = True
                coarse_frame_in_interval = 0

        if need_detect:
            target = find_nearest_target(img, True)
            if target is not None:
                x1, y1, x2, y2, label, score = target
                mid_x = (x1 + x2) // 2
                bottom_y = y2
                bev_x, bev_y = ipm_transform(mid_x, bottom_y)
                bev_x, bev_y = normalize_bev_point(bev_x, bev_y)

                if ema_bev_x is None:
                    ema_bev_x = bev_x
                    ema_bev_y = bev_y
                else:
                    ema_bev_x = Ema_Alpha * bev_x + (1.0 - Ema_Alpha) * ema_bev_x
                    ema_bev_y = Ema_Alpha * bev_y + (1.0 - Ema_Alpha) * ema_bev_y

                error_x = int(ema_bev_x) - BEV_CENTER_X
                error_y = BEV_TARGET_Y - int(ema_bev_y)
                frozen_error = (error_x, error_y)
                if DEBUG_DRAW_BOX:
                    frozen_overlay = (x1, y1, x2, y2)

                dx_mm, dy_mm = bev_to_mm(ema_bev_x, ema_bev_y)
                send_x, send_y = send_error(error_x, error_y)
                print_state_log(
                    mode_name, "EMA", coarse_frame_count, coarse_frame_in_interval,
                    score, label, mid_x, bottom_y, ema_bev_x, ema_bev_y,
                    dx_mm, dy_mm, error_x, error_y, send_x, send_y,
                )
            else:
                frozen_error = None
                frozen_overlay = None
                ema_bev_x = None
                ema_bev_y = None
                coarse_frame_in_interval = 0
                print_state_log(mode_name, "MISS", coarse_frame_count,
                                coarse_frame_in_interval, verbose=True)

        if frozen_error is not None:
            if DEBUG_DRAW_BOX and frozen_overlay is not None:
                draw_target_overlay(img, frozen_overlay[0], frozen_overlay[1],
                                    frozen_overlay[2], frozen_overlay[3])
            if not need_detect:
                send_error(frozen_error[0], frozen_error[1])
        else:
            if SEND_NEUTRAL_WHEN_EMPTY:
                send_no_target()
            print_state_log(mode_name, "SEARCH", coarse_frame_count,
                            coarse_frame_in_interval, verbose=True)
    else:
        target = find_nearest_target(img, False)

        if target is None:
            if SEND_NEUTRAL_WHEN_EMPTY:
                send_no_target()

            print_state_log("FINE", "NO_TARGET", coarse_frame_count, freeze_count, verbose = True)
            if (not PRINT_VERBOSE) and PRINT_FPS:
                print(clock.fps())
        else:
            x1, y1, x2, y2, label, score = target
            mid_x = (x1 + x2) // 2
            bottom_y = y2
            bev_x, bev_y = ipm_transform(mid_x, bottom_y)
            bev_x, bev_y = normalize_bev_point(bev_x, bev_y)
            error_x = int(bev_x) - BEV_CENTER_X
            error_y = BEV_TARGET_Y - int(bev_y)
            if DEBUG_DRAW_BOX:
                draw_target_overlay(img, x1, y1, x2, y2)
            dx_mm, dy_mm = bev_to_mm(bev_x, bev_y)
            send_x, send_y = send_error(error_x, error_y)
            print_state_log(
                "FINE", "TRACK", coarse_frame_count, freeze_count,
                score, label, mid_x, bottom_y, bev_x, bev_y,
                dx_mm, dy_mm, error_x, error_y, send_x, send_y,
                verbose = True
            )
            if (not PRINT_VERBOSE) and PRINT_FPS:
                print(clock.fps())

    frame_count += 1
    if (frame_count & 0x07) == 0:
        gc.collect()
