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


# ================= Model =================
detect_model = "/sd/detect.tflite"
classify_model = "/sd/classify.tflite"
net = tf.load(detect_model)
classify_net = None
try:
    classify_net = tf.load(classify_model, load_to_fb = True)
except Exception:
    pass


# ================= UART =================
uart = UART(12, 9600)
uart.init(9600, timeout_char = 1000)
SEARCH_TRIGGER = b"SEARCH\n"
COARSE_TRIGGER = b"COARSE\n"
FOLLOW_FINE_TRIGGER = b"FOLLOW_FINE\n"
FINE_TRIGGER = b"FINE\n"
CLASSIFY_TRIGGER = b"CLASSIFY\n"
PUSH_TRIGGER = b"PUSH\n"
LINE_TRIGGER = b"LINE\n"
IDLE_TRIGGER = b"IDLE\n"


# ================= Runtime =================
DETECT_SCORE_TH = 0.70
SEND_NEUTRAL_WHEN_EMPTY = True

ERROR_OFFSET = 120
ERROR_LIMIT = 240
ERROR_SCALE = 1.6
UART_FRAME_HEAD = 0xFF
RED_BRICK_PACKET_TAG = 0xFB
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
# Production path: no draw and no red-brick log. Thresholds are copied from
# the validated bench script for model filtering and independent avoidance data.
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
MODEL_MAX_COUNT = 6
RED_BRICK_CENTER_HALF_W = int(WORK_W * 0.16)
RED_BRICK_SCORE_Y_WEIGHT = 2
RED_BRICK_SCORE_PIXELS_WEIGHT = 1
RED_BRICK_CONFIRM_FRAMES = 2
RED_BRICK_NONE = 0
RED_BRICK_LEFT = 1
RED_BRICK_CENTER = 2
RED_BRICK_RIGHT = 3
RED_BRICK_PACKET_PRESENT = 0x01
RED_BRICK_STABLE_X = 16
RED_BRICK_STABLE_Y = 20

REDBAG_SIZE_TOL = 0.30
CALIB_FILE = "/sd/calib_91.txt"
V2_REDBAG = 0
V2_BRICK = 1
V2_REJECTED = 2

PUSH_BAG_CENTER_HALF_W = int(WORK_W * 0.20)
PUSH_BAG_MIN_W = int(WORK_W * 0.18)
PUSH_BAG_MIN_H = int(WORK_H * 0.18)
PUSH_BAG_MIN_AREA = int(WORK_W * WORK_H * 0.04)
PUSH_BAG_BOTTOM_MARGIN = 6
PUSH_BAG_LOW_MIN_Y1 = 80
PUSH_BAG_LOW_MIN_BOTTOM = 153
PUSH_BAG_LOW_MIN_W = 44
PUSH_BAG_LOW_MIN_H = 60
PUSH_BAG_LOW_MIN_AREA = 3000

RB_X1 = 0
RB_Y1 = 1
RB_X2 = 2
RB_Y2 = 3
RB_W = 4
RB_H = 5
RB_PIXELS = 6
RB_AREA = 7

M_X1 = 0
M_Y1 = 1
M_X2 = 2
M_Y2 = 3
M_LABEL = 4
M_SCORE = 5


def load_redbag_calib(path):
    points = bytearray()
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split(",")
                if len(parts) >= 4:
                    points.append(int(parts[0]))
                    points.append(int(parts[1]))
                    points.append(int(parts[2]))
                    points.append(int(parts[3]))
    except OSError:
        pass
    return points


REDBAG_CALIB = load_redbag_calib(CALIB_FILE)
gc.collect()


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

BEV_CENTER_X = (IPM_MATRIX_DST_W // 2) + 8   # 96
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


def send_red_brick_state():
    red_brick_tx_buf[0] = UART_FRAME_HEAD
    red_brick_tx_buf[1] = RED_BRICK_PACKET_TAG
    if red_brick_code == RED_BRICK_NONE:
        red_brick_tx_buf[2] = 0
        red_brick_tx_buf[3] = 0
        red_brick_tx_buf[4] = 0
    else:
        red_brick_tx_buf[2] = RED_BRICK_PACKET_PRESENT
        red_brick_tx_buf[3] = red_brick_mid_x & 0xFF
        red_brick_tx_buf[4] = red_brick_bottom_y & 0xFF
    uart.write(red_brick_tx_buf)


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
        item = (x1, y1, x1 + w, y1 + h, w, h, pixels, area)
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


def collect_model_candidates(img):
    models = []
    img_w = img.width()
    img_h = img.height()
    for obj in tf.detect(net, img):
        x1, y1, x2, y2, label, score = obj
        if score <= DETECT_SCORE_TH:
            continue
        x1 = clamp(int(x1 * img_w), 0, img_w - 1)
        y1 = clamp(int(y1 * img_h), 0, img_h - 1)
        x2 = clamp(int(x2 * img_w), 0, img_w - 1)
        y2 = clamp(int(y2 * img_h), 0, img_h - 1)
        if x2 > x1 and y2 > y1 and len(models) < MODEL_MAX_COUNT:
            models.append((x1, y1, x2, y2, label, score))
    return models


def find_matching_model(rb, models):
    best = None
    best_overlap = 0
    rb_rect = (rb[RB_X1], rb[RB_Y1], rb[RB_X2], rb[RB_Y2])
    for model in models:
        overlap = rect_intersection_area(
            rb_rect,
            (model[M_X1], model[M_Y1], model[M_X2], model[M_Y2]),
        )
        if overlap > best_overlap:
            best_overlap = overlap
            best = model
    return best


def find_matching_red_blob_index(model, red_blobs):
    best_index = -1
    best_overlap = 0
    model_rect = (model[M_X1], model[M_Y1], model[M_X2], model[M_Y2])
    for index in range(len(red_blobs)):
        rb = red_blobs[index]
        overlap = rect_intersection_area(
            model_rect,
            (rb[RB_X1], rb[RB_Y1], rb[RB_X2], rb[RB_Y2]),
        )
        if overlap > best_overlap:
            best_overlap = overlap
            best_index = index
    return best_index


def expected_redbag_size(bottom_y, center_x):
    if not REDBAG_CALIB:
        return None, None
    total_weight = 0.0
    expected_w = 0.0
    expected_h = 0.0
    idx = 0
    while idx < len(REDBAG_CALIB):
        dy = bottom_y - REDBAG_CALIB[idx]
        dx = center_x - REDBAG_CALIB[idx + 1]
        weight = 1.0 / (dy * dy + dx * dx + 0.000001)
        total_weight += weight
        expected_w += weight * REDBAG_CALIB[idx + 2]
        expected_h += weight * REDBAG_CALIB[idx + 3]
        idx += 4
    return expected_w / total_weight, expected_h / total_weight


def is_false_redbag(rb):
    expected_w, expected_h = expected_redbag_size(
        rb[RB_Y2], (rb[RB_X1] + rb[RB_X2]) // 2,
    )
    if expected_w is None or expected_h is None:
        return False
    width_error = abs(rb[RB_W] - expected_w) / expected_w
    height_error = abs(rb[RB_H] - expected_h) / expected_h
    return width_error > REDBAG_SIZE_TOL or height_error > REDBAG_SIZE_TOL


def classify_red_blobs(red_blobs, models):
    classes = bytearray(len(red_blobs))
    for index in range(len(red_blobs)):
        rb = red_blobs[index]
        if redbag_completed:
            classes[index] = V2_BRICK
            continue
        matched_model = find_matching_model(rb, models)
        if matched_model is None:
            classes[index] = V2_BRICK
        elif is_false_redbag(rb):
            classes[index] = V2_REJECTED
        else:
            classes[index] = V2_REDBAG
    return classes


def calc_red_brick_code(red_blobs, red_classes, push_foreground = None):
    best = None
    best_score = -1
    for index in range(len(red_blobs)):
        rb = red_blobs[index]
        if push_foreground is not None and rb == push_foreground:
            continue
        if red_classes[index] == V2_REDBAG:
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
    global red_brick_mid_x, red_brick_bottom_y
    global red_brick_last_mid_x, red_brick_last_bottom_y

    red_brick_code = RED_BRICK_NONE
    red_brick_error_x = 0
    red_brick_error_y = 0
    red_brick_confirm_count = 0
    red_brick_last_code = RED_BRICK_NONE
    red_brick_mid_x = 0
    red_brick_bottom_y = 0
    red_brick_last_mid_x = -1000
    red_brick_last_bottom_y = -1000


def update_red_brick_state(red_blobs, red_classes, push_foreground = None):
    global red_brick_code, red_brick_error_x, red_brick_error_y
    global red_brick_confirm_count, red_brick_last_code
    global red_brick_mid_x, red_brick_bottom_y
    global red_brick_last_mid_x, red_brick_last_bottom_y

    raw_code, rb = calc_red_brick_code(red_blobs, red_classes, push_foreground)
    if raw_code == RED_BRICK_NONE:
        clear_red_brick_state()
        return

    mid_x = (rb[RB_X1] + rb[RB_X2]) // 2
    bottom_y = rb[RB_Y2]
    stable = (
        raw_code == red_brick_last_code
        and abs(mid_x - red_brick_last_mid_x) <= RED_BRICK_STABLE_X
        and abs(bottom_y - red_brick_last_bottom_y) <= RED_BRICK_STABLE_Y
    )
    if stable:
        if red_brick_confirm_count < RED_BRICK_CONFIRM_FRAMES:
            red_brick_confirm_count += 1
    else:
        red_brick_last_code = raw_code
        red_brick_confirm_count = 1
    red_brick_last_mid_x = mid_x
    red_brick_last_bottom_y = bottom_y

    if red_brick_confirm_count >= RED_BRICK_CONFIRM_FRAMES:
        red_brick_code = raw_code
        red_brick_error_x, red_brick_error_y = calc_red_brick_error(rb)
        red_brick_mid_x = mid_x
        red_brick_bottom_y = bottom_y
    else:
        red_brick_code = RED_BRICK_NONE
        red_brick_error_x = 0
        red_brick_error_y = 0
        red_brick_mid_x = 0
        red_brick_bottom_y = 0


def select_push_foreground(red_blobs):
    center_left = WORK_W // 2 - PUSH_BAG_CENTER_HALF_W
    center_right = WORK_W // 2 + PUSH_BAG_CENTER_HALF_W
    best = None
    best_pixels = -1
    for rb in red_blobs:
        bottom_center = (
            rb[RB_Y2] >= WORK_H - 1 - PUSH_BAG_BOTTOM_MARGIN
            and rb[RB_X2] >= center_left
            and rb[RB_X1] <= center_right
            and rb[RB_W] >= PUSH_BAG_MIN_W
            and rb[RB_H] >= PUSH_BAG_MIN_H
            and rb[RB_AREA] >= PUSH_BAG_MIN_AREA
        )
        large_low = (
            rb[RB_Y1] >= PUSH_BAG_LOW_MIN_Y1
            and rb[RB_Y2] >= PUSH_BAG_LOW_MIN_BOTTOM
            and rb[RB_W] >= PUSH_BAG_LOW_MIN_W
            and rb[RB_H] >= PUSH_BAG_LOW_MIN_H
            and rb[RB_AREA] >= PUSH_BAG_LOW_MIN_AREA
        )
        if not bottom_center and not large_low:
            continue
        if best is None or rb[RB_PIXELS] > best_pixels:
            best = rb
            best_pixels = rb[RB_PIXELS]
    return best


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
    best = None
    red_blobs = None
    if use_redbrick_filter:
        red_blobs = collect_red_blobs(img)
    models = collect_model_candidates(img)
    red_classes = None
    if use_redbrick_filter:
        red_classes = classify_red_blobs(red_blobs, models)

    for model in models:
        if yellow_model_filter_enabled and model_is_yellow_interference(img, model):
            continue
        if use_redbrick_filter:
            red_index = find_matching_red_blob_index(model, red_blobs)
            if red_index >= 0 and red_classes[red_index] != V2_REDBAG:
                continue
        if (
            best is None
            or model[M_Y2] > best[M_Y2]
            or (
                model[M_Y2] == best[M_Y2]
                and model[M_SCORE] > best[M_SCORE]
            )
        ):
            best = model

    if use_redbrick_filter:
        update_red_brick_state(red_blobs, red_classes)
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

    if best_label == "redbag" and redbag_completed:
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


detect_mode = "SEARCH"
frozen_error = None
ema_bev_x = None
ema_bev_y = None
coarse_frame_in_interval = 0
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
red_brick_mid_x = 0
red_brick_bottom_y = 0
red_brick_last_mid_x = -1000
red_brick_last_bottom_y = -1000
red_brick_tx_buf = bytearray(5)
redbag_class_latched = False
redbag_completed = False
yellow_model_filter_enabled = True


def set_detect_mode(new_mode, clear_state = True):
    global detect_mode, frozen_error, ema_bev_x, ema_bev_y
    global coarse_frame_in_interval, classify_sent
    global line_confirm_count, line_ball_slant_state
    global whitebear_line_confirm_enabled
    global red_brick_code, red_brick_error_x, red_brick_error_y
    global red_brick_confirm_count, red_brick_last_code
    global yellow_model_filter_enabled
    global redbag_class_latched, redbag_completed

    old_mode = detect_mode
    if old_mode == "PUSH" and new_mode != "PUSH" and redbag_class_latched:
        redbag_completed = True
    detect_mode = new_mode
    if clear_state:
        frozen_error = None
        ema_bev_x = None
        ema_bev_y = None
        coarse_frame_in_interval = 0
        classify_sent = False
        line_confirm_count = 0
        clear_red_brick_state()
    if new_mode == "CLASSIFY":
        redbag_class_latched = False
    if old_mode in ("LINE", "PUSH") and new_mode not in ("LINE", "PUSH"):
        whitebear_line_confirm_enabled = False
    if new_mode in ("SEARCH", "COARSE", "CLASSIFY"):
        line_ball_slant_state = 0
        if new_mode == "CLASSIFY":
            whitebear_line_confirm_enabled = False
    elif new_mode in ("LINE", "PUSH"):
        yellow_model_filter_enabled = False
        if line_ball_slant_state == 1:
            line_ball_slant_state = 2
        elif old_mode not in ("LINE", "PUSH"):
            line_ball_slant_state = 0
    elif old_mode in ("LINE", "PUSH") and line_ball_slant_state == 2:
        line_ball_slant_state = 0


while True:
    green.on()        # 打开红灯
    if uart.any():
        uart_data = uart.read()
        if uart_data:
            uart_rx_buf += uart_data
            if SEARCH_TRIGGER in uart_rx_buf:
                set_detect_mode("SEARCH")
                uart_rx_buf = bytearray()
            elif COARSE_TRIGGER in uart_rx_buf:
                set_detect_mode("COARSE")
                uart_rx_buf = bytearray()
            elif FOLLOW_FINE_TRIGGER in uart_rx_buf:
                set_detect_mode("FOLLOW_FINE")
                uart_rx_buf = bytearray()
            elif FINE_TRIGGER in uart_rx_buf:
                set_detect_mode("FINE")
                uart_rx_buf = bytearray()
            elif CLASSIFY_TRIGGER in uart_rx_buf:
                set_detect_mode("CLASSIFY")
                uart_rx_buf = bytearray()
            elif PUSH_TRIGGER in uart_rx_buf:
                set_detect_mode("PUSH")
                uart_rx_buf = bytearray()
            elif LINE_TRIGGER in uart_rx_buf:
                set_detect_mode("LINE")
                uart_rx_buf = bytearray()
            elif IDLE_TRIGGER in uart_rx_buf:
                set_detect_mode("IDLE")
                uart_rx_buf = bytearray()
            elif len(uart_rx_buf) > 20:
                uart_rx_buf = bytearray()

    img = sensor.snapshot().scale(x_scale = WORK_W / float(FRAME_W),
                                  y_scale = WORK_H / float(FRAME_H))

    if detect_mode == "IDLE":
        if SEND_NEUTRAL_WHEN_EMPTY:
            send_no_target()
        send_red_brick_state()
    elif detect_mode == "CLASSIFY":
        if not classify_sent:
            target = find_nearest_target(img, False)
            if target is not None:
                dir_code, class_label, _ = classify_target(img, target)
                if dir_code is not None:
                    redbag_class_latched = class_label == "redbag"
                    send_classify_dir(dir_code)
                    line_ball_slant_state = 1 if dir_code == CLASSIFY_DIR_UP else 0
                    whitebear_line_confirm_enabled = class_label == "whitebear"
                    classify_sent = True
        send_red_brick_state()
    elif detect_mode == "LINE" or detect_mode == "PUSH":
        allow_ball_slant = line_ball_slant_state == 2
        if whitebear_line_confirm_enabled:
            required_frames = LINE_WHITEBEAR_CONFIRM_FRAMES
        elif allow_ball_slant:
            required_frames = LINE_BALL_CONFIRM_FRAMES
        else:
            required_frames = LINE_CONFIRM_FRAMES
        line_crossed, _ = detect_yellow_line(img, allow_ball_slant)
        if line_crossed:
            if line_confirm_count < required_frames:
                line_confirm_count += 1
        else:
            line_confirm_count = 0
        if line_confirm_count >= required_frames:
            send_line_state(LINE_STATE_CROSSED)
        else:
            send_line_state(LINE_STATE_NONE)
        if detect_mode == "PUSH":
            red_blobs = collect_red_blobs(img)
            red_classes = classify_red_blobs(red_blobs, ())
            push_foreground = None
            if redbag_class_latched and not redbag_completed:
                push_foreground = select_push_foreground(red_blobs)
            update_red_brick_state(red_blobs, red_classes, push_foreground)
        else:
            clear_red_brick_state()
        send_red_brick_state()
    elif detect_mode == "SEARCH" or detect_mode == "COARSE":
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
                x1, y1, x2, y2, _, _ = target
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
                send_error(error_x, error_y)
            else:
                frozen_error = None
                ema_bev_x = None
                ema_bev_y = None
                coarse_frame_in_interval = 0

        if frozen_error is not None:
            if not need_detect:
                send_error(frozen_error[0], frozen_error[1])
        else:
            if SEND_NEUTRAL_WHEN_EMPTY:
                send_no_target()
        send_red_brick_state()
    else:
        use_redbrick_filter = detect_mode == "FOLLOW_FINE"
        target = find_nearest_target(img, use_redbrick_filter)

        if target is None:
            if SEND_NEUTRAL_WHEN_EMPTY:
                send_no_target()
        else:
            x1, y1, x2, y2, _, _ = target
            mid_x = (x1 + x2) // 2
            bottom_y = y2
            bev_x, bev_y = ipm_transform(mid_x, bottom_y)
            bev_x, bev_y = normalize_bev_point(bev_x, bev_y)
            error_x = int(bev_x) - BEV_CENTER_X
            error_y = BEV_TARGET_Y - int(bev_y)
            send_error(error_x, error_y)
        send_red_brick_state()

    frame_count += 1
    if (frame_count & 0x07) == 0:
        gc.collect()
