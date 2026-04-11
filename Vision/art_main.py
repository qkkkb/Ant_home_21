import seekfree, pyb
import sensor, image, time, tf, gc
from pyb import LED #导入LED
from machine import UART
#test
red = LED(1)    # 定义一个LED1   红灯
green = LED(2)
blue = LED(3)  
white = LED(4)   
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565.
sensor.set_framesize(sensor.QVGA)      # 320x240, scaled to 240x240 in code
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_exposure(True, exposure_us = 150)
clock = time.clock()                # Create a clock object to track the FPS.


# ================= Model =================
detect_model = "/sd/detect.tflite"
net = tf.load(detect_model)


# ================= UART =================
uart = UART(12, 9600)
uart.init(9600, timeout_char = 1000)
FINE_TRIGGER = b"FINE\n"


# ================= Runtime =================
DETECT_SCORE_TH = 0.70
SEND_NEUTRAL_WHEN_EMPTY = True
DEBUG_DRAW_BOX = True
PRINT_FPS = False
PRINT_EVENT = True
PRINT_VERBOSE = False

ERROR_OFFSET = 120
ERROR_LIMIT = 120
UART_FRAME_HEAD = 0xAA
FINE_ENTER_ERR_Y = 100
FINE_FORCE_FRAMES = 23  #强制进入 FINE 模式的帧数上限
STOP_ALIGN_ERR_Y = 1
STOP_ALIGN_ERR_X = 6

FRAME_W = 320
FRAME_H = 240
WORK_W  = 240
WORK_H  = 240
COARSE_FREEZE_TIMEOUT = 20   # 冻结后超过此帧数未收到 FINE -> 重置重新检测


# ================= Inverse Perspective =================
# Replace this matrix with your calibration result.
# Default assumption: image coordinates -> bird-view coordinates.
IPM_ENABLE = True
IPM_MATRIX_IS_IMAGE_TO_BEV = True

# If calibration was done on another image size, update these values too.
IPM_MATRIX_DST_W = WORK_W
IPM_MATRIX_DST_H = WORK_H

# TODO: Replace with calibration output from ipm_calibration_pc.py
IPM_MATRIX = [
    [-1.55542265, -2.18141946, 293.44258373],
    [0.00000000, -5.52711324, 514.02153110],
    [0.00000000, -0.01820840, 1.00000000],
]

# BEV 像素 -> 物理距离换算 (mm)
# 标定：594mm x 210mm 纸，_bev_w=84, BEV 240x240
MM_PER_PIX_X = 210.0 / 84.0    # 2.500 mm/px
MM_PER_PIX_Y = 594.0 / 240.0   # 2.475 mm/px
BEV_CENTER_X = IPM_MATRIX_DST_W // 2   # 120
BEV_CENTER_Y = IPM_MATRIX_DST_H // 2   # 120
BEV_TARGET_Y = IPM_MATRIX_DST_H - 1    # 239, measured target bottom position
BEV_FLIP_X = True
BEV_FLIP_Y = False


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


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
    return clamp(value + ERROR_OFFSET, 0, ERROR_OFFSET * 2)


def send_error(error_x, error_y):
    send_x = encode_error(error_x)
    send_y = encode_error(error_y)
    uart.write(bytearray([UART_FRAME_HEAD, send_x, send_y]))
    return send_x, send_y


def find_nearest_target(img):
    img_w = img.width()
    img_h = img.height()
    best = None

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

        if (best is None) or (y2 > best[3]) or ((y2 == best[3]) and (score > best[5])):
            best = (x1, y1, x2, y2, label, score)

    return best


def draw_target_overlay(img, x1, y1, x2, y2):
    img.draw_rectangle(x1, y1, x2 - x1, y2 - y1, color = (0, 255, 0), thickness = 2)


def print_state_cfg():
    if not (PRINT_EVENT or PRINT_VERBOSE):
        return
    print(
        "STATE CFG fine_y<=%d force_fine=%d freeze=%d stop=(y<=%d,|x|<=%d) bev_target_y=%d mm_per_px=(%.3f,%.3f)" %
        (
            FINE_ENTER_ERR_Y,
            FINE_FORCE_FRAMES,
            COARSE_FREEZE_TIMEOUT,
            STOP_ALIGN_ERR_Y,
            STOP_ALIGN_ERR_X,
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
detect_mode = "COARSE"
frozen_error = None
frozen_overlay = None
freeze_count = 0
coarse_frame_count = 0
uart_rx_buf = bytearray()
frame_count = 0
while True:
    clock.tick()
    green.on()        # 打开红灯
    if uart.any():
        uart_data = uart.read()
        if uart_data:
            uart_rx_buf += uart_data
            if FINE_TRIGGER in uart_rx_buf:
                detect_mode = "FINE"
                frozen_error = None
                if DEBUG_DRAW_BOX:
                    frozen_overlay = None
                freeze_count = 0
                coarse_frame_count = 0
                print_state_log("UART", "TRIGGER->FINE", coarse_frame_count, freeze_count)
                uart_rx_buf = bytearray()
            elif len(uart_rx_buf) > 20:
                uart_rx_buf = bytearray()

    img = sensor.snapshot().scale(x_scale = WORK_W / float(FRAME_W),
                                  y_scale = WORK_H / float(FRAME_H))

    if detect_mode == "DONE":
        if SEND_NEUTRAL_WHEN_EMPTY:
            send_error(0, 0)
    elif detect_mode == "COARSE":
        coarse_entered_fine = False
        coarse_frame_count += 1
        if (frozen_error is None) or (freeze_count >= COARSE_FREEZE_TIMEOUT):
            target = find_nearest_target(img)
            freeze_count = 0
            if target is not None:
                x1, y1, x2, y2, label, score = target
                mid_x = (x1 + x2) // 2
                bottom_y = y2
                bev_x, bev_y = ipm_transform(mid_x, bottom_y)
                bev_x, bev_y = normalize_bev_point(bev_x, bev_y)
                error_x = int(bev_x) - BEV_CENTER_X
                error_y = BEV_TARGET_Y - int(bev_y)
                dx_mm, dy_mm = bev_to_mm(bev_x, bev_y)
                if (error_y <= FINE_ENTER_ERR_Y) or (coarse_frame_count >= FINE_FORCE_FRAMES):
                    detect_mode = "FINE"
                    frozen_error = None
                    coarse_frame_count = 0
                    if DEBUG_DRAW_BOX:
                        frozen_overlay = None
                        draw_target_overlay(img, x1, y1, x2, y2)
                    coarse_entered_fine = True
                    send_x, send_y = send_error(error_x, error_y)
                    if error_y <= FINE_ENTER_ERR_Y:
                        print_state_log(
                            "COARSE", "TO_FINE(Y)", coarse_frame_count, freeze_count,
                            score, label, mid_x, bottom_y, bev_x, bev_y,
                            dx_mm, dy_mm, error_x, error_y, send_x, send_y
                        )
                    else:
                        print_state_log(
                            "COARSE", "TO_FINE(%dF)" % FINE_FORCE_FRAMES, coarse_frame_count, freeze_count,
                            score, label, mid_x, bottom_y, bev_x, bev_y,
                            dx_mm, dy_mm, error_x, error_y, send_x, send_y
                        )
                else:
                    frozen_error = (error_x, error_y)
                    if DEBUG_DRAW_BOX:
                        frozen_overlay = (x1, y1, x2, y2)
                    print_state_log(
                        "COARSE", "LOCK", coarse_frame_count, freeze_count,
                        score, label, mid_x, bottom_y, bev_x, bev_y,
                        dx_mm, dy_mm, error_x, error_y,
                        verbose = True
                    )
            else:
                frozen_error = None
                if DEBUG_DRAW_BOX:
                    frozen_overlay = None
        if coarse_entered_fine:
            pass
        elif (frozen_error is not None) and (coarse_frame_count >= FINE_FORCE_FRAMES):
            detect_mode = "FINE"
            coarse_frame_count = 0
            if DEBUG_DRAW_BOX and frozen_overlay is not None:
                draw_target_overlay(img, frozen_overlay[0], frozen_overlay[1], frozen_overlay[2], frozen_overlay[3])
            send_x, send_y = send_error(frozen_error[0], frozen_error[1])
            print_state_log(
                "COARSE", "TO_FINE(%dF)_HOLD" % FINE_FORCE_FRAMES, coarse_frame_count, freeze_count,
                error_x = frozen_error[0], error_y = frozen_error[1],
                send_x = send_x, send_y = send_y
            )
        elif frozen_error is not None:
            if DEBUG_DRAW_BOX and frozen_overlay is not None:
                draw_target_overlay(img, frozen_overlay[0], frozen_overlay[1], frozen_overlay[2], frozen_overlay[3])
            send_x, send_y = send_error(frozen_error[0], frozen_error[1])
            freeze_count += 1
            print_state_log(
                "COARSE", "HOLD", coarse_frame_count, freeze_count,
                error_x = frozen_error[0], error_y = frozen_error[1],
                send_x = send_x, send_y = send_y,
                verbose = True
            )
        else:
            if SEND_NEUTRAL_WHEN_EMPTY:
                send_error(0, 0)
            if coarse_frame_count >= FINE_FORCE_FRAMES:
                detect_mode = "FINE"
                coarse_frame_count = 0
                print_state_log("COARSE", "TO_FINE(%dF)_NO_TARGET" % FINE_FORCE_FRAMES, coarse_frame_count, freeze_count)
            else:
                print_state_log("COARSE", "SEARCH", coarse_frame_count, freeze_count, verbose = True)
        # COARSE 持续发送冻结误差，到期后重新检测并更新
    else:
        target = find_nearest_target(img)

        if target is None:
            if SEND_NEUTRAL_WHEN_EMPTY:
                send_error(0, 0)

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
            if (error_y <= STOP_ALIGN_ERR_Y) and (-STOP_ALIGN_ERR_X <= error_x <= STOP_ALIGN_ERR_X):
                detect_mode = "DONE"
                frozen_error = None
                if DEBUG_DRAW_BOX:
                    frozen_overlay = None
                send_x, send_y = send_error(0, 0)
                print_state_log(
                    "FINE", "DONE", coarse_frame_count, freeze_count,
                    score, label, mid_x, bottom_y, bev_x, bev_y,
                    dx_mm, dy_mm, error_x, error_y, send_x, send_y
                )
            else:
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
