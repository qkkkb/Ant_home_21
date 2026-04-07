import seekfree, pyb
import sensor, image, time, tf, gc
from machine import UART
#test

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
DRAW_RESULT = True
PRINT_FPS = False
PRINT_TARGET = True

ERROR_OFFSET = 120
ERROR_LIMIT = 120

FRAME_W = 320
FRAME_H = 240
WORK_W  = 240
WORK_H  = 240
COARSE_FREEZE_TIMEOUT = 100   # 冻结后超过此帧数未收到 FINE -> 重置重新检测


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
    [-3.40383827, -2.33693919, 504.43734026],
    [0.15752834, -6.14360543, 563.00630259],
    [0.00065911, -0.01991198, 1.00000000],
]

# BEV 像素 -> 物理距离换算 (mm)
# 标定：594mm x 210mm 纸，_bev_w=169, BEV 240x240
MM_PER_PIX_X = 210.0 / 169.0   # 1.243 mm/px
MM_PER_PIX_Y = 594.0 / 240.0   # 2.475 mm/px
BEV_CENTER_X = IPM_MATRIX_DST_W // 2   # 120
BEV_CENTER_Y = IPM_MATRIX_DST_H // 2   # 120


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
    uart.write(bytearray([send_x, send_y]))
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


print("IPM_ENABLE=%s IS_IMG_TO_BEV=%s DST=%dx%d" %
      (IPM_ENABLE, IPM_MATRIX_IS_IMAGE_TO_BEV, IPM_MATRIX_DST_W, IPM_MATRIX_DST_H))
detect_mode = "COARSE"
frozen_error = None
coarse_sent = False
freeze_count = 0
uart_rx_buf = bytearray()
frame_count = 0
while True:
    clock.tick()
    if uart.any():
        uart_data = uart.read()
        if uart_data:
            uart_rx_buf += uart_data
            if FINE_TRIGGER in uart_rx_buf:
                detect_mode = "FINE"
                uart_rx_buf = bytearray()
            elif len(uart_rx_buf) > 20:
                uart_rx_buf = bytearray()

    img = sensor.snapshot().scale(x_scale = WORK_W / float(FRAME_W),
                                  y_scale = WORK_H / float(FRAME_H))

    if detect_mode == "COARSE":
        if frozen_error is None:
            target = find_nearest_target(img)
            if target is not None:
                x1, y1, x2, y2, label, score = target
                mid_x = (x1 + x2) // 2
                bottom_y = y2
                bev_x, bev_y = ipm_transform(mid_x, bottom_y)
                error_x = int(bev_x) - BEV_CENTER_X
                error_y = int(bev_y) - BEV_CENTER_Y
                frozen_error = (error_x, error_y)
                freeze_count = 0
                dx_mm, dy_mm = bev_to_mm(bev_x, bev_y)
                if PRINT_TARGET:
                    print("COARSE LOCK s=%.2f px=(%d,%d) mm=(%.0f,%.0f)" %
                          (score, mid_x, bottom_y, dx_mm, dy_mm))
        if frozen_error is not None:
            if not coarse_sent:
                send_error(frozen_error[0], frozen_error[1])
                coarse_sent = True
            freeze_count += 1
            if PRINT_TARGET:
                print("COARSE FROZEN #%d err=(%d,%d)" %
                      (freeze_count, frozen_error[0], frozen_error[1]))
            if freeze_count >= COARSE_FREEZE_TIMEOUT:
                if PRINT_TARGET:
                    print("COARSE TIMEOUT reset")
                frozen_error = None
                coarse_sent = False
                freeze_count = 0
        else:
            if SEND_NEUTRAL_WHEN_EMPTY:
                send_error(0, 0)
            if PRINT_TARGET:
                print("COARSE searching")
        # coarse_sent=True 后不再发送，控制器保持上次速度
    else:
        target = find_nearest_target(img)

        if target is None:
            if SEND_NEUTRAL_WHEN_EMPTY:
                send_error(0, 0)

            if PRINT_TARGET:
                print("FINE no target")
            elif PRINT_FPS:
                print(clock.fps())
        else:
            x1, y1, x2, y2, label, score = target
            mid_x = (x1 + x2) // 2
            bottom_y = y2
            bev_x, bev_y = ipm_transform(mid_x, bottom_y)
            error_x = int(bev_x) - BEV_CENTER_X
            error_y = int(bev_y) - BEV_CENTER_Y
            send_x, send_y = send_error(error_x, error_y)
            dx_mm, dy_mm = bev_to_mm(bev_x, bev_y)

            if PRINT_TARGET:
                print("FINE s=%.2f px=(%d,%d) mm=(%.0f,%.0f) send=(%d,%d)" %
                      (score, mid_x, bottom_y, dx_mm, dy_mm, send_x, send_y))
            elif PRINT_FPS:
                print(clock.fps())

    frame_count += 1
    if (frame_count & 0x07) == 0:
        gc.collect()
