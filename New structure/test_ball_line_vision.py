import gc
import pyb
import sensor
from pyb import LED


FRAME_W = 320
FRAME_H = 240
WORK_W = 192
WORK_H = 192

YELLOW_LINE_THRESHOLDS = [(60, 100, -128, 2, 22, 127)]
LINE_ROI_Y = WORK_H // 2
LINE_MIN_PIXELS = 77
LINE_MIN_AREA = 77
LINE_SIDE_MIN_WIDTH = int(WORK_W * 0.16)
LINE_SIDE_MIN_FILL_PCT = 35
LINE_BALL_SLANT_MIN_WIDTH = int(WORK_W * 0.12)
LINE_BALL_SLANT_MIN_FILL_PCT = 18
LINE_BALL_SLANT_MIN_ELONGATION_PCT = 68
LINE_BALL_SLANT_MIN_BOTTOM = (WORK_H * 78 + 99) // 100
LINE_BALL_EDGE_MARGIN = 2
LINE_CENTER_MASK_W = int(WORK_W * 0.42)
LINE_BALL_CONFIRM_FRAMES = 2
LOG_PERIOD_MS = 100


def print_diag(prefix, now, side, x, w, h, bottom, fill, elong, mask, count):
    print(prefix, now, side, x, w, h, bottom, fill, elong, mask, count)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True, exposure_us=150)
sensor.skip_frames(time=3000)

green = LED(2)
green.off()
gc.collect()

print(
    "C",
    LINE_BALL_SLANT_MIN_WIDTH,
    LINE_BALL_SLANT_MIN_BOTTOM,
    LINE_BALL_SLANT_MIN_FILL_PCT,
    LINE_BALL_SLANT_MIN_ELONGATION_PCT,
    LINE_BALL_EDGE_MARGIN,
    LINE_BALL_CONFIRM_FRAMES,
)

confirm_count = 0
last_log_ms = pyb.millis()

while True:
    img = sensor.snapshot().scale(
        x_scale=WORK_W / float(FRAME_W),
        y_scale=WORK_H / float(FRAME_H),
    )

    side_w = (img.width() - LINE_CENTER_MASK_W) // 2
    right_x = img.width() - side_w
    roi_h = img.height() - LINE_ROI_Y

    best_score = -1
    best_pixels = -1
    best_side = 2
    best_x = 0
    best_w = 0
    best_h = 0
    best_bottom = 0
    best_fill = 0
    best_elong = 0
    best_mask = 0
    best_accepted = False

    for roi_x in (0, right_x):
        side = 0 if roi_x == 0 else 1
        for blob in img.find_blobs(
            YELLOW_LINE_THRESHOLDS,
            roi=(roi_x, LINE_ROI_Y, side_w, roi_h),
            pixels_threshold=LINE_MIN_PIXELS,
            area_threshold=LINE_MIN_AREA,
            merge=True,
        ):
            x = blob.x()
            w = blob.w()
            h = blob.h()
            bottom = blob.y() + h
            area = w * h
            pixels = blob.pixels()
            fill = pixels * 100 // area if area else 0
            elong = int(blob.elongation() * 100)

            if side == 0:
                edge_ok = x <= LINE_BALL_EDGE_MARGIN
            else:
                edge_ok = (x + w) >= (img.width() - LINE_BALL_EDGE_MARGIN)
            bottom_ok = bottom >= LINE_BALL_SLANT_MIN_BOTTOM
            width_ok = w >= LINE_BALL_SLANT_MIN_WIDTH
            fill_ok = fill >= LINE_BALL_SLANT_MIN_FILL_PCT
            elong_ok = elong >= LINE_BALL_SLANT_MIN_ELONGATION_PCT
            normal_ok = (
                w >= LINE_SIDE_MIN_WIDTH
                and w * 2 >= h * 3
                and fill >= LINE_SIDE_MIN_FILL_PCT
            )
            slant_ok = width_ok and fill_ok and elong_ok
            accepted = edge_ok and bottom_ok and (normal_ok or slant_ok)

            mask = 1
            score = 0
            if edge_ok:
                mask |= 2
                score += 1
            if bottom_ok:
                mask |= 4
                score += 1
            if width_ok:
                mask |= 8
                score += 1
            if fill_ok:
                mask |= 16
                score += 1
            if elong_ok:
                mask |= 32
                score += 1
            if normal_ok:
                mask |= 64
                score += 1
            if accepted:
                mask |= 128
                score += 16

            if score > best_score or (score == best_score and pixels > best_pixels):
                best_score = score
                best_pixels = pixels
                best_side = side
                best_x = x
                best_w = w
                best_h = h
                best_bottom = bottom
                best_fill = fill
                best_elong = elong
                best_mask = mask
                best_accepted = accepted

    if best_accepted:
        if confirm_count < LINE_BALL_CONFIRM_FRAMES:
            confirm_count += 1
    else:
        confirm_count = 0

    now = pyb.millis()
    if confirm_count >= LINE_BALL_CONFIRM_FRAMES:
        print_diag(
            "X",
            now,
            best_side,
            best_x,
            best_w,
            best_h,
            best_bottom,
            best_fill,
            best_elong,
            best_mask,
            confirm_count,
        )
        green.on()
        break

    if now - last_log_ms >= LOG_PERIOD_MS:
        last_log_ms = now
        print_diag(
            "D",
            now,
            best_side,
            best_x,
            best_w,
            best_h,
            best_bottom,
            best_fill,
            best_elong,
            best_mask,
            confirm_count,
        )

while True:
    pyb.delay(1000)
