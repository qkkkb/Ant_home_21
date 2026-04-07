import sensor
import image
import time
from machine import UART

# OpenMV inverse-perspective calibration helper
# Usage:
# 1. Fix the camera position.
# 2. Put 4 markers on the ground.
# 3. Move each marker onto the center cross in order:
#    left-bottom, right-bottom, right-top, left-top
# 4. Send b'1' by UART to save current center point.
# 5. Send b'2' to clear all saved points.
# 6. Read the printed IMAGE_CAL_POINTS and copy into your main script.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(2, 9600)
uart.init(9600, timeout_char=1000)

clock = time.clock()

CENTER_X = 160
CENTER_Y = 120
CROSS_SIZE = 10

POINT_NAMES = [
    "left-bottom",
    "right-bottom",
    "right-top",
    "left-top"
]

saved_points = []
last_cmd = None


def draw_ui(img):
    img.draw_cross(CENTER_X, CENTER_Y, color=(255, 0, 0), size=CROSS_SIZE, thickness=2)
    img.draw_circle(CENTER_X, CENTER_Y, 4, color=(255, 255, 0), thickness=1)
    img.draw_string(4, 4, "Aim marker at center", color=(255, 255, 255), scale=1)

    if len(saved_points) < 4:
        msg = "Next: %s" % POINT_NAMES[len(saved_points)]
    else:
        msg = "Done: send 2 to clear"
    img.draw_string(4, 20, msg, color=(0, 255, 0), scale=1)

    img.draw_string(4, 36, "Center=(%d,%d)" % (CENTER_X, CENTER_Y), color=(0, 255, 255), scale=1)

    y = 54
    for i, pt in enumerate(saved_points):
        img.draw_string(4, y, "%d:%s %s" % (i + 1, POINT_NAMES[i], pt), color=(255, 255, 0), scale=1)
        y += 16


def save_current_point():
    if len(saved_points) >= 4:
        print("Already saved 4 points. Send b'2' to clear.")
        return

    saved_points.append([CENTER_X, CENTER_Y])
    print("Saved %s -> [%d, %d]" % (POINT_NAMES[len(saved_points) - 1], CENTER_X, CENTER_Y))

    if len(saved_points) == 4:
        print("IMAGE_CAL_POINTS = [")
        for pt in saved_points:
            print("    [%d, %d]," % (pt[0], pt[1]))
        print("]")


def clear_points():
    saved_points[:] = []
    print("Cleared all saved points.")


while True:
    clock.tick()
    img = sensor.snapshot()
    draw_ui(img)

    if uart.any():
        data = uart.read()
        if data:
            last_cmd = data

    if last_cmd == b"1":
        save_current_point()
        last_cmd = None
    elif last_cmd == b"2":
        clear_points()
        last_cmd = None

