import os
import sys

import cv2
import numpy as np


WINDOW_NAME = "IPM Calibration"
BEV_WINDOW_NAME = "BEV Preview"
IMAGE_SIZE = 192
POINT_NAMES = [
    "1 LB",
    "2 RB",
    "3 RT",
    "4 LT",
]
POINT_COLORS = [
    (0, 0, 255),
    (0, 255, 0),
    (255, 0, 0),
    (0, 255, 255),
]
MEASURE_COLOR = (255, 0, 255)

_bev_w = int(IMAGE_SIZE * 210 / 594)
_x0 = (IMAGE_SIZE - _bev_w) // 2
dst_pts = np.array(
    [
        [_x0, IMAGE_SIZE - 1],
        [_x0 + _bev_w - 1, IMAGE_SIZE - 1],
        [_x0 + _bev_w - 1, 0],
        [_x0, 0],
    ],
    dtype=np.float32,
)
MM_PER_PIX_X = 210.0 / _bev_w
MM_PER_PIX_Y = 594.0 / IMAGE_SIZE
BEV_CENTER_X = IMAGE_SIZE // 2
BEV_CENTER_Y = IMAGE_SIZE // 2


def format_matrix(matrix):
    rows = []
    for row in matrix:
        rows.append("    [{:.8f}, {:.8f}, {:.8f}]".format(row[0], row[1], row[2]))
    return "IPM_MATRIX = [\n" + ",\n".join(rows) + "\n]"


def draw_points(canvas, points):
    for idx, point in enumerate(points):
        x, y = point
        color = POINT_COLORS[idx]
        cv2.circle(canvas, (x, y), 5, color, -1)
        cv2.putText(
            canvas,
            POINT_NAMES[idx],
            (x + 8, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

    if len(points) > 1:
        for idx in range(len(points) - 1):
            cv2.line(canvas, points[idx], points[idx + 1], (200, 200, 200), 1, cv2.LINE_AA)


def draw_overlay(canvas, points):
    draw_points(canvas, points)
    tips = [
        "Left click: left-bottom -> right-bottom -> right-top -> left-top",
        "ENTER: compute matrix    R: reset    Q/ESC: quit",
        "After ENTER: click target bottom point to measure BEV_TARGET_Y",
        "Points: {}/4".format(len(points)),
    ]
    for idx, text in enumerate(tips):
        cv2.putText(
            canvas,
            text,
            (8, 20 + idx * 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def draw_bev_overlay(bev):
    draw_points(bev, [(int(x), int(y)) for x, y in dst_pts])
    cv2.putText(
        bev,
        "dst_pts",
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def draw_measure_point(canvas, point, label):
    if point is None:
        return
    x, y = point
    cv2.circle(canvas, (int(round(x)), int(round(y))), 5, MEASURE_COLOR, -1)
    cv2.putText(
        canvas,
        label,
        (int(round(x)) + 8, int(round(y)) - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        MEASURE_COLOR,
        1,
        cv2.LINE_AA,
    )


def load_image():
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
    else:
        image_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "snapshot.jpg")

    img = cv2.imread(image_path)
    if img is None:
        print("Failed to load image:", image_path)
        sys.exit(1)

    img = cv2.resize(img, (IMAGE_SIZE, IMAGE_SIZE))

    return image_path, img


def main():
    image_path, base_img = load_image()
    points = []
    H = None
    measure_src = None
    measure_bev = None
    bev_preview_base = None

    def refresh():
        canvas = base_img.copy()
        draw_overlay(canvas, points)
        draw_measure_point(canvas, measure_src, "M src")
        cv2.imshow(WINDOW_NAME, canvas)

    def refresh_bev():
        if bev_preview_base is None:
            return
        bev_preview = bev_preview_base.copy()
        draw_bev_overlay(bev_preview)
        draw_measure_point(bev_preview, measure_bev, "M bev")
        if measure_bev is not None:
            target_y = int(round(measure_bev[1]))
            cv2.line(
                bev_preview,
                (0, target_y),
                (IMAGE_SIZE - 1, target_y),
                MEASURE_COLOR,
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                bev_preview,
                "BEV_TARGET_Y={}".format(target_y),
                (8, IMAGE_SIZE - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                MEASURE_COLOR,
                1,
                cv2.LINE_AA,
            )
        cv2.imshow(BEV_WINDOW_NAME, bev_preview)

    def on_mouse(event, x, y, flags, param):
        nonlocal H, measure_src, measure_bev, bev_preview_base
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if len(points) >= 4:
            if H is None:
                print("Already collected 4 points. Press ENTER to compute or R to reset.")
                return

            measure_src = (x, y)
            bev_pt = cv2.perspectiveTransform(
                np.array([[[x, y]]], dtype=np.float32), H
            )[0][0]
            measure_bev = (float(bev_pt[0]), float(bev_pt[1]))
            target_y = int(round(measure_bev[1]))
            dx_mm = (measure_bev[0] - BEV_CENTER_X) * MM_PER_PIX_X
            dy_mm = (measure_bev[1] - BEV_CENTER_Y) * MM_PER_PIX_Y
            print(
                "Measure px=({},{}) -> bev=({:.2f},{:.2f})".format(
                    x, y, measure_bev[0], measure_bev[1]
                )
            )
            print("BEV_TARGET_Y = {}".format(target_y))
            print(
                "Offset from BEV center: dx_mm={:.1f}, dy_mm={:.1f}".format(
                    dx_mm, dy_mm
                )
            )
            refresh()
            refresh_bev()
            return

        points.append((x, y))
        H = None
        measure_src = None
        measure_bev = None
        bev_preview_base = None
        print("Point {} {} -> ({}, {})".format(len(points), POINT_NAMES[len(points) - 1], x, y))
        refresh()

    print("Image:", image_path)
    print("Click 4 points in order: left-bottom -> right-bottom -> right-top -> left-top")

    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    refresh()

    while True:
        key = cv2.waitKey(20) & 0xFF

        if key in (27, ord("q"), ord("Q")):
            break

        if key in (ord("r"), ord("R")):
            points[:] = []
            H = None
            measure_src = None
            measure_bev = None
            bev_preview_base = None
            try:
                cv2.destroyWindow(BEV_WINDOW_NAME)
            except cv2.error:
                pass
            print("Reset points.")
            refresh()
            continue

        if key in (13, 10):
            if len(points) != 4:
                print("Need 4 points before computing. Current:", len(points))
                continue

            src = np.array(points, dtype=np.float32)
            H = cv2.getPerspectiveTransform(src, dst_pts)
            print(format_matrix(H))
            print("MM_PER_PIX_X = {:.6f}".format(MM_PER_PIX_X))
            print("MM_PER_PIX_Y = {:.6f}".format(MM_PER_PIX_Y))
            print("Click the target bottom point in the source window to read BEV_TARGET_Y.")

            bev_preview_base = cv2.warpPerspective(base_img, H, (IMAGE_SIZE, IMAGE_SIZE))
            refresh()
            refresh_bev()
            continue

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
