import os
import sys

import cv2
import numpy as np


WINDOW_NAME = "IPM Calibration"
BEV_WINDOW_NAME = "BEV Preview"
IMAGE_SIZE = 240
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

_bev_w = int(240 * 210 / 297)
_x0 = (240 - _bev_w) // 2
dst_pts = np.array(
    [[_x0, 239], [_x0 + _bev_w - 1, 239], [_x0 + _bev_w - 1, 0], [_x0, 0]],
    dtype=np.float32,
)


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
    bev_preview = None

    def refresh():
        canvas = base_img.copy()
        draw_overlay(canvas, points)
        cv2.imshow(WINDOW_NAME, canvas)

    def on_mouse(event, x, y, flags, param):
        nonlocal bev_preview
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if len(points) >= 4:
            print("Already collected 4 points. Press R to reset or ENTER to compute.")
            return

        points.append((x, y))
        bev_preview = None
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
            bev_preview = None
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

            bev_preview = cv2.warpPerspective(base_img, H, (IMAGE_SIZE, IMAGE_SIZE))
            draw_bev_overlay(bev_preview)
            cv2.imshow(BEV_WINDOW_NAME, bev_preview)
            continue

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
