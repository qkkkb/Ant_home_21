import sensor, image, time
import gc
import tf
from machine import UART
import ustruct

# ================= Camera =================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True, exposure_us=150)

# ================= UART =================
uart = UART(2, 9600)
uart.init(9600, timeout_char=1000)

# ================= Model =================
detect_model_path = "detect.tflite"
classify_model_path = "classify.tflite"

net_detect = tf.load(detect_model_path)
net_classify = tf.load(classify_model_path, load_to_fb=True)

labels = [
    "ball",
    "redbag",
    "bluebag",
    "brownbear",
    "whitebear"
]

# ================= Detect ROI =================
DETECT_ROI_X = 40
DETECT_ROI_Y = 0
DETECT_ROI_W = 240
DETECT_ROI_H = 240

DETECT_SCORE_TH = 0.60
CLASSIFY_SCORE_TH = 0.50

# ================= Inverse Perspective =================
# 世界坐标单位建议用 mm。
# 4 个点顺序必须和图像点顺序一致，推荐：
# 左下、右下、右上、左上
#
# IMAGE_CAL_POINTS 需要你自己标定后填入。
# 下面这组只是示例值，第一次使用请务必改成你的相机实际像素点。
ENABLE_IPM = True
IMAGE_CAL_POINTS = [
    [58, 230],   # 左下
    [262, 230],  # 右下
    [214, 92],   # 右上
    [106, 92]    # 左上
]

# 对应地面上的真实坐标，单位 mm。
# 这里按 A4 纸竖放举例：宽 210，高 297。
# 车体中线为 X=0，离车最近的一边为 Y=0。
WORLD_CAL_POINTS = [
    [-105, 0],
    [105, 0],
    [105, 297],
    [-105, 297]
]

# 框底边中点通常更接近物体与地面的接触点。
GROUND_POINT_X_SHIFT = 0
GROUND_POINT_Y_SHIFT = -3

# 如果你发现 Y 方向整体偏大/偏小，可直接加这个平移补偿。
WORLD_X_OFFSET = 0
WORLD_Y_OFFSET = 0

DRAW_IPM_INFO = True
PRINT_IPM_INFO = True

# 保留原框坐标包，新增世界坐标包。
SEND_BBOX_PACKET = True
SEND_WORLD_PACKET = True

_ipm_warned = False
H = None


def mat_mul(a, b):
    rows = len(a)
    cols = len(b[0])
    inner = len(b)
    out = []
    for i in range(rows):
        row = []
        for j in range(cols):
            s = 0.0
            for k in range(inner):
                s += a[i][k] * b[k][j]
            row.append(s)
        out.append(row)
    return out


def solve_linear_system(a, b):
    n = len(a)
    aug = []
    for i in range(n):
        aug.append(a[i][:] + [b[i][0]])

    for col in range(n):
        pivot = col
        max_value = abs(aug[col][col])
        for row in range(col + 1, n):
            if abs(aug[row][col]) > max_value:
                max_value = abs(aug[row][col])
                pivot = row

        if max_value < 1e-6:
            raise ValueError("bad calibration matrix")

        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]

        pivot_value = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] = aug[col][j] / pivot_value

        for row in range(n):
            if row != col:
                factor = aug[row][col]
                for j in range(col, n + 1):
                    aug[row][j] = aug[row][j] - factor * aug[col][j]

    x = []
    for i in range(n):
        x.append([aug[i][n]])
    return x


def cal_mtx(uv, xy):
    a = []
    b = []
    for i in range(4):
        u = uv[i][0]
        v = uv[i][1]
        x = xy[i][0]
        y = xy[i][1]
        a.append([u, v, 1, 0, 0, 0, -x * u, -x * v])
        a.append([0, 0, 0, u, v, 1, -y * u, -y * v])
        b.append([x])
        b.append([y])

    x = solve_linear_system(a, b)
    return [
        [x[0][0], x[1][0], x[2][0]],
        [x[3][0], x[4][0], x[5][0]],
        [x[6][0], x[7][0], 1]
    ]


def init_ipm():
    global H, _ipm_warned

    if not ENABLE_IPM:
        H = None
        return

    if len(IMAGE_CAL_POINTS) != 4 or len(WORLD_CAL_POINTS) != 4:
        if not _ipm_warned:
            print("IPM disabled: calibration points must be 4 pairs.")
            _ipm_warned = True
        H = None
        return

    try:
        H = cal_mtx(IMAGE_CAL_POINTS, WORLD_CAL_POINTS)
        print("IPM matrix ready.")
    except Exception as e:
        H = None
        if not _ipm_warned:
            print("IPM init failed:", e)
            _ipm_warned = True


def image_to_world(u, v):
    if H is None:
        return None, None

    point = [[u], [v], [1]]
    mapped = mat_mul(H, point)
    s = mapped[2][0]
    if abs(s) < 1e-6:
        return None, None

    x = mapped[0][0] / s + WORLD_X_OFFSET
    y = mapped[1][0] / s + WORLD_Y_OFFSET
    return int(x), int(y)


def get_ground_contact_point(x1, y1, x2, y2):
    u = (x1 + x2) // 2 + GROUND_POINT_X_SHIFT
    v = y2 + GROUND_POINT_Y_SHIFT

    if u < 0:
        u = 0
    if u > 319:
        u = 319
    if v < 0:
        v = 0
    if v > 239:
        v = 239
    return u, v


def send_bbox(index, x1, y1, x2, y2):
    data = ustruct.pack("<BBHHHH", 0xAA, index, x1, y1, x2, y2)
    uart.write(data)


def send_world(index, world_x, world_y, img_x, img_y):
    data = ustruct.pack("<BBhhHH", 0xAB, index, world_x, world_y, img_x, img_y)
    uart.write(data)


def detect_object():
    img = sensor.snapshot()
    detect_img = img.copy(roi=(DETECT_ROI_X, DETECT_ROI_Y, DETECT_ROI_W, DETECT_ROI_H))

    for obj in tf.detect(net_detect, detect_img):
        x1, y1, x2, y2, label, score = obj

        if score > DETECT_SCORE_TH:
            x1 = DETECT_ROI_X + int(x1 * DETECT_ROI_W)
            y1 = DETECT_ROI_Y + int(y1 * DETECT_ROI_H)
            x2 = DETECT_ROI_X + int(x2 * DETECT_ROI_W)
            y2 = DETECT_ROI_Y + int(y2 * DETECT_ROI_H)

            if x2 <= x1 or y2 <= y1:
                continue

            w = x2 - x1
            h = y2 - y1
            roi_img = img.copy(roi=(x1, y1, w, h))

            img.draw_rectangle(x1, y1, w, h, color=(0, 255, 0))
            return img, roi_img, x1, y1, x2, y2, score

    return img, None, None, None, None, None, None


def classify_object(img, img_roi, x1, y1, x2, y2, det_score):
    for obj in tf.classify(
        net_classify,
        img_roi,
        min_scale=1,
        scale_mul=0.5,
        x_overlap=0.0,
        y_overlap=0.0
    ):
        sorted_list = sorted(zip(labels, obj.output()), key=lambda x: x[1], reverse=True)

        if sorted_list[0][1] > CLASSIFY_SCORE_TH:
            label = sorted_list[0][0]
            index = labels.index(label)

            contact_u, contact_v = get_ground_contact_point(x1, y1, x2, y2)
            world_x, world_y = image_to_world(contact_u, contact_v)

            img.draw_cross(contact_u, contact_v, color=(255, 0, 0), size=8)

            if DRAW_IPM_INFO:
                img.draw_string(
                    x1,
                    max(0, y1 - 18),
                    "%s %.2f" % (label, det_score),
                    color=(255, 255, 255),
                    scale=1
                )
                if world_x is not None and world_y is not None:
                    img.draw_string(
                        x1,
                        max(0, y1 - 8),
                        "(%d,%d)" % (world_x, world_y),
                        color=(255, 255, 0),
                        scale=1
                    )

            if PRINT_IPM_INFO:
                if world_x is None:
                    print(
                        "label=%s det=%.2f cls=%.2f img=(%d,%d) world=NA" %
                        (label, det_score, sorted_list[0][1], contact_u, contact_v)
                    )
                else:
                    print(
                        "label=%s det=%.2f cls=%.2f img=(%d,%d) world=(%d,%d)" %
                        (label, det_score, sorted_list[0][1], contact_u, contact_v, world_x, world_y)
                    )

            if SEND_BBOX_PACKET:
                send_bbox(index, x1, y1, x2, y2)

            if SEND_WORLD_PACKET and world_x is not None:
                send_world(index, world_x, world_y, contact_u, contact_v)

            return True

        return False

    return False


if __name__ == "__main__":
    init_ipm()
    send_flag = False

    while True:
        gc.collect()

        if uart.any():
            data = uart.read()
            if data == b"\x01":
                send_flag = True
            elif data == b"\x02":
                send_flag = False

        while send_flag:
            img, roi, x1, y1, x2, y2, det_score = detect_object()
            if roi is not None:
                if classify_object(img, roi, x1, y1, x2, y2, det_score):
                    send_flag = False
                    break
