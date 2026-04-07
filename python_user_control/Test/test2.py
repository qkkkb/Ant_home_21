from machine import *
from smartcar import ticker, encoder
from seekfree import WIRELESS_UART, MOTOR_CONTROLLER

from models import MoveBase
from move_base import calc_wheel_spd

import gc
import time

# ---------------------- Config ----------------------
TEST_MODE = "straight_vy"  # "straight_vy", "straight_vx", "rotate", "custom", "motor_1", "motor_2", "motor_3"
PWM_MODE = "raw"           # "raw": direct duty value, "percent": duty percentage
TEST_DUTY = 1000            # raw mode magnitude, range about 0~10000
TEST_DUTY_PERCENT = 10.0    # percent mode, 0~100
TEST_DIR = 1                # 1 forward/ccw, -1 backward/cw
TEST_SECONDS = 20
LOG_PERIOD_MS = 100         # keep the same rhythm as official motor demo

EXIT_TRIGGER_CHANNEL = 7
CH7_TOLERANCE = 1.0
PWM_MAX_VALUE = 10000

# Role mapping: official motor order mapped to wheel roles
MOTOR_1_ROLE = "fr"        # C30/C31
MOTOR_2_ROLE = "fl"        # C28/C29
MOTOR_3_ROLE = "b"         # D4/D5

# Per-motor command sign after move_base role mapping.
# If "straight" becomes in-place rotation after changing invert, adjust here first.
MOTOR_1_SIGN = 1.0
MOTOR_2_SIGN = 1.0
MOTOR_3_SIGN = 1.0

# Per-motor open-loop compensation. If one motor is visibly slow, raise only its gain.
MOTOR_1_GAIN = 1.00
MOTOR_2_GAIN = 1.00
MOTOR_3_GAIN = 1.00

# Custom body command, only used when TEST_MODE == "custom"
CUSTOM_VX = 0.0
CUSTOM_VY = 0.0
CUSTOM_VZ = 0.0

# ---------------------- Hardware ----------------------
time.sleep_ms(100)

led = Pin('C4', Pin.OUT, value=True)

# Exact motor initialization style from V3.0.x motor demo
motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty=0, invert=False)  ##enc3 +
motor_2 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty=0, invert=False)  ##enc1 +
motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty=0, invert=False)    ##enc2 -

encoder_1 = encoder("D15", "D16")
encoder_2 = encoder("C2", "C3")
encoder_3 = encoder("D13", "D14")

wireless = WIRELESS_UART(460800)
move_cmd = MoveBase()

MOTOR_CONTROLLER.help()
motor_1.info()
motor_2.info()
motor_3.info()

def encoder_capture_cb(_):
    pass

pit1 = ticker(1)
pit1.capture_list(encoder_1, encoder_2, encoder_3)
pit1.callback(encoder_capture_cb)
pit1.start(10)

# ---------------------- Exit calibration ----------------------
print("=== CH7 calibration start ===")
wireless.send_str("=== CH7 calibration start ===\r\n")

ch7_calibrate_list = []
for _ in range(5):
    wireless.data_analysis()
    time.sleep_ms(10)
    ch7_calibrate_list.append(wireless.get_data(EXIT_TRIGGER_CHANNEL))

ch7_init_value = sum(ch7_calibrate_list) / len(ch7_calibrate_list)
calibrate_str = "CH7 baseline calibrated: %.1f" % ch7_init_value
print(calibrate_str)
wireless.send_str(calibrate_str + "\r\n")

tips_str = "Exit when CH7 differs from baseline by more than +/-%.1f" % CH7_TOLERANCE
print(tips_str)
wireless.send_str(tips_str + "\r\n")

# ---------------------- Helpers ----------------------
def stop_all():
    motor_1.duty(0)
    motor_2.duty(0)
    motor_3.duty(0)
    print("[STOP] all motor duty outputs cleared")
    wireless.send_str("[STOP] all motor duty outputs cleared\r\n")


def check_upper_exit():
    wireless.data_analysis()
    ch7_current = wireless.get_data(EXIT_TRIGGER_CHANNEL)
    if abs(ch7_current - ch7_init_value) > CH7_TOLERANCE:
        exit_str = "CH7 changed: baseline=%.1f current=%.1f" % (ch7_init_value, ch7_current)
        print(exit_str)
        wireless.send_str(exit_str + "\r\n")
        return True
    return False


def duty_value_to_percent(value):
    return float(abs(value)) / PWM_MAX_VALUE * 100.0


def percent_to_value(percent):
    value = int(float(percent) / 100.0 * PWM_MAX_VALUE)
    if value < 0:
        value = 0
    elif value > PWM_MAX_VALUE:
        value = PWM_MAX_VALUE
    return value


def get_test_magnitude():
    if PWM_MODE == "percent":
        return percent_to_value(TEST_DUTY_PERCENT)
    value = int(TEST_DUTY)
    if value < 0:
        value = 0
    elif value > PWM_MAX_VALUE:
        value = PWM_MAX_VALUE
    return value


def get_body_command():
    mag = float(get_test_magnitude()) * (1 if TEST_DIR >= 0 else -1)
    if TEST_MODE == "straight_vy":
        return 0.0, mag, 0.0
    if TEST_MODE == "straight_vx":
        return mag, 0.0, 0.0
    if TEST_MODE == "rotate":
        return 0.0, 0.0, mag
    return float(CUSTOM_VX), float(CUSTOM_VY), float(CUSTOM_VZ)


def get_role_speed(role):
    if role == "fr":
        return move_cmd.speed_fr
    if role == "fl":
        return move_cmd.speed_fl
    if role == "b":
        return move_cmd.speed_b
    return 0.0


def clamp_duty(value):
    value = int(value)
    if value > PWM_MAX_VALUE:
        return PWM_MAX_VALUE
    if value < -PWM_MAX_VALUE:
        return -PWM_MAX_VALUE
    return value


def get_role_gain(role):
    if role == MOTOR_1_ROLE:
        return MOTOR_1_GAIN
    if role == MOTOR_2_ROLE:
        return MOTOR_2_GAIN
    if role == MOTOR_3_ROLE:
        return MOTOR_3_GAIN
    return 1.0


def calc_motor_targets():
    if TEST_MODE == "motor_1":
        mag = int(get_test_magnitude()) * (1 if TEST_DIR >= 0 else -1)
        return 0.0, 0.0, 0.0, mag, 0.0, 0.0, clamp_duty(mag), 0, 0
    if TEST_MODE == "motor_2":
        mag = int(get_test_magnitude()) * (1 if TEST_DIR >= 0 else -1)
        return 0.0, 0.0, 0.0, 0.0, mag, 0.0, 0, clamp_duty(mag), 0
    if TEST_MODE == "motor_3":
        mag = int(get_test_magnitude()) * (1 if TEST_DIR >= 0 else -1)
        return 0.0, 0.0, 0.0, 0.0, 0.0, mag, 0, 0, clamp_duty(mag)

    vx, vy, vz = get_body_command()
    calc_wheel_spd(move_cmd, vx, vy, vz)
    raw_1 = get_role_speed(MOTOR_1_ROLE) * MOTOR_1_SIGN
    raw_2 = get_role_speed(MOTOR_2_ROLE) * MOTOR_2_SIGN
    raw_3 = get_role_speed(MOTOR_3_ROLE) * MOTOR_3_SIGN
    d1 = clamp_duty(raw_1 * MOTOR_1_GAIN)
    d2 = clamp_duty(raw_2 * MOTOR_2_GAIN)
    d3 = clamp_duty(raw_3 * MOTOR_3_GAIN)
    return vx, vy, vz, raw_1, raw_2, raw_3, d1, d2, d3


def send_debug(elapsed_s, vx, vy, vz, raw_1, raw_2, raw_3, duty_1, duty_2, duty_3):
    e1 = encoder_1.get()
    e2 = encoder_2.get()
    e3 = encoder_3.get()
    debug_str = (
        "[MOVE TEST] t=%.1fs mode=%s body=(%.0f,%.0f,%.0f) raw=(%.0f,%.0f,%.0f) duty=(%d,%d,%d) enc=(e1:%d,e2:%d,e3:%d)"
        % (
            elapsed_s, TEST_MODE, vx, vy, vz, raw_1, raw_2, raw_3, duty_1, duty_2, duty_3, int(e1), int(e2), int(e3)
        )
    )
    print(debug_str)
    wireless.send_str(debug_str + "\r\n")


# ---------------------- Main loop ----------------------
run_str = "=== move_base test start (straight / rotate, CH7 exit, 20s timeout) ==="
print(run_str)
wireless.send_str(run_str + "\r\n")

if PWM_MODE == "percent":
    mode_str = "mode=%s target=%.2f%%" % (TEST_MODE, TEST_DUTY_PERCENT)
else:
    mode_str = "mode=%s target=%d" % (TEST_MODE, get_test_magnitude())
print(mode_str)
wireless.send_str(mode_str + "\r\n")

map_str = "roles=(m1:%s,m2:%s,m3:%s) signs=(%.0f,%.0f,%.0f)" % (
    MOTOR_1_ROLE, MOTOR_2_ROLE, MOTOR_3_ROLE, MOTOR_1_SIGN, MOTOR_2_SIGN, MOTOR_3_SIGN
)
print(map_str)
wireless.send_str(map_str + "\r\n")

enc_str = "encoders=(e1:C0/C1,e2:C2/C3,e3:D13/D14) use mode=motor_1/motor_2/motor_3 to check motor-encoder mapping"
print(enc_str)
wireless.send_str(enc_str + "\r\n")

start_ms = time.ticks_ms()
while True:
    time.sleep_ms(LOG_PERIOD_MS)
    elapsed_s = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0

    vx, vy, vz, raw_1, raw_2, raw_3, duty_1, duty_2, duty_3 = calc_motor_targets()
    motor_1.duty(duty_1)
    motor_2.duty(duty_2)
    motor_3.duty(duty_3)

    led.value(duty_3 < 0)
    send_debug(elapsed_s, vx, vy, vz, raw_1, raw_2, raw_3, duty_1, duty_2, duty_3)

    if check_upper_exit():
        stop_all()
        wireless.send_str("=== CH7 exit triggered, program stopped ===\r\n")
        print("=== CH7 exit triggered, program stopped ===")
        break

    if elapsed_s >= TEST_SECONDS:
        stop_all()
        wireless.send_str("=== timeout reached, program stopped ===\r\n")
        print("=== timeout reached, program stopped ===")
        break

    gc.collect()

pit1.stop()
led.value(True)
wireless.send_str("=== program fully stopped ===\r\n")
print("=== program fully stopped ===")
