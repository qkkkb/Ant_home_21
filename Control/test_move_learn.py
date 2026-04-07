from machine import Pin

import gc
import utime

from smartcar import ticker, encoder
from seekfree import WIRELESS_UART, MOTOR_CONTROLLER

from imu_runtime import IMUYawRuntime
from models import AnglePID, MoveBase, SpeedPID
from move_base import calc_wheel_spd
import pid as _pid_mod


_pid_mod.PWM_MAX = 10000.0
speed_ctrl = _pid_mod.speed_ctrl
gyro_ctrl = _pid_mod.gyro_ctrl


# ---------------------- Bring-up config ----------------------
TICK_PERIOD_MS = 5
MOTOR_FREQ = 13000
MOTOR_DUTY_MAX = 10000
MOTOR_DUTY_MIN = 300
PWM_SMOOTH_FACTOR = 0.4
MAX_PWM_CHANGE = 800

# Speed-loop-only test by default. Turn the gyro loop on only when you need heading control.
ENABLE_GYRO_LOOP = False
ENABLE_OSCILLOSCOPE = False
SHOW_DEVICE_INFO = False

# Run mode:
# - "wheel_dir_cal": old C style direction bring-up, only calibrate wheel direction at the source
# - "speed_loop": normal closed-loop body command test
RUN_MODE = "speed_loop"
CAL_WHEEL = "fr"   # "fl", "fr", "b"
CAL_PWM = 1200
CAL_SIGN = 1       # +1: test positive wheel direction, -1: test negative wheel direction

# Body motion test:
# - "straight": forward/backward along vx
# - "translate": lateral move along vy
# - "rotate": in-place rotation with vz
BODY_TEST_MODE = "straight"  # "straight", "translate", "rotate"
TEST_LINEAR_SPEED = 3.0
TEST_ROTATE_SPEED = 3.0
GYRO_SIGN = -1.0
# Old C style gyro calibration:
# - current yaw / gyro loop only uses the Z axis
# - manual offset: fill in GYRO_OFFSET_Z after you measure the static bias
# - auto offset: enable AUTO_GYRO_OFFSET_CAL and average 2000 samples at startup
GYRO_OFFSET_Z = 1.0
AUTO_GYRO_OFFSET_CAL = False
GYRO_CALIB_SAMPLES = 2000
GYRO_CALIB_DELAY_MS = 1
GYRO_SCALE = -1.0 / 16.54052
GYRO_DEADBAND_DPS = 0.8
GYRO_KP = 0.4
GYRO_KI = 0.012
GYRO_OUTPUT_LIMIT = 8.0

if BODY_TEST_MODE == "straight":
    STRAIGHT_VX = TEST_LINEAR_SPEED
    STRAIGHT_VY = 0.0
    STRAIGHT_VZ = 0.0
elif BODY_TEST_MODE == "translate":
    STRAIGHT_VX = 0.0
    STRAIGHT_VY = TEST_LINEAR_SPEED
    STRAIGHT_VZ = 0.0
elif BODY_TEST_MODE == "rotate":
    STRAIGHT_VX = 0.0
    STRAIGHT_VY = 0.0
    STRAIGHT_VZ = TEST_ROTATE_SPEED
else:
    raise ValueError("BODY_TEST_MODE must be 'straight', 'translate', or 'rotate'")

ENABLE_IMU = ENABLE_GYRO_LOOP or AUTO_GYRO_OFFSET_CAL or SHOW_DEVICE_INFO

# Official demo style: make invert/capture_div explicit.
# Follow the old C project: normalize encoder direction at the encoder layer,
# then feed the unified wheel speed directly into the speed loop.
ENCODER_CAPTURE_DIV = 1
IMU_CAPTURE_DIV = 1
ENC_FL_INVERT = False
ENC_FR_INVERT = False
ENC_B_INVERT = False

DEBUG_DIV = 20
EXIT_CHECK_DIV = 5
GC_DIV = 50
TIMEOUT_SECOND = 6

EXIT_TRIGGER_CHANNEL = 7
CH7_TOLERANCE = 1.0


# ---------------------- Demo-style hardware init ----------------------
utime.sleep_ms(100)

led = Pin("C4", Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

# Keep the role mapping that matched the current vehicle bring-up:
# FL -> C28/C29, FR -> C30/C31, B -> D4/D5.
motor_fl = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, MOTOR_FREQ, duty=0, invert=False)
motor_fr = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, MOTOR_FREQ, duty=0, invert=False)
motor_b = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, MOTOR_FREQ, duty=0, invert=True)

enc_fl = encoder("D15", "D16", ENC_FL_INVERT)
enc_fr = encoder("D13", "D14", ENC_FR_INVERT)
enc_b = encoder("C2", "C3", ENC_B_INVERT)

wireless = WIRELESS_UART(460800)
imu_runtime = None
if ENABLE_IMU:
    imu_runtime = IMUYawRuntime(
        sign=GYRO_SIGN,
        offset_z=GYRO_OFFSET_Z,
        scale=GYRO_SCALE,
        deadband_dps=GYRO_DEADBAND_DPS,
        tick_period_ms=TICK_PERIOD_MS,
    )

if SHOW_DEVICE_INFO:
    MOTOR_CONTROLLER.help()
    motor_fl.info()
    motor_fr.info()
    motor_b.info()
    if ENABLE_IMU:
        IMUYawRuntime.help()
        imu_runtime.info()


# ---------------------- Helpers ----------------------
def log(msg):
    print(msg)
    wireless.send_str(msg + "\r\n")


def stop_all():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)
    log("[STOP] all motor duty outputs cleared")


def clamp_duty(value):
    value = int(value)
    if value > MOTOR_DUTY_MAX:
        return MOTOR_DUTY_MAX
    if value < -MOTOR_DUTY_MAX:
        return -MOTOR_DUTY_MAX
    return value


def smooth_value(target, last):
    delta = target - last
    if abs(delta) > MAX_PWM_CHANGE:
        target = last + MAX_PWM_CHANGE * (1 if delta > 0 else -1)
    target = int(last * (1.0 - PWM_SMOOTH_FACTOR) + target * PWM_SMOOTH_FACTOR)
    return clamp_duty(target)


def apply_motor_duty(cmd, motor):
    cmd = int(cmd)
    if 0 < abs(cmd) < MOTOR_DUTY_MIN:
        cmd = MOTOR_DUTY_MIN if cmd > 0 else -MOTOR_DUTY_MIN
    motor.duty(cmd)


def set_three_pwm_smooth(u_fl, u_fr, u_b):
    global last_pwm_fl, last_pwm_fr, last_pwm_b

    s_fl = smooth_value(int(u_fl), last_pwm_fl)
    s_fr = smooth_value(int(u_fr), last_pwm_fr)
    s_b = smooth_value(int(u_b), last_pwm_b)

    apply_motor_duty(s_fl, motor_fl)
    apply_motor_duty(s_fr, motor_fr)
    apply_motor_duty(s_b, motor_b)

    last_pwm_fl = s_fl
    last_pwm_fr = s_fr
    last_pwm_b = s_b

    return s_fl, s_fr, s_b


def get_role_motor(role):
    if role == "fl":
        return motor_fl
    if role == "fr":
        return motor_fr
    if role == "b":
        return motor_b
    return None


def get_role_encoder_value(role, e_fl, e_fr, e_b):
    if role == "fl":
        return e_fl
    if role == "fr":
        return e_fr
    if role == "b":
        return e_b
    return 0


def apply_single_wheel_duty(role, duty):
    apply_motor_duty(duty if role == "fl" else 0, motor_fl)
    apply_motor_duty(duty if role == "fr" else 0, motor_fr)
    apply_motor_duty(duty if role == "b" else 0, motor_b)


def analyze_wheel_dir(role, e_fl, e_fr, e_b):
    focus = get_role_encoder_value(role, e_fl, e_fr, e_b)
    other_1 = get_role_encoder_value("fl" if role != "fl" else "fr", e_fl, e_fr, e_b)
    other_2 = get_role_encoder_value("b" if role != "b" else "fr", e_fl, e_fr, e_b)
    other_peak = max(abs(other_1), abs(other_2))

    if abs(focus) <= 1 and other_peak > 1:
        return "focus encoder inactive; role mapping may be wrong"
    if focus < -1:
        return "positive pwm -> negative encoder; flip this encoder invert"
    if focus > 1 and other_peak <= abs(focus):
        return "direction aligned"
    if focus > 1:
        return "direction aligned, but another encoder also changes"
    return "encoder response too small; lift wheel or raise CAL_PWM"


def check_upper_exit():
    wireless.data_analysis()
    ch7_current = wireless.get_data(EXIT_TRIGGER_CHANNEL)
    if abs(ch7_current - ch7_init_value) > CH7_TOLERANCE:
        log("CH7 changed: baseline=%.1f current=%.1f" % (ch7_init_value, ch7_current))
        return True
    return False


# ---------------------- CH7 exit calibration ----------------------
log("=== CH7 calibration start ===")

ch7_samples = []
for _ in range(5):
    wireless.data_analysis()
    utime.sleep_ms(10)
    ch7_samples.append(wireless.get_data(EXIT_TRIGGER_CHANNEL))

ch7_init_value = sum(ch7_samples) / len(ch7_samples)
log("CH7 baseline calibrated: %.1f" % ch7_init_value)
log("Exit when CH7 differs from baseline by more than +/-%.1f" % CH7_TOLERANCE)

gyro_offset_z = GYRO_OFFSET_Z
if RUN_MODE != "wheel_dir_cal":
    if ENABLE_IMU and AUTO_GYRO_OFFSET_CAL:
        gyro_offset_z = imu_runtime.calibrate_offset(
            samples=GYRO_CALIB_SAMPLES,
            delay_ms=GYRO_CALIB_DELAY_MS,
            logger=log,
        )
    elif ENABLE_IMU:
        log("gyro offset preset: axis=z offset_z=%.2f" % gyro_offset_z)
    else:
        log("imu disabled: speed loop only")


# ---------------------- Ticker ----------------------
pit_flag = False
pit_count = 0


def time_pit_handler(_):
    global pit_flag, pit_count
    pit_flag = True
    pit_count += 1


pit1 = ticker(1)
if ENABLE_IMU:
    pit1.capture_list(enc_fl, enc_fr, enc_b, imu_runtime.capture_device())
else:
    pit1.capture_list(enc_fl, enc_fr, enc_b)
pit1.callback(time_pit_handler)
pit1.start(TICK_PERIOD_MS)


# ---------------------- Controller state ----------------------
move_cmd = MoveBase()

pid_fl = SpeedPID()
pid_fr = SpeedPID()
pid_b = SpeedPID()
pid_fl.init_c()
pid_fr.init_c()
pid_b.init_c()

gyro_pid = None
if ENABLE_GYRO_LOOP:
    gyro_pid = AnglePID()
    gyro_pid.output = 0.0
    gyro_pid.err = 0.0
    gyro_pid.err_last = 0.0
    gyro_pid.gyro_kp = GYRO_KP
    gyro_pid.gyro_ki = GYRO_KI
    gyro_pid.gyro_output_limit = GYRO_OUTPUT_LIMIT

last_pwm_fl = 0
last_pwm_fr = 0
last_pwm_b = 0

start_time = utime.ticks_ms()
last_status_ms = start_time
loop_count = 0
last_vz_cmd = 0.0


def calc_speed_closed_loop():
    global last_vz_cmd

    if ENABLE_IMU:
        gyro_z = imu_runtime.read_gyro_z()
        raw_gyro_z = imu_runtime.raw_gyro_z
        yaw_deg = imu_runtime.read_yaw()
    else:
        gyro_z = 0.0
        raw_gyro_z = 0.0
        yaw_deg = 0.0

    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        vz_cmd = gyro_ctrl(gyro_pid, STRAIGHT_VZ - gyro_z)
    else:
        if gyro_pid is not None:
            gyro_pid.output = 0.0
            gyro_pid.err = 0.0
            gyro_pid.err_last = 0.0
        vz_cmd = STRAIGHT_VZ
    last_vz_cmd = vz_cmd

    calc_wheel_spd(move_cmd, STRAIGHT_VX, STRAIGHT_VY, vz_cmd)

    e_fl = enc_fl.get()
    e_fr = enc_fr.get()
    e_b = enc_b.get()

    t_fl = move_cmd.speed_fl
    t_fr = move_cmd.speed_fr
    t_b = move_cmd.speed_b

    u_fl = speed_ctrl(pid_fl, e_fl, t_fl)
    u_fr = speed_ctrl(pid_fr, e_fr, t_fr)
    u_b = speed_ctrl(pid_b, e_b, t_b)

    s_fl, s_fr, s_b = set_three_pwm_smooth(u_fl, u_fr, u_b)

    return {
        "enc_fl": e_fl,
        "enc_fr": e_fr,
        "enc_b": e_b,
        "tar_fl": t_fl,
        "tar_fr": t_fr,
        "tar_b": t_b,
        "out_fl": u_fl,
        "out_fr": u_fr,
        "out_b": u_b,
        "pwm_fl": s_fl,
        "pwm_fr": s_fr,
        "pwm_b": s_b,
        "raw_gyro_z": raw_gyro_z,
        "gyro_z": gyro_z,
        "yaw_deg": yaw_deg,
        "vz_cmd": vz_cmd,
    }


def run_wheel_dir_cal():
    duty = clamp_duty(int(CAL_PWM) * (1 if CAL_SIGN >= 0 else -1))
    apply_single_wheel_duty(CAL_WHEEL, duty)
    e_fl = enc_fl.get()
    e_fr = enc_fr.get()
    e_b = enc_b.get()
    return {
        "wheel": CAL_WHEEL,
        "duty": duty,
        "enc_fl": e_fl,
        "enc_fr": e_fr,
        "enc_b": e_b,
        "focus": get_role_encoder_value(CAL_WHEEL, e_fl, e_fr, e_b),
        "note": analyze_wheel_dir(CAL_WHEEL, e_fl, e_fr, e_b),
    }


if RUN_MODE == "wheel_dir_cal":
    log("=== wheel direction calibration start ===")
    log(
        "tick=%dms mode=%s wheel=%s duty=%d sign=%+d"
        % (TICK_PERIOD_MS, RUN_MODE, CAL_WHEEL, CAL_PWM, CAL_SIGN)
    )
    log(
        "rule: positive pwm should make the selected wheel encoder positive; if not, flip only that encoder invert"
    )
else:
    log("=== speed closed loop bring-up start ===")
    log(
        "tick=%dms speed_loop=on gyro_loop=%s mode=%s target_body=(%.1f, %.1f, %.1f)"
        % (
            TICK_PERIOD_MS,
            "on" if ENABLE_GYRO_LOOP else "off",
            BODY_TEST_MODE,
            STRAIGHT_VX,
            STRAIGHT_VY,
            STRAIGHT_VZ,
        )
    )
log(
    "motor map: fl=C28/C29 invert=False, fr=C30/C31 invert=False, b=D4/D5 invert=True"
)
log(
    "encoder map: fl=D15/D16 inv=%s, fr=D13/D14 inv=%s, b=C2/C3 inv=%s, capture_div=%d"
    % (ENC_FL_INVERT, ENC_FR_INVERT, ENC_B_INVERT, ENCODER_CAPTURE_DIV)
)
log(
    "encoder dir handled at source (C-style): fl=%s fr=%s b=%s"
    % ("normal", "normal", "invert" if ENC_B_INVERT else "normal")
)
if ENABLE_IMU:
    log(
        "imu: official_demo_style yaw_axis=z offset_z=%.2f scale=%.8f deadband=%.2f"
        % (gyro_offset_z, GYRO_SCALE, GYRO_DEADBAND_DPS)
    )
    log(
        "gyro loop cfg: sign=%.1f kp=%.3f ki=%.3f limit=%.1f"
        % (GYRO_SIGN, GYRO_KP, GYRO_KI, GYRO_OUTPUT_LIMIT)
    )


try:
    while True:
        loop_count += 1
        now = utime.ticks_ms()
        elapsed_s = utime.ticks_diff(now, start_time) / 1000.0

        if pit_flag:
            pit_flag = False
            if RUN_MODE == "wheel_dir_cal":
                snap = run_wheel_dir_cal()
            else:
                snap = calc_speed_closed_loop()

            if pit_count % DEBUG_DIV == 0 and RUN_MODE == "wheel_dir_cal":
                log(
                    "[WHEEL CAL] t=%.2fs wheel=%s duty=%d enc=(%d,%d,%d) focus=%d note=%s"
                    % (
                        elapsed_s,
                        snap["wheel"],
                        int(snap["duty"]),
                        int(snap["enc_fl"]),
                        int(snap["enc_fr"]),
                        int(snap["enc_b"]),
                        int(snap["focus"]),
                        snap["note"],
                    )
                )
            elif pit_count % DEBUG_DIV == 0:
                if ENABLE_IMU:
                    log(
                        "[SPEED LOOP] t=%.2fs enc=(%d,%d,%d) tar=(%.1f,%.1f,%.1f) out=(%.1f,%.1f,%.1f) pwm=(%d,%d,%d) raw_gz=%.1f gyro=%.2f yaw=%.2f vz=%.2f"
                        % (
                            elapsed_s,
                            int(snap["enc_fl"]),
                            int(snap["enc_fr"]),
                            int(snap["enc_b"]),
                            snap["tar_fl"],
                            snap["tar_fr"],
                            snap["tar_b"],
                            snap["out_fl"],
                            snap["out_fr"],
                            snap["out_b"],
                            int(snap["pwm_fl"]),
                            int(snap["pwm_fr"]),
                            int(snap["pwm_b"]),
                            snap["raw_gyro_z"],
                            snap["gyro_z"],
                            snap["yaw_deg"],
                            snap["vz_cmd"],
                        )
                    )
                else:
                    log(
                        "[SPEED LOOP] t=%.2fs mode=%s enc=(%d,%d,%d) tar=(%.1f,%.1f,%.1f) out=(%.1f,%.1f,%.1f) pwm=(%d,%d,%d)"
                        % (
                            elapsed_s,
                            BODY_TEST_MODE,
                            int(snap["enc_fl"]),
                            int(snap["enc_fr"]),
                            int(snap["enc_b"]),
                            snap["tar_fl"],
                            snap["tar_fr"],
                            snap["tar_b"],
                            snap["out_fl"],
                            snap["out_fr"],
                            snap["out_b"],
                            int(snap["pwm_fl"]),
                            int(snap["pwm_fr"]),
                            int(snap["pwm_b"]),
                        )
                    )
                if ENABLE_OSCILLOSCOPE:
                    wireless.send_oscilloscope(
                        int(snap["enc_fl"]),
                        int(snap["enc_fr"]),
                        int(snap["enc_b"]),
                        int(snap["tar_fl"]),
                        int(snap["tar_fr"]),
                        int(snap["tar_b"]),
                        int(snap["out_fl"]),
                        int(snap["out_fr"]),
                    )

        if utime.ticks_diff(now, last_status_ms) >= 1000:
            led.toggle()
            last_status_ms = now
            if RUN_MODE == "wheel_dir_cal":
                log(
                    "[STATUS] t=%.1fs ch7=%.1f wheel=%s duty=%d"
                    % (elapsed_s, ch7_init_value, CAL_WHEEL, int(CAL_PWM) * (1 if CAL_SIGN >= 0 else -1))
                )
            else:
                if ENABLE_IMU:
                    log(
                        "[STATUS] t=%.1fs ch7=%.1f raw_gz=%.1f gyro=%.2f yaw=%.2f vz=%.2f"
                        % (
                            elapsed_s,
                            ch7_init_value,
                            imu_runtime.raw_gyro_z,
                            imu_runtime.gyro_z_deg,
                            imu_runtime.yaw_deg,
                            last_vz_cmd,
                        )
                    )
                else:
                    log(
                        "[STATUS] t=%.1fs ch7=%.1f mode=%s target_body=(%.1f, %.1f, %.1f)"
                        % (
                            elapsed_s,
                            ch7_init_value,
                            BODY_TEST_MODE,
                            STRAIGHT_VX,
                            STRAIGHT_VY,
                            STRAIGHT_VZ,
                        )
                    )

        if loop_count % EXIT_CHECK_DIV == 0 and check_upper_exit():
            log("=== CH7 exit triggered, program stopped ===")
            break

        if elapsed_s >= TIMEOUT_SECOND:
            log("=== timeout reached, program stopped ===")
            break

        if loop_count % GC_DIV == 0:
            gc.collect()

        utime.sleep_ms(1)

finally:
    pit1.stop()
    stop_all()
    led.value(True)
    log("=== program fully stopped ===")
