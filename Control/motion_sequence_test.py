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


# ---------------------- Motion config ----------------------
TICK_PERIOD_MS = 5
MOTOR_FREQ = 13000
MOTOR_DUTY_MAX = 10000
MOTOR_DUTY_MIN = 300
PWM_SMOOTH_FACTOR = 0.4
MAX_PWM_CHANGE = 800

ENABLE_GYRO_LOOP = True

GYRO_SIGN = -1.0
GYRO_OFFSET_Z = 1.0
GYRO_SCALE = -1.0 / 16.54052
GYRO_DEADBAND_DPS = 0.8
GYRO_KP = 0.6
GYRO_KI = 0.0125
GYRO_OUTPUT_LIMIT = 8.18

FORWARD_VX = 3.0
BACKWARD_VX = -3.0
RIGHT_VY = 3.0
LEFT_VY = -3.0

FORWARD_MS = 4000
RIGHT_MS = 4000
BACKWARD_MS = 4000
LEFT_MS = 4000
TOTAL_MS = FORWARD_MS + RIGHT_MS + BACKWARD_MS + LEFT_MS

DEBUG_DIV = 20
EXIT_CHECK_DIV = 5
GC_DIV = 50

EXIT_TRIGGER_CHANNEL = 7
CH7_TOLERANCE = 1.0


# ---------------------- Hardware ----------------------
utime.sleep_ms(100)

led = Pin("C4", Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

motor_fl = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, MOTOR_FREQ, duty=0, invert=False)
motor_fr = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, MOTOR_FREQ, duty=0, invert=False)
motor_b = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, MOTOR_FREQ, duty=0, invert=True)

enc_fl = encoder("D15", "D16", False)
enc_fr = encoder("D13", "D14", False)
enc_b = encoder("C2", "C3", False)

wireless = WIRELESS_UART(460800)
imu_runtime = IMUYawRuntime(
    sign=GYRO_SIGN,
    offset_z=GYRO_OFFSET_Z,
    scale=GYRO_SCALE,
    deadband_dps=GYRO_DEADBAND_DPS,
    tick_period_ms=TICK_PERIOD_MS,
)


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


def reset_speed_pid(pid):
    pid.err = 0.0
    pid.err_last = 0.0
    pid.tar_spd_last = 0.0
    pid.output = 0.0
    pid.delta_tar_last = 0.0
    pid.delta_tar = 0.0
    pid.delta_ud = 0.0


def reset_all_pid():
    reset_speed_pid(pid_fl)
    reset_speed_pid(pid_fr)
    reset_speed_pid(pid_b)
    gyro_pid.output = 0.0
    gyro_pid.err = 0.0
    gyro_pid.err_last = 0.0


def check_upper_exit():
    wireless.data_analysis()
    ch7_current = wireless.get_data(EXIT_TRIGGER_CHANNEL)
    if abs(ch7_current - ch7_init_value) > CH7_TOLERANCE:
        log("CH7 changed: baseline=%.1f current=%.1f" % (ch7_init_value, ch7_current))
        return True
    return False


def get_stage(elapsed_ms):
    if elapsed_ms < FORWARD_MS:
        return "forward", FORWARD_VX, 0.0, 0.0
    if elapsed_ms < FORWARD_MS + RIGHT_MS:
        return "right", 0.0, RIGHT_VY, 0.0
    if elapsed_ms < FORWARD_MS + RIGHT_MS + BACKWARD_MS:
        return "backward", BACKWARD_VX, 0.0, 0.0
    if elapsed_ms < TOTAL_MS:
        return "left", 0.0, LEFT_VY, 0.0
    return "done", 0.0, 0.0, 0.0


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


# ---------------------- Ticker ----------------------
pit_flag = False
pit_count = 0


def time_pit_handler(_):
    global pit_flag, pit_count
    pit_flag = True
    pit_count += 1


pit1 = ticker(1)
pit1.capture_list(enc_fl, enc_fr, enc_b, imu_runtime.capture_device())
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
stage_name = ""


def run_stage(vx, vy, vz):
    gyro_z = imu_runtime.read_gyro_z()
    raw_gyro_z = imu_runtime.raw_gyro_z
    yaw_deg = imu_runtime.read_yaw()

    if ENABLE_GYRO_LOOP:
        vz_cmd = gyro_ctrl(gyro_pid, vz - gyro_z)
    else:
        gyro_pid.output = 0.0
        gyro_pid.err = 0.0
        gyro_pid.err_last = 0.0
        vz_cmd = vz

    calc_wheel_spd(move_cmd, vx, vy, vz_cmd)

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
        "pwm_fl": s_fl,
        "pwm_fr": s_fr,
        "pwm_b": s_b,
        "raw_gyro_z": raw_gyro_z,
        "gyro_z": gyro_z,
        "yaw_deg": yaw_deg,
        "vz_cmd": vz_cmd,
    }


log("=== motion sequence test start ===")
log(
    "sequence: forward %.1fs -> right %.1fs -> backward %.1fs -> left %.1fs"
    % (
        FORWARD_MS / 1000.0,
        RIGHT_MS / 1000.0,
        BACKWARD_MS / 1000.0,
        LEFT_MS / 1000.0,
    )
)
log(
    "body speed: forward_vx=%.1f backward_vx=%.1f right_vy=%.1f left_vy=%.1f"
    % (FORWARD_VX, BACKWARD_VX, RIGHT_VY, LEFT_VY)
)
log(
    "gyro loop: %s sign=%.1f offset_z=%.2f kp=%.3f ki=%.4f limit=%.2f"
    % (
        "on" if ENABLE_GYRO_LOOP else "off",
        GYRO_SIGN,
        GYRO_OFFSET_Z,
        GYRO_KP,
        GYRO_KI,
        GYRO_OUTPUT_LIMIT,
    )
)
log(
    "motor map: fl=C28/C29 invert=False, fr=C30/C31 invert=False, b=D4/D5 invert=True"
)
log("encoder map: fl=D15/D16, fr=D13/D14, b=C2/C3")


try:
    while True:
        loop_count += 1
        now = utime.ticks_ms()
        elapsed_ms = utime.ticks_diff(now, start_time)
        elapsed_s = elapsed_ms / 1000.0

        cur_stage, vx, vy, vz = get_stage(elapsed_ms)
        if cur_stage != stage_name:
            stage_name = cur_stage
            reset_all_pid()
            if stage_name == "done":
                log("=== sequence finished ===")
                break
            log("[STAGE] %s body=(%.1f, %.1f, %.1f)" % (stage_name, vx, vy, vz))

        if pit_flag:
            pit_flag = False
            snap = run_stage(vx, vy, vz)

            if pit_count % DEBUG_DIV == 0:
                log(
                    "[SEQ] t=%.2fs stage=%s enc=(%d,%d,%d) tar=(%.1f,%.1f,%.1f) pwm=(%d,%d,%d) raw_gz=%.1f gyro=%.2f yaw=%.2f vz=%.2f"
                    % (
                        elapsed_s,
                        stage_name,
                        int(snap["enc_fl"]),
                        int(snap["enc_fr"]),
                        int(snap["enc_b"]),
                        snap["tar_fl"],
                        snap["tar_fr"],
                        snap["tar_b"],
                        int(snap["pwm_fl"]),
                        int(snap["pwm_fr"]),
                        int(snap["pwm_b"]),
                        snap["raw_gyro_z"],
                        snap["gyro_z"],
                        snap["yaw_deg"],
                        snap["vz_cmd"],
                    )
                )

        if utime.ticks_diff(now, last_status_ms) >= 1000:
            led.toggle()
            last_status_ms = now
            log(
                "[STATUS] t=%.1fs stage=%s ch7=%.1f raw_gz=%.1f gyro=%.2f yaw=%.2f"
                % (
                    elapsed_s,
                    stage_name,
                    ch7_init_value,
                    imu_runtime.raw_gyro_z,
                    imu_runtime.gyro_z_deg,
                    imu_runtime.yaw_deg,
                )
            )

        if loop_count % EXIT_CHECK_DIV == 0 and check_upper_exit():
            log("=== CH7 exit triggered, program stopped ===")
            break

        if loop_count % GC_DIV == 0:
            gc.collect()

        utime.sleep_ms(1)

finally:
    pit1.stop()
    stop_all()
    led.value(True)
    log("=== program fully stopped ===")
