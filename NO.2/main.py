from machine import Pin, UART
from array import array
import gc
import utime
from smartcar import ticker, encoder
from seekfree import WIRELESS_UART
from imu_runtime import IMUYawRuntime
from models import AnglePID, MoveBase, SpeedPID
from move_base import calc_wheel_spd
import pid as _pid_mod
import config as cfg
from hardware import Motor
from coop_protocol import (
    CoopFrameParser,
    MASTER_MOTION_FLAG_STARTED,
    MSG_MASTER_MOTION,
    decode_master_motion,
)


_pid_mod.PWM_MAX = cfg.PWM_MAX
speed_ctrl = _pid_mod.speed_ctrl
gyro_ctrl = _pid_mod.gyro_ctrl


# ====================== Base config ======================
TICK_PERIOD_MS = cfg.TICK_PERIOD_MS
MOTOR_DUTY_MAX = cfg.MOTOR_DUTY_MAX
MOTOR_DUTY_MIN = cfg.MOTOR_DUTY_MIN
PWM_SMOOTH_FACTOR = cfg.PWM_SMOOTH_FACTOR
MAX_PWM_CHANGE = cfg.MAX_PWM_CHANGE

ENABLE_GYRO_LOOP = True
ENABLE_IMU = ENABLE_GYRO_LOOP
GYRO_SIGN = 1.0
GYRO_OFFSET_Z = 3.16
GYRO_SCALE = -1.0 / 16.54052
GYRO_DEADBAND_DPS = 0.8
GYRO_KP = 0.22
GYRO_KI = 0.004
GYRO_OUTPUT_LIMIT = 7.0
AUTO_CALIBRATE_GYRO_ON_LAUNCH = True
GYRO_CALIBRATE_SAMPLES = 1000
GYRO_CALIBRATE_DELAY_MS = 2

EXIT_CHECK_DIV = 5
GC_DIV = 50
FOLLOW_LOG_ENABLE = False
TURN_DEBUG_LOG_ENABLE = False
TURN_DEBUG_LOG_INTERVAL_MS = 100
MAP_DEBUG_LOG_ENABLE = True
MAP_DEBUG_LOG_INTERVAL_MS = 100
DEBUG_DIV = 50
FORCE_MOTOR_OFF = False
AUTO_START_ON_BOOT = False
AUTO_START_DELAY_MS = 2000


# ====================== Camera protocol ======================
Cam_Error_Offset = 120
Cam_Error_Scale = 2
Cam_Packet_Timeout_Ms = 200
Cam_Frame_Head = 0xFF
No_Target_Marker = 0xFE
Line_Packet_Tag = 0xFC
Classify_Packet_Tag = 0xFD
ART_MODE_TRACK_CMD = b"TRACK\n"
ART_MODE_IDLE_CMD = b"IDLE\n"


# ====================== Follow control ======================
Follow_Forward_Gain = 0.060
Follow_Lateral_Gain = 0.045
Follow_Forward_Error_Sign = -1.0
Follow_Forward_Limit = 12.0
Follow_Lateral_Limit = 7.0
Follow_Forward_Deadband = 4
Follow_Lateral_Deadband = 4
Follow_Feedforward_Gain = 1.0
Follow_Hold_Feedforward_Gain = 0.80
Follow_Wz_Feedforward_Gain = 0.4
Follow_Yaw_Enable = False
Follow_Yaw_Gain = 0.08
Follow_Yaw_Limit = 15.0
Follow_Target_Lost_Hold_Ms = 250
Master_Motion_Timeout_Ms = 250
Follow_Master_Extra_Vx = 2.0
Follow_Master_Extra_Vy = 1.5
Camera_Right_Yaw_Cos = 0.5
Camera_Right_Yaw_Sin = 0.8660254


# ====================== Runtime state ======================
car_started = False
auto_start_done = False
last_c9_state = 1
last_c8_state = 1

cam_error_x = 0
cam_error_y = 0
cam_target_vx = 0.0
cam_target_vy = 0.0
cam_last_rx_ms = 0
cam_rx_buf = bytearray()
cam_has_target = False
cam_rx_started = False
cam_valid_target_since_ms = 0
target_lost_since_ms = 0

master_vx = 0.0
master_vy = 0.0
master_wz = 0.0
master_yaw = 0.0
master_flags = 0
master_last_rx_ms = 0

coop_parser = CoopFrameParser()
COOP_LED_PULSE_MS = 40
coop_rx_led_until_ms = 0

pit_flag = False
pit_count = 0
start_time = utime.ticks_ms()
last_status_ms = start_time
loop_count = 0
last_pwm_fl = 0
last_pwm_fr = 0
last_pwm_b = 0
turn_debug_last_ms = 0
map_debug_last_ms = 0
last_turn_rate_cmd = 0.0
last_vz_cmd = 0.0
yaw_ref_deg = 0.0


def log(msg):
    if FOLLOW_LOG_ENABLE:
        print(msg)


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def wrapped_yaw_error(ref_deg, now_deg):
    err = now_deg - ref_deg
    if err > 180.0:
        err -= 360.0
    elif err < -180.0:
        err += 360.0
    return err


def update_cam_target(err_x, err_y):
    global cam_error_x, cam_error_y, cam_last_rx_ms

    cam_error_x = int(err_x)
    cam_error_y = int(err_y)
    cam_last_rx_ms = utime.ticks_ms()


def clear_cam_target_state():
    global cam_error_x, cam_error_y, cam_last_rx_ms, cam_rx_buf
    global cam_has_target, cam_valid_target_since_ms, target_lost_since_ms

    cam_error_x = 0
    cam_error_y = 0
    cam_has_target = False
    cam_valid_target_since_ms = 0
    target_lost_since_ms = 0
    cam_last_rx_ms = 0
    cam_rx_buf = bytearray()


def cam_packet_fresh():
    return utime.ticks_diff(utime.ticks_ms(), cam_last_rx_ms) <= Cam_Packet_Timeout_Ms


def cam_target_seen():
    return cam_has_target and cam_packet_fresh()


def master_motion_fresh():
    return utime.ticks_diff(utime.ticks_ms(), master_last_rx_ms) <= Master_Motion_Timeout_Ms


def master_started():
    return master_motion_fresh() and ((master_flags & MASTER_MOTION_FLAG_STARTED) != 0)


def rotate_camera_velocity_to_body(cam_vx, cam_vy):
    body_vx = cam_vx * Camera_Right_Yaw_Cos + cam_vy * Camera_Right_Yaw_Sin
    body_vy = -cam_vx * Camera_Right_Yaw_Sin + cam_vy * Camera_Right_Yaw_Cos
    return body_vx, body_vy


def follow_limit(base_limit, master_value, extra):
    limit = base_limit
    master_abs = abs(master_value) + extra
    if master_abs > limit:
        limit = master_abs
    return limit


def poll_art_uart():
    global cam_rx_buf, cam_has_target, cam_rx_started, cam_valid_target_since_ms
    global cam_last_rx_ms, target_lost_since_ms

    pending = cam_uart.any()
    if pending:
        if pending > 32:
            pending = 32
        data = cam_uart.read(pending)
        if data:
            cam_rx_buf += data
            while len(cam_rx_buf) >= 3:
                if cam_rx_buf[0] != Cam_Frame_Head:
                    cam_rx_buf = cam_rx_buf[1:]
                    continue
                if not cam_rx_started:
                    cam_rx_started = True
                if cam_rx_buf[1] == No_Target_Marker and cam_rx_buf[2] == No_Target_Marker:
                    cam_has_target = False
                    cam_valid_target_since_ms = 0
                    cam_last_rx_ms = utime.ticks_ms()
                elif cam_rx_buf[1] in (Line_Packet_Tag, Classify_Packet_Tag):
                    cam_last_rx_ms = utime.ticks_ms()
                else:
                    cam_has_target = True
                    target_lost_since_ms = 0
                    if cam_valid_target_since_ms == 0:
                        cam_valid_target_since_ms = utime.ticks_ms()
                    update_cam_target(
                        (int(cam_rx_buf[1]) - Cam_Error_Offset) * Cam_Error_Scale,
                        (int(cam_rx_buf[2]) - Cam_Error_Offset) * Cam_Error_Scale,
                    )
                cam_rx_buf = cam_rx_buf[3:]

    if len(cam_rx_buf) > 20:
        cam_rx_buf = bytearray()


def handle_coop_frame(msg_type, seq, payload, payload_len):
    global master_vx, master_vy, master_wz, master_yaw
    global master_flags, master_last_rx_ms

    if msg_type != MSG_MASTER_MOTION:
        return
    motion = decode_master_motion(payload, payload_len)
    if motion is None:
        return
    master_vx = motion[0]
    master_vy = motion[1]
    master_wz = motion[2]
    master_yaw = motion[3]
    master_flags = int(motion[4])
    master_last_rx_ms = utime.ticks_ms()
    coop_flash_rx()


def poll_coop_uart():
    try:
        n = wireless.receive_bytearray(coop_rx_buf, len(coop_rx_buf))
        if n:
            coop_parser.feed(coop_rx_buf, n, handle_coop_frame)
    except Exception:
        pass


def update_follow_targets(yaw_deg, gyro_z):
    global cam_target_vx, cam_target_vy, target_lost_since_ms
    global yaw_ref_deg, last_turn_rate_cmd, last_vz_cmd
    global turn_debug_last_ms, map_debug_last_ms

    now = utime.ticks_ms()
    seen = cam_target_seen()
    fresh_motion = master_motion_fresh()
    ff_vx = master_vx if fresh_motion else 0.0
    ff_vy = master_vy if fresh_motion else 0.0
    ff_wz = master_wz if fresh_motion else 0.0
    cam_vx = 0.0
    cam_vy = 0.0
    body_vx = 0.0
    body_vy = 0.0

    if seen:
        target_lost_since_ms = 0
        cam_vx = cam_error_y * Follow_Forward_Gain * Follow_Forward_Error_Sign
        cam_vy = -cam_error_x * Follow_Lateral_Gain
        if -Follow_Forward_Deadband <= cam_error_y <= Follow_Forward_Deadband:
            cam_vx = 0.0
        if -Follow_Lateral_Deadband <= cam_error_x <= Follow_Lateral_Deadband:
            cam_vy = 0.0
        body_vx, body_vy = rotate_camera_velocity_to_body(cam_vx, cam_vy)
        vx = body_vx + ff_vx * Follow_Feedforward_Gain
        vy = body_vy + ff_vy * Follow_Feedforward_Gain
    else:
        if target_lost_since_ms == 0:
            target_lost_since_ms = now
        if fresh_motion and utime.ticks_diff(now, target_lost_since_ms) <= Follow_Target_Lost_Hold_Ms:
            vx = ff_vx * Follow_Hold_Feedforward_Gain
            vy = ff_vy * Follow_Hold_Feedforward_Gain
        else:
            vx = 0.0
            vy = 0.0

    vx_limit = Follow_Forward_Limit
    vy_limit = Follow_Lateral_Limit
    if fresh_motion:
        vx_limit = follow_limit(vx_limit, ff_vx, Follow_Master_Extra_Vx)
        vy_limit = follow_limit(vy_limit, ff_vy, Follow_Master_Extra_Vy)
    vx = clamp(vx, -vx_limit, vx_limit)
    vy = clamp(vy, -vy_limit, vy_limit)
    cam_target_vx = vx
    cam_target_vy = vy

    if MAP_DEBUG_LOG_ENABLE:
        if utime.ticks_diff(now, map_debug_last_ms) >= MAP_DEBUG_LOG_INTERVAL_MS:
            map_debug_last_ms = now
            print(
                "[MAP] seen=%d err=(%d,%d) cam=(%.2f,%.2f) body=(%.2f,%.2f) ff=(%.2f,%.2f) out=(%.2f,%.2f)"
                % (
                    1 if seen else 0,
                    cam_error_x,
                    cam_error_y,
                    cam_vx,
                    cam_vy,
                    body_vx,
                    body_vy,
                    ff_vx,
                    ff_vy,
                    vx,
                    vy,
                )
            )

    turn_rate_cmd = ff_wz * Follow_Wz_Feedforward_Gain
    yaw_err = 0.0
    yaw_correction = 0.0
    if Follow_Yaw_Enable and fresh_motion and ENABLE_IMU:
        yaw_ref_deg = master_yaw
        yaw_err = -wrapped_yaw_error(yaw_ref_deg, yaw_deg)
        yaw_correction = clamp(yaw_err * Follow_Yaw_Gain, -Follow_Yaw_Limit, Follow_Yaw_Limit)
        turn_rate_cmd += yaw_correction

    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        vz_cmd = gyro_ctrl(gyro_pid, turn_rate_cmd - gyro_z)
    else:
        vz_cmd = turn_rate_cmd

    if TURN_DEBUG_LOG_ENABLE:
        if utime.ticks_diff(now, turn_debug_last_ms) >= TURN_DEBUG_LOG_INTERVAL_MS:
            turn_debug_last_ms = now
            print(
                "[TURN] fwz=%.2f gain=%.2f yaw_en=%d yaw_err=%.2f yaw_fix=%.2f cmd=%.2f gyro=%.2f vz=%.2f"
                % (
                    ff_wz,
                    Follow_Wz_Feedforward_Gain,
                    1 if Follow_Yaw_Enable else 0,
                    yaw_err,
                    yaw_correction,
                    turn_rate_cmd,
                    gyro_z,
                    vz_cmd,
                )
            )

    last_turn_rate_cmd = turn_rate_cmd
    last_vz_cmd = vz_cmd
    return vz_cmd


def stop_all():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)
    log("[STOP] motors off")


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


def update_nav_led_display():
    straight_value = 1 if cam_target_seen() else 0
    translate_value = 1 if master_motion_fresh() else 0
    rotate_value = 1 if car_started else 0
    now = utime.ticks_ms()
    rx_active = coop_rx_led_until_ms and utime.ticks_diff(coop_rx_led_until_ms, now) > 0
    if rx_active:
        translate_value = 0 if translate_value else 1
    led_straight.value(straight_value)
    led_translate.value(translate_value)
    led_rotate.value(rotate_value)


def coop_flash_rx():
    global coop_rx_led_until_ms
    coop_rx_led_until_ms = utime.ticks_add(utime.ticks_ms(), COOP_LED_PULSE_MS)


def calibrate_gyro_before_launch():
    if (
        AUTO_CALIBRATE_GYRO_ON_LAUNCH
        and ENABLE_IMU
        and imu_runtime is not None
    ):
        pit1.stop()
        try:
            imu_runtime.calibrate_offset(
                samples=GYRO_CALIBRATE_SAMPLES,
                delay_ms=GYRO_CALIBRATE_DELAY_MS,
                logger=log,
            )
            imu_runtime.reset_yaw(0.0)
        finally:
            pit1.start(TICK_PERIOD_MS)


def start_follow(reason):
    global car_started, auto_start_done, start_time, yaw_ref_deg

    if car_started:
        return
    calibrate_gyro_before_launch()
    if ENABLE_IMU and imu_runtime is not None:
        yaw_ref_deg = imu_runtime.read_yaw()
    else:
        yaw_ref_deg = 0.0
    clear_cam_target_state()
    car_started = True
    auto_start_done = True
    start_time = utime.ticks_ms()
    cam_uart.write(ART_MODE_TRACK_CMD)
    log("[FOLLOW] started: %s" % reason)


def check_c9_start():
    global last_c9_state

    current_c9 = key_start.value()
    if current_c9 == 0 and last_c9_state == 1:
        utime.sleep_ms(10)
        if key_start.value() == 0:
            start_follow("C9")
    last_c9_state = current_c9


def check_c8_exit():
    global last_c8_state

    current_c8 = key_exit.value()
    if current_c8 == 0 and last_c8_state == 1:
        utime.sleep_ms(10)
        if key_exit.value() == 0:
            log("[C8] exit")
            raise KeyboardInterrupt
    last_c8_state = current_c8


def check_upper_exit():
    return False


def time_pit_handler(_):
    global pit_flag, pit_count
    pit_flag = True
    pit_count += 1


def calc_speed_closed_loop():
    global last_pwm_fl, last_pwm_fr, last_pwm_b

    if not car_started:
        set_three_pwm_smooth(0, 0, 0)
        return None

    if ENABLE_IMU:
        gyro_z = imu_runtime.read_gyro_z()
        raw_gyro_z = imu_runtime.raw_gyro_z
        yaw_deg = imu_runtime.read_yaw()
    else:
        gyro_z = 0.0
        raw_gyro_z = 0.0
        yaw_deg = 0.0

    vz_cmd = update_follow_targets(yaw_deg, gyro_z)
    calc_wheel_spd(move_cmd, cam_target_vx, cam_target_vy, vz_cmd)

    e_fl = enc_fl.get()
    e_fr = enc_fr.get()
    e_b = enc_b.get()
    t_fl = move_cmd.speed_fl
    t_fr = move_cmd.speed_fr
    t_b = move_cmd.speed_b
    u_fl = speed_ctrl(pid_fl, e_fl, t_fl)
    u_fr = speed_ctrl(pid_fr, e_fr, t_fr)
    u_b = speed_ctrl(pid_b, e_b, t_b)

    if FORCE_MOTOR_OFF:
        s_fl, s_fr, s_b = set_three_pwm_smooth(0, 0, 0)
    else:
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
        "turn_rate_cmd": last_turn_rate_cmd,
        "vz_cmd": last_vz_cmd,
    }


key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)
led_straight = Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
led_translate = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
led_rotate = Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)

utime.sleep_ms(100)
led = Pin(cfg.LED_HB_PIN, Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

motor_fl = Motor(cfg.MOTOR_FL_PH, cfg.MOTOR_FL_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FL_INVERT)
motor_fr = Motor(cfg.MOTOR_FR_PH, cfg.MOTOR_FR_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FR_INVERT)
motor_b = Motor(cfg.MOTOR_B_PH, cfg.MOTOR_B_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_B_INVERT)
enc_fl = encoder(cfg.ENC_FL_A, cfg.ENC_FL_B, cfg.ENC_FL_INVERT)
enc_fr = encoder(cfg.ENC_FR_A, cfg.ENC_FR_B, cfg.ENC_FR_INVERT)
enc_b = encoder(cfg.ENC_B_A, cfg.ENC_B_B, cfg.ENC_B_INVERT)

wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
coop_rx_buf = array('b', [0] * 32)
cam_uart = UART(cfg.CAM_UART_ID, cfg.CAM_UART_BAUD)
cam_uart.init(cfg.CAM_UART_BAUD, timeout_char=100)
cam_uart.write(ART_MODE_TRACK_CMD)

imu_runtime = None
if ENABLE_IMU:
    imu_runtime = IMUYawRuntime(
        sign=GYRO_SIGN,
        offset_z=GYRO_OFFSET_Z,
        scale=GYRO_SCALE,
        deadband_dps=GYRO_DEADBAND_DPS,
        tick_period_ms=TICK_PERIOD_MS,
    )

pit1 = ticker(1)
if ENABLE_IMU:
    pit1.capture_list(enc_fl, enc_fr, enc_b, imu_runtime.capture_device())
else:
    pit1.capture_list(enc_fl, enc_fr, enc_b)
pit1.callback(time_pit_handler)
pit1.start(TICK_PERIOD_MS)

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

log("[INIT] follower IR follow ready")
log("[INIT] camera=%d@%d wireless=%d" % (cfg.CAM_UART_ID, cfg.CAM_UART_BAUD, cfg.COOP_WIRELESS_BAUD))

try:
    while True:
        loop_count += 1
        now = utime.ticks_ms()
        elapsed_s = utime.ticks_diff(now, start_time) / 1000.0

        check_c8_exit()
        check_c9_start()
        poll_art_uart()
        poll_coop_uart()
        update_nav_led_display()

        if AUTO_START_ON_BOOT and (not auto_start_done):
            if utime.ticks_diff(now, start_time) >= AUTO_START_DELAY_MS:
                start_follow("auto")
        if (not car_started) and (cam_target_seen() or master_started()):
            start_follow("signal")

        snap = None
        if pit_flag:
            pit_flag = False
            snap = calc_speed_closed_loop()

        if utime.ticks_diff(now, last_status_ms) >= 1000:
            led.toggle()
            last_status_ms = now
            if FOLLOW_LOG_ENABLE:
                log(
                    "[FOLLOW] t=%.1f started=%d seen=%d err=(%d,%d) cmd=(%.1f,%.1f,%.1f) master=(%.1f,%.1f,%.1f) fresh=%d"
                    % (
                        elapsed_s,
                        1 if car_started else 0,
                        1 if cam_target_seen() else 0,
                        cam_error_x,
                        cam_error_y,
                        cam_target_vx,
                        cam_target_vy,
                        last_turn_rate_cmd,
                        master_vx,
                        master_vy,
                        master_wz,
                        1 if master_motion_fresh() else 0,
                    )
                )

        if FOLLOW_LOG_ENABLE and snap is not None and (pit_count % DEBUG_DIV) == 0:
            log(
                "[LOOP] enc=(%d,%d,%d) tar=(%.1f,%.1f,%.1f) pwm=(%d,%d,%d) gyro=%.2f yaw=%.2f vz=%.2f"
                % (
                    int(snap["enc_fl"]),
                    int(snap["enc_fr"]),
                    int(snap["enc_b"]),
                    snap["tar_fl"],
                    snap["tar_fr"],
                    snap["tar_b"],
                    int(snap["pwm_fl"]),
                    int(snap["pwm_fr"]),
                    int(snap["pwm_b"]),
                    snap["gyro_z"],
                    snap["yaw_deg"],
                    snap["vz_cmd"],
                )
            )

        if loop_count % EXIT_CHECK_DIV == 0 and check_upper_exit():
            break
        if loop_count % GC_DIV == 0:
            gc.collect()

        utime.sleep_ms(1)

finally:
    pit1.stop()
    cam_uart.write(ART_MODE_IDLE_CMD)
    stop_all()
    led.value(True)
    led_straight.value(0)
    led_translate.value(0)
    led_rotate.value(0)
    log("[EXIT] follower stopped")
