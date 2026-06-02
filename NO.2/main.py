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
speed_reset = _pid_mod.speed_reset


# ====================== Base config ======================
TICK_PERIOD_MS = cfg.TICK_PERIOD_MS
MOTOR_DUTY_MAX = cfg.MOTOR_DUTY_MAX
MOTOR_DUTY_MIN = cfg.MOTOR_DUTY_MIN
PWM_SMOOTH_FACTOR = cfg.PWM_SMOOTH_FACTOR
MAX_PWM_CHANGE = cfg.MAX_PWM_CHANGE

ENABLE_GYRO_LOOP = True
FOLLOW_GYRO_HOLD_ENABLE = False
ENABLE_IMU = ENABLE_GYRO_LOOP
GYRO_SIGN = 1.0
GYRO_OFFSET_Z = 3.16
GYRO_SCALE = -1.0 / 16.54052
GYRO_DEADBAND_DPS = 0.8
GYRO_KP = 0.24
GYRO_KI = 0.0005
GYRO_PRIORITY_KP = 0.58
GYRO_PRIORITY_KI = 0.0
GYRO_OUTPUT_LIMIT = 14.0
GYRO_OUTPUT_BASE_LIMIT = 6.0
GYRO_OUTPUT_TARGET_GAIN = 2.2
GYRO_OUTPUT_MAX_LIMIT = 22.0
GYRO_PRIORITY_OUTPUT_BASE_LIMIT = 4.5
GYRO_PRIORITY_OUTPUT_TARGET_GAIN = 0.85
GYRO_PRIORITY_OUTPUT_MAX_LIMIT = 18.0
GYRO_PRIORITY_MIN_OUTPUT = 4.2
GYRO_PRIORITY_MIN_RATE_RATIO = 0.35
GYRO_PRIORITY_MIN_CMD = 6.5
GYRO_PRIORITY_OVERSPEED_RATIO = 1.20
GYRO_PRIORITY_BRAKE_KP = 0.22
GYRO_PRIORITY_BRAKE_LIMIT = 5.0
AUTO_CALIBRATE_GYRO_ON_LAUNCH = True
GYRO_CALIBRATE_SAMPLES = 1000
GYRO_CALIBRATE_DELAY_MS = 2

EXIT_CHECK_DIV = 5
GC_DIV = 50
USE_MASTER_MOTION_FEEDFORWARD = True
FOLLOW_WIRELESS_TUNE_LOG_ENABLE = True
FOLLOW_TUNE_LOG_INTERVAL_MS = 100
FORCE_MOTOR_OFF = False
AUTO_START_ON_BOOT = False
AUTO_START_DELAY_MS = 2000
WHEEL_TARGET_STOP_EPS = 0.05
WHEEL_TARGET_IDLE_EPS = 0.35
FOLLOW_START_PWM = 6200
FOLLOW_START_PWM_MID = 3600
FOLLOW_START_PWM_LOW = 0
FOLLOW_STALL_BOOST_PWM = 8800
FOLLOW_START_PWM_LOW_TARGET = 1.2
FOLLOW_START_PWM_MID_TARGET = 2.8
FOLLOW_STALL_BOOST_TARGET = 2.8
FOLLOW_RUN_PWM_LIMIT = 24000
FOLLOW_STALL_BOOST_FRAMES = 3


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
Follow_Forward_Gain = 0.50
Follow_Lateral_Gain = 0.22
Follow_Orbit_Forward_Gain = 0.76
Follow_Orbit_Lateral_Gain = 0.38
Follow_Forward_Error_Sign = 1.0
Follow_Lateral_Error_Sign = -1.0
Follow_Forward_Limit = 58.0
Follow_Lateral_Limit = 36.0
Follow_Forward_Deadband = 4
Follow_Lateral_Deadband = 6
Follow_Orbit_Forward_Deadband = 3
Follow_Orbit_Lateral_Deadband = 2
Follow_Orbit_Position_X_Error = 4
Follow_Orbit_Position_Y_Error = 4
Follow_Distance_Far_Boost_Error = 6
Follow_Distance_Far_Boost_Gain = 0.65
Follow_Distance_Close_Gain = 0.76
Follow_Distance_Close_Limit = 28.0
Follow_Distance_Back_Min_Vx = 8.0
Follow_Distance_Back_Max_Vx = 22.0
Follow_Distance_Back_Vy_Limit = 5.0
Follow_Distance_No_Forward_Error = 0
Follow_Distance_Feedforward_Enable_Error = 20
Follow_Distance_Min_Chase_Vx = 4.2
Follow_Orbit_Min_Chase_Vx = 6.0
Follow_Distance_Approach_Slow_Error = 4
Follow_Distance_Approach_Vx_Limit = 0.0
Follow_Distance_Lock_Error = 4
Follow_Distance_Lock_Hold_Ms = 450
Follow_Feedforward_Forward_Gain = 0.95
Follow_Feedforward_Lateral_Gain = 0.35
Follow_Feedforward_Forward_Limit = 6.0
Follow_Feedforward_Lateral_Limit = 3.0
Follow_Hold_Feedforward_Gain = 1.70
Follow_Wz_Feedforward_Gain = 1.60
Follow_Wz_Feedforward_Limit = 9.0
Follow_Wz_Forward_Gain = 0.0
Follow_Wz_Lateral_Gain = -0.18
Follow_Pose_Angle_Gain = -0.70
Follow_Pose_Angle_Limit = 42.0
Follow_Pose_Angle_Deadband = 4
Follow_Pose_Angle_Active_Error = 6
Follow_Pose_Wheel_Target_Limit = 34.0
Follow_Command_Ramp_Vx = 5.0
Follow_Command_Ramp_Vy = 2.4
Follow_Orbit_Command_Ramp_Vx = 8.0
Follow_Orbit_Command_Ramp_Vy = 5.0
Follow_Command_Ramp_Wz = 9.0
Follow_Pose_Gyro_Output_Ramp = 2.4
Follow_Yaw_Enable = False
Follow_Yaw_Gain = 0.08
Follow_Yaw_Limit = 15.0
Follow_Target_Lost_Hold_Ms = 250
Master_Motion_Timeout_Ms = 250
Follow_Master_Extra_Vx = 10.0
Follow_Master_Extra_Vy = 8.0
Camera_Right_Yaw_Cos = 0.5
Camera_Right_Yaw_Sin = 0.8660254


# ====================== Runtime state ======================
car_started = False
auto_start_done = False
last_c9_state = 1
last_c8_state = 1

cam_error_x = 0
cam_error_y = 0
cam_error_angle = 0
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
last_turn_rate_cmd = 0.0
last_vz_cmd = 0.0
last_follow_seen = False
last_visual_vx = 0.0
last_visual_vy = 0.0
last_ff_vx = 0.0
last_ff_vy = 0.0
last_ff_wz = 0.0
last_cmd_vx = 0.0
last_cmd_vy = 0.0
last_cmd_wz = 0.0
last_ap_vz_cmd = 0.0
last_distance_lock_ms = 0
last_angle_priority_active = False
last_back_priority_active = False
last_tune_log_ms = 0
last_hard_stop = False
last_stall_count = 0
last_stall_boost = False
yaw_ref_deg = 0.0


def log(msg):
    pass


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def ramp_value(target, last, step):
    delta = target - last
    if delta > step:
        return last + step
    if delta < -step:
        return last - step
    return target


def gyro_limit_for_turn(turn_rate_cmd, priority=False):
    if priority:
        base_limit = GYRO_PRIORITY_OUTPUT_BASE_LIMIT
        target_gain = GYRO_PRIORITY_OUTPUT_TARGET_GAIN
        max_limit = GYRO_PRIORITY_OUTPUT_MAX_LIMIT
    else:
        base_limit = GYRO_OUTPUT_BASE_LIMIT
        target_gain = GYRO_OUTPUT_TARGET_GAIN
        max_limit = GYRO_OUTPUT_MAX_LIMIT
    limit = base_limit + abs(turn_rate_cmd) * target_gain
    if limit > max_limit:
        return max_limit
    if limit < base_limit:
        return base_limit
    return limit


def wrapped_yaw_error(ref_deg, now_deg):
    err = now_deg - ref_deg
    if err > 180.0:
        err -= 360.0
    elif err < -180.0:
        err += 360.0
    return err


def update_cam_target(err_x, err_y, err_angle=0):
    global cam_error_x, cam_error_y, cam_error_angle, cam_last_rx_ms

    cam_error_x = int(err_x)
    cam_error_y = int(err_y)
    cam_error_angle = int(err_angle)
    cam_last_rx_ms = utime.ticks_ms()


def clear_cam_target_state():
    global cam_error_x, cam_error_y, cam_last_rx_ms, cam_rx_buf
    global cam_error_angle
    global cam_has_target, cam_valid_target_since_ms, target_lost_since_ms
    global last_cmd_vx, last_cmd_vy, last_cmd_wz, last_ap_vz_cmd
    global last_distance_lock_ms, last_angle_priority_active
    global last_back_priority_active

    cam_error_x = 0
    cam_error_y = 0
    cam_error_angle = 0
    last_cmd_vx = 0.0
    last_cmd_vy = 0.0
    last_cmd_wz = 0.0
    last_ap_vz_cmd = 0.0
    last_distance_lock_ms = 0
    last_angle_priority_active = False
    last_back_priority_active = False
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
    return (
        USE_MASTER_MOTION_FEEDFORWARD
        and utime.ticks_diff(utime.ticks_ms(), master_last_rx_ms) <= Master_Motion_Timeout_Ms
    )


def master_motion_rx_fresh():
    return utime.ticks_diff(utime.ticks_ms(), master_last_rx_ms) <= Master_Motion_Timeout_Ms


def master_started():
    return master_motion_fresh() and ((master_flags & MASTER_MOTION_FLAG_STARTED) != 0)


def rotate_camera_velocity_to_body(cam_vx, cam_vy):
    body_vx = cam_vx
    body_vy = cam_vy
    return body_vx, body_vy


def follow_limit(base_limit, master_value, extra):
    limit = base_limit
    master_abs = abs(master_value) + extra
    if master_abs > limit:
        limit = master_abs
    return limit


def position_priority_needed(error_x, error_y, angle_active):
    return (
        angle_active
        or error_x >= Follow_Orbit_Position_X_Error
        or error_x <= -Follow_Orbit_Position_X_Error
        or error_y >= Follow_Orbit_Position_Y_Error
        or error_y <= -Follow_Orbit_Position_Y_Error
    )


def calc_follow_forward(error_y, position_priority=False):
    deadband = Follow_Orbit_Forward_Deadband if position_priority else Follow_Forward_Deadband
    min_chase_vx = Follow_Orbit_Min_Chase_Vx if position_priority else Follow_Distance_Min_Chase_Vx
    if -deadband <= error_y <= deadband:
        return 0.0

    if error_y > 0:
        gain = Follow_Orbit_Forward_Gain if position_priority else Follow_Forward_Gain
        out = error_y * gain
        if error_y > Follow_Distance_Far_Boost_Error:
            out += (
                error_y - Follow_Distance_Far_Boost_Error
            ) * Follow_Distance_Far_Boost_Gain
        out *= Follow_Forward_Error_Sign
        if Follow_Forward_Error_Sign >= 0:
            if 0.0 < out < min_chase_vx:
                out = min_chase_vx
            return clamp(out, 0.0, Follow_Forward_Limit)
        if -min_chase_vx < out < 0.0:
            out = -min_chase_vx
        return clamp(out, -Follow_Forward_Limit, 0.0)

    out = error_y * Follow_Distance_Close_Gain * Follow_Forward_Error_Sign
    if Follow_Forward_Error_Sign >= 0:
        if -Follow_Distance_Back_Min_Vx < out < 0.0:
            out = -Follow_Distance_Back_Min_Vx
        return clamp(out, -Follow_Distance_Close_Limit, 0.0)
    if 0.0 < out < Follow_Distance_Back_Min_Vx:
        out = Follow_Distance_Back_Min_Vx
    return clamp(out, 0.0, Follow_Distance_Close_Limit)


def apply_distance_guard(vx, visual_vx, error_y, position_priority=False):
    if (
        (not position_priority)
        and 0 < error_y <= Follow_Distance_Approach_Slow_Error
    ):
        if visual_vx > Follow_Distance_Approach_Vx_Limit:
            visual_vx = Follow_Distance_Approach_Vx_Limit
        elif visual_vx < -Follow_Distance_Approach_Vx_Limit:
            visual_vx = -Follow_Distance_Approach_Vx_Limit
        if vx > visual_vx:
            vx = visual_vx
    if error_y <= Follow_Distance_No_Forward_Error:
        if vx > visual_vx:
            vx = visual_vx
        if vx > 0.0:
            vx = 0.0
    elif error_y <= Follow_Distance_Feedforward_Enable_Error and vx > visual_vx:
        vx = visual_vx
    return vx


def calc_follow_lateral(error_x, position_priority=False):
    deadband = Follow_Orbit_Lateral_Deadband if position_priority else Follow_Lateral_Deadband
    if -deadband <= error_x <= deadband:
        return 0.0
    gain = Follow_Orbit_Lateral_Gain if position_priority else Follow_Lateral_Gain
    return clamp(
        -error_x * gain * Follow_Lateral_Error_Sign,
        -Follow_Lateral_Limit,
        Follow_Lateral_Limit,
    )


def calc_follow_angle(error_angle):
    if -Follow_Pose_Angle_Deadband <= error_angle <= Follow_Pose_Angle_Deadband:
        return 0.0
    return clamp(
        error_angle * Follow_Pose_Angle_Gain,
        -Follow_Pose_Angle_Limit,
        Follow_Pose_Angle_Limit,
    )


def add_feedforward_assist(base, feedforward, gain, limit):
    assist = clamp(feedforward * gain, -limit, limit)
    if base > 0.001 and assist < -0.001:
        return base
    if base < -0.001 and assist > 0.001:
        return base
    return base + assist


def solve_follow_pose_twist(error_x, error_y, error_angle, ff_vx, ff_vy, ff_wz, use_ff):
    vision_wz = calc_follow_angle(error_angle)
    angle_active = (
        error_angle >= Follow_Pose_Angle_Active_Error
        or error_angle <= -Follow_Pose_Angle_Active_Error
    )
    position_priority = position_priority_needed(error_x, error_y, angle_active)
    cam_vx = calc_follow_forward(error_y, position_priority)
    cam_vy = calc_follow_lateral(error_x, position_priority)
    body_vx, body_vy = rotate_camera_velocity_to_body(cam_vx, cam_vy)
    vx = body_vx
    vy = body_vy
    wz = vision_wz
    if use_ff:
        vx = add_feedforward_assist(
            vx,
            ff_vx,
            Follow_Feedforward_Forward_Gain,
            Follow_Feedforward_Forward_Limit,
        )
        vy = add_feedforward_assist(
            vy,
            ff_vy,
            Follow_Feedforward_Lateral_Gain,
            Follow_Feedforward_Lateral_Limit,
        )
        vx += ff_wz * Follow_Wz_Forward_Gain
        vy += ff_wz * Follow_Wz_Lateral_Gain
        wz = add_feedforward_assist(
            wz,
            ff_wz,
            Follow_Wz_Feedforward_Gain,
            Follow_Wz_Feedforward_Limit,
        )
    return vx, vy, wz, body_vx, body_vy, angle_active, position_priority


def pose_wheel_targets(vx, vy, vz):
    wheel_fr = -vx * 0.866025 + vy * 0.5 + vz
    wheel_fl = vx * 0.866025 + vy * 0.5 + vz
    wheel_b = -vy + vz
    return wheel_fr, wheel_fl, wheel_b


def max_wheel_abs(wheel_fr, wheel_fl, wheel_b):
    max_abs = abs(wheel_fr)
    tmp = abs(wheel_fl)
    if tmp > max_abs:
        max_abs = tmp
    tmp = abs(wheel_b)
    if tmp > max_abs:
        max_abs = tmp
    return max_abs


def limit_pose_twist_for_wheels(vx, vy, vz):
    if Follow_Pose_Wheel_Target_Limit <= 0.0:
        return vx, vy, vz

    wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(vx, vy, 0.0)
    pos_max = max_wheel_abs(wheel_fr, wheel_fl, wheel_b)
    if pos_max > Follow_Pose_Wheel_Target_Limit:
        pos_scale = Follow_Pose_Wheel_Target_Limit / pos_max
        vx *= pos_scale
        vy *= pos_scale
        wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(vx, vy, 0.0)

    if -0.001 < vz < 0.001:
        return vx, vy, 0.0

    if vz > 0.0:
        remain = Follow_Pose_Wheel_Target_Limit - wheel_fr
        tmp = Follow_Pose_Wheel_Target_Limit - wheel_fl
        if tmp < remain:
            remain = tmp
        tmp = Follow_Pose_Wheel_Target_Limit - wheel_b
        if tmp < remain:
            remain = tmp
        if remain < 0.0:
            remain = 0.0
        if vz > remain:
            vz = remain
    else:
        remain = Follow_Pose_Wheel_Target_Limit + wheel_fr
        tmp = Follow_Pose_Wheel_Target_Limit + wheel_fl
        if tmp < remain:
            remain = tmp
        tmp = Follow_Pose_Wheel_Target_Limit + wheel_b
        if tmp < remain:
            remain = tmp
        if remain < 0.0:
            remain = 0.0
        if -vz > remain:
            vz = -remain

    return vx, vy, vz


def reset_turn_loop_state():
    global last_cmd_wz, last_ap_vz_cmd

    last_cmd_wz = 0.0
    last_ap_vz_cmd = 0.0
    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        gyro_pid.output = 0.0
        gyro_pid.err = 0.0
        gyro_pid.err_last = 0.0


def priority_gyro_rate_ctrl(turn_rate_cmd, gyro_z):
    err = turn_rate_cmd - gyro_z
    limit = gyro_limit_for_turn(turn_rate_cmd, True)
    turn_abs = abs(turn_rate_cmd)
    gyro_abs = abs(gyro_z)
    same_dir = (
        (turn_rate_cmd > 0.0 and gyro_z > 0.0)
        or (turn_rate_cmd < 0.0 and gyro_z < 0.0)
    )
    if same_dir and gyro_abs > turn_abs * GYRO_PRIORITY_OVERSPEED_RATIO:
        out = err * GYRO_PRIORITY_BRAKE_KP
        return clamp(out, -GYRO_PRIORITY_BRAKE_LIMIT, GYRO_PRIORITY_BRAKE_LIMIT)

    out = clamp(err * GYRO_PRIORITY_KP, -limit, limit)
    if (
        turn_abs >= GYRO_PRIORITY_MIN_CMD
        and (not same_dir or gyro_abs < turn_abs * GYRO_PRIORITY_MIN_RATE_RATIO)
    ):
        if 0.0 < out < GYRO_PRIORITY_MIN_OUTPUT:
            out = GYRO_PRIORITY_MIN_OUTPUT
        elif -GYRO_PRIORITY_MIN_OUTPUT < out < 0.0:
            out = -GYRO_PRIORITY_MIN_OUTPUT
    return clamp(out, -limit, limit)


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
                frame_len = 3
                if cam_rx_buf[1] == No_Target_Marker and cam_rx_buf[2] == No_Target_Marker:
                    cam_has_target = False
                    cam_valid_target_since_ms = 0
                    cam_last_rx_ms = utime.ticks_ms()
                elif cam_rx_buf[1] in (Line_Packet_Tag, Classify_Packet_Tag):
                    cam_last_rx_ms = utime.ticks_ms()
                else:
                    if len(cam_rx_buf) < 4:
                        return
                    if cam_rx_buf[3] == Cam_Frame_Head:
                        err_angle = 0
                    else:
                        err_angle = (int(cam_rx_buf[3]) - Cam_Error_Offset) * Cam_Error_Scale
                        frame_len = 4
                    cam_has_target = True
                    target_lost_since_ms = 0
                    if cam_valid_target_since_ms == 0:
                        cam_valid_target_since_ms = utime.ticks_ms()
                    update_cam_target(
                        (int(cam_rx_buf[1]) - Cam_Error_Offset) * Cam_Error_Scale,
                        (int(cam_rx_buf[2]) - Cam_Error_Offset) * Cam_Error_Scale,
                        err_angle,
                    )
                cam_rx_buf = cam_rx_buf[frame_len:]

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
    global last_follow_seen, last_visual_vx, last_visual_vy
    global last_ff_vx, last_ff_vy, last_ff_wz
    global last_cmd_vx, last_cmd_vy, last_cmd_wz
    global last_ap_vz_cmd
    global last_distance_lock_ms, last_angle_priority_active
    global last_back_priority_active

    now = utime.ticks_ms()
    seen = cam_target_seen()
    fresh_motion = master_motion_fresh()
    ff_vx = master_vx if fresh_motion else 0.0
    ff_vy = master_vy if fresh_motion else 0.0
    ff_wz = master_wz if fresh_motion else 0.0
    body_vx = 0.0
    body_vy = 0.0
    turn_rate_cmd = 0.0
    use_motion_feedforward = False
    angle_priority_active = False
    position_priority_active = False
    back_priority_active = False
    prev_angle_priority_active = last_angle_priority_active

    if seen:
        target_lost_since_ms = 0
        use_motion_feedforward = fresh_motion
        back_priority_active = cam_error_y < -Follow_Forward_Deadband
        if -Follow_Distance_Lock_Error <= cam_error_y <= Follow_Distance_Lock_Error:
            last_distance_lock_ms = now
        (
            vx,
            vy,
            turn_rate_cmd,
            body_vx,
            body_vy,
            angle_priority_active,
            position_priority_active,
        ) = solve_follow_pose_twist(
            cam_error_x,
            cam_error_y,
            cam_error_angle,
            ff_vx,
            ff_vy,
            ff_wz,
            use_motion_feedforward,
        )
        last_angle_priority_active = angle_priority_active
        last_back_priority_active = back_priority_active
    else:
        if target_lost_since_ms == 0:
            target_lost_since_ms = now
        if fresh_motion and utime.ticks_diff(now, target_lost_since_ms) <= Follow_Target_Lost_Hold_Ms:
            vx = ff_vx * Follow_Hold_Feedforward_Gain
            vy = ff_vy * Follow_Hold_Feedforward_Gain
            use_motion_feedforward = True
        else:
            vx = 0.0
            vy = 0.0
        if prev_angle_priority_active:
            reset_turn_loop_state()
        last_angle_priority_active = False
        last_back_priority_active = False

    if seen:
        vx = apply_distance_guard(
            vx,
            body_vx,
            cam_error_y,
            position_priority_active,
        )
        if back_priority_active:
            if Follow_Forward_Error_Sign >= 0:
                if vx > -Follow_Distance_Back_Min_Vx:
                    vx = -Follow_Distance_Back_Min_Vx
                elif vx < -Follow_Distance_Back_Max_Vx:
                    vx = -Follow_Distance_Back_Max_Vx
            else:
                if vx < Follow_Distance_Back_Min_Vx:
                    vx = Follow_Distance_Back_Min_Vx
                elif vx > Follow_Distance_Back_Max_Vx:
                    vx = Follow_Distance_Back_Max_Vx
            vy = clamp(
                vy,
                -Follow_Distance_Back_Vy_Limit,
                Follow_Distance_Back_Vy_Limit,
            )

    vx_limit = Follow_Forward_Limit
    vy_limit = Follow_Lateral_Limit
    if fresh_motion:
        vx_limit = follow_limit(vx_limit, ff_vx, Follow_Master_Extra_Vx)
        vy_limit = follow_limit(vy_limit, ff_vy, Follow_Master_Extra_Vy)
    vx = clamp(vx, -vx_limit, vx_limit)
    vy = clamp(vy, -vy_limit, vy_limit)
    if seen and cam_error_y <= Follow_Forward_Deadband and vx <= 0.0 and last_cmd_vx > 0.0:
        # Do not let the ramp coast forward through the distance stop zone.
        last_cmd_vx = 0.0
    if back_priority_active:
        if last_cmd_vx > 0.0:
            last_cmd_vx = 0.0
        last_cmd_vy = clamp(
            last_cmd_vy,
            -Follow_Distance_Back_Vy_Limit,
            Follow_Distance_Back_Vy_Limit,
        )
    if position_priority_active:
        vx_ramp = Follow_Orbit_Command_Ramp_Vx
        vy_ramp = Follow_Orbit_Command_Ramp_Vy
    else:
        vx_ramp = Follow_Command_Ramp_Vx
        vy_ramp = Follow_Command_Ramp_Vy
    vx = ramp_value(vx, last_cmd_vx, vx_ramp)
    vy = ramp_value(vy, last_cmd_vy, vy_ramp)
    last_cmd_vx = vx
    last_cmd_vy = vy
    cam_target_vx = vx
    cam_target_vy = vy

    if (not seen) and use_motion_feedforward:
        turn_rate_cmd = ff_wz * Follow_Wz_Feedforward_Gain
    if -0.001 < turn_rate_cmd < 0.001:
        reset_turn_loop_state()
        turn_rate_cmd = 0.0
    else:
        turn_rate_cmd = ramp_value(turn_rate_cmd, last_cmd_wz, Follow_Command_Ramp_Wz)
        last_cmd_wz = turn_rate_cmd
    yaw_err = 0.0
    yaw_correction = 0.0
    if Follow_Yaw_Enable and fresh_motion and ENABLE_IMU:
        yaw_ref_deg = master_yaw
        yaw_err = -wrapped_yaw_error(yaw_ref_deg, yaw_deg)
        yaw_correction = clamp(yaw_err * Follow_Yaw_Gain, -Follow_Yaw_Limit, Follow_Yaw_Limit)
        turn_rate_cmd += yaw_correction

    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        if FOLLOW_GYRO_HOLD_ENABLE or abs(turn_rate_cmd) > 0.001:
            if angle_priority_active:
                gyro_pid.gyro_kp = GYRO_PRIORITY_KP
                gyro_pid.gyro_ki = GYRO_PRIORITY_KI
                gyro_pid.gyro_output_limit = gyro_limit_for_turn(turn_rate_cmd, True)
                gyro_pid.err = turn_rate_cmd - gyro_z
                vz_cmd = priority_gyro_rate_ctrl(turn_rate_cmd, gyro_z)
                vz_cmd = ramp_value(vz_cmd, last_ap_vz_cmd, Follow_Pose_Gyro_Output_Ramp)
                last_ap_vz_cmd = vz_cmd
                gyro_pid.output = vz_cmd
                gyro_pid.err_last = gyro_pid.err
            else:
                last_ap_vz_cmd = 0.0
                gyro_pid.gyro_kp = GYRO_KP
                gyro_pid.gyro_ki = GYRO_KI
                gyro_pid.gyro_output_limit = gyro_limit_for_turn(turn_rate_cmd)
                vz_cmd = gyro_ctrl(gyro_pid, turn_rate_cmd - gyro_z)
        else:
            gyro_pid.gyro_kp = GYRO_KP
            gyro_pid.gyro_ki = GYRO_KI
            gyro_pid.output = 0.0
            gyro_pid.err = 0.0
            gyro_pid.err_last = 0.0
            vz_cmd = 0.0
            last_cmd_wz = 0.0
            last_ap_vz_cmd = 0.0
    else:
        last_ap_vz_cmd = 0.0
        vz_cmd = turn_rate_cmd

    cam_target_vx, cam_target_vy, vz_cmd = limit_pose_twist_for_wheels(
        cam_target_vx,
        cam_target_vy,
        vz_cmd,
    )
    last_cmd_vx = cam_target_vx
    last_cmd_vy = cam_target_vy
    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        gyro_pid.output = vz_cmd

    last_turn_rate_cmd = turn_rate_cmd
    last_vz_cmd = vz_cmd
    last_follow_seen = seen
    last_visual_vx = body_vx
    last_visual_vy = body_vy
    last_ff_vx = ff_vx
    last_ff_vy = ff_vy
    last_ff_wz = ff_wz
    return vz_cmd


def stop_all():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)
    log("[STOP] motors off")


def reset_speed_outputs():
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    global last_stall_count, last_stall_boost
    global last_cmd_vx, last_cmd_vy, last_cmd_wz, last_ap_vz_cmd
    global last_angle_priority_active, last_back_priority_active

    speed_reset(pid_fl)
    speed_reset(pid_fr)
    speed_reset(pid_b)
    last_pwm_fl = 0
    last_pwm_fr = 0
    last_pwm_b = 0
    last_stall_count = 0
    last_stall_boost = False
    last_cmd_vx = 0.0
    last_cmd_vy = 0.0
    last_cmd_wz = 0.0
    last_ap_vz_cmd = 0.0
    last_angle_priority_active = False
    last_back_priority_active = False


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


def apply_start_pwm(cmd, min_pwm):
    if min_pwm <= 0:
        return 0
    cmd = int(cmd)
    if cmd > 0:
        if cmd < min_pwm:
            cmd = min_pwm
    elif cmd < 0:
        if cmd > -min_pwm:
            cmd = -min_pwm
    if cmd > FOLLOW_RUN_PWM_LIMIT:
        cmd = FOLLOW_RUN_PWM_LIMIT
    elif cmd < -FOLLOW_RUN_PWM_LIMIT:
        cmd = -FOLLOW_RUN_PWM_LIMIT
    return cmd


def follow_start_pwm_for_target(target, stall_boost):
    target_abs = abs(target)
    if target_abs <= WHEEL_TARGET_IDLE_EPS:
        return 0
    if stall_boost and target_abs >= FOLLOW_STALL_BOOST_TARGET:
        return FOLLOW_STALL_BOOST_PWM
    if target_abs < FOLLOW_START_PWM_LOW_TARGET:
        return FOLLOW_START_PWM_LOW
    if target_abs < FOLLOW_START_PWM_MID_TARGET:
        return FOLLOW_START_PWM_MID
    return FOLLOW_START_PWM


def follow_channel_pwm(cmd, target, stall_boost, last_pwm):
    min_pwm = follow_start_pwm_for_target(target, stall_boost)
    if min_pwm <= 0:
        return 0
    return smooth_value(apply_start_pwm(cmd, min_pwm), last_pwm)


def set_three_pwm_follow(u_fl, u_fr, u_b, t_fl, t_fr, t_b, stall_boost):
    global last_pwm_fl, last_pwm_fr, last_pwm_b

    s_fl = follow_channel_pwm(u_fl, t_fl, stall_boost, last_pwm_fl)
    s_fr = follow_channel_pwm(u_fr, t_fr, stall_boost, last_pwm_fr)
    s_b = follow_channel_pwm(u_b, t_b, stall_boost, last_pwm_b)
    apply_motor_duty(s_fl, motor_fl)
    apply_motor_duty(s_fr, motor_fr)
    apply_motor_duty(s_b, motor_b)
    last_pwm_fl = s_fl
    last_pwm_fr = s_fr
    last_pwm_b = s_b
    return s_fl, s_fr, s_b


def set_three_pwm_zero():
    global last_pwm_fl, last_pwm_fr, last_pwm_b

    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)
    last_pwm_fl = 0
    last_pwm_fr = 0
    last_pwm_b = 0
    return 0, 0, 0


def wheel_targets_zero(t_fl, t_fr, t_b):
    return (
        -WHEEL_TARGET_STOP_EPS <= t_fl <= WHEEL_TARGET_STOP_EPS
        and -WHEEL_TARGET_STOP_EPS <= t_fr <= WHEEL_TARGET_STOP_EPS
        and -WHEEL_TARGET_STOP_EPS <= t_b <= WHEEL_TARGET_STOP_EPS
    )


def encoders_stalled(e_fl, e_fr, e_b):
    return e_fl == 0 and e_fr == 0 and e_b == 0


def wheel_target_idle(target):
    return -WHEEL_TARGET_IDLE_EPS <= target <= WHEEL_TARGET_IDLE_EPS


def speed_ctrl_follow(pid, actual_speed, target_speed):
    if wheel_target_idle(target_speed):
        speed_reset(pid)
        return 0.0
    return speed_ctrl(pid, actual_speed, target_speed)


def update_nav_led_display():
    straight_value = 1 if cam_target_seen() else 0
    translate_value = 1 if master_motion_rx_fresh() else 0
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


def wireless_tune_log(snap):
    global last_tune_log_ms

    if not FOLLOW_WIRELESS_TUNE_LOG_ENABLE or snap is None:
        return
    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_tune_log_ms) < FOLLOW_TUNE_LOG_INTERVAL_MS:
        return
    last_tune_log_ms = now
    try:
        wireless.send_str(
            "FT seen=%d err=%d,%d,%d vis=%.2f,%.2f out=%.2f,%.2f,%.2f "
            "ff=%.2f,%.2f,%.2f wz=%.2f,%.2f,%.2f "
            "tar=%.1f,%.1f,%.1f enc=%d,%d,%d pid=%d,%d,%d pwm=%d,%d,%d stop=%d boost=%d ap=%d bp=%d "
            "g=%.2f glim=%.1f yaw=%.1f mf=%d rx=%d\r\n"
            % (
                1 if last_follow_seen else 0,
                cam_error_x,
                cam_error_y,
                cam_error_angle,
                last_visual_vx,
                last_visual_vy,
                cam_target_vx,
                cam_target_vy,
                last_vz_cmd,
                last_ff_vx,
                last_ff_vy,
                last_ff_wz,
                last_ff_wz,
                last_turn_rate_cmd,
                last_vz_cmd,
                snap["tar_fl"],
                snap["tar_fr"],
                snap["tar_b"],
                int(snap["enc_fl"]),
                int(snap["enc_fr"]),
                int(snap["enc_b"]),
                int(snap["pid_fl"]),
                int(snap["pid_fr"]),
                int(snap["pid_b"]),
                int(snap["pwm_fl"]),
                int(snap["pwm_fr"]),
                int(snap["pwm_b"]),
                int(snap["hard_stop"]),
                int(snap["stall_boost"]),
                1 if last_angle_priority_active else 0,
                1 if last_back_priority_active else 0,
                snap["gyro_z"],
                snap["gyro_limit"],
                snap["yaw_deg"],
                1 if USE_MASTER_MOTION_FEEDFORWARD else 0,
                1 if master_motion_rx_fresh() else 0,
            )
        )
    except Exception:
        pass


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
    global last_hard_stop
    global last_stall_count, last_stall_boost

    if not car_started:
        reset_speed_outputs()
        set_three_pwm_zero()
        last_hard_stop = True
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

    if wheel_targets_zero(t_fl, t_fr, t_b):
        reset_speed_outputs()
        s_fl, s_fr, s_b = set_three_pwm_zero()
        u_fl = 0.0
        u_fr = 0.0
        u_b = 0.0
        last_hard_stop = True
    else:
        last_hard_stop = False
        if encoders_stalled(e_fl, e_fr, e_b):
            last_stall_count += 1
        else:
            last_stall_count = 0
        last_stall_boost = last_stall_count >= FOLLOW_STALL_BOOST_FRAMES

        u_fl = speed_ctrl_follow(pid_fl, e_fl, t_fl)
        u_fr = speed_ctrl_follow(pid_fr, e_fr, t_fr)
        u_b = speed_ctrl_follow(pid_b, e_b, t_b)

        if FORCE_MOTOR_OFF:
            reset_speed_outputs()
            s_fl, s_fr, s_b = set_three_pwm_zero()
            last_hard_stop = True
        else:
            s_fl, s_fr, s_b = set_three_pwm_follow(
                u_fl, u_fr, u_b, t_fl, t_fr, t_b, last_stall_boost
            )

    return {
        "enc_fl": e_fl,
        "enc_fr": e_fr,
        "enc_b": e_b,
        "tar_fl": t_fl,
        "tar_fr": t_fr,
        "tar_b": t_b,
        "pid_fl": u_fl,
        "pid_fr": u_fr,
        "pid_b": u_b,
        "pwm_fl": s_fl,
        "pwm_fr": s_fr,
        "pwm_b": s_b,
        "hard_stop": 1 if last_hard_stop else 0,
        "stall_boost": 1 if last_stall_boost else 0,
        "raw_gyro_z": raw_gyro_z,
        "gyro_z": gyro_z,
        "gyro_limit": gyro_pid.gyro_output_limit if gyro_pid is not None else 0.0,
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
        check_c8_exit()
        check_c9_start()
        poll_art_uart()
        poll_coop_uart()
        update_nav_led_display()

        if AUTO_START_ON_BOOT and (not auto_start_done):
            if utime.ticks_diff(now, start_time) >= AUTO_START_DELAY_MS:
                start_follow("auto")
        if (not car_started) and cam_target_seen():
            start_follow("signal")

        snap = None
        if pit_flag:
            pit_flag = False
            snap = calc_speed_closed_loop()
            if FOLLOW_WIRELESS_TUNE_LOG_ENABLE:
                wireless_tune_log(snap)

        if utime.ticks_diff(now, last_status_ms) >= 1000:
            led.toggle()
            last_status_ms = now

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
