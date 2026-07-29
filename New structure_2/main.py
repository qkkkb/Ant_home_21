from machine import Pin, UART
import gc
import utime
from smartcar import ticker, encoder
from seekfree import WIRELESS_UART
from lsm6dsv16x_gyro_runtime import LSM6DSV16XYawRuntime
from models import AnglePID, MoveBase, SpeedPID
from move_base import calc_wheel_spd
import pid as _pid_mod
import config as cfg
from hardware import Motor
from coop_protocol import (
    CoopFrameParser,
    MASTER_MOTION_FLAG_BACK,
    MASTER_MOTION_FLAG_ORBIT,
    MASTER_MOTION_FLAG_PUSH,
    MASTER_MOTION_FLAG_SPIN,
    MSG_MASTER_MOTION,
    decode_i16,
)


speed_ctrl = _pid_mod.speed_ctrl
gyro_ctrl = _pid_mod.gyro_ctrl
speed_reset = _pid_mod.speed_reset
speed_follow_guard = _pid_mod.speed_follow_guard
follow_low_pwm = _pid_mod.follow_low_pwm


# ====================== Base config ======================
TICK_PERIOD_MS = cfg.TICK_PERIOD_MS
MOTOR_DUTY_MAX = cfg.MOTOR_DUTY_MAX
MOTOR_DUTY_MIN = cfg.MOTOR_DUTY_MIN
PWM_SMOOTH_FACTOR = cfg.PWM_SMOOTH_FACTOR
MAX_PWM_CHANGE = cfg.MAX_PWM_CHANGE

ENABLE_GYRO_LOOP = True
ENABLE_IMU = ENABLE_GYRO_LOOP
GYRO_SIGN = 1.0
GYRO_OFFSET_Z = 0.0
GYRO_SCALE = -1.0
GYRO_DEADBAND_DPS = 0.8
GYRO_KP = 0.16
GYRO_KI = 0.0005
GYRO_PRIORITY_KP = 0.46
GYRO_PRIORITY_KI = 0.0
GYRO_OUTPUT_LIMIT = 14.0
GYRO_OUTPUT_BASE_LIMIT = 4.5
GYRO_OUTPUT_TARGET_GAIN = 2.2
GYRO_OUTPUT_MAX_LIMIT = 22.0
GYRO_PRIORITY_OUTPUT_BASE_LIMIT = 6.5
GYRO_PRIORITY_OUTPUT_TARGET_GAIN = 0.90
GYRO_PRIORITY_OUTPUT_MAX_LIMIT = 32.0
GYRO_PRIORITY_MIN_OUTPUT = 2.4
GYRO_PRIORITY_MIN_RATE_RATIO = 0.45
GYRO_PRIORITY_MIN_CMD = 6.5
GYRO_PRIORITY_OVERSPEED_RATIO = 2.25
GYRO_PRIORITY_BRAKE_KP = 0.16
GYRO_PRIORITY_BRAKE_LIMIT = 8.0
GYRO_SPIN_PRIORITY_OUTPUT_BASE_LIMIT = 10.0
GYRO_SPIN_PRIORITY_OUTPUT_TARGET_GAIN = 0.45
GYRO_SPIN_PRIORITY_OUTPUT_MAX_LIMIT = 24.0
GYRO_SPIN_PRIORITY_OVERSPEED_RATIO = 1.25
AUTO_CALIBRATE_GYRO_ON_LAUNCH = True
GYRO_CALIBRATE_SAMPLES = 1000
GYRO_CALIBRATE_DELAY_MS = 2

GC_DIV = 50
DEBUG_LOG_PERIOD_MS = 100
WHEEL_TARGET_STOP_EPS = 0.05
WHEEL_TARGET_IDLE_EPS = 1.2
WHEEL_TARGET_NORMAL_IDLE_EPS = 0.35
FOLLOW_STATIC_LOCK_PWM_LIMIT = 12000
FOLLOW_RUN_PWM_LIMIT = 50000
_pid_mod.PWM_MAX = FOLLOW_RUN_PWM_LIMIT


# ====================== Camera protocol ======================
Cam_Error_Offset = 120
Cam_Error_Scale = 2
Cam_Packet_Timeout_Ms = 300
Cam_Frame_Head = 0xFF
No_Target_Marker = 0xFE
Line_Packet_Tag = 0xFC
Classify_Packet_Tag = 0xFD
ART_MODE_TRACK_CMD = b"TRACK\n"
ART_MODE_IDLE_CMD = b"IDLE\n"


# ====================== Follow control ======================
Follow_Forward_Gain = 0.46
Follow_Lateral_Gain = 0.92
Follow_Orbit_Forward_Gain = 1.00
Follow_Orbit_Lateral_Gain = 0.92
Follow_Forward_Limit = 38.0
Follow_Lateral_Limit = 30.0
Follow_Forward_Deadband = 2
Follow_Lateral_Deadband = 2
Follow_Orbit_Forward_Deadband = 2
Follow_Orbit_Lateral_Deadband = 2
Follow_Orbit_Position_X_Error = 4
Follow_Orbit_Position_Y_Error = 4
Follow_Distance_Far_Boost_Error = 6
Follow_Distance_Far_Boost_Gain = 0.70
Follow_Distance_Close_Gain = 0.70
Follow_Distance_Close_Limit = 22.0
Follow_Feedforward_Forward_Gain = 1.00
Follow_Feedforward_Lateral_Gain = 1.60
Follow_Feedforward_Forward_Limit = 35.0
Follow_Feedforward_Lateral_Limit = 31.0
Follow_Push_Feedforward_Forward_Gain = 1.25
Follow_Push_Feedforward_Forward_Limit = 30.0
Follow_Normal_Visual_Forward_Scale = 0.72
Follow_Static_Visual_Scale = 0.60
Follow_Close_Guard_Full_Error = 8.0
Follow_Close_Feedforward_Min_Scale = 0.25
Follow_Normal_Hold_Feedforward_Gain = 1.15
Follow_Hold_Feedforward_Gain = 1.70
Follow_Normal_Wz_Feedforward_Gain = 1.00
Follow_Orbit_Wz_Feedforward_Gain = 1.00
Follow_Orbit_Wz_Feedforward_Limit = 128.0
Follow_Orbit_Turn_Rate_Limit = 128.0
Follow_Target_Point_Wz_To_Vx = -0.18
Follow_Target_Point_Wz_To_Vy = -0.45
Follow_Orbit_Target_Point_Wz_To_Vx = -0.17
Follow_Orbit_Target_Point_Wz_To_Vy = -0.28
Follow_Orbit_Feedforward_Forward_Gain = 1.30
Follow_Orbit_Feedforward_Lateral_Gain = 0.76
Follow_Orbit_Feedforward_Forward_Limit = 22.0
Follow_Orbit_Feedforward_Lateral_Limit = 20.0
Follow_Orbit_Feedforward_Close_Error = 6
Follow_Orbit_Feedforward_Full_Error = 22
Follow_Orbit_Feedforward_Close_Scale = 0.78
Follow_Orbit_Close_Feedforward_Full_Error = 6
Follow_Pose_Angle_Gain = -0.68
Follow_Pose_Angle_Limit = 40.0
Follow_Orbit_Pose_Angle_Gain = -1.35
Follow_Orbit_Pose_Angle_Limit = 74.0
Follow_Orbit_Pose_Angle_Min_Error = 14
Follow_Orbit_Pose_Angle_Min_Turn = 30.0
Follow_Normal_Pose_Angle_Deadband = 8
Follow_Normal_Pose_Angle_Active_Error = 18
Follow_Spin_Target_Point_Wz_To_Vy = -0.08
Follow_Spin_Wz_Feedforward_Gain = 0.85
Follow_Spin_Wz_Feedforward_Limit = 100.0
Follow_Spin_Turn_Rate_Limit = 128.0
Follow_Pose_Angle_Deadband = 4
Follow_Pose_Angle_Active_Error = 6
Follow_Angle_XY_Mode_On_Error = 10
Follow_Angle_XY_Mode_Full_Error = 42
Follow_Angle_XY_Min_Scale = 0.38
Follow_Spin_XY_Min_Scale = 0.94
Follow_Orbit_XY_Max_Scale = 0.78
Follow_Spin_XY_Max_Scale = 1.00
Follow_Pose_Wheel_Target_Limit = 45.0
Follow_Command_Ramp_Vx = 2.0
Follow_Command_Ramp_Vy = 3.0
Follow_Orbit_Command_Ramp_Vx = 20.0
Follow_Orbit_Command_Ramp_Vy = 14.0
Follow_Normal_Target_Lost_Hold_Ms = 1000
Follow_Target_Lost_Hold_Ms = 250
Follow_Orbit_Mode_FfWz_Off = 10.0
Follow_Orbit_Mode_Exit_Ms = 200
Follow_Orbit_Mode_FfWz_Filter = 0.22
Follow_Normal_Wz_Feedforward_Limit = 15.0
Follow_Spin_Latch_Min_Wz = 26.0
Follow_Spin_Command_Hold_Ms = 1100
Follow_Spin_Latch_Release_Angle = 3
Follow_Orbit_Brake_Gyro_Threshold = 4.0
Follow_Orbit_Brake_Output_Limit = 8.0
Follow_Normal_Brake_Wheel_Reserve = 1.5
Master_Motion_Timeout_Ms = 250
Follow_Master_Edge_Delta = 4.0
Follow_Master_Edge_Hold_Ms = 120
# ====================== Runtime state ======================
car_started = False
last_c9_state = 1
last_c8_state = 1

cam_error_x = 0
cam_error_y = 0
cam_error_angle = 0
cam_target_vx = 0.0
cam_target_vy = 0.0
cam_last_rx_ms = 0
cam_has_target = False
target_lost_since_ms = 0
cam_uart_buf = None
cam_parse_state = 0
cam_parse_b1 = 0
cam_parse_b2 = 0

master_vx = 0.0
master_vy = 0.0
master_wz = 0.0
master_flags = 0
master_last_rx_ms = 0

coop_parser = CoopFrameParser()
coop_rx_led_until_ms = 0

pit_flag = False
start_time = utime.ticks_ms()
last_status_ms = start_time
loop_count = 0
debug_log_last_ms = start_time
last_pwm_fl = 0
last_pwm_fr = 0
last_pwm_b = 0
last_turn_rate_cmd = 0.0
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
last_angle_priority_active = False
orbit_follow_active = False
orbit_follow_exit_since_ms = 0
filtered_ff_wz = 0.0
spin_latched_wz = 0.0
spin_latch_until_ms = 0
push_yaw_target = None
last_follow_mode_key = -1
master_edge_until_ms = 0
last_hard_stop = False
last_stall_count = 0
last_stall_boost = False
follow_output_limit = FOLLOW_RUN_PWM_LIMIT


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


def soft_deadband(value, deadband, full_error):
    value_abs = abs(value)
    if value_abs <= deadband:
        return 0.0
    if value_abs < full_error:
        value_abs = (value_abs - deadband) * full_error / (full_error - deadband)
    return value_abs if value > 0.0 else -value_abs


def angle_pose_mode_needed(error_angle, orbit_mode=False, spin_mode=False):
    if orbit_mode or spin_mode:
        return True
    if last_angle_priority_active:
        return (
            error_angle > Follow_Normal_Pose_Angle_Deadband
            or error_angle < -Follow_Normal_Pose_Angle_Deadband
        )
    return (
        error_angle >= Follow_Normal_Pose_Angle_Active_Error
        or error_angle <= -Follow_Normal_Pose_Angle_Active_Error
    )


def angle_xy_lock_scale(error_angle, orbit_mode=False, spin_mode=False):
    angle_abs = abs(error_angle)
    if angle_abs <= Follow_Angle_XY_Mode_On_Error:
        scale = 1.0
    elif angle_abs >= Follow_Angle_XY_Mode_Full_Error:
        scale = Follow_Angle_XY_Min_Scale
    else:
        span = Follow_Angle_XY_Mode_Full_Error - Follow_Angle_XY_Mode_On_Error
        scale = 1.0 - (
            (angle_abs - Follow_Angle_XY_Mode_On_Error)
            * (1.0 - Follow_Angle_XY_Min_Scale)
            / span
        )
    if spin_mode:
        if scale < Follow_Spin_XY_Min_Scale:
            scale = Follow_Spin_XY_Min_Scale
        if scale > Follow_Spin_XY_Max_Scale:
            scale = Follow_Spin_XY_Max_Scale
    elif orbit_mode and scale > Follow_Orbit_XY_Max_Scale:
        scale = Follow_Orbit_XY_Max_Scale
    return scale


def orbit_feedforward_position_scale(error_x, error_y):
    err_abs = error_x if error_x >= 0 else -error_x
    tmp = error_y if error_y >= 0 else -error_y
    if tmp > err_abs:
        err_abs = tmp
    if err_abs <= Follow_Orbit_Feedforward_Close_Error:
        scale = Follow_Orbit_Feedforward_Close_Scale
    elif err_abs >= Follow_Orbit_Feedforward_Full_Error:
        scale = 1.0
    else:
        span = Follow_Orbit_Feedforward_Full_Error - Follow_Orbit_Feedforward_Close_Error
        scale = Follow_Orbit_Feedforward_Close_Scale + (
            (err_abs - Follow_Orbit_Feedforward_Close_Error)
            * (1.0 - Follow_Orbit_Feedforward_Close_Scale)
            / span
        )
    depth = orbit_close_depth(error_y)
    if depth <= 0.0:
        return scale
    if depth >= Follow_Orbit_Close_Feedforward_Full_Error:
        return Follow_Close_Feedforward_Min_Scale
    close_scale = 1.0 - (
        (1.0 - Follow_Close_Feedforward_Min_Scale)
        * depth
        / Follow_Orbit_Close_Feedforward_Full_Error
    )
    if close_scale < scale:
        return close_scale
    return scale


def orbit_close_depth(error_y):
    depth = error_y - Follow_Forward_Deadband
    if depth < 0:
        return 0.0
    return depth


def gyro_limit_for_turn(turn_rate_cmd, priority=False, spin_priority=False):
    if spin_priority:
        base_limit = GYRO_SPIN_PRIORITY_OUTPUT_BASE_LIMIT
        target_gain = GYRO_SPIN_PRIORITY_OUTPUT_TARGET_GAIN
        max_limit = GYRO_SPIN_PRIORITY_OUTPUT_MAX_LIMIT
    elif priority:
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


def update_cam_target(err_x, err_y, err_angle=0):
    global cam_error_x, cam_error_y, cam_error_angle, cam_last_rx_ms

    err_x = int(err_x)
    err_y = int(err_y)
    err_angle = int(err_angle)
    cam_error_x = err_x
    cam_error_y = err_y
    cam_error_angle = err_angle
    cam_last_rx_ms = utime.ticks_ms()


def clear_cam_target_state():
    global cam_error_x, cam_error_y, cam_last_rx_ms, cam_parse_state
    global cam_error_angle
    global cam_has_target, target_lost_since_ms
    global last_cmd_vx, last_cmd_vy, last_cmd_wz, last_ap_vz_cmd
    global last_angle_priority_active
    global orbit_follow_active, orbit_follow_exit_since_ms, filtered_ff_wz
    global spin_latched_wz, spin_latch_until_ms
    global push_yaw_target
    global last_follow_mode_key
    global master_edge_until_ms

    cam_error_x = 0
    cam_error_y = 0
    cam_error_angle = 0
    last_cmd_vx = 0.0
    last_cmd_vy = 0.0
    last_cmd_wz = 0.0
    last_ap_vz_cmd = 0.0
    last_angle_priority_active = False
    orbit_follow_active = False
    orbit_follow_exit_since_ms = 0
    filtered_ff_wz = 0.0
    spin_latched_wz = 0.0
    spin_latch_until_ms = 0
    push_yaw_target = None
    last_follow_mode_key = -1
    master_edge_until_ms = 0
    cam_has_target = False
    target_lost_since_ms = 0
    cam_last_rx_ms = 0
    cam_parse_state = 0


def cam_target_seen():
    return cam_has_target and utime.ticks_diff(utime.ticks_ms(), cam_last_rx_ms) <= Cam_Packet_Timeout_Ms


def master_motion_fresh():
    return utime.ticks_diff(utime.ticks_ms(), master_last_rx_ms) <= Master_Motion_Timeout_Ms


def update_spin_feedforward_latch(now, fresh_motion, explicit_spin, ff_wz, seen, error_angle):
    global spin_latched_wz, spin_latch_until_ms

    if explicit_spin and fresh_motion and (
        ff_wz >= Follow_Spin_Latch_Min_Wz
        or ff_wz <= -Follow_Spin_Latch_Min_Wz
    ):
        if last_follow_mode_key == 0:
            reset_speed_outputs(True)
        spin_latched_wz = ff_wz
        spin_latch_until_ms = utime.ticks_add(now, Follow_Spin_Command_Hold_Ms)
        return ff_wz

    if spin_latched_wz != 0.0:
        if (
            seen
            and -10 < cam_error_x < 10
            and -10 < cam_error_y < 10
            and -Follow_Spin_Latch_Release_Angle
            <= error_angle
            <= Follow_Spin_Latch_Release_Angle
        ):
            spin_latched_wz = 0.0
            spin_latch_until_ms = 0
        elif utime.ticks_diff(spin_latch_until_ms, now) > 0:
            return spin_latched_wz
        else:
            spin_latched_wz = 0.0
            spin_latch_until_ms = 0

    if not explicit_spin:
        spin_latched_wz = 0.0
        spin_latch_until_ms = 0
    return ff_wz


def update_filtered_ff_wz(ff_wz, fresh_motion):
    global filtered_ff_wz

    target = ff_wz if fresh_motion else 0.0
    filtered_ff_wz += (target - filtered_ff_wz) * Follow_Orbit_Mode_FfWz_Filter
    if -0.05 < filtered_ff_wz < 0.05:
        filtered_ff_wz = 0.0
    return filtered_ff_wz


def update_orbit_follow_mode(
    now,
    ff_wz,
    gyro_z,
    explicit_orbit,
    explicit_push,
    explicit_spin,
    explicit_back,
):
    global orbit_follow_active, orbit_follow_exit_since_ms

    if explicit_spin or explicit_back:
        orbit_follow_active = False
        orbit_follow_exit_since_ms = 0
        return False
    if explicit_push:
        orbit_follow_active = True
        orbit_follow_exit_since_ms = 0
        return True

    if explicit_orbit:
        orbit_follow_active = True
        orbit_follow_exit_since_ms = 0
        return True

    if not orbit_follow_active:
        orbit_follow_exit_since_ms = 0
        return False

    if (
        ff_wz > Follow_Orbit_Mode_FfWz_Off
        or ff_wz < -Follow_Orbit_Mode_FfWz_Off
        or gyro_z > 8
        or gyro_z < -8
        or not cam_target_seen()
        or cam_error_angle > 8
        or cam_error_angle < -8
        or cam_error_y > 8
        or cam_error_y < -8
        or cam_error_x + cam_error_angle > 10
        or cam_error_x + cam_error_angle < -10
    ):
        orbit_follow_exit_since_ms = 0
    elif orbit_follow_exit_since_ms == 0:
        orbit_follow_exit_since_ms = now
    elif utime.ticks_diff(now, orbit_follow_exit_since_ms) >= Follow_Orbit_Mode_Exit_Ms:
        orbit_follow_active = False
        orbit_follow_exit_since_ms = 0
    return orbit_follow_active


def follow_limit(base_limit, master_value):
    limit = base_limit
    master_abs = abs(master_value)
    if master_abs > limit:
        limit = master_abs
    return limit


def calc_follow_forward(error_y, position_priority=False):
    deadband = Follow_Orbit_Forward_Deadband if position_priority else Follow_Forward_Deadband
    if follow_output_limit == FOLLOW_STATIC_LOCK_PWM_LIMIT:
        deadband = 1
    error_y = soft_deadband(error_y, deadband, deadband * 2)
    if error_y == 0.0:
        return 0.0

    if error_y > 0:
        gain = Follow_Orbit_Forward_Gain if position_priority else Follow_Forward_Gain
        out = error_y * gain
        if error_y > Follow_Distance_Far_Boost_Error:
            out += (
                error_y - Follow_Distance_Far_Boost_Error
            ) * Follow_Distance_Far_Boost_Gain
        return clamp(out, 0.0, Follow_Forward_Limit)

    out = error_y * Follow_Distance_Close_Gain
    return clamp(out, -Follow_Distance_Close_Limit, 0.0)


def calc_follow_lateral(error_x, position_priority=False):
    deadband = Follow_Orbit_Lateral_Deadband if position_priority else Follow_Lateral_Deadband
    if follow_output_limit == FOLLOW_STATIC_LOCK_PWM_LIMIT:
        deadband = 1
    error_x = soft_deadband(error_x, deadband, deadband * 2)
    if error_x == 0.0:
        return 0.0
    gain = Follow_Orbit_Lateral_Gain if position_priority else Follow_Lateral_Gain
    out = error_x * gain
    return clamp(out, -Follow_Lateral_Limit, Follow_Lateral_Limit)


def calc_follow_angle(error_angle, orbit_mode=False, spin_mode=False):
    if orbit_mode or spin_mode:
        deadband = Follow_Pose_Angle_Deadband
        full_error = Follow_Pose_Angle_Active_Error
    else:
        deadband = (
            10
            if follow_output_limit < FOLLOW_RUN_PWM_LIMIT
            and -0.001 < last_turn_rate_cmd < 0.001
            else Follow_Normal_Pose_Angle_Deadband
        )
        full_error = Follow_Normal_Pose_Angle_Active_Error
    error_angle = soft_deadband(error_angle, deadband, full_error)
    if error_angle == 0.0:
        return 0.0
    if orbit_mode:
        out = clamp(
            error_angle * Follow_Orbit_Pose_Angle_Gain,
            -Follow_Orbit_Pose_Angle_Limit,
            Follow_Orbit_Pose_Angle_Limit,
        )
        if (
            error_angle >= Follow_Orbit_Pose_Angle_Min_Error
            or error_angle <= -Follow_Orbit_Pose_Angle_Min_Error
        ):
            if 0.0 < out < Follow_Orbit_Pose_Angle_Min_Turn:
                out = Follow_Orbit_Pose_Angle_Min_Turn
            elif -Follow_Orbit_Pose_Angle_Min_Turn < out < 0.0:
                out = -Follow_Orbit_Pose_Angle_Min_Turn
        return out
    if spin_mode:
        error_angle *= 0.35 if last_ff_wz else 0.60
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


def add_feedforward_direct(base, feedforward, gain, limit, conflict_scale=1.0):
    assist = clamp(feedforward * gain, -limit, limit)
    if base * assist < -0.001 and conflict_scale < 1.0:
        if (
            not master_edge_until_ms
            or last_follow_mode_key != 0
            or conflict_scale <= 0.0
        ):
            assist *= conflict_scale
    return base + assist


def solve_follow_pose_twist(
    error_x,
    error_y,
    error_angle,
    ff_vx,
    ff_vy,
    ff_wz,
    use_ff,
    orbit_mode,
    spin_mode,
    push_mode,
):
    vision_wz = (
        0.0
        if push_mode
        else calc_follow_angle(error_angle, orbit_mode, spin_mode)
    )
    active_error = (
        Follow_Pose_Angle_Active_Error
        if (orbit_mode or spin_mode)
        else Follow_Normal_Pose_Angle_Active_Error
    )
    angle_active = (
        error_angle >= active_error
        or error_angle <= -active_error
    )
    # Local pose features around the calibrated nonparallel formation.
    cam_vx = error_x if push_mode else error_x + error_angle
    cam_vy = error_y
    if cam_vx > 0.0:
        cam_vy -= cam_vx * 5 // 13
    if orbit_mode or spin_mode:
        position_priority = (
            (angle_active and (not spin_mode))
            or cam_vy >= Follow_Orbit_Position_X_Error
            or cam_vy <= -Follow_Orbit_Position_X_Error
            or cam_vx >= Follow_Orbit_Position_Y_Error
            or cam_vx <= -Follow_Orbit_Position_Y_Error
        )
    else:
        position_priority = False
    body_vx = -calc_follow_forward(cam_vx, position_priority)
    if push_mode and -8.0 < cam_vx < 8.0:
        body_vx *= 0.55
    body_vy = calc_follow_lateral(cam_vy, position_priority)
    if (
        push_mode
        and error_x < 40
        and (error_x <= -40 or abs(error_y) >= 8)
    ):
        body_vx *= 0.65 if error_x <= -40 else 0.45
    if (not orbit_mode) and (not spin_mode):
        body_vx *= Follow_Normal_Visual_Forward_Scale
    if use_ff and not spin_mode and ff_vx == 0.0 and ff_vy == 0.0:
        if (not orbit_mode) and (cam_vy >= 6.0 or cam_vy <= -6.0):
            body_vx = 0.0
        elif -8.0 < body_vx < 8.0:
            body_vx *= 1.5
        if -8.0 < body_vy < 8.0:
            body_vy *= 1.5
    vx = body_vx
    vy = body_vy
    wz = vision_wz
    if use_ff:
        if orbit_mode:
            ff_scale = orbit_feedforward_position_scale(cam_vy, cam_vx)
            target_ff_vx = ff_vx + ff_wz * Follow_Orbit_Target_Point_Wz_To_Vx
            target_ff_vy = ff_vy + ff_wz * Follow_Orbit_Target_Point_Wz_To_Vy
            target_ff_vx *= ff_scale
            target_ff_vy *= ff_scale
            vx = add_feedforward_direct(
                vx,
                target_ff_vx,
                Follow_Orbit_Feedforward_Forward_Gain,
                Follow_Orbit_Feedforward_Forward_Limit,
                Follow_Close_Feedforward_Min_Scale,
            )
            vy = add_feedforward_direct(
                vy,
                target_ff_vy,
                Follow_Orbit_Feedforward_Lateral_Gain,
                Follow_Orbit_Feedforward_Lateral_Limit,
                Follow_Close_Feedforward_Min_Scale,
            )
            wz = add_feedforward_direct(
                wz,
                ff_wz,
                Follow_Orbit_Wz_Feedforward_Gain,
                Follow_Orbit_Wz_Feedforward_Limit,
            )
        elif spin_mode:
            target_ff_vx = ff_vx + ff_wz * 0.08
            target_ff_vy = ff_vy + ff_wz * Follow_Spin_Target_Point_Wz_To_Vy
            vx = add_feedforward_direct(
                target_ff_vx,
                vx,
                1.0,
                Follow_Forward_Limit,
                0.15,
            )
            vy = add_feedforward_direct(
                target_ff_vy * 1.05,
                vy,
                1.0,
                Follow_Lateral_Limit,
                0.15,
            )
            wz = add_feedforward_direct(
                wz,
                ff_wz,
                Follow_Spin_Wz_Feedforward_Gain,
                Follow_Spin_Wz_Feedforward_Limit,
            )
        else:
            target_ff_vx = ff_vx + ff_wz * Follow_Target_Point_Wz_To_Vx
            target_ff_vy = ff_vy + ff_wz * Follow_Target_Point_Wz_To_Vy
            if master_flags & MASTER_MOTION_FLAG_BACK:
                target_ff_vx *= 0.80
                target_ff_vy *= 0.80
                ff_scale = 0.0
            else:
                ff_scale = (
                    Follow_Close_Feedforward_Min_Scale
                    + (1.0 - Follow_Close_Feedforward_Min_Scale)
                    * clamp(
                        (Follow_Close_Guard_Full_Error - cam_vx)
                        / Follow_Close_Guard_Full_Error,
                        0.0,
                        1.0,
                    )
                )
            vx = add_feedforward_direct(
                vx,
                target_ff_vx,
                Follow_Feedforward_Forward_Gain,
                Follow_Feedforward_Forward_Limit,
                ff_scale,
            )
            vy = add_feedforward_direct(
                target_ff_vy * Follow_Feedforward_Lateral_Gain,
                vy,
                1.0,
                Follow_Feedforward_Lateral_Limit,
                0.60,
            )
            wz = add_feedforward_assist(
                wz,
                ff_wz,
                Follow_Normal_Wz_Feedforward_Gain,
                Follow_Normal_Wz_Feedforward_Limit,
            )
    if orbit_mode and Follow_Orbit_Turn_Rate_Limit > 0.0:
        wz = clamp(
            wz,
            -Follow_Orbit_Turn_Rate_Limit,
            Follow_Orbit_Turn_Rate_Limit,
        )
    elif spin_mode and Follow_Spin_Turn_Rate_Limit > 0.0:
        wz = clamp(
            wz,
            -Follow_Spin_Turn_Rate_Limit,
            Follow_Spin_Turn_Rate_Limit,
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


def limit_pose_twist_for_wheels(
    vx,
    vy,
    vz,
    preserve_pose_ratio,
    preserve_turn,
    preserve_vy,
):
    if Follow_Pose_Wheel_Target_Limit <= 0.0:
        return vx, vy, vz

    if preserve_turn:
        limit = Follow_Pose_Wheel_Target_Limit
        if vz > limit:
            vz = limit
        elif vz < -limit:
            vz = -limit
        wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(vx, vy, 0.0)
        wheel_max = wheel_fr
        wheel_min = wheel_fr
        if wheel_fl > wheel_max:
            wheel_max = wheel_fl
        if wheel_fl < wheel_min:
            wheel_min = wheel_fl
        if wheel_b > wheel_max:
            wheel_max = wheel_b
        if wheel_b < wheel_min:
            wheel_min = wheel_b
        scale = 1.0
        if wheel_max > 0.001:
            scale = (limit - vz) / wheel_max
        if wheel_min < -0.001:
            tmp = (-limit - vz) / wheel_min
            if tmp < scale:
                scale = tmp
        if scale < 0.0:
            scale = 0.0
        if scale < 1.0:
            vx *= scale
            vy *= scale
        return vx, vy, vz

    if preserve_pose_ratio:
        wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(vx, vy, vz)
        target_max = max_wheel_abs(wheel_fr, wheel_fl, wheel_b)
        if target_max > Follow_Pose_Wheel_Target_Limit:
            scale = Follow_Pose_Wheel_Target_Limit / target_max
            vx *= scale
            vy *= scale
            vz *= scale
        return vx, vy, vz

    if preserve_vy:
        # Keep translation near 37 units so PUSH yaw retains wheel headroom.
        pos_max = 43.0 - abs(vy) * 0.57735
        vx = clamp(vx, -pos_max, pos_max)

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


def priority_gyro_rate_ctrl(turn_rate_cmd, gyro_z, spin_priority=False):
    err = turn_rate_cmd - gyro_z
    limit = gyro_limit_for_turn(turn_rate_cmd, True, spin_priority)
    turn_abs = abs(turn_rate_cmd)
    gyro_abs = abs(gyro_z)
    if spin_priority:
        overspeed_ratio = GYRO_SPIN_PRIORITY_OVERSPEED_RATIO
    elif last_follow_mode_key == 1:
        overspeed_ratio = 1.0
    else:
        overspeed_ratio = GYRO_PRIORITY_OVERSPEED_RATIO
    same_dir = (
        (turn_rate_cmd > 0.0 and gyro_z > 0.0)
        or (turn_rate_cmd < 0.0 and gyro_z < 0.0)
    )
    if same_dir and gyro_abs > turn_abs * overspeed_ratio:
        brake_gain = GYRO_PRIORITY_BRAKE_KP
        if not spin_priority and last_follow_mode_key != 1:
            brake_gain = GYRO_KP
        out = err * brake_gain
        return clamp(out, -GYRO_PRIORITY_BRAKE_LIMIT, GYRO_PRIORITY_BRAKE_LIMIT)

    gain = GYRO_PRIORITY_KP
    min_output = GYRO_PRIORITY_MIN_OUTPUT
    if not spin_priority and last_follow_mode_key != 1:
        gain = GYRO_KP
        min_output = 0.8
    out = clamp(err * gain, -limit, limit)
    if (
        turn_abs >= GYRO_PRIORITY_MIN_CMD
        and (not same_dir or gyro_abs < turn_abs * GYRO_PRIORITY_MIN_RATE_RATIO)
    ):
        if 0.0 < out < min_output:
            out = min_output
        elif -min_output < out < 0.0:
            out = -min_output
    return clamp(out, -limit, limit)


def poll_art_uart():
    global cam_parse_state, cam_parse_b1, cam_parse_b2
    global cam_has_target
    global cam_last_rx_ms, target_lost_since_ms

    pending = cam_uart.any()
    if not pending:
        return
    if pending > 32:
        pending = 32
    n = cam_uart.readinto(cam_uart_buf, pending)
    if not n:
        return

    i = 0
    while i < n:
        b = cam_uart_buf[i]
        i += 1

        if cam_parse_state == 0:
            if b == Cam_Frame_Head:
                cam_parse_state = 1
        elif cam_parse_state == 1:
            if b != Cam_Frame_Head:
                cam_parse_b1 = b
                cam_parse_state = 2
        elif cam_parse_state == 2:
            cam_parse_b2 = b
            if cam_parse_b1 == No_Target_Marker and b == No_Target_Marker:
                cam_has_target = False
                cam_last_rx_ms = utime.ticks_ms()
                cam_parse_state = 0
            elif cam_parse_b1 == Line_Packet_Tag or cam_parse_b1 == Classify_Packet_Tag:
                cam_last_rx_ms = utime.ticks_ms()
                cam_parse_state = 0
            else:
                cam_parse_state = 3
        else:
            if b == Cam_Frame_Head:
                err_angle = 0
                cam_parse_state = 1
            else:
                err_angle = (b - Cam_Error_Offset) * Cam_Error_Scale
                cam_parse_state = 0
            cam_has_target = True
            target_lost_since_ms = 0
            update_cam_target(
                (cam_parse_b1 - Cam_Error_Offset) * Cam_Error_Scale,
                (cam_parse_b2 - Cam_Error_Offset) * Cam_Error_Scale,
                err_angle,
            )


def handle_coop_frame(msg_type, seq, payload, payload_len):
    global master_vx, master_vy, master_wz
    global master_flags, master_last_rx_ms

    if msg_type != MSG_MASTER_MOTION or payload_len < 9:
        return
    master_vx = decode_i16(payload, 0) / 10.0
    master_vy = decode_i16(payload, 2) / 10.0
    master_wz = decode_i16(payload, 4) / 10.0
    master_flags = payload[8]
    master_last_rx_ms = utime.ticks_ms()
    coop_flash_rx()


def poll_coop_uart():
    try:
        n = wireless.receive_bytearray(coop_rx_buf, len(coop_rx_buf))
        if n:
            coop_parser.feed(coop_rx_buf, n, handle_coop_frame)
    except Exception:
        pass


def debug_due(now):
    global debug_log_last_ms
    if utime.ticks_diff(now, debug_log_last_ms) < DEBUG_LOG_PERIOD_MS:
        return False
    debug_log_last_ms = now
    return True


def debug_send(text):
    try:
        wireless.send_str(text)
        wireless.send_str("\r\n")
    except Exception:
        pass


def debug_send_idle(log_id):
    debug_send(
        "I %d %d %d %d %d %d %d"
        % (
            log_id,
            1 if cam_target_seen() else 0,
            1 if master_motion_fresh() else 0,
            master_flags,
            cam_error_x,
            cam_error_y,
            cam_error_angle,
        )
    )


def debug_send_state(log_id, vz_cmd, gyro_z):
    debug_send(
        "S %d %d %d %d %d %d %d %d %d %d %d %d %d %d"
        % (
            log_id,
            master_flags,
            1 if last_follow_seen else 0,
            1 if last_hard_stop else 0,
            1 if last_stall_boost else 0,
            1 if last_angle_priority_active else 0,
            1 if master_edge_until_ms else 0,
            cam_error_x,
            cam_error_y,
            cam_error_angle,
            int(cam_target_vx),
            int(cam_target_vy),
            int(vz_cmd),
            int(gyro_z),
        )
    )


def debug_send_wheel(log_id, u_fl, u_fr, u_b):
    debug_send(
        "W %d %d %d %d %d %d %d %d %d %d %d %d %d"
        % (
            log_id,
            pid_fl.enc_sum,
            pid_fr.enc_sum,
            pid_b.enc_sum,
            int(move_cmd.speed_fl * 4),
            int(move_cmd.speed_fr * 4),
            int(move_cmd.speed_b * 4),
            last_pwm_fl,
            last_pwm_fr,
            last_pwm_b,
            int(u_fl),
            int(u_fr),
            int(u_b),
        )
    )


def debug_send_flow(log_id, yaw_deg, master_age_ms, cam_age_ms):
    debug_send(
        "F %d %d %d %d %d %d %d %d %d %d %d"
        % (
            log_id,
            int(yaw_deg),
            int(last_visual_vx),
            int(last_visual_vy),
            int(last_ff_vx),
            int(last_ff_vy),
            int(last_ff_wz),
            int(last_turn_rate_cmd),
            last_follow_mode_key,
            master_age_ms,
            cam_age_ms,
        )
    )


def update_follow_targets(gyro_z):
    global cam_target_vx, cam_target_vy, target_lost_since_ms
    global last_turn_rate_cmd
    global last_follow_seen, last_visual_vx, last_visual_vy
    global last_ff_vx, last_ff_vy, last_ff_wz
    global last_cmd_vx, last_cmd_vy, last_cmd_wz
    global last_ap_vz_cmd
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    global last_stall_count, last_stall_boost
    global follow_output_limit
    global last_angle_priority_active
    global last_follow_mode_key, _orbit
    global master_edge_until_ms
    global push_yaw_target

    now = utime.ticks_ms()
    seen = cam_target_seen()
    fresh_motion = master_motion_fresh()
    if fresh_motion:
        ff_vx = master_vx * 0.5 + master_vy * 0.8660254
        ff_vy = master_vy * 0.5 - master_vx * 0.8660254
    else:
        ff_vx = 0.0
        ff_vy = 0.0
    ff_wz = master_wz if fresh_motion else 0.0
    explicit_orbit = fresh_motion and ((master_flags & MASTER_MOTION_FLAG_ORBIT) != 0)
    explicit_push = fresh_motion and ((master_flags & MASTER_MOTION_FLAG_PUSH) != 0)
    explicit_spin = fresh_motion and ((master_flags & MASTER_MOTION_FLAG_SPIN) != 0)
    spin_ff_wz = update_spin_feedforward_latch(
        now,
        fresh_motion,
        explicit_spin,
        ff_wz,
        seen,
        cam_error_angle,
    )
    spin_mode_active = (
        explicit_spin
        and (not explicit_orbit)
        and (not explicit_push)
    )
    filtered_wz = update_filtered_ff_wz(spin_ff_wz if spin_mode_active else ff_wz, fresh_motion)
    orbit_mode_active = update_orbit_follow_mode(
        now,
        ff_wz,
        gyro_z,
        explicit_orbit,
        explicit_push,
        spin_mode_active,
        fresh_motion and (master_flags & MASTER_MOTION_FLAG_BACK),
    )
    if orbit_mode_active:
        follow_ff_wz = filtered_wz
    elif spin_mode_active:
        follow_ff_wz = spin_ff_wz
    else:
        follow_ff_wz = clamp(
            filtered_wz,
            -Follow_Normal_Wz_Feedforward_Limit,
            Follow_Normal_Wz_Feedforward_Limit,
        )
    push_follow_active = explicit_push and orbit_mode_active
    mode_key = 3 if spin_mode_active else (1 if orbit_mode_active else 0)
    _orbit = mode_key == 1
    if last_follow_mode_key != mode_key:
        if last_follow_mode_key > 0 and mode_key == 0:
            reset_speed_outputs()
            follow_ff_wz = 0.0
        elif last_follow_mode_key < 0:
            reset_speed_outputs(True)
            reset_turn_loop_state()
        elif push_follow_active:
            speed_reset(pid_b)
    follow_output_limit = FOLLOW_RUN_PWM_LIMIT
    if (
        mode_key == 0
        and -Follow_Master_Edge_Delta < ff_vx < Follow_Master_Edge_Delta
        and -Follow_Master_Edge_Delta < ff_vy < Follow_Master_Edge_Delta
        and -Follow_Master_Edge_Delta < follow_ff_wz < Follow_Master_Edge_Delta
        and (
            not (master_flags & 1)
            or (-10 < cam_error_x < 10 and -10 < cam_error_y < 10)
        )
    ):
        follow_output_limit = FOLLOW_STATIC_LOCK_PWM_LIMIT
    if (
        (mode_key != 0 and (not push_follow_active))
        or (not seen)
        or (not fresh_motion)
        or (last_follow_mode_key != mode_key and mode_key == 0)
    ):
        master_edge_until_ms = 0
    elif (
        abs(ff_vx - last_ff_vx) >= Follow_Master_Edge_Delta
        or abs(ff_vy - last_ff_vy) >= Follow_Master_Edge_Delta
    ):
        master_edge_until_ms = utime.ticks_add(now, Follow_Master_Edge_Hold_Ms)
    elif master_edge_until_ms and utime.ticks_diff(master_edge_until_ms, now) <= 0:
        master_edge_until_ms = 0
    last_follow_mode_key = mode_key
    if (
        spin_mode_active
        and (follow_ff_wz >= 0.001 or follow_ff_wz <= -0.001)
        and last_cmd_wz * follow_ff_wz < 0.0
    ):
        reset_turn_loop_state()
    angle_pose_mode_active = angle_pose_mode_needed(
        cam_error_angle,
        orbit_mode_active,
        spin_mode_active,
    ) if seen else mode_key != 0
    body_vx = 0.0
    body_vy = 0.0
    turn_rate_cmd = 0.0
    use_motion_feedforward = False
    angle_priority_active = False
    position_priority_active = False
    prev_angle_priority_active = last_angle_priority_active

    if seen:
        target_lost_since_ms = 0
        use_motion_feedforward = fresh_motion and (not push_follow_active)
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
            follow_ff_wz,
            use_motion_feedforward,
            orbit_mode_active,
            spin_mode_active,
            push_follow_active,
        )
        if (
            mode_key == 0
            and (
                cam_error_y >= 8
                or cam_error_y <= -8
                or body_vx * ff_vx > 120
            )
        ):
            vx -= body_vx
            body_vx *= 0.20 if (
                abs(cam_error_x) < 28 or body_vx * ff_vx > 120
            ) else (0.70 if abs(ff_vy) <= 8.0 else 0.45)
            vx += body_vx
        if follow_output_limit == FOLLOW_STATIC_LOCK_PWM_LIMIT:
            vx -= body_vx * (1.0 - Follow_Static_Visual_Scale)
            vy -= body_vy * (1.0 - Follow_Static_Visual_Scale)
            body_vx *= Follow_Static_Visual_Scale
            body_vy *= Follow_Static_Visual_Scale
        if mode_key != 0:
            position_priority_active = True
        last_angle_priority_active = angle_priority_active or angle_pose_mode_active
    else:
        if target_lost_since_ms == 0:
            target_lost_since_ms = now
        if fresh_motion:
            use_motion_feedforward = utime.ticks_diff(
                now,
                target_lost_since_ms,
            ) <= (
                Follow_Normal_Target_Lost_Hold_Ms
                if mode_key == 0
                else Follow_Target_Lost_Hold_Ms
            )
        if use_motion_feedforward:
            xy_scale = (
                Follow_Normal_Hold_Feedforward_Gain
                if mode_key == 0 or push_follow_active
                else Follow_Hold_Feedforward_Gain
            )
            vx = ff_vx * xy_scale
            vy = ff_vy * xy_scale
        else:
            vx = 0.0
            vy = 0.0
        if prev_angle_priority_active:
            reset_turn_loop_state()
        last_angle_priority_active = False

    if seen and angle_pose_mode_active and not push_follow_active:
        xy_scale = angle_xy_lock_scale(
            cam_error_angle,
            orbit_mode_active,
            spin_mode_active,
        )
        if mode_key != 0:
            vx *= xy_scale
            vy *= xy_scale
        else:
            vx = (vx - body_vx) + body_vx * xy_scale

    if push_follow_active and seen:
        vx = add_feedforward_direct(
            vx,
            ff_vx,
            Follow_Push_Feedforward_Forward_Gain,
            Follow_Push_Feedforward_Forward_Limit,
            0.75,
        )
        vy = add_feedforward_direct(
            ff_vy * 1.35,
            vy,
            1.0,
            Follow_Lateral_Limit,
            Follow_Close_Feedforward_Min_Scale,
        )

    if push_follow_active:
        if push_yaw_target is None:
            push_yaw_target = imu_runtime.yaw_deg
            reset_turn_loop_state()
        # Short PUSH stages use gyro-integrated relative yaw as the heading anchor.
        turn_rate_cmd = (
            imu_runtime.yaw_deg - push_yaw_target + 180.0
        ) % 360.0 - 180.0
        turn_rate_cmd = calc_follow_angle(turn_rate_cmd)
    else:
        push_yaw_target = None

    if mode_key == 0 and seen and master_edge_until_ms and not ff_vx and not ff_vy:
        vx = last_cmd_vx
        vy = last_cmd_vy

    vx_limit = Follow_Forward_Limit
    vy_limit = (
        29.0 if cam_error_y <= -16 else 27.0
    ) if push_follow_active else Follow_Lateral_Limit
    if fresh_motion:
        vx_limit = follow_limit(vx_limit, ff_vx)
        vy_limit = follow_limit(vy_limit, ff_vy)
    vx = clamp(vx, -vx_limit, vx_limit)
    vy = clamp(vy, -vy_limit, vy_limit)
    if master_edge_until_ms or position_priority_active:
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
        if orbit_mode_active:
            turn_rate_cmd = clamp(
                follow_ff_wz * Follow_Orbit_Wz_Feedforward_Gain,
                -Follow_Orbit_Wz_Feedforward_Limit,
                Follow_Orbit_Wz_Feedforward_Limit,
            )
        elif spin_mode_active:
            turn_rate_cmd = clamp(
                follow_ff_wz * Follow_Spin_Wz_Feedforward_Gain,
                -Follow_Spin_Wz_Feedforward_Limit,
                Follow_Spin_Wz_Feedforward_Limit,
            )
        else:
            turn_rate_cmd = follow_ff_wz * Follow_Normal_Wz_Feedforward_Gain
    priority_turn_mode = mode_key != 0 or angle_pose_mode_active
    orbit_brake_active = (
        mode_key != 0
        and (
            gyro_z >= Follow_Orbit_Brake_Gyro_Threshold
            or gyro_z <= -Follow_Orbit_Brake_Gyro_Threshold
        )
    )
    normal_brake_active = (
        seen
        and not master_flags
        and follow_output_limit < FOLLOW_RUN_PWM_LIMIT
    )
    normal_brake_active = (
        mode_key == 0
        and abs(gyro_z) >= (10.0 if normal_brake_active else 40.0)
        and (
            (
                fresh_motion
                and (master_flags or normal_brake_active)
                and -0.001 < turn_rate_cmd < 0.001
            )
            or (
                abs(gyro_z) >= 40.0
                and not (-0.001 < turn_rate_cmd < 0.001)
                and (
                    (master_flags and turn_rate_cmd * gyro_z < 0.0)
                    or abs(gyro_z) > abs(turn_rate_cmd) * GYRO_PRIORITY_OVERSPEED_RATIO
                )
            )
        )
    )
    if -0.001 < turn_rate_cmd < 0.001:
        turn_rate_cmd = 0.0
        if orbit_brake_active or normal_brake_active:
            last_cmd_wz = 0.0
        else:
            reset_turn_loop_state()
    else:
        if spin_mode_active:
            output_ramp = 36.0
        elif priority_turn_mode:
            output_ramp = 18.0
        else:
            output_ramp = 11.0
        turn_rate_cmd = ramp_value(turn_rate_cmd, last_cmd_wz, output_ramp)
        last_cmd_wz = turn_rate_cmd
    if (
        priority_turn_mode
        and (
            turn_rate_cmd >= GYRO_PRIORITY_MIN_CMD
            or turn_rate_cmd <= -GYRO_PRIORITY_MIN_CMD
        )
    ):
        angle_priority_active = True
        last_angle_priority_active = True
    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        if abs(turn_rate_cmd) > 0.001 or orbit_brake_active or normal_brake_active:
            if priority_turn_mode or normal_brake_active:
                gyro_pid.gyro_kp = GYRO_PRIORITY_KP
                gyro_pid.gyro_ki = GYRO_PRIORITY_KI
                gyro_pid.gyro_output_limit = gyro_limit_for_turn(
                    turn_rate_cmd,
                    True,
                    spin_mode_active,
                )
                gyro_pid.err = turn_rate_cmd - gyro_z
                vz_cmd = priority_gyro_rate_ctrl(
                    turn_rate_cmd,
                    gyro_z,
                    spin_mode_active,
                )
                output_ramp = (
                    GYRO_PRIORITY_OUTPUT_MAX_LIMIT
                    if push_follow_active
                    else (
                        GYRO_SPIN_PRIORITY_OUTPUT_MAX_LIMIT
                        if spin_mode_active
                        else (0.6 if mode_key == 0 else 2.0)
                    )
                )
                vz_cmd = ramp_value(vz_cmd, last_ap_vz_cmd, output_ramp)
                last_ap_vz_cmd = vz_cmd
                gyro_pid.output = vz_cmd
                gyro_pid.err_last = gyro_pid.err
            else:
                last_ap_vz_cmd = 0.0
                gyro_pid.gyro_kp = GYRO_KP
                gyro_pid.gyro_ki = GYRO_KI
                gyro_pid.gyro_output_limit = (
                    Follow_Orbit_Brake_Output_Limit
                    if orbit_brake_active
                    else gyro_limit_for_turn(turn_rate_cmd)
                )
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

    if mode_key == 0:
        vz_cmd = clamp(
            vz_cmd,
            -GYRO_OUTPUT_BASE_LIMIT,
            GYRO_OUTPUT_BASE_LIMIT,
        )
    if normal_brake_active and (not priority_turn_mode):
        normal_brake_reserve = Follow_Normal_Brake_Wheel_Reserve
        if master_flags and abs(gyro_z) >= 40.0 and (
            abs(gyro_z) > abs(turn_rate_cmd) * GYRO_PRIORITY_OVERSPEED_RATIO
        ):
            normal_brake_reserve *= 2.0
        if vz_cmd > normal_brake_reserve:
            vz_cmd = normal_brake_reserve
        elif vz_cmd < -normal_brake_reserve:
            vz_cmd = -normal_brake_reserve
    cam_target_vx, cam_target_vy, vz_cmd = limit_pose_twist_for_wheels(
        cam_target_vx,
        cam_target_vy,
        vz_cmd,
        (priority_turn_mode and (not push_follow_active))
        or (normal_brake_active and (not master_edge_until_ms)),
        (
            priority_turn_mode
            or (normal_brake_active and (not master_edge_until_ms))
            or (
                mode_key == 0
                and fresh_motion
                and (
                    master_vy >= Follow_Master_Edge_Delta
                    or master_vy <= -Follow_Master_Edge_Delta
                )
            )
        )
        and mode_key == 0
        and (vz_cmd >= 0.001 or vz_cmd <= -0.001),
        push_follow_active,
    )
    last_cmd_vx = cam_target_vx
    last_cmd_vy = cam_target_vy
    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        gyro_pid.output = vz_cmd

    last_turn_rate_cmd = turn_rate_cmd
    last_follow_seen = seen
    last_visual_vx = body_vx
    last_visual_vy = body_vy
    last_ff_vx = ff_vx
    last_ff_vy = ff_vy
    last_ff_wz = follow_ff_wz
    return vz_cmd


def stop_all():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)


def reset_speed_outputs(keep_orbit_state=False):
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    global last_stall_count, last_stall_boost
    global last_cmd_vx, last_cmd_vy, last_cmd_wz, last_ap_vz_cmd
    global last_angle_priority_active
    global orbit_follow_active, orbit_follow_exit_since_ms, filtered_ff_wz
    global spin_latched_wz, spin_latch_until_ms
    global last_follow_mode_key
    global master_edge_until_ms

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
    if not keep_orbit_state:
        orbit_follow_active = False
        orbit_follow_exit_since_ms = 0
        filtered_ff_wz = 0.0
        last_follow_mode_key = -1
    spin_latched_wz = 0.0
    spin_latch_until_ms = 0
    master_edge_until_ms = 0


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


def apply_start_pwm(cmd, min_pwm):
    cmd = int(cmd)
    if 0 < cmd < min_pwm:
        cmd = min_pwm
    elif -min_pwm < cmd < 0:
        cmd = -min_pwm
    return clamp(cmd, -FOLLOW_RUN_PWM_LIMIT, FOLLOW_RUN_PWM_LIMIT)


def follow_start_pwm_for_target(target, stall_boost):
    target_abs = abs(target)
    if wheel_target_idle(target):
        return 0
    if stall_boost and target_abs >= 2.8:
        return 8800
    if (
        follow_output_limit < FOLLOW_RUN_PWM_LIMIT
        and -0.001 < cam_target_vx < 0.001
        and -0.001 < cam_target_vy < 0.001
    ):
        return 2600
    return 3600 if target_abs < 2.8 else 6200


def follow_channel_pwm(cmd, target, speed_err, stall_boost, last_pwm):
    fast_reverse = (
        (not _orbit or (master_flags & MASTER_MOTION_FLAG_PUSH))
        and (
            master_edge_until_ms
            or last_ff_wz >= Follow_Spin_Latch_Min_Wz
            or last_ff_wz <= -Follow_Spin_Latch_Min_Wz
        )
    )
    min_pwm = follow_start_pwm_for_target(target, stall_boost)
    if min_pwm <= 0:
        if not _orbit:
            return 0
        return follow_low_pwm(
            cmd,
            target,
            speed_err,
            last_pwm,
            WHEEL_TARGET_STOP_EPS,
            MOTOR_DUTY_MIN,
            smooth_value,
        )
    if (
        fast_reverse
        and (target >= 4.0 or target <= -4.0)
    ):
        if last_pwm * target < 0.0:
            return 8800 if target > 0.0 else -8800
        if last_pwm == 0 and (target - speed_err) * target < 0.0:
            return 8800 if target > 0.0 else -8800
    cmd = apply_start_pwm(cmd, min_pwm)
    if not fast_reverse and last_follow_mode_key == 0 and last_pwm * cmd < 0:
        return smooth_value(cmd, smooth_value(cmd, last_pwm))
    if master_edge_until_ms and last_follow_mode_key == 0:
        return smooth_value(cmd, smooth_value(cmd, last_pwm))
    if last_follow_mode_key == 3:
        return smooth_value(cmd, smooth_value(cmd, last_pwm))
    return smooth_value(cmd, last_pwm)


def set_three_pwm_follow(u_fl, u_fr, u_b, t_fl, t_fr, t_b, stall_boost):
    global last_pwm_fl, last_pwm_fr, last_pwm_b

    s_fl = follow_channel_pwm(u_fl, t_fl, pid_fl.err, stall_boost, last_pwm_fl)
    s_fr = follow_channel_pwm(u_fr, t_fr, pid_fr.err, stall_boost, last_pwm_fr)
    s_b = follow_channel_pwm(u_b, t_b, pid_b.err, stall_boost, last_pwm_b)
    s_fl = clamp(s_fl, -follow_output_limit, follow_output_limit)
    s_fr = clamp(s_fr, -follow_output_limit, follow_output_limit)
    s_b = clamp(s_b, -follow_output_limit, follow_output_limit)
    s_fl = motor_fl.duty(s_fl, MOTOR_DUTY_MIN)
    s_fr = motor_fr.duty(s_fr, MOTOR_DUTY_MIN)
    s_b = motor_b.duty(s_b, MOTOR_DUTY_MIN)
    last_pwm_fl = s_fl
    last_pwm_fr = s_fr
    last_pwm_b = s_b
    pid_fl.output = s_fl
    pid_fr.output = s_fr
    pid_b.output = s_b
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
    if (
        last_follow_mode_key == 0
        and (last_turn_rate_cmd >= 0.001 or last_turn_rate_cmd <= -0.001)
        and -0.001 < cam_target_vx < 0.001
        and -0.001 < cam_target_vy < 0.001
    ):
        return -WHEEL_TARGET_STOP_EPS <= target <= WHEEL_TARGET_STOP_EPS
    if last_follow_mode_key == 0:
        return -WHEEL_TARGET_NORMAL_IDLE_EPS <= target <= WHEEL_TARGET_NORMAL_IDLE_EPS
    return -WHEEL_TARGET_IDLE_EPS <= target <= WHEEL_TARGET_IDLE_EPS


def speed_ctrl_follow(pid, actual_speed, target_speed):
    if last_follow_mode_key and not (
        (master_flags & MASTER_MOTION_FLAG_PUSH)
        and (cam_target_vy >= 8.0 or cam_target_vy <= -8.0)
    ):
        pid.ki = 18.0 if last_follow_mode_key == 3 else 8.0
    elif master_flags and (cam_target_vy >= 8.0 or cam_target_vy <= -8.0):
        pid.ki = 18.0
    else:
        pid.ki = 12.0
    if wheel_target_idle(target_speed):
        if not _orbit:
            speed_reset(pid)
            return 0.0
    output = speed_ctrl(pid, actual_speed, target_speed)
    output = speed_follow_guard(
        pid,
        output,
        actual_speed,
        target_speed,
        WHEEL_TARGET_STOP_EPS,
        not master_edge_until_ms,
    )
    return output


def update_nav_led_display():
    straight_value = 1 if cam_target_seen() else 0
    translate_value = 1 if utime.ticks_diff(utime.ticks_ms(), master_last_rx_ms) <= Master_Motion_Timeout_Ms else 0
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
    coop_rx_led_until_ms = utime.ticks_add(utime.ticks_ms(), 40)


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
                logger=None,
            )
            imu_runtime.reset_yaw(0.0)
        finally:
            pit1.start(TICK_PERIOD_MS)


def start_follow():
    global car_started, start_time

    if car_started:
        return
    calibrate_gyro_before_launch()
    clear_cam_target_state()
    car_started = True
    start_time = utime.ticks_ms()
    cam_uart.write(ART_MODE_TRACK_CMD)


def check_c9_start():
    global last_c9_state

    current_c9 = key_start.value()
    if current_c9 == 0 and last_c9_state == 1:
        utime.sleep_ms(10)
        if key_start.value() == 0:
            start_follow()
    last_c9_state = current_c9


def check_c8_exit():
    global last_c8_state

    current_c8 = key_exit.value()
    if current_c8 == 0 and last_c8_state == 1:
        utime.sleep_ms(10)
        if key_exit.value() == 0:
            raise KeyboardInterrupt
    last_c8_state = current_c8


def time_pit_handler(_):
    global pit_flag
    pit_flag = True


def calc_speed_closed_loop():
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    global last_hard_stop
    global last_stall_count, last_stall_boost

    if not car_started:
        reset_speed_outputs()
        set_three_pwm_zero()
        last_hard_stop = True
        now_log = utime.ticks_ms()
        if debug_due(now_log):
            debug_send_idle(now_log & 0x7FFF)
        return None

    if ENABLE_IMU:
        gyro_z = imu_runtime.read_gyro_z()
        yaw_deg = imu_runtime.read_yaw()
    else:
        gyro_z = 0.0
        yaw_deg = 0.0

    vz_cmd = update_follow_targets(gyro_z)
    calc_wheel_spd(move_cmd, cam_target_vx, cam_target_vy, vz_cmd)

    e_fl = _pid_mod.encoder_window(pid_fl, enc_fl.get())
    e_fr = _pid_mod.encoder_window(pid_fr, enc_fr.get())
    e_b = _pid_mod.encoder_window(pid_b, enc_b.get())
    t_fl = move_cmd.speed_fl
    t_fr = move_cmd.speed_fr
    t_b = move_cmd.speed_b

    if wheel_targets_zero(t_fl, t_fr, t_b):
        reset_speed_outputs(orbit_follow_active)
        s_fl, s_fr, s_b = set_three_pwm_zero()
        u_fl = 0.0
        u_fr = 0.0
        u_b = 0.0
        last_hard_stop = True
    else:
        last_hard_stop = False
        if (
            wheel_target_idle(t_fl)
            and wheel_target_idle(t_fr)
            and wheel_target_idle(t_b)
        ):
            last_stall_count = 0
        elif encoders_stalled(e_fl, e_fr, e_b):
            last_stall_count += 1
        else:
            last_stall_count = 0
        last_stall_boost = last_stall_count >= 3

        u_fl = speed_ctrl_follow(pid_fl, e_fl, t_fl)
        u_fr = speed_ctrl_follow(pid_fr, e_fr, t_fr)
        u_b = speed_ctrl_follow(pid_b, e_b, t_b)

        s_fl, s_fr, s_b = set_three_pwm_follow(
            u_fl, u_fr, u_b, t_fl, t_fr, t_b, last_stall_boost
        )

    now_log = utime.ticks_ms()
    if debug_due(now_log):
        log_id = now_log & 0x7FFF
        master_age_ms = (
            utime.ticks_diff(now_log, master_last_rx_ms)
            if master_last_rx_ms
            else -1
        )
        cam_age_ms = (
            utime.ticks_diff(now_log, cam_last_rx_ms)
            if cam_last_rx_ms
            else -1
        )
        debug_send_state(log_id, vz_cmd, gyro_z)
        debug_send_wheel(log_id, u_fl, u_fr, u_b)
        debug_send_flow(log_id, yaw_deg, master_age_ms, cam_age_ms)

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
coop_rx_buf = bytearray(32)
cam_uart = UART(cfg.CAM_UART_ID, cfg.CAM_UART_BAUD)
cam_uart.init(cfg.CAM_UART_BAUD, timeout_char=100)
cam_uart_buf = bytearray(32)
cam_uart.write(ART_MODE_TRACK_CMD)

imu_runtime = None
if ENABLE_IMU:
    imu_runtime = LSM6DSV16XYawRuntime(
        sign=GYRO_SIGN,
        offset_z=GYRO_OFFSET_Z,
        scale=GYRO_SCALE,
        deadband_dps=GYRO_DEADBAND_DPS,
        tick_period_ms=TICK_PERIOD_MS,
    )

pit1 = ticker(1)
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

try:
    while True:
        loop_count += 1
        now = utime.ticks_ms()
        check_c8_exit()
        check_c9_start()
        poll_art_uart()
        poll_coop_uart()
        update_nav_led_display()

        if (not car_started) and cam_target_seen():
            start_follow()

        if pit_flag:
            pit_flag = False
            calc_speed_closed_loop()

        if utime.ticks_diff(now, last_status_ms) >= 1000:
            led.toggle()
            last_status_ms = now

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
