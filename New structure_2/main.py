from machine import Pin, UART
from micropython import const
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
    MASTER_MOTION_FLAG_RETURN,
    MASTER_MOTION_FLAG_SPIN,
    MASTER_MOTION_FLAG_STARTED,
    MSG_MASTER_MOTION,
    decode_i16,
)


speed_reset = _pid_mod.speed_reset
fit_wheel_delta_scale = _pid_mod.fit_wheel_delta_scale


# ====================== Base config ======================
TICK_PERIOD_MS = cfg.TICK_PERIOD_MS
ENC_SCALE = cfg.ENC_SCALE
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
GYRO_KP = 0.04
GYRO_KI = 0.004
GYRO_TURN_KI = 0.002
GYRO_OUTPUT_LIMIT = 12.0
GYRO_NORMAL_POSE_OUTPUT_LIMIT = 16.0
GYRO_PUSH_OUTPUT_LIMIT = 16.0
GYRO_ORBIT_OUTPUT_LIMIT = 18.0
GYRO_SPIN_OUTPUT_LIMIT = 18.0
GYRO_STOP_RATE = 4.0
GYRO_NORMAL_STOP_RATE = 12.0
GYRO_ANGLE_PRIORITY_MIN_CMD = 6.5
AUTO_CALIBRATE_GYRO_ON_LAUNCH = True
_GYRO_CALIBRATE_SAMPLES = const(1000)
_GYRO_CALIBRATE_DELAY_MS = const(2)

_GC_DIV = const(50)
_DEBUG_LOG_PERIOD_MS = const(100)
WHEEL_TARGET_STOP_EPS = 0.05
WHEEL_TARGET_IDLE_EPS = 1.2
WHEEL_TARGET_NORMAL_IDLE_EPS = 0.35
_FOLLOW_STATIC_LOCK_PWM_LIMIT = const(12000)
_FOLLOW_RUN_PWM_LIMIT = const(60000)
_pid_mod.PWM_MAX = _FOLLOW_RUN_PWM_LIMIT


# ====================== Camera protocol ======================
_Cam_Error_Offset = const(120)
_Cam_Error_Scale = const(2)
_Cam_Packet_Timeout_Ms = const(300)
_Cam_Frame_Head = const(0xFF)
_No_Target_Marker = const(0xFE)
_Line_Packet_Tag = const(0xFC)
_Classify_Packet_Tag = const(0xFD)
ART_MODE_TRACK_CMD = b"TRACK\n"
ART_MODE_IDLE_CMD = b"IDLE\n"


# ====================== Follow control ======================
Follow_Forward_Gain = 0.46
Follow_Lateral_Gain = 0.92
Follow_Orbit_Forward_Gain = 1.00
Follow_Orbit_Lateral_Gain = 0.92
Follow_Forward_Limit = 38.0
Follow_Lateral_Limit = 30.0
Follow_Return_Lateral_Limit = 32.0
_Follow_Forward_Deadband = const(2)
_Follow_Lateral_Deadband = const(2)
_Follow_Orbit_Forward_Deadband = const(2)
_Follow_Orbit_Lateral_Deadband = const(2)
_Follow_Orbit_Position_X_Error = const(4)
_Follow_Orbit_Position_Y_Error = const(4)
_Follow_Distance_Far_Boost_Error = const(6)
Follow_Distance_Far_Boost_Gain = 0.70
Follow_Distance_Close_Gain = 0.70
Follow_Distance_Close_Limit = 22.0
Follow_Feedforward_Forward_Gain = 1.00
Follow_Feedforward_Lateral_Gain = 1.00
Follow_Feedforward_Forward_Limit = 35.0
Follow_Feedforward_Lateral_Limit = 42.0
Follow_Push_Feedforward_Forward_Gain = 1.25
Follow_Push_Feedforward_Forward_Limit = 30.0
Follow_Normal_Visual_Forward_Scale = 0.72
Follow_Static_Visual_Scale = 0.60
Follow_Close_Feedforward_Min_Scale = 0.25
Follow_Hold_Feedforward_Gain = 1.70
Follow_Normal_Wz_Feedforward_Gain = 1.00
Follow_Orbit_Wz_Feedforward_Gain = 1.00
Follow_Orbit_Wz_Feedforward_Limit = 150.0
Follow_Orbit_Turn_Rate_Limit = 180.0
Follow_Push_Wz_Feedforward_Limit = 128.0
Follow_Push_Turn_Rate_Limit = 160.0
Follow_Target_Point_Wz_To_Vx = -0.18
Follow_Target_Point_Wz_To_Vy = -0.45
Follow_Orbit_Target_Point_Wz_To_Vx = -0.17
Follow_Orbit_Target_Point_Wz_To_Vy = -0.28
Follow_Orbit_Feedforward_Forward_Gain = 1.30
Follow_Orbit_Feedforward_Lateral_Gain = 0.76
Follow_Orbit_Feedforward_Forward_Limit = 22.0
Follow_Orbit_Feedforward_Lateral_Limit = 20.0
_Follow_Orbit_Feedforward_Close_Error = const(6)
_Follow_Orbit_Feedforward_Full_Error = const(22)
Follow_Orbit_Feedforward_Close_Scale = 0.78
_Follow_Orbit_Close_Feedforward_Full_Error = const(6)
Follow_Pose_Angle_Gain = -0.68
Follow_Pose_Angle_Limit = 40.0
Follow_Normal_Pose_Angle_Limit = 24.0
Follow_Orbit_Pose_Angle_Gain = -1.20
Follow_Orbit_Pose_Angle_Limit = 32.0
_Follow_Normal_Pose_Angle_Deadband = const(8)
_Follow_Normal_Pose_Angle_Active_Error = const(18)
Follow_Spin_Target_Point_Wz_To_Vy = -0.08
Follow_Spin_Wz_Feedforward_Gain = 0.95
Follow_Spin_Wz_Feedforward_Limit = 120.0
Follow_Spin_Turn_Rate_Limit = 128.0
_Follow_Pose_Angle_Deadband = const(4)
_Follow_Pose_Angle_Active_Error = const(6)
Follow_Pose_Wheel_Target_Limit = 33.0
Follow_Normal_Wheel_Target_Limit = 38.0
Follow_Normal_Allocation_Reserve = 8.0
Follow_Normal_Correction_Reserve = 6.0
Follow_Normal_Conflict_Start_Error = 4
Follow_Normal_Conflict_Stop_Error = 16
Follow_Normal_Preview_Gain = 0.50
Follow_Normal_Velocity_Damping = 0.55
Follow_Safety_Speed_Margin = 0.12
_Master_State_Preview_Flag = const(0x80)
Follow_Static_Velocity_Damping = 0.85
Follow_Orbit_Velocity_Damping = 0.55
Follow_Normal_Reverse_Release_Speed = 3.0
Follow_Command_Ramp_Vx = 2.0
Follow_Command_Ramp_Vy = 3.0
Follow_Orbit_Command_Ramp_Vx = 2.0
Follow_Orbit_Command_Ramp_Vy = 3.0
Follow_Orbit_Entry_Ramp_Vx = 0.25
Follow_Orbit_Entry_Ramp_Vy = 0.35
Follow_Orbit_Entry_Ramp_Wz = 3.0
_Follow_Orbit_Entry_Hold_Ms = const(250)
_Follow_Target_Lost_Hold_Ms = const(450)
_Follow_Orbit_Settle_Position_Error = const(6)
_Follow_Orbit_Settle_Angle_Error = const(6)
Follow_Orbit_Settle_Gyro_Rate = 5.0
Follow_Orbit_Settle_XY_Limit = 14.0
Follow_Orbit_Settle_Turn_Limit = 32.0
_Follow_Orbit_Settle_Hold_Ms = const(160)
_Follow_Orbit_Settle_Timeout_Ms = const(1400)
Follow_Orbit_Mode_FfWz_Filter = 0.22
Follow_Normal_Wz_Feedforward_Limit = 15.0
Follow_Normal_Target_Point_Wz_Limit = 15.0
Follow_Spin_Latch_Min_Wz = 26.0
_Follow_Spin_Command_Hold_Ms = const(900)
_Follow_Spin_Latch_Release_Angle = const(3)
_Master_Motion_Timeout_Ms = const(250)
Follow_Master_Edge_Delta = 4.0
_Follow_Master_Edge_Hold_Ms = const(120)
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
master_preview_vx = 0.0
master_preview_vy = 0.0
master_flags = 0
master_state_code = -1
master_last_rx_ms = 0

coop_parser = CoopFrameParser()

pit_flag = False
last_status_ms = utime.ticks_ms()
loop_count = 0
debug_log_last_ms = last_status_ms
last_pwm_fl = 0
last_pwm_fr = 0
last_pwm_b = 0
last_enc_fl = 0.0
last_enc_fr = 0.0
last_enc_b = 0.0
last_turn_rate_cmd = 0.0
last_follow_seen = False
last_ff_vx = 0.0
last_ff_vy = 0.0
last_ff_wz = 0.0
last_cmd_vx = 0.0
last_cmd_vy = 0.0
last_cmd_wz = 0.0
last_angle_priority_active = False
orbit_follow_active = False
orbit_follow_exit_since_ms = 0
orbit_follow_settle_since_ms = 0
orbit_follow_entry_until_ms = 0
filtered_ff_wz = 0.0
spin_latched_wz = 0.0
spin_latch_until_ms = 0
last_follow_mode_key = -1
last_control_master_state = -1
master_edge_until_ms = 0
last_stall_count = 0
last_stall_boost = False
last_alloc_scale = 100
follow_output_limit = _FOLLOW_RUN_PWM_LIMIT


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def update_push_reacquire(now, seen):
    global master_edge_until_ms
    if (not seen) or last_follow_mode_key != 1:
        master_edge_until_ms = 0
    elif not last_follow_seen:
        master_edge_until_ms = utime.ticks_add(
            now,
            _Follow_Master_Edge_Hold_Ms,
        )


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


def normal_feedforward_conflict_scale(error):
    error_abs = error if error >= 0 else -error
    if error_abs <= Follow_Normal_Conflict_Start_Error:
        return 1.0
    if error_abs >= Follow_Normal_Conflict_Stop_Error:
        return 0.0
    return (
        Follow_Normal_Conflict_Stop_Error - error_abs
    ) / (
        Follow_Normal_Conflict_Stop_Error
        - Follow_Normal_Conflict_Start_Error
    )


def angle_pose_mode_needed(error_angle, orbit_mode=False, spin_mode=False):
    if orbit_mode or spin_mode:
        return True
    if last_angle_priority_active:
        return (
            error_angle > _Follow_Normal_Pose_Angle_Deadband
            or error_angle < -_Follow_Normal_Pose_Angle_Deadband
        )
    return (
        error_angle >= _Follow_Normal_Pose_Angle_Active_Error
        or error_angle <= -_Follow_Normal_Pose_Angle_Active_Error
    )


def orbit_feedforward_position_scale(error_x, error_y):
    err_abs = error_x if error_x >= 0 else -error_x
    tmp = error_y if error_y >= 0 else -error_y
    if tmp > err_abs:
        err_abs = tmp
    if err_abs <= _Follow_Orbit_Feedforward_Close_Error:
        scale = Follow_Orbit_Feedforward_Close_Scale
    elif err_abs >= _Follow_Orbit_Feedforward_Full_Error:
        scale = 1.0
    else:
        span = _Follow_Orbit_Feedforward_Full_Error - _Follow_Orbit_Feedforward_Close_Error
        scale = Follow_Orbit_Feedforward_Close_Scale + (
            (err_abs - _Follow_Orbit_Feedforward_Close_Error)
            * (1.0 - Follow_Orbit_Feedforward_Close_Scale)
            / span
        )
    depth = orbit_close_depth(error_y)
    if depth <= 0.0:
        return scale
    if depth >= _Follow_Orbit_Close_Feedforward_Full_Error:
        return Follow_Close_Feedforward_Min_Scale
    close_scale = 1.0 - (
        (1.0 - Follow_Close_Feedforward_Min_Scale)
        * depth
        / _Follow_Orbit_Close_Feedforward_Full_Error
    )
    if close_scale < scale:
        return close_scale
    return scale


def orbit_close_depth(error_y):
    depth = error_y - _Follow_Forward_Deadband
    if depth < 0:
        return 0.0
    return depth


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
    global last_cmd_vx, last_cmd_vy, last_cmd_wz
    global last_angle_priority_active
    global orbit_follow_active, orbit_follow_exit_since_ms
    global orbit_follow_settle_since_ms, filtered_ff_wz
    global orbit_follow_entry_until_ms
    global spin_latched_wz, spin_latch_until_ms
    global last_follow_mode_key
    global last_control_master_state
    global master_edge_until_ms

    cam_error_x = 0
    cam_error_y = 0
    cam_error_angle = 0
    last_cmd_vx = 0.0
    last_cmd_vy = 0.0
    last_cmd_wz = 0.0
    last_angle_priority_active = False
    orbit_follow_active = False
    orbit_follow_exit_since_ms = 0
    orbit_follow_settle_since_ms = 0
    orbit_follow_entry_until_ms = 0
    filtered_ff_wz = 0.0
    spin_latched_wz = 0.0
    spin_latch_until_ms = 0
    last_follow_mode_key = -1
    last_control_master_state = -1
    master_edge_until_ms = 0
    cam_has_target = False
    target_lost_since_ms = 0
    cam_last_rx_ms = 0
    cam_parse_state = 0


def cam_target_state():
    if (
        not cam_last_rx_ms
        or utime.ticks_diff(utime.ticks_ms(), cam_last_rx_ms)
        > _Cam_Packet_Timeout_Ms
    ):
        return -1
    return 1 if cam_has_target else 0


def lost_motion_scale(now, target_state):
    age = utime.ticks_diff(now, target_lost_since_ms)
    if target_state == 0:
        return clamp((560 - age) / 300.0, 0.0, 1.0)
    return clamp((240 - age) / 160.0, 0.0, 1.0)


def normal_visual_scale(now, seen):
    global master_edge_until_ms

    if not seen:
        master_edge_until_ms = 0
        return 1.0
    if not last_follow_seen:
        master_edge_until_ms = utime.ticks_add(now, 180)
    remaining = utime.ticks_diff(master_edge_until_ms, now)
    if remaining <= 0:
        master_edge_until_ms = 0
        return 1.0
    return 1.0 - remaining / 240.0


def master_motion_fresh():
    return utime.ticks_diff(utime.ticks_ms(), master_last_rx_ms) <= _Master_Motion_Timeout_Ms


def update_spin_feedforward_latch(now, fresh_motion, explicit_spin, ff_wz, seen, error_angle):
    global spin_latched_wz, spin_latch_until_ms

    if explicit_spin and fresh_motion and (
        ff_wz >= Follow_Spin_Latch_Min_Wz
        or ff_wz <= -Follow_Spin_Latch_Min_Wz
    ):
        if last_follow_mode_key == 0:
            reset_speed_outputs(True)
        spin_latched_wz = ff_wz
        spin_latch_until_ms = utime.ticks_add(now, _Follow_Spin_Command_Hold_Ms)
        return ff_wz

    if spin_latched_wz != 0.0:
        if (
            seen
            and -10 < cam_error_x < 10
            and -10 < cam_error_y < 10
            and -_Follow_Spin_Latch_Release_Angle
            <= error_angle
            <= _Follow_Spin_Latch_Release_Angle
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
    gyro_z,
    explicit_orbit,
    explicit_push,
    explicit_spin,
    explicit_back,
):
    global orbit_follow_active, orbit_follow_exit_since_ms
    global orbit_follow_settle_since_ms

    if explicit_spin or explicit_back:
        orbit_follow_active = False
        orbit_follow_exit_since_ms = orbit_follow_settle_since_ms = 0
        return False
    if explicit_push or explicit_orbit:
        orbit_follow_active = True
        orbit_follow_exit_since_ms = orbit_follow_settle_since_ms = 0
        return True

    if not orbit_follow_active:
        orbit_follow_exit_since_ms = orbit_follow_settle_since_ms = 0
        return False

    if orbit_follow_exit_since_ms == 0:
        orbit_follow_exit_since_ms = now
        orbit_follow_settle_since_ms = 0
        reset_turn_loop_state()

    settled = (
        cam_has_target
        and utime.ticks_diff(utime.ticks_ms(), cam_last_rx_ms)
        <= _Cam_Packet_Timeout_Ms
        and -_Follow_Orbit_Settle_Position_Error
        <= cam_error_x
        <= _Follow_Orbit_Settle_Position_Error
        and -_Follow_Orbit_Settle_Position_Error
        <= cam_error_y
        <= _Follow_Orbit_Settle_Position_Error
        and -_Follow_Orbit_Settle_Angle_Error
        <= cam_error_angle
        <= _Follow_Orbit_Settle_Angle_Error
        and -Follow_Orbit_Settle_Gyro_Rate
        <= gyro_z
        <= Follow_Orbit_Settle_Gyro_Rate
    )
    if settled:
        if orbit_follow_settle_since_ms == 0:
            orbit_follow_settle_since_ms = now
        elif (
            utime.ticks_diff(now, orbit_follow_settle_since_ms)
            >= _Follow_Orbit_Settle_Hold_Ms
        ):
            orbit_follow_active = False
    else:
        orbit_follow_settle_since_ms = 0

    if (
        orbit_follow_active
        and utime.ticks_diff(now, orbit_follow_exit_since_ms)
        >= _Follow_Orbit_Settle_Timeout_Ms
    ):
        orbit_follow_active = False

    if not orbit_follow_active:
        orbit_follow_exit_since_ms = orbit_follow_settle_since_ms = 0
    return orbit_follow_active


def follow_limit(base_limit, master_value):
    limit = base_limit
    master_abs = abs(master_value)
    if master_abs > limit:
        limit = master_abs
    return limit


def calc_follow_forward(error_y, position_priority=False):
    deadband = _Follow_Orbit_Forward_Deadband if position_priority else _Follow_Forward_Deadband
    if follow_output_limit == _FOLLOW_STATIC_LOCK_PWM_LIMIT:
        deadband = 1
    error_y = soft_deadband(error_y, deadband, deadband * 2)
    if error_y == 0.0:
        return 0.0

    if error_y > 0:
        gain = Follow_Orbit_Forward_Gain if position_priority else Follow_Forward_Gain
        out = error_y * gain
        if error_y > _Follow_Distance_Far_Boost_Error:
            out += (
                error_y - _Follow_Distance_Far_Boost_Error
            ) * Follow_Distance_Far_Boost_Gain
        return clamp(out, 0.0, Follow_Forward_Limit)

    out = error_y * Follow_Distance_Close_Gain
    return clamp(out, -Follow_Distance_Close_Limit, 0.0)


def calc_follow_lateral(error_x, position_priority=False):
    deadband = _Follow_Orbit_Lateral_Deadband if position_priority else _Follow_Lateral_Deadband
    if follow_output_limit == _FOLLOW_STATIC_LOCK_PWM_LIMIT:
        deadband = 1
    error_x = soft_deadband(error_x, deadband, deadband * 2)
    if error_x == 0.0:
        return 0.0
    gain = Follow_Orbit_Lateral_Gain if position_priority else Follow_Lateral_Gain
    out = error_x * gain
    return clamp(out, -Follow_Lateral_Limit, Follow_Lateral_Limit)


def calc_follow_angle(error_angle, orbit_mode=False, spin_mode=False):
    if orbit_mode or spin_mode:
        deadband = _Follow_Pose_Angle_Deadband
        full_error = _Follow_Pose_Angle_Active_Error
    else:
        deadband = (
            10
            if follow_output_limit < _FOLLOW_RUN_PWM_LIMIT
            and -0.001 < last_turn_rate_cmd < 0.001
            else _Follow_Normal_Pose_Angle_Deadband
        )
        full_error = _Follow_Normal_Pose_Angle_Active_Error
    error_angle = soft_deadband(error_angle, deadband, full_error)
    if error_angle == 0.0:
        return 0.0
    if orbit_mode:
        return clamp(
            error_angle * Follow_Orbit_Pose_Angle_Gain,
            -Follow_Orbit_Pose_Angle_Limit,
            Follow_Orbit_Pose_Angle_Limit,
        )
    if spin_mode:
        error_angle *= 0.35 if last_ff_wz else 0.60
    return clamp(
        error_angle * Follow_Pose_Angle_Gain,
        -Follow_Pose_Angle_Limit,
        Follow_Pose_Angle_Limit,
    )


def add_feedforward_direct(base, feedforward, gain, limit, conflict_scale=1.0):
    assist = clamp(feedforward * gain, -limit, limit)
    if base * assist < -0.001 and conflict_scale < 1.0:
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
    moving_visual_scale,
):
    vision_wz = (
        calc_follow_angle(error_angle)
        if push_mode
        else calc_follow_angle(error_angle, orbit_mode, spin_mode)
    )
    if orbit_mode or push_mode:
        vision_wz *= moving_visual_scale
    if not orbit_mode and not spin_mode and not push_mode:
        vision_wz = clamp(
            vision_wz,
            -Follow_Normal_Pose_Angle_Limit,
            Follow_Normal_Pose_Angle_Limit,
        )
    active_error = (
        _Follow_Pose_Angle_Active_Error
        if (orbit_mode or spin_mode)
        else _Follow_Normal_Pose_Angle_Active_Error
    )
    angle_active = (
        error_angle >= active_error
        or error_angle <= -active_error
    )
    # Vision already reports independent position and heading errors.
    cam_vx = error_x
    cam_vy = error_y
    if orbit_mode or spin_mode:
        position_priority = (
            (angle_active and (not spin_mode))
            or cam_vy >= _Follow_Orbit_Position_X_Error
            or cam_vy <= -_Follow_Orbit_Position_X_Error
            or cam_vx >= _Follow_Orbit_Position_Y_Error
            or cam_vx <= -_Follow_Orbit_Position_Y_Error
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
        body_vx *= 0.65
    if (not orbit_mode) and (not spin_mode):
        body_vx *= Follow_Normal_Visual_Forward_Scale
    if use_ff and not spin_mode and ff_vx == 0.0 and ff_vy == 0.0:
        if -8.0 < body_vx < 8.0:
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
            point_wz = clamp(
                ff_wz,
                -Follow_Normal_Target_Point_Wz_Limit,
                Follow_Normal_Target_Point_Wz_Limit,
            )
            target_ff_vx = ff_vx + point_wz * Follow_Target_Point_Wz_To_Vx
            target_ff_vy = ff_vy + point_wz * Follow_Target_Point_Wz_To_Vy
            if master_flags & MASTER_MOTION_FLAG_BACK:
                target_ff_vx *= 0.80
                target_ff_vy *= 0.42
                ff_scale = 0.0
            else:
                ff_scale = 1.0
                if master_flags & MASTER_MOTION_FLAG_RETURN:
                    if abs(error_x) >= 48 and body_vx * target_ff_vx < 0.0:
                        target_ff_vx *= 0.35
                    else:
                        target_ff_vx *= 1.15
            if 0 < master_flags < MASTER_MOTION_FLAG_ORBIT:
                if body_vx * target_ff_vx < -0.001:
                    target_ff_vx *= normal_feedforward_conflict_scale(error_x)
                if body_vy * target_ff_vy < -0.001:
                    target_ff_vy *= normal_feedforward_conflict_scale(error_y)
            vx = add_feedforward_direct(
                vx,
                target_ff_vx,
                Follow_Feedforward_Forward_Gain,
                Follow_Feedforward_Forward_Limit,
                ff_scale,
            )
            vy = add_feedforward_direct(
                vy,
                target_ff_vy,
                Follow_Feedforward_Lateral_Gain,
                Follow_Feedforward_Lateral_Limit,
                0.0 if master_flags & MASTER_MOTION_FLAG_BACK else (
                    (
                        0.66
                        if error_x > -48
                        else (0.82 if error_x > -72 else 1.0)
                    )
                    if (
                        (master_flags & MASTER_MOTION_FLAG_RETURN)
                        and ff_vy < 0.0
                    )
                    else 1.0
                ),
            )
            wz = add_feedforward_direct(
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
    base_vx,
    base_vy,
    feedback_vx,
    feedback_vy,
    safety_vx,
    preview_vx,
    preview_vy,
    preserve_feedforward,
    preserve_rotation,
):
    global last_alloc_scale

    limit = (
        Follow_Normal_Wheel_Target_Limit
        if preserve_feedforward
        else Follow_Pose_Wheel_Target_Limit
    )
    if limit <= 0.0:
        last_alloc_scale = 100
        return vx, vy, vz

    normal_priority = (
        preserve_feedforward
        and last_follow_mode_key == 0
        and 0 < master_flags < MASTER_MOTION_FLAG_ORBIT
    )
    if normal_priority:
        wfr = -vx * 0.866025 + vy * 0.5 + vz
        wfl = vx * 0.866025 + vy * 0.5 + vz
        wb = -vy + vz
        if max_wheel_abs(wfr, wfl, wb) <= limit:
            last_alloc_scale = 100
            return vx, vy, vz
        cvx = vx - base_vx
        cvy = vy - base_vy
        preview_vx = clamp(
            preview_vx,
            cvx if cvx < 0.0 else 0.0,
            cvx if cvx > 0.0 else 0.0,
        )
        preview_vy = clamp(
            preview_vy,
            cvy if cvy < 0.0 else 0.0,
            cvy if cvy > 0.0 else 0.0,
        )
        corr_vx = cvx - preview_vx
        corr_vy = cvy - preview_vy
        safety_vx = clamp(
            safety_vx,
            0.0,
            min(
                Follow_Normal_Allocation_Reserve
                + Follow_Normal_Correction_Reserve,
                corr_vx if corr_vx > 0.0 else 0.0,
            ),
        )
        core_vy = clamp(
            corr_vy,
            -Follow_Normal_Correction_Reserve,
            Follow_Normal_Correction_Reserve,
        )
        core_wz = clamp(
            vz,
            -Follow_Normal_Allocation_Reserve,
            Follow_Normal_Allocation_Reserve,
        )
        core_vx = safety_vx
        ox = core_vx
        oy = core_vy
        oz = core_wz
        wfr, wfl, wb = pose_wheel_targets(ox, oy, oz)
        dfr = -base_vx * 0.866025 + base_vy * 0.5
        dfl = base_vx * 0.866025 + base_vy * 0.5
        db = -base_vy
        scale = fit_wheel_delta_scale(wfr, wfl, wb, dfr, dfl, db, limit)
        ox += base_vx * scale
        oy += base_vy * scale
        wfr += dfr * scale
        wfl += dfl * scale
        wb += db * scale
        base_scale = scale
        dfr, dfl, db = pose_wheel_targets(preview_vx, preview_vy, 0.0)
        preview_scale = fit_wheel_delta_scale(
            wfr, wfl, wb, dfr, dfl, db, limit
        )
        ox += preview_vx * preview_scale
        oy += preview_vy * preview_scale
        wfr += dfr * preview_scale
        wfl += dfl * preview_scale
        wb += db * preview_scale
        yaw = vz - oz
        corr_vx -= core_vx
        corr_vy -= core_vy
        dfr = -corr_vx * 0.866025 + corr_vy * 0.5 + yaw
        dfl = corr_vx * 0.866025 + corr_vy * 0.5 + yaw
        db = -corr_vy + yaw
        scale = fit_wheel_delta_scale(wfr, wfl, wb, dfr, dfl, db, limit)
        if base_scale < 0.999:
            last_alloc_scale = -max(1, int(base_scale * 100.0))
        else:
            extra_scale = scale if scale < preview_scale else preview_scale
            last_alloc_scale = int(extra_scale * 100.0)
        return (
            ox + corr_vx * scale,
            oy + corr_vy * scale,
            oz + yaw * scale,
        )

    if preserve_rotation:
        # Relative heading is the orbit constraint.  Keep the gyro-loop yaw
        # output and fit translation into the wheel headroom that remains.
        vz = clamp(vz, -limit, limit)
        wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(vx, vy, 0.0)
        scale = fit_wheel_delta_scale(
            vz, vz, vz, wheel_fr, wheel_fl, wheel_b, limit
        )
        last_alloc_scale = int(scale * 100.0)
        return vx * scale, vy * scale, vz

    if not preserve_feedforward:
        wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(vx, vy, vz)
        target_max = max_wheel_abs(wheel_fr, wheel_fl, wheel_b)
        if target_max > limit:
            scale = limit / target_max
            vx *= scale
            vy *= scale
            vz *= scale
            last_alloc_scale = int(scale * 100.0)
        else:
            last_alloc_scale = 100
        return vx, vy, vz

    # Keep the master's rigid-motion feedforward, but reserve wheel-speed
    # headroom for the visual correction.  The correction still uses one
    # common scale, so its X/Y direction cannot be distorted by saturation.
    corr_vx = vx - base_vx
    corr_vy = vy - base_vy
    wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(base_vx, base_vy, 0.0)
    base_max = max_wheel_abs(wheel_fr, wheel_fl, wheel_b)
    corr_fr, corr_fl, corr_b = pose_wheel_targets(corr_vx, corr_vy, vz)
    reserve_fr, reserve_fl, reserve_b = pose_wheel_targets(
        feedback_vx,
        feedback_vy,
        vz,
    )
    reserve = max_wheel_abs(reserve_fr, reserve_fl, reserve_b)
    if reserve > Follow_Normal_Allocation_Reserve:
        reserve = Follow_Normal_Allocation_Reserve
    base_limit = limit - reserve
    base_scale = 1.0
    if base_max > base_limit:
        base_scale = base_limit / base_max
        base_vx *= base_scale
        base_vy *= base_scale
        wheel_fr *= base_scale
        wheel_fl *= base_scale
        wheel_b *= base_scale

    scale = fit_wheel_delta_scale(
        wheel_fr, wheel_fl, wheel_b, corr_fr, corr_fl, corr_b, limit
    )
    last_alloc_scale = int(
        (scale if scale < base_scale else base_scale) * 100.0
    )
    return (
        base_vx + corr_vx * scale,
        base_vy + corr_vy * scale,
        vz * scale,
    )


def reset_turn_loop_state():
    global last_cmd_wz

    last_cmd_wz = 0.0
    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        gyro_pid.output = 0.0
        gyro_pid.err = 0.0
        gyro_pid.err_last = 0.0


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
            if b == _Cam_Frame_Head:
                cam_parse_state = 1
        elif cam_parse_state == 1:
            if b != _Cam_Frame_Head:
                cam_parse_b1 = b
                cam_parse_state = 2
        elif cam_parse_state == 2:
            cam_parse_b2 = b
            if cam_parse_b1 == _No_Target_Marker and b == _No_Target_Marker:
                cam_has_target = False
                cam_last_rx_ms = utime.ticks_ms()
                cam_parse_state = 0
            elif cam_parse_b1 == _Line_Packet_Tag or cam_parse_b1 == _Classify_Packet_Tag:
                cam_last_rx_ms = utime.ticks_ms()
                cam_parse_state = 0
            else:
                cam_parse_state = 3
        else:
            if b == _Cam_Frame_Head:
                err_angle = 0
                cam_parse_state = 1
            else:
                err_angle = (b - _Cam_Error_Offset) * _Cam_Error_Scale
                cam_parse_state = 0
            cam_has_target = True
            target_lost_since_ms = 0
            update_cam_target(
                (cam_parse_b1 - _Cam_Error_Offset) * _Cam_Error_Scale,
                (cam_parse_b2 - _Cam_Error_Offset) * _Cam_Error_Scale,
                err_angle,
            )


def handle_coop_frame(msg_type, seq, payload, payload_len):
    global master_vx, master_vy, master_wz
    global master_preview_vx, master_preview_vy
    global master_flags, master_state_code, master_last_rx_ms

    if msg_type != MSG_MASTER_MOTION or payload_len < 9:
        return
    measured_vx = decode_i16(payload, 0) / 10.0
    measured_vy = decode_i16(payload, 2) / 10.0
    master_vx = measured_vx * 0.5 + measured_vy * 0.8660254
    master_vy = measured_vy * 0.5 - measured_vx * 0.8660254
    master_wz = decode_i16(payload, 4) / 10.0
    master_state_code = payload[9] if payload_len >= 10 else -1
    if master_state_code >= _Master_State_Preview_Flag:
        preview_vx = payload[6]
        preview_vy = payload[7]
        if preview_vx >= 128:
            preview_vx -= 256
        if preview_vy >= 128:
            preview_vy -= 256
        master_preview_vx = Follow_Normal_Preview_Gain * (
            preview_vx * 0.25 + preview_vy * 0.4330127
        )
        master_preview_vy = Follow_Normal_Preview_Gain * (
            preview_vy * 0.25 - preview_vx * 0.4330127
        )
        master_state_code -= _Master_State_Preview_Flag
    else:
        master_preview_vx = 0.0
        master_preview_vy = 0.0
    master_flags = payload[8]
    master_last_rx_ms = utime.ticks_ms()


def poll_coop_uart():
    try:
        n = wireless.receive_bytearray(coop_rx_buf, len(coop_rx_buf))
        if n:
            coop_parser.feed(coop_rx_buf, n, handle_coop_frame)
    except Exception:
        pass


def debug_due(now):
    global debug_log_last_ms
    if utime.ticks_diff(now, debug_log_last_ms) < _DEBUG_LOG_PERIOD_MS:
        return False
    debug_log_last_ms = now
    return True


def debug_send(text):
    try:
        wireless.send_str(text)
        wireless.send_str("\r\n")
    except Exception:
        pass


def debug_send_state(log_id, gyro_z):
    debug_send(
        "D %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d"
        % (
            log_id,
            master_flags,
            cam_target_state(),
            last_alloc_scale,
            int(move_cmd.speed_fl * 10.0),
            int(move_cmd.speed_fr * 10.0),
            int(move_cmd.speed_b * 10.0),
            int(last_enc_fl * 10.0),
            int(last_enc_fr * 10.0),
            int(last_enc_b * 10.0),
            cam_error_x,
            cam_error_y,
            cam_error_angle,
            int(gyro_z),
            master_state_code,
            int(last_ff_wz),
            int(last_ff_vx),
            int(last_ff_vy),
        )
    )


def calc_follow_yaw_output(
    gyro_z,
    turn_rate_cmd,
    mode_key,
    seen,
    orbit_settling,
    spin_mode_active,
    orbit_entry_active,
    push_follow_active,
    orbit_mode_active,
    angle_pose_mode_active,
    explicit_orbit,
    explicit_push,
):
    global last_cmd_wz, last_angle_priority_active

    priority_turn_mode = mode_key or angle_pose_mode_active
    gyro_brake_active = False
    if -0.001 < turn_rate_cmd < 0.001:
        turn_rate_cmd = 0.0
        gyro_stop_rate = (
            GYRO_STOP_RATE
            if priority_turn_mode
            else GYRO_NORMAL_STOP_RATE
        )
        if (
            ENABLE_GYRO_LOOP
            and gyro_pid is not None
            and (priority_turn_mode or seen)
            and (
                gyro_z >= gyro_stop_rate
                or gyro_z <= -gyro_stop_rate
            )
        ):
            gyro_brake_active = True
            last_cmd_wz = 0.0
        else:
            reset_turn_loop_state()
    else:
        if spin_mode_active:
            output_ramp = 36.0
        elif orbit_entry_active:
            output_ramp = Follow_Orbit_Entry_Ramp_Wz
        elif priority_turn_mode:
            output_ramp = 18.0
        else:
            output_ramp = 11.0
        turn_rate_cmd = ramp_value(turn_rate_cmd, last_cmd_wz, output_ramp)
        last_cmd_wz = turn_rate_cmd
    if (
        priority_turn_mode
        and (
            turn_rate_cmd >= GYRO_ANGLE_PRIORITY_MIN_CMD
            or turn_rate_cmd <= -GYRO_ANGLE_PRIORITY_MIN_CMD
        )
    ):
        last_angle_priority_active = True
    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        if turn_rate_cmd:
            gyro_pid.gyro_kp = GYRO_KP
            if orbit_settling:
                gyro_pid.gyro_ki = GYRO_TURN_KI
                gyro_pid.gyro_output_limit = GYRO_ORBIT_OUTPUT_LIMIT
            elif spin_mode_active:
                gyro_pid.gyro_ki = GYRO_TURN_KI
                gyro_pid.gyro_output_limit = GYRO_SPIN_OUTPUT_LIMIT
            elif push_follow_active:
                gyro_pid.gyro_ki = GYRO_KI
                gyro_pid.gyro_output_limit = GYRO_PUSH_OUTPUT_LIMIT
            elif orbit_mode_active:
                gyro_pid.gyro_ki = GYRO_TURN_KI
                gyro_pid.gyro_output_limit = GYRO_ORBIT_OUTPUT_LIMIT
            elif angle_pose_mode_active:
                gyro_pid.gyro_ki = GYRO_KI
                gyro_pid.gyro_output_limit = GYRO_NORMAL_POSE_OUTPUT_LIMIT
            else:
                gyro_pid.gyro_ki = GYRO_KI
                gyro_pid.gyro_output_limit = GYRO_OUTPUT_LIMIT
        elif gyro_brake_active and (
            orbit_settling or push_follow_active or not mode_key
        ):
            gyro_pid.gyro_kp = GYRO_KP
            gyro_pid.gyro_ki = 0.0
            gyro_pid.gyro_output_limit = (
                GYRO_ORBIT_OUTPUT_LIMIT
                if orbit_settling
                else (
                    GYRO_PUSH_OUTPUT_LIMIT
                    if push_follow_active
                    else GYRO_OUTPUT_LIMIT
                )
            )
        if turn_rate_cmd or gyro_brake_active:
            gyro_error = turn_rate_cmd - gyro_z
            if (
                (
                    orbit_settling
                    or not mode_key
                    or (explicit_orbit and not explicit_push)
                    or push_follow_active
                )
                and gyro_pid.err_last * gyro_error < 0.0
            ):
                gyro_pid.output = 0.0
                gyro_pid.err_last = 0.0
            return _pid_mod.gyro_ctrl(gyro_pid, gyro_error)
        gyro_pid.gyro_kp = GYRO_KP
        gyro_pid.gyro_ki = GYRO_KI
        gyro_pid.output = 0.0
        gyro_pid.err = 0.0
        gyro_pid.err_last = 0.0
        last_cmd_wz = 0.0
        return 0.0
    return turn_rate_cmd


def update_follow_targets(gyro_z):
    global cam_target_vx, cam_target_vy, target_lost_since_ms
    global last_turn_rate_cmd
    global last_follow_seen
    global last_ff_vx, last_ff_vy, last_ff_wz
    global last_cmd_vx, last_cmd_vy, last_cmd_wz
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    global last_stall_count, last_stall_boost
    global follow_output_limit
    global last_angle_priority_active
    global last_follow_mode_key, _orbit
    global last_control_master_state
    global master_edge_until_ms
    global orbit_follow_entry_until_ms

    now = utime.ticks_ms()
    target_state = cam_target_state()
    seen = target_state == 1
    fresh_motion = master_motion_fresh()
    if fresh_motion:
        measured_ff_vx = master_vx
        measured_ff_vy = master_vy
        ff_vx = measured_ff_vx + master_preview_vx
        ff_vy = measured_ff_vy + master_preview_vy
    else:
        measured_ff_vx = 0.0
        measured_ff_vy = 0.0
        ff_vx = 0.0
        ff_vy = 0.0
    ff_wz = master_wz if fresh_motion else 0.0
    explicit_orbit = fresh_motion and (master_flags & MASTER_MOTION_FLAG_ORBIT)
    explicit_push = fresh_motion and (master_flags & MASTER_MOTION_FLAG_PUSH)
    explicit_spin = fresh_motion and (master_flags & MASTER_MOTION_FLAG_SPIN)
    spin_ff_wz = update_spin_feedforward_latch(
        now,
        fresh_motion,
        explicit_spin,
        ff_wz,
        seen,
        cam_error_angle,
    )
    spin_mode_active = explicit_spin
    filtered_wz = update_filtered_ff_wz(spin_ff_wz if spin_mode_active else ff_wz, fresh_motion)
    orbit_mode_active = update_orbit_follow_mode(
        now,
        gyro_z,
        explicit_orbit,
        explicit_push,
        spin_mode_active,
        fresh_motion and (master_flags & MASTER_MOTION_FLAG_BACK),
    )
    orbit_settling = orbit_mode_active and orbit_follow_exit_since_ms != 0
    moving_visual_scale = 1.0
    if orbit_mode_active and explicit_push:
        # PUSH remains on its previous protection until it is retuned for the
        # new encoder.  ORBIT must keep relative heading feedback active at
        # high rate; otherwise the rate mismatch integrates into yaw error.
        gyro_abs = abs(gyro_z)
        if gyro_abs >= 120.0:
            moving_visual_scale = 0.0
        elif gyro_abs > 40.0:
            moving_visual_scale = (120.0 - gyro_abs) / 80.0
    if orbit_mode_active:
        follow_ff_wz = 0.0 if orbit_settling else filtered_wz
    elif spin_mode_active:
        follow_ff_wz = spin_ff_wz
    else:
        follow_ff_wz = clamp(
            filtered_wz,
            -Follow_Normal_Wz_Feedforward_Limit,
            Follow_Normal_Wz_Feedforward_Limit,
        )
    push_follow_active = explicit_push and orbit_mode_active
    classification_exit = (
        fresh_motion
        and last_control_master_state == 4
        and master_state_code != 4
    )
    mode_key = 3 if spin_mode_active else (1 if orbit_mode_active else 0)
    _orbit = mode_key == 1
    if last_follow_mode_key != mode_key:
        if last_follow_mode_key == 1 and not mode_key:
            speed_reset(pid_fl)
            speed_reset(pid_fr)
            speed_reset(pid_b)
            reset_turn_loop_state()
            follow_ff_wz = 0.0
        elif last_follow_mode_key > 0 and not mode_key:
            reset_speed_outputs()
            follow_ff_wz = 0.0
        elif last_follow_mode_key == 0 and mode_key == 1:
            speed_reset(pid_fl)
            speed_reset(pid_fr)
            speed_reset(pid_b)
            reset_turn_loop_state()
        elif last_follow_mode_key < 0:
            reset_speed_outputs(True)
            reset_turn_loop_state()
        if mode_key == 1 and explicit_orbit and not explicit_push:
            orbit_follow_entry_until_ms = utime.ticks_add(
                now,
                _Follow_Orbit_Entry_Hold_Ms,
            )
        else:
            orbit_follow_entry_until_ms = 0
        if not mode_key:
            master_edge_until_ms = 0
    if (
        orbit_follow_entry_until_ms
        and utime.ticks_diff(orbit_follow_entry_until_ms, now) <= 0
    ):
        orbit_follow_entry_until_ms = 0
    orbit_entry_active = (
        explicit_orbit
        and not explicit_push
        and orbit_follow_entry_until_ms
    )
    follow_output_limit = _FOLLOW_RUN_PWM_LIMIT
    if (
        not mode_key
        and -Follow_Master_Edge_Delta < ff_vx < Follow_Master_Edge_Delta
        and -Follow_Master_Edge_Delta < ff_vy < Follow_Master_Edge_Delta
        and -Follow_Master_Edge_Delta < follow_ff_wz < Follow_Master_Edge_Delta
        and (
            not (master_flags & 1)
            or (-10 < cam_error_x < 10 and -10 < cam_error_y < 10)
        )
    ):
        follow_output_limit = _FOLLOW_STATIC_LOCK_PWM_LIMIT
    if push_follow_active:
        update_push_reacquire(now, seen)
    elif mode_key:
        master_edge_until_ms = 0
    last_follow_mode_key = mode_key
    if (
        spin_mode_active
        and follow_ff_wz
        and last_cmd_wz * follow_ff_wz < 0.0
    ):
        reset_turn_loop_state()
    body_vx = 0.0
    body_vy = 0.0
    turn_rate_cmd = 0.0
    use_motion_feedforward = False
    angle_priority_active = False
    position_priority_active = False
    prev_angle_priority_active = last_angle_priority_active
    alloc_base_vx = 0.0
    alloc_base_vy = 0.0
    safety_vx = 0.0
    lost_scale = 1.0
    visual_scale = 1.0

    if seen:
        target_lost_since_ms = 0
        if not mode_key:
            visual_scale = normal_visual_scale(now, True)
        control_error_angle = cam_error_angle * visual_scale
        angle_pose_mode_active = angle_pose_mode_needed(
            control_error_angle,
            orbit_mode_active,
            spin_mode_active,
        )
        use_motion_feedforward = (
            fresh_motion
            and (not push_follow_active)
            and (not orbit_settling)
        )
        (
            vx,
            vy,
            turn_rate_cmd,
            body_vx,
            body_vy,
            angle_priority_active,
            position_priority_active,
        ) = solve_follow_pose_twist(
            cam_error_x * visual_scale,
            cam_error_y * visual_scale,
            control_error_angle,
            ff_vx,
            ff_vy,
            follow_ff_wz,
            use_motion_feedforward,
            orbit_mode_active,
            spin_mode_active,
            push_follow_active,
            moving_visual_scale,
        )
        if follow_output_limit == _FOLLOW_STATIC_LOCK_PWM_LIMIT:
            vx -= body_vx * (1.0 - Follow_Static_Visual_Scale)
            vy -= body_vy * (1.0 - Follow_Static_Visual_Scale)
            body_vx *= Follow_Static_Visual_Scale
            body_vy *= Follow_Static_Visual_Scale
        if orbit_settling:
            # Converge translation, relative angle and yaw rate together.
            # Waiting for a low-rate pre-phase can repeatedly reset on an
            # overshoot and leave the relative pose uncontrolled.
            settle_ff_vx = clamp(
                ff_vx * Follow_Feedforward_Forward_Gain,
                -Follow_Feedforward_Forward_Limit,
                Follow_Feedforward_Forward_Limit,
            )
            settle_ff_vy = clamp(
                ff_vy * Follow_Feedforward_Lateral_Gain,
                -Follow_Feedforward_Lateral_Limit,
                Follow_Feedforward_Lateral_Limit,
            )
            body_vx = clamp(
                vx,
                -Follow_Orbit_Settle_XY_Limit,
                Follow_Orbit_Settle_XY_Limit,
            )
            body_vy = clamp(
                vy,
                -Follow_Orbit_Settle_XY_Limit,
                Follow_Orbit_Settle_XY_Limit,
            )
            vx = settle_ff_vx + body_vx
            vy = settle_ff_vy + body_vy
            turn_rate_cmd = clamp(
                turn_rate_cmd,
                -Follow_Orbit_Settle_Turn_Limit,
                Follow_Orbit_Settle_Turn_Limit,
            )
        normal_damping_active = (
            not mode_key and master_flags < MASTER_MOTION_FLAG_ORBIT
        )
        orbit_damping_active = explicit_orbit and not explicit_push
        if normal_damping_active:
            alloc_base_vx = measured_ff_vx
            alloc_base_vy = measured_ff_vy
            body_vx = vx - alloc_base_vx
            body_vy = vy - alloc_base_vy
        else:
            alloc_base_vx = vx - body_vx
            alloc_base_vy = vy - body_vy
        if normal_damping_active or orbit_damping_active:
            # Dampen motion relative to the preserved master feedforward.
            actual_body_vx = (last_enc_fl - last_enc_fr) * 0.5773503
            actual_body_vy = (
                last_enc_fl + last_enc_fr - 2.0 * last_enc_b
            ) / 3.0
            if orbit_damping_active:
                velocity_damping = Follow_Orbit_Velocity_Damping
            elif follow_output_limit < _FOLLOW_RUN_PWM_LIMIT:
                velocity_damping = Follow_Static_Velocity_Damping
            else:
                velocity_damping = Follow_Normal_Velocity_Damping
            damp_vx = -velocity_damping * (
                actual_body_vx - alloc_base_vx
            )
            damp_vy = -velocity_damping * (
                actual_body_vy - alloc_base_vy
            )
            damping_max = abs(damp_vx)
            damping_tmp = abs(damp_vy)
            if damping_tmp > damping_max:
                damping_max = damping_tmp
            if damping_max > Follow_Normal_Correction_Reserve:
                damping_scale = Follow_Normal_Correction_Reserve / damping_max
                damp_vx *= damping_scale
                damp_vy *= damping_scale
            body_vx += damp_vx
            body_vy += damp_vy
            # Do not open the distance loop during a fast turn.  The final
            # command ramp already limits how quickly this correction changes;
            # suppressing it here lets a real close-distance error accumulate.
            vx = alloc_base_vx + body_vx
            vy = alloc_base_vy + body_vy
            if normal_damping_active:
                speed_ref = abs(measured_ff_vx)
                if abs(measured_ff_vy) > speed_ref:
                    speed_ref = abs(measured_ff_vy)
                safety_trigger = (
                    -_Follow_Distance_Far_Boost_Error
                    + speed_ref * Follow_Safety_Speed_Margin
                )
                if cam_error_x < safety_trigger:
                    safety_vx = clamp(
                        (safety_trigger - cam_error_x)
                        * Follow_Distance_Close_Gain
                        + max(0.0, measured_ff_vx - actual_body_vx)
                        * Follow_Normal_Velocity_Damping,
                        0.0,
                        Follow_Normal_Allocation_Reserve
                        + Follow_Normal_Correction_Reserve,
                    )
                    if body_vx < safety_vx:
                        body_vx = safety_vx
                        vx = alloc_base_vx + body_vx
        if push_follow_active:
            old_body_vx = body_vx
            body_vx *= moving_visual_scale
            vx += body_vx - old_body_vx
        if mode_key:
            position_priority_active = True
        last_angle_priority_active = angle_priority_active or angle_pose_mode_active
    else:
        angle_pose_mode_active = mode_key
        if not mode_key:
            normal_visual_scale(now, False)
        if target_lost_since_ms == 0:
            target_lost_since_ms = now
        if not mode_key:
            lost_scale = lost_motion_scale(now, target_state)
        if fresh_motion and not orbit_settling and not mode_key and lost_scale:
            use_motion_feedforward = True
        elif fresh_motion and not orbit_settling:
            use_motion_feedforward = utime.ticks_diff(
                now,
                target_lost_since_ms,
            ) <= (
                180
                if push_follow_active
                else (
                    300
                    if not mode_key
                    else _Follow_Target_Lost_Hold_Ms
                )
            )
        if use_motion_feedforward:
            if not mode_key:
                ff_vx = measured_ff_vx
                ff_vy = measured_ff_vy
                vx = ff_vx * lost_scale
                vy = ff_vy * lost_scale
                if (
                    cam_error_x < -_Follow_Distance_Far_Boost_Error
                    and vx < 0.0
                ):
                    vx = 0.0
                alloc_base_vx = vx
                alloc_base_vy = vy
            else:
                xy_scale = (
                    0.65
                    if push_follow_active
                    else Follow_Hold_Feedforward_Gain
                )
                vx = ff_vx * (
                    1.0
                    if master_flags & MASTER_MOTION_FLAG_RETURN
                    else xy_scale
                )
                vy = ff_vy * xy_scale
            if orbit_mode_active and (master_flags & MASTER_MOTION_FLAG_RETURN):
                vx += follow_ff_wz * Follow_Orbit_Target_Point_Wz_To_Vx * Follow_Orbit_Feedforward_Forward_Gain
                vy += follow_ff_wz * Follow_Orbit_Target_Point_Wz_To_Vy * Follow_Orbit_Feedforward_Lateral_Gain
        else:
            vx = 0.0
            vy = 0.0
        if prev_angle_priority_active:
            reset_turn_loop_state()
        last_angle_priority_active = False

    if (
        push_follow_active
        and master_edge_until_ms
        and body_vx < -22.0
    ):
        vx = -22.0
        body_vx = vx

    if push_follow_active and seen:
        # Keep the leader motion as the allocation base.  Visual correction
        # may use the remaining wheel headroom but must not reduce that base.
        alloc_base_vx = clamp(
            ff_vx * Follow_Push_Feedforward_Forward_Gain,
            -Follow_Push_Feedforward_Forward_Limit,
            Follow_Push_Feedforward_Forward_Limit,
        )
        alloc_base_vy = clamp(
            ff_vy * 1.20,
            -Follow_Lateral_Limit,
            Follow_Lateral_Limit,
        )
        vx = alloc_base_vx + body_vx
        vy = alloc_base_vy + body_vy
        turn_rate_cmd = add_feedforward_direct(
            turn_rate_cmd,
            follow_ff_wz,
            Follow_Orbit_Wz_Feedforward_Gain,
            Follow_Push_Wz_Feedforward_Limit,
        )
        turn_rate_cmd = clamp(
            turn_rate_cmd,
            -Follow_Push_Turn_Rate_Limit,
            Follow_Push_Turn_Rate_Limit,
        )

    vx_limit = Follow_Forward_Limit
    if master_flags & MASTER_MOTION_FLAG_RETURN:
        vy_limit = Follow_Return_Lateral_Limit
    elif push_follow_active:
        vy_limit = 29.0 if cam_error_y <= -16 else 27.0
    else:
        vy_limit = 32.0
    if fresh_motion:
        vx_limit = follow_limit(vx_limit, ff_vx)
        vy_limit = follow_limit(vy_limit, ff_vy)
    vx = clamp(vx, -vx_limit, vx_limit)
    vy = clamp(vy, -vy_limit, vy_limit)
    if seen and not mode_key and master_flags < MASTER_MOTION_FLAG_ORBIT:
        if (
            vx * actual_body_vx < -0.001
            and abs(actual_body_vx) > Follow_Normal_Reverse_Release_Speed
        ):
            vx = 0.0
        if (
            vy * actual_body_vy < -0.001
            and abs(actual_body_vy) > Follow_Normal_Reverse_Release_Speed
        ):
            vy = 0.0
    if orbit_entry_active:
        vx_ramp = Follow_Orbit_Entry_Ramp_Vx
        vy_ramp = Follow_Orbit_Entry_Ramp_Vy
    elif position_priority_active:
        vx_ramp = Follow_Orbit_Command_Ramp_Vx
        vy_ramp = Follow_Orbit_Command_Ramp_Vy
    else:
        vx_ramp = Follow_Command_Ramp_Vx
        vy_ramp = Follow_Command_Ramp_Vy
    if not (classification_exit and not mode_key):
        vx = ramp_value(vx, last_cmd_vx, vx_ramp)
        vy = ramp_value(vy, last_cmd_vy, vy_ramp)
    # A normal state-4 exit can apply the new leader feedforward immediately,
    # because vision correction stayed continuous.  ORBIT keeps its dedicated
    # entry ramp so the first high-rate command cannot kick the follower.
    last_cmd_vx = vx
    last_cmd_vy = vy
    cam_target_vx = vx
    cam_target_vy = vy

    if (not seen) and use_motion_feedforward:
        if orbit_mode_active:
            wz_limit = (
                Follow_Push_Wz_Feedforward_Limit
                if push_follow_active
                else Follow_Orbit_Wz_Feedforward_Limit
            )
            turn_rate_cmd = clamp(
                follow_ff_wz * Follow_Orbit_Wz_Feedforward_Gain,
                -wz_limit,
                wz_limit,
            )
        elif spin_mode_active:
            turn_rate_cmd = clamp(
                follow_ff_wz * Follow_Spin_Wz_Feedforward_Gain,
                -Follow_Spin_Wz_Feedforward_Limit,
                Follow_Spin_Wz_Feedforward_Limit,
            )
        else:
            turn_rate_cmd = follow_ff_wz * Follow_Normal_Wz_Feedforward_Gain
            turn_rate_cmd *= lost_scale
    vz_cmd = calc_follow_yaw_output(
        gyro_z,
        turn_rate_cmd,
        mode_key,
        seen,
        orbit_settling,
        spin_mode_active,
        orbit_entry_active,
        push_follow_active,
        orbit_mode_active,
        angle_pose_mode_active,
        explicit_orbit,
        explicit_push,
    )

    cam_target_vx, cam_target_vy, vz_cmd = limit_pose_twist_for_wheels(
        cam_target_vx,
        cam_target_vy,
        vz_cmd,
        alloc_base_vx,
        alloc_base_vy,
        body_vx,
        body_vy,
        safety_vx,
        master_preview_vx if seen and fresh_motion and not mode_key else 0.0,
        master_preview_vy if seen and fresh_motion and not mode_key else 0.0,
        fresh_motion
        and (
            (0 < master_flags < 8 and not mode_key)
            or (seen and push_follow_active)
        ),
        explicit_orbit and not explicit_push,
    )
    last_cmd_vx = cam_target_vx
    last_cmd_vy = cam_target_vy
    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        gyro_pid.output = vz_cmd

    last_turn_rate_cmd = turn_rate_cmd
    last_follow_seen = seen
    last_ff_vx = ff_vx
    last_ff_vy = ff_vy
    last_ff_wz = follow_ff_wz
    last_control_master_state = master_state_code
    return vz_cmd


def stop_all():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)


def reset_speed_outputs(keep_orbit_state=False):
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    global last_stall_count, last_stall_boost
    global last_cmd_vx, last_cmd_vy, last_cmd_wz
    global last_angle_priority_active
    global orbit_follow_active, orbit_follow_exit_since_ms
    global orbit_follow_settle_since_ms, filtered_ff_wz
    global orbit_follow_entry_until_ms
    global spin_latched_wz, spin_latch_until_ms
    global last_follow_mode_key
    global last_control_master_state
    global master_edge_until_ms
    global last_alloc_scale

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
    last_angle_priority_active = False
    last_alloc_scale = 100
    if not keep_orbit_state:
        orbit_follow_active = False
        orbit_follow_exit_since_ms = 0
        orbit_follow_settle_since_ms = 0
        orbit_follow_entry_until_ms = 0
        filtered_ff_wz = 0.0
        last_follow_mode_key = -1
        last_control_master_state = -1
    spin_latched_wz = 0.0
    spin_latch_until_ms = 0
    master_edge_until_ms = 0


def smooth_value(target, last):
    delta = target - last
    if abs(delta) > MAX_PWM_CHANGE:
        target = last + MAX_PWM_CHANGE * (1 if delta > 0 else -1)
    target = int(last * (1.0 - PWM_SMOOTH_FACTOR) + target * PWM_SMOOTH_FACTOR)
    return clamp(int(target), -MOTOR_DUTY_MAX, MOTOR_DUTY_MAX)


def apply_start_pwm(cmd, min_pwm):
    cmd = int(cmd)
    if 0 < cmd < min_pwm:
        cmd = min_pwm
    elif -min_pwm < cmd < 0:
        cmd = -min_pwm
    return clamp(cmd, -_FOLLOW_RUN_PWM_LIMIT, _FOLLOW_RUN_PWM_LIMIT)


def follow_start_pwm_for_target(target, stall_boost):
    target_abs = abs(target)
    if wheel_target_idle(target):
        return 0
    if stall_boost and target_abs >= 2.8:
        return 8800
    if (
        follow_output_limit < _FOLLOW_RUN_PWM_LIMIT
        and target_abs < 2.8
    ):
        return 2600
    return 3600 if target_abs < 2.8 else 6200


def follow_channel_pwm(cmd, target, speed_err, stall_boost, last_pwm):
    fast_reverse = (
        last_follow_mode_key == 3
        or (
            _orbit
            and (master_flags & MASTER_MOTION_FLAG_PUSH)
            and master_edge_until_ms
        )
    )
    min_pwm = follow_start_pwm_for_target(target, stall_boost)
    if min_pwm <= 0:
        if not _orbit:
            if follow_output_limit < _FOLLOW_RUN_PWM_LIMIT and cmd:
                if last_pwm * cmd < 0:
                    return 0
                return smooth_value(cmd, last_pwm)
            return 0
        return _pid_mod.follow_low_pwm(
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
    u_fl = motor_fl.duty(s_fl, MOTOR_DUTY_MIN)
    u_fr = motor_fr.duty(s_fr, MOTOR_DUTY_MIN)
    u_b = motor_b.duty(s_b, MOTOR_DUTY_MIN)
    last_pwm_fl = u_fl
    last_pwm_fr = u_fr
    last_pwm_b = u_b


def set_three_pwm_zero():
    global last_pwm_fl, last_pwm_fr, last_pwm_b

    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)
    last_pwm_fl = 0
    last_pwm_fr = 0
    last_pwm_b = 0


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


def speed_ctrl_follow(pid, motor, actual_speed, target_speed):
    if wheel_target_idle(target_speed):
        if not _orbit:
            speed_reset(pid)
            if (
                follow_output_limit < _FOLLOW_RUN_PWM_LIMIT
                and (
                    actual_speed > WHEEL_TARGET_NORMAL_IDLE_EPS
                    or actual_speed < -WHEEL_TARGET_NORMAL_IDLE_EPS
                )
            ):
                return clamp(
                    -actual_speed * 1450.0,
                    -6000.0,
                    6000.0,
                )
            return 0.0
    return _pid_mod.speed_ctrl(
        pid,
        actual_speed,
        target_speed,
        motor.direction_waiting(target_speed),
    )


def calibrate_gyro_before_launch():
    if (
        AUTO_CALIBRATE_GYRO_ON_LAUNCH
        and ENABLE_IMU
        and imu_runtime is not None
    ):
        pit1.stop()
        try:
            imu_runtime.calibrate_offset(
                samples=_GYRO_CALIBRATE_SAMPLES,
                delay_ms=_GYRO_CALIBRATE_DELAY_MS,
                logger=None,
            )
            imu_runtime.reset_yaw(0.0)
        finally:
            pit1.start(TICK_PERIOD_MS)


def start_follow():
    global car_started

    if car_started:
        return
    calibrate_gyro_before_launch()
    clear_cam_target_state()
    car_started = True
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
    global last_stall_count, last_stall_boost
    global cam_target_vx, cam_target_vy
    global last_enc_fl, last_enc_fr, last_enc_b

    if not car_started:
        reset_speed_outputs()
        set_three_pwm_zero()
        return None

    if master_motion_fresh() and not (master_flags & MASTER_MOTION_FLAG_STARTED):
        reset_speed_outputs()
        set_three_pwm_zero()
        return None

    if ENABLE_IMU:
        gyro_z = imu_runtime.read_gyro_z()
    else:
        gyro_z = 0.0

    vz_cmd = update_follow_targets(gyro_z)
    calc_wheel_spd(move_cmd, cam_target_vx, cam_target_vy, vz_cmd)

    e_fl = enc_fl.get() * ENC_SCALE
    e_fr = enc_fr.get() * ENC_SCALE
    e_b = enc_b.get() * ENC_SCALE
    t_fl = move_cmd.speed_fl
    t_fr = move_cmd.speed_fr
    t_b = move_cmd.speed_b
    last_enc_fl = e_fl
    last_enc_fr = e_fr
    last_enc_b = e_b

    if (
        -WHEEL_TARGET_STOP_EPS <= t_fl <= WHEEL_TARGET_STOP_EPS
        and -WHEEL_TARGET_STOP_EPS <= t_fr <= WHEEL_TARGET_STOP_EPS
        and -WHEEL_TARGET_STOP_EPS <= t_b <= WHEEL_TARGET_STOP_EPS
    ):
        reset_speed_outputs(orbit_follow_active)
        set_three_pwm_zero()
    else:
        if (
            wheel_target_idle(t_fl)
            and wheel_target_idle(t_fr)
            and wheel_target_idle(t_b)
        ):
            last_stall_count = 0
        elif e_fl == 0 and e_fr == 0 and e_b == 0:
            last_stall_count += 1
        else:
            last_stall_count = 0
        last_stall_boost = last_stall_count >= 3

        u_fl = speed_ctrl_follow(pid_fl, motor_fl, e_fl, t_fl)
        u_fr = speed_ctrl_follow(pid_fr, motor_fr, e_fr, t_fr)
        u_b = speed_ctrl_follow(pid_b, motor_b, e_b, t_b)

        set_three_pwm_follow(
            u_fl, u_fr, u_b, t_fl, t_fr, t_b, last_stall_boost
        )

    now_log = utime.ticks_ms()
    if debug_due(now_log):
        log_id = now_log & 0x7FFF
        debug_send_state(log_id, gyro_z)

key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)

utime.sleep_ms(100)
led = Pin(cfg.LED_HB_PIN, Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

motor_fl = Motor(cfg.MOTOR_FL_PH, cfg.MOTOR_FL_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FL_INVERT)
motor_fr = Motor(cfg.MOTOR_FR_PH, cfg.MOTOR_FR_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FR_INVERT)
motor_b = Motor(cfg.MOTOR_B_PH, cfg.MOTOR_B_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_B_INVERT)
enc_fl = encoder(cfg.ENC_FL_DIR, cfg.ENC_FL_PULSE, cfg.ENC_FL_INVERT)
enc_fr = encoder(cfg.ENC_FR_DIR, cfg.ENC_FR_PULSE, cfg.ENC_FR_INVERT)
enc_b = encoder(cfg.ENC_B_DIR, cfg.ENC_B_PULSE, cfg.ENC_B_INVERT)

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

        if (
            not car_started
            and cam_has_target
            and utime.ticks_diff(utime.ticks_ms(), cam_last_rx_ms)
            <= _Cam_Packet_Timeout_Ms
        ):
            start_follow()

        if pit_flag:
            pit_flag = False
            calc_speed_closed_loop()

        if utime.ticks_diff(now, last_status_ms) >= 1000:
            led.toggle()
            last_status_ms = now

        if loop_count % _GC_DIV == 0:
            gc.collect()

        utime.sleep_ms(1)

finally:
    pit1.stop()
    cam_uart.write(ART_MODE_IDLE_CMD)
    stop_all()
    led.value(True)
