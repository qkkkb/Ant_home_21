from machine import Pin, UART
from micropython import const
import gc
import utime
from smartcar import ticker, encoder
from seekfree import WIRELESS_UART
from lsm6dsv16x_gyro_runtime import LSM6DSV16XYawRuntime
from move_base import calc_wheel_spd_into
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

gc.collect()

speed_reset = _pid_mod.speed_reset


# ====================== Base config ======================
TICK_PERIOD_MS = cfg.TICK_PERIOD_MS
ENC_SCALE = cfg.ENC_SCALE
MOTOR_DUTY_MAX = cfg.MOTOR_DUTY_MAX
MOTOR_DUTY_MIN = cfg.MOTOR_DUTY_MIN
PWM_SMOOTH_FACTOR = cfg.PWM_SMOOTH_FACTOR
MAX_PWM_CHANGE = cfg.MAX_PWM_CHANGE

GYRO_KP = 0.04
GYRO_KI = 0.004
GYRO_TURN_KI = 0.002
GYRO_OUTPUT_LIMIT = 12.0
GYRO_ORBIT_OUTPUT_LIMIT = 18.0
GYRO_ANGLE_PRIORITY_MIN_CMD = 6.5

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
_Follow_Orbit_Position_X_Error = const(6)
_Follow_Orbit_Position_Y_Error = const(6)
_Follow_Distance_Far_Boost_Error = const(6)
Follow_Distance_Far_Boost_Gain = 0.70
Follow_Distance_Close_Gain = 0.70
Follow_Distance_Close_Limit = 22.0
Follow_Feedforward_Forward_Gain = 1.00
Follow_Feedforward_Lateral_Gain = 1.00
Follow_Feedforward_Forward_Limit = 35.0
Follow_Feedforward_Lateral_Limit = 42.0
Follow_Normal_Visual_Forward_Scale = 0.72
Follow_Static_Visual_Scale = 0.60
Follow_Hold_Feedforward_Gain = 1.70
Follow_Normal_Wz_Feedforward_Gain = 1.00
Follow_Orbit_Wz_Feedforward_Gain = 1.00
Follow_Orbit_Wz_Feedforward_Limit = 150.0
Follow_Orbit_Turn_Rate_Limit = 180.0
Follow_Target_Point_Wz_To_Vx = -0.18
Follow_Target_Point_Wz_To_Vy = -0.45
Follow_Orbit_Target_Point_Wz_To_Vx = -0.17
Follow_Orbit_Target_Point_Wz_To_Vy = -0.28
Follow_Orbit_Feedforward_Forward_Gain = 1.30
Follow_Orbit_Feedforward_Lateral_Gain = 0.76
Follow_Orbit_Feedforward_Forward_Limit = 14.0
Follow_Orbit_Feedforward_Lateral_Limit = 20.0
Follow_Orbit_Forward_Command_Limit = 26.0
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
Follow_Normal_Allocation_Reserve = 8.0
Follow_Normal_Correction_Reserve = 6.0
Follow_Normal_Conflict_Start_Error = 4
Follow_Normal_Conflict_Stop_Error = 16
Follow_Normal_Preview_Gain = 0.50
Follow_Normal_Velocity_Damping = 0.55
Follow_Push_Velocity_Damping = 0.75
_Follow_Push_Emergency_Error = const(48)
Follow_Safety_Speed_Margin = 0.12
_Master_State_Preview_Flag = const(0x80)
Follow_Static_Velocity_Damping = 0.85
Follow_Orbit_Velocity_Damping = 0.55
Follow_Normal_Reverse_Release_Speed = 3.0
Follow_Command_Ramp_Vx = 2.0
Follow_Command_Ramp_Vy = 3.0
Follow_Orbit_Command_Ramp_Vx = 1.0
Follow_Orbit_Command_Ramp_Vy = 1.5
Follow_Orbit_Command_Ramp_Wz = 6.0
Follow_Orbit_Entry_Ramp_Vx = 0.25
Follow_Orbit_Entry_Ramp_Vy = 0.35
Follow_Orbit_Entry_Ramp_Wz = 3.0
_Follow_Orbit_Entry_Hold_Ms = const(400)
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
master_orbit_wz = 0.0
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
    strict_follow,
    explicit_spin,
    explicit_back,
):
    global orbit_follow_active, orbit_follow_exit_since_ms
    global orbit_follow_settle_since_ms

    if explicit_spin or explicit_back or strict_follow:
        if orbit_follow_active or orbit_follow_exit_since_ms:
            follow_state[0] = follow_state[1] = 0.0
            follow_state[2] = follow_state[3] = 0
        orbit_follow_active = False
        orbit_follow_exit_since_ms = orbit_follow_settle_since_ms = 0
        follow_state[4] = follow_state[5] = 0
        return False
    if explicit_orbit:
        if not orbit_follow_active or orbit_follow_exit_since_ms:
            follow_state[0] = follow_state[1] = 0.0
            follow_state[2] = follow_state[3] = 0
            follow_state[4] = follow_state[5] = 0.0
        orbit_follow_active = True
        orbit_follow_exit_since_ms = orbit_follow_settle_since_ms = 0
        return True

    if not orbit_follow_active:
        orbit_follow_exit_since_ms = orbit_follow_settle_since_ms = 0
        follow_state[4] = follow_state[5] = 0
        return False

    if orbit_follow_exit_since_ms == 0:
        orbit_follow_exit_since_ms = now
        orbit_follow_settle_since_ms = 0
        follow_state[4] = cam_error_x
        follow_state[5] = cam_error_y
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
        follow_state[4] = follow_state[5] = 0
    return orbit_follow_active


def calc_follow_angle(error_angle, orbit_mode=False, spin_mode=False, push_mode=False):
    if orbit_mode or spin_mode:
        deadband = _Follow_Pose_Angle_Deadband
        full_error = _Follow_Pose_Angle_Active_Error
    elif push_mode:
        deadband = 4
        full_error = 12
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
    out,
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
    vision_wz = calc_follow_angle(error_angle, orbit_mode, spin_mode, push_mode)
    if not orbit_mode and not spin_mode:
        vision_wz = clamp(
            vision_wz,
            -Follow_Normal_Pose_Angle_Limit,
            Follow_Normal_Pose_Angle_Limit,
        )
    active_error = _Follow_Pose_Angle_Active_Error if (
        orbit_mode or spin_mode
    ) else (12 if push_mode else _Follow_Normal_Pose_Angle_Active_Error)
    angle_active = (
        error_angle >= active_error
        or error_angle <= -active_error
    )
    # Vision already reports independent position and heading errors.
    cam_vx = error_x
    cam_vy = error_y
    if push_mode:
        cam_vx += 4
        cam_vy += 4
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
    deadband = (
        _Follow_Orbit_Forward_Deadband
        if position_priority
        else _Follow_Forward_Deadband
    )
    if push_mode or follow_output_limit == _FOLLOW_STATIC_LOCK_PWM_LIMIT:
        deadband = 1
    forward_error = soft_deadband(cam_vx, deadband, deadband * 2)
    if forward_error == 0.0:
        body_vx = 0.0
    elif forward_error > 0:
        gain = (
            Follow_Orbit_Forward_Gain
            if position_priority
            else Follow_Forward_Gain
        )
        body_vx = forward_error * gain
        if forward_error > _Follow_Distance_Far_Boost_Error:
            body_vx += (
                forward_error - _Follow_Distance_Far_Boost_Error
            ) * Follow_Distance_Far_Boost_Gain
        body_vx = clamp(body_vx, 0.0, Follow_Forward_Limit)
    else:
        body_vx = clamp(
            forward_error * Follow_Distance_Close_Gain,
            -Follow_Distance_Close_Limit,
            0.0,
        )
    body_vx = -body_vx
    deadband = (
        _Follow_Orbit_Lateral_Deadband
        if position_priority
        else _Follow_Lateral_Deadband
    )
    if push_mode or follow_output_limit == _FOLLOW_STATIC_LOCK_PWM_LIMIT:
        deadband = 1
    body_vy = soft_deadband(cam_vy, deadband, deadband * 2)
    if body_vy != 0.0:
        gain = (
            Follow_Orbit_Lateral_Gain
            if position_priority
            else Follow_Lateral_Gain
        )
        body_vy = clamp(
            body_vy * gain,
            -Follow_Lateral_Limit,
            Follow_Lateral_Limit,
        )
    if (not orbit_mode) and (not spin_mode):
        body_vx *= Follow_Normal_Visual_Forward_Scale
    if (
        use_ff
        and not spin_mode
        and not push_mode
        and ff_vx == 0.0
        and ff_vy == 0.0
    ):
        if -8.0 < body_vx < 8.0:
            body_vx *= 1.5
        if -8.0 < body_vy < 8.0:
            body_vy *= 1.5
    vx = body_vx
    vy = body_vy
    wz = vision_wz
    if use_ff:
        if orbit_mode:
            if master_state_code == 5 or master_state_code == 16:
                _pid_mod.orbit_translation(
                    out, body_vx, body_vy, ff_vx, ff_vy, ff_wz,
                    master_state_code == 5,
                )
                vx = out[0]
                vy = out[1]
                body_vx = out[3]
                body_vy = out[4]
            else:
                ff_scale = _pid_mod.return_orbit_scale(cam_vy, cam_vx)
                vx = add_feedforward_direct(
                    vx,
                    ff_vx * ff_scale,
                    Follow_Orbit_Feedforward_Forward_Gain,
                    Follow_Orbit_Feedforward_Forward_Limit,
                    0.25,
                )
                vy = add_feedforward_direct(
                    vy,
                    ff_vy * ff_scale,
                    Follow_Orbit_Feedforward_Lateral_Gain,
                    Follow_Orbit_Feedforward_Lateral_Limit,
                    0.25,
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
            if push_mode or 0 < master_flags < MASTER_MOTION_FLAG_ORBIT:
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
    out[0] = vx
    out[1] = vy
    out[2] = wz
    out[3] = body_vx
    out[4] = body_vy
    out[5] = angle_active
    out[6] = position_priority


def reset_turn_loop_state():
    global last_cmd_wz

    last_cmd_wz = 0.0
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
    global master_vx, master_vy, master_wz, master_orbit_wz
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
    master_preview_vx = master_preview_vy = 0.0
    master_orbit_wz = 0.0
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
    elif master_state_code == 5 or master_state_code == 16:
        master_orbit_wz = decode_i16(payload, 6) / 10.0
    master_flags = payload[8]
    master_last_rx_ms = utime.ticks_ms()


def poll_coop_uart():
    try:
        n = wireless.receive_bytearray(coop_rx_buf, len(coop_rx_buf))
        if n:
            coop_parser.feed(coop_rx_buf, n, handle_coop_frame)
    except Exception:
        pass


def debug_send_state(log_id, gyro_z):
    try:
        wireless.send_str(
            "D %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d"
            % (
                log_id,
                master_flags,
                cam_target_state(),
                last_alloc_scale,
                int(control_buf[0] * 10.0),
                int(control_buf[1] * 10.0),
                int(control_buf[2] * 10.0),
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
        wireless.send_str("\r\n")
    except Exception:
        pass


def calc_follow_yaw_output(
    gyro_z,
    turn_rate_cmd,
    mode_key,
    seen,
    orbit_settling,
    spin_mode_active,
    orbit_entry_active,
    orbit_mode_active,
    angle_pose_mode_active,
    explicit_orbit,
    orbit_rate_base,
    push_mode_active,
):
    global last_cmd_wz, last_angle_priority_active

    priority_turn_mode = mode_key or angle_pose_mode_active or push_mode_active
    gyro_brake_active = False
    if -0.001 < turn_rate_cmd < 0.001:
        turn_rate_cmd = 0.0
        gyro_stop_rate = 4.0 if priority_turn_mode else 12.0
        if (
            (priority_turn_mode or seen)
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
        if explicit_orbit and not orbit_settling:
            output_ramp = (
                Follow_Orbit_Entry_Ramp_Wz
                if orbit_entry_active
                else (
                    3.0
                    if master_state_code == 5 or master_state_code == 16
                    else Follow_Orbit_Command_Ramp_Wz
                )
            )
            last_cmd_wz = orbit_rate_base + follow_state[0]
        else:
            if spin_mode_active:
                output_ramp = 36.0
            elif orbit_entry_active:
                output_ramp = Follow_Orbit_Entry_Ramp_Wz
            elif orbit_mode_active:
                output_ramp = Follow_Orbit_Command_Ramp_Wz
            elif push_mode_active:
                output_ramp = 12.0
            elif priority_turn_mode:
                output_ramp = 18.0
            else:
                output_ramp = 11.0
        turn_rate_cmd = ramp_value(turn_rate_cmd, last_cmd_wz, output_ramp)
        if explicit_orbit and not orbit_settling:
            follow_state[0] = turn_rate_cmd - orbit_rate_base
        last_cmd_wz = turn_rate_cmd
    if (
        priority_turn_mode
        and (
            turn_rate_cmd >= GYRO_ANGLE_PRIORITY_MIN_CMD
            or turn_rate_cmd <= -GYRO_ANGLE_PRIORITY_MIN_CMD
        )
    ):
        last_angle_priority_active = True
    if turn_rate_cmd:
        gyro_pid.gyro_kp = GYRO_KP
        if orbit_settling:
            gyro_pid.gyro_ki = GYRO_TURN_KI
            gyro_pid.gyro_output_limit = GYRO_ORBIT_OUTPUT_LIMIT
        elif spin_mode_active:
            gyro_pid.gyro_ki = GYRO_TURN_KI
            gyro_pid.gyro_output_limit = 18.0
        elif orbit_mode_active:
            gyro_pid.gyro_ki = GYRO_TURN_KI
            gyro_pid.gyro_output_limit = GYRO_ORBIT_OUTPUT_LIMIT
        elif push_mode_active:
            gyro_pid.gyro_ki = GYRO_KI
            gyro_pid.gyro_output_limit = 16.0
        elif angle_pose_mode_active:
            gyro_pid.gyro_ki = GYRO_KI
            gyro_pid.gyro_output_limit = 16.0
        else:
            gyro_pid.gyro_ki = GYRO_KI
            gyro_pid.gyro_output_limit = GYRO_OUTPUT_LIMIT
    elif gyro_brake_active and (orbit_settling or not mode_key):
        gyro_pid.gyro_kp = GYRO_KP
        gyro_pid.gyro_ki = 0.0
        gyro_pid.gyro_output_limit = (
            GYRO_ORBIT_OUTPUT_LIMIT
            if orbit_settling
            else GYRO_OUTPUT_LIMIT
        )
    if turn_rate_cmd or gyro_brake_active:
        gyro_error = turn_rate_cmd - gyro_z
        if (
            (
                orbit_settling
                or not mode_key
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


def lost_follow_translation(
    out, mode_key, orbit_mode, lost_scale,
    ff_vx, ff_vy, ff_wz,
):
    alloc_vx = alloc_vy = 0.0
    if orbit_mode and (
        master_state_code == 5 or master_state_code == 16
    ):
        _pid_mod.orbit_translation(
            out, 0, 0, ff_vx, ff_vy, ff_wz,
            master_state_code == 5,
        )
        vx = alloc_vx = out[0]
        vy = alloc_vy = out[1]
    elif not mode_key:
        vx = ff_vx * lost_scale
        vy = ff_vy * lost_scale
        if (
            not (master_flags & MASTER_MOTION_FLAG_RETURN)
            and cam_error_x < -_Follow_Distance_Far_Boost_Error
            and vx < 0.0
        ):
            vx = 0.0
        alloc_vx = vx
        alloc_vy = vy
    else:
        scale = 1.0 if master_flags & MASTER_MOTION_FLAG_RETURN else (
            Follow_Hold_Feedforward_Gain
        )
        vx = ff_vx * scale
        vy = ff_vy * Follow_Hold_Feedforward_Gain
    if orbit_mode and (master_flags & MASTER_MOTION_FLAG_RETURN):
        vx += ff_wz * Follow_Orbit_Target_Point_Wz_To_Vx * Follow_Orbit_Feedforward_Forward_Gain
        vy += ff_wz * Follow_Orbit_Target_Point_Wz_To_Vy * Follow_Orbit_Feedforward_Lateral_Gain
    out[0] = vx
    out[1] = vy
    out[3] = alloc_vx
    out[4] = alloc_vy


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
    global last_alloc_scale

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
    return_follow = master_flags & MASTER_MOTION_FLAG_RETURN
    strict_follow_active = fresh_motion and (
        explicit_push or master_state_code == 6
    )
    spin_ff_wz = update_spin_feedforward_latch(
        now,
        fresh_motion,
        explicit_spin,
        ff_wz,
        seen,
        cam_error_angle,
    )
    spin_mode_active = explicit_spin
    orbit_rate = master_orbit_wz
    if explicit_orbit and orbit_follow_entry_until_ms:
        orbit_rate += clamp(master_wz - orbit_rate, -24.0, 24.0) * 0.25
    filtered_wz = update_filtered_ff_wz(
        spin_ff_wz if spin_mode_active else (
            orbit_rate
            if explicit_orbit and (
                master_state_code == 5 or master_state_code == 16
            )
            else ff_wz
        ),
        fresh_motion,
    )
    orbit_mode_active = update_orbit_follow_mode(
        now,
        gyro_z,
        explicit_orbit,
        strict_follow_active or (
            last_control_master_state == 16
            and (
                not fresh_motion
                or (master_state_code != 16 and not explicit_orbit)
            )
        ),
        spin_mode_active,
        fresh_motion and (master_flags & MASTER_MOTION_FLAG_BACK),
    )
    orbit_settling = orbit_mode_active and orbit_follow_exit_since_ms != 0
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
    push_follow_active = strict_follow_active
    if not push_follow_active and not explicit_orbit:
        follow_state[0] = follow_state[1] = 0
        follow_state[2] = follow_state[3] = 0
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
    if mode_key:
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
            and (not orbit_settling)
        )
        solve_follow_pose_twist(
            control_buf,
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
        )
        vx = control_buf[0]
        vy = control_buf[1]
        turn_rate_cmd = control_buf[2]
        body_vx = control_buf[3]
        body_vy = control_buf[4]
        angle_priority_active = control_buf[5]
        position_priority_active = control_buf[6]
        actual_body_vx = (last_enc_fl - last_enc_fr) * 0.5773503
        actual_body_vy = (last_enc_fl + last_enc_fr - 2.0 * last_enc_b) / 3.0
        if follow_output_limit == _FOLLOW_STATIC_LOCK_PWM_LIMIT:
            vx -= body_vx * (1.0 - Follow_Static_Visual_Scale)
            vy -= body_vy * (1.0 - Follow_Static_Visual_Scale)
            body_vx *= Follow_Static_Visual_Scale
            body_vy *= Follow_Static_Visual_Scale
        if orbit_settling:
            _pid_mod.orbit_settle_translation(
                control_buf, follow_state, vx, vy,
                actual_body_vx, actual_body_vy,
                cam_error_x, cam_error_y, cam_error_angle, gyro_z,
                Follow_Orbit_Settle_XY_Limit, _Follow_Orbit_Settle_Angle_Error,
                Follow_Orbit_Settle_Gyro_Rate, 1.0,
            )
            body_vx = control_buf[0]
            body_vy = control_buf[1]
            vx = body_vx
            vy = body_vy
            turn_rate_cmd = clamp(
                turn_rate_cmd,
                -Follow_Orbit_Settle_Turn_Limit,
                Follow_Orbit_Settle_Turn_Limit,
            )
        normal_damping_active = (
            not mode_key
            and (
                master_flags < MASTER_MOTION_FLAG_ORBIT
                or push_follow_active
                or return_follow
            )
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
            if orbit_damping_active:
                velocity_damping = Follow_Orbit_Velocity_Damping
            elif push_follow_active:
                velocity_damping = Follow_Push_Velocity_Damping
            elif follow_output_limit < _FOLLOW_RUN_PWM_LIMIT:
                velocity_damping = Follow_Static_Velocity_Damping
            else:
                velocity_damping = Follow_Normal_Velocity_Damping
            _pid_mod.velocity_damping(
                control_buf, follow_state, actual_body_vx, actual_body_vy,
                alloc_base_vx, alloc_base_vy,
                0.18
                if orbit_damping_active and (
                    master_state_code == 5 or master_state_code == 16
                )
                else 0.0,
                velocity_damping, Follow_Normal_Correction_Reserve,
            )
            damp_vx = control_buf[0]
            damp_vy = control_buf[1]
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
            if master_flags & MASTER_MOTION_FLAG_RETURN:
                lost_scale = 1.0
        if (
            fresh_motion
            and not orbit_settling
            and not mode_key
            and lost_scale
        ):
            use_motion_feedforward = True
        elif fresh_motion and not orbit_settling:
            use_motion_feedforward = master_state_code == 16 or (
                utime.ticks_diff(now, target_lost_since_ms)
                <= (300 if not mode_key else _Follow_Target_Lost_Hold_Ms)
            )
        if use_motion_feedforward:
            if not mode_key:
                ff_vx = measured_ff_vx
                ff_vy = measured_ff_vy
            lost_follow_translation(
                control_buf, mode_key, orbit_mode_active, lost_scale,
                ff_vx, ff_vy, follow_ff_wz,
            )
            vx = control_buf[0]
            vy = control_buf[1]
            alloc_base_vx = control_buf[3]
            alloc_base_vy = control_buf[4]
        else:
            vx = 0.0
            vy = 0.0
        if prev_angle_priority_active:
            reset_turn_loop_state()
        last_angle_priority_active = False

    vx_limit = (
        Follow_Orbit_Forward_Command_Limit
        if explicit_orbit
        else Follow_Forward_Limit
    )
    if master_flags & MASTER_MOTION_FLAG_RETURN:
        vy_limit = Follow_Return_Lateral_Limit
    elif push_follow_active:
        vy_limit = 27.0
    else:
        vy_limit = 32.0
    if fresh_motion:
        vx_limit = max(vx_limit, abs(ff_vx))
        vy_limit = max(vy_limit, abs(ff_vy))
    if not (explicit_orbit and master_state_code == 16):
        vx = clamp(vx, -vx_limit, vx_limit)
        vy = clamp(vy, -vy_limit, vy_limit)
    if (
        seen
        and not mode_key
        and (
            master_flags < MASTER_MOTION_FLAG_ORBIT
            or push_follow_active
        )
    ):
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
    if orbit_settling:
        vx_ramp = 3.0
        vy_ramp = 4.0
    elif orbit_entry_active:
        vx_ramp = Follow_Orbit_Entry_Ramp_Vx
        vy_ramp = Follow_Orbit_Entry_Ramp_Vy
    elif position_priority_active:
        if master_state_code == 5 or master_state_code == 16:
            vx_ramp = 0.45
            vy_ramp = 0.65
        else:
            vx_ramp = Follow_Orbit_Command_Ramp_Vx
            vy_ramp = Follow_Orbit_Command_Ramp_Vy
    else:
        vx_ramp = Follow_Command_Ramp_Vx
        vy_ramp = Follow_Command_Ramp_Vy
    if explicit_orbit and not orbit_settling:
        last_cmd_vx = alloc_base_vx + follow_state[4]
        last_cmd_vy = alloc_base_vy + follow_state[5]
    if not (classification_exit and not mode_key):
        vx = ramp_value(vx, last_cmd_vx, vx_ramp)
        vy = ramp_value(vy, last_cmd_vy, vy_ramp)
    if explicit_orbit:
        if not orbit_settling:
            follow_state[4] = vx - alloc_base_vx
            follow_state[5] = vy - alloc_base_vy
        if master_state_code != 16:
            vx = clamp(
                vx,
                -Follow_Orbit_Forward_Command_Limit,
                Follow_Orbit_Forward_Command_Limit,
            )
            vy = clamp(vy, -vy_limit, vy_limit)
    elif push_follow_active:
        _pid_mod.push_correction_envelope(
            control_buf, follow_state, cam_error_x, cam_error_y,
            alloc_base_vx, explicit_push,
        )
        push_catchup_limit = control_buf[0]
        push_brake_limit = control_buf[1]
        push_lateral_limit = control_buf[2]
        if (
            cam_error_x <= -_Follow_Push_Emergency_Error
            and safety_vx > push_catchup_limit
        ):
            push_catchup_limit += (safety_vx - push_catchup_limit) * (
                (push_catchup_limit - 4.0) / 2.0
            )
        body_vx = clamp(
            vx - alloc_base_vx,
            -push_brake_limit,
            push_catchup_limit,
        )
        body_vy = clamp(
            vy - alloc_base_vy,
            -push_lateral_limit,
            push_lateral_limit,
        )
        vx = alloc_base_vx + body_vx
        vy = alloc_base_vy + body_vy
    if not fresh_motion and return_follow:
        vx = vy = turn_rate_cmd = 0.0
    # A normal state-4 exit can apply the new leader feedforward immediately,
    # because vision correction stayed continuous.  ORBIT keeps its dedicated
    # entry ramp so the first high-rate command cannot kick the follower.
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
            turn_rate_cmd *= lost_scale
    vz_cmd = calc_follow_yaw_output(
        gyro_z,
        turn_rate_cmd,
        mode_key,
        seen,
        orbit_settling,
        spin_mode_active,
        orbit_entry_active,
        orbit_mode_active,
        angle_pose_mode_active,
        explicit_orbit,
        clamp(
            follow_ff_wz * Follow_Orbit_Wz_Feedforward_Gain,
            -Follow_Orbit_Wz_Feedforward_Limit,
            Follow_Orbit_Wz_Feedforward_Limit,
        ) if explicit_orbit else 0.0,
        push_follow_active,
    )
    if not fresh_motion and return_follow:
        vz_cmd = 0.0
        reset_turn_loop_state()

    _pid_mod.limit_pose_twist_for_wheels(
        control_buf,
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
            or (return_follow and not mode_key)
        ),
        (
            2
            if explicit_orbit and (
                master_state_code == 5 or master_state_code == 16
            )
            else explicit_orbit and not explicit_push
        ),
        fresh_motion
        and last_follow_mode_key == 0
        and (
            0 < master_flags < 8
            or push_follow_active
            or return_follow
        ),
    )
    cam_target_vx = control_buf[0]
    cam_target_vy = control_buf[1]
    vz_cmd = control_buf[2]
    last_alloc_scale = control_buf[3]
    last_cmd_vx = cam_target_vx
    last_cmd_vy = cam_target_vy
    gyro_pid.output = vz_cmd

    last_turn_rate_cmd = turn_rate_cmd
    last_follow_seen = seen
    last_ff_vx = ff_vx
    last_ff_vy = ff_vy
    last_ff_wz = follow_ff_wz
    last_control_master_state = master_state_code
    return vz_cmd


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


def follow_channel_pwm(cmd, target, speed_err, stall_boost, last_pwm):
    fast_reverse = last_follow_mode_key == 3
    target_abs = abs(target)
    if wheel_target_idle(target):
        min_pwm = 0
    elif stall_boost and target_abs >= 2.8:
        min_pwm = 8800
    elif follow_output_limit < _FOLLOW_RUN_PWM_LIMIT and target_abs < 2.8:
        min_pwm = 2600
    else:
        min_pwm = 3600 if target_abs < 2.8 else 6200
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
    cmd = int(cmd)
    if 0 < cmd < min_pwm:
        cmd = min_pwm
    elif -min_pwm < cmd < 0:
        cmd = -min_pwm
    cmd = clamp(cmd, -_FOLLOW_RUN_PWM_LIMIT, _FOLLOW_RUN_PWM_LIMIT)
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


def start_follow():
    global car_started

    if car_started:
        return
    pit1.stop()
    try:
        imu_runtime.calibrate_offset(
            samples=1000,
            delay_ms=2,
            logger=None,
        )
        imu_runtime.reset_yaw(0.0)
    finally:
        pit1.start(TICK_PERIOD_MS)
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
    global debug_log_last_ms

    if not car_started:
        reset_speed_outputs()
        set_three_pwm_zero()
        return None

    if master_motion_fresh() and not (master_flags & MASTER_MOTION_FLAG_STARTED):
        reset_speed_outputs()
        set_three_pwm_zero()
        return None

    gyro_z = imu_runtime.read_gyro_z()

    vz_cmd = update_follow_targets(gyro_z)
    calc_wheel_spd_into(control_buf, cam_target_vx, cam_target_vy, vz_cmd)

    e_fl = enc_fl.get() * ENC_SCALE
    e_fr = enc_fr.get() * ENC_SCALE
    e_b = enc_b.get() * ENC_SCALE
    t_fl = control_buf[0]
    t_fr = control_buf[1]
    t_b = control_buf[2]
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
    if utime.ticks_diff(now_log, debug_log_last_ms) >= 100:
        debug_log_last_ms = now_log
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

imu_runtime = LSM6DSV16XYawRuntime(
    sign=1.0,
    offset_z=0.0,
    scale=-1.0,
    deadband_dps=0.8,
    tick_period_ms=TICK_PERIOD_MS,
)

pit1 = ticker(1)
pit1.capture_list(enc_fl, enc_fr, enc_b)
pit1.callback(time_pit_handler)
pit1.start(TICK_PERIOD_MS)

control_buf = [0.0, 0.0, 0.0, 0.0, 0.0, False, False]
follow_state = [0, 0, 0, 0, 0, 0]
pid_fl = _pid_mod.SpeedPID()
pid_fr = _pid_mod.SpeedPID()
pid_b = _pid_mod.SpeedPID()

gyro_pid = _pid_mod.AnglePID()
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

        if loop_count % 50 == 0:
            gc.collect()

        utime.sleep_ms(1)

finally:
    pit1.stop()
    cam_uart.write(b"IDLE\n")
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)
    led.value(True)
