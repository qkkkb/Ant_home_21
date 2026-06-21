from machine import UART
import gc
import utime
from seekfree import WIRELESS_UART

import config as cfg
import pid as _pid_mod
from models import MoveBase, SpeedPID
from move_base import calc_wheel_spd


_pid_mod.PWM_MAX = cfg.PWM_MAX
speed_ctrl = _pid_mod.speed_ctrl
speed_reset = _pid_mod.speed_reset


LOG_PERIOD_MS = 200
ART_MODE_TRACK_CMD = b"TRACK\n"

CAM_ERROR_OFFSET = 120
CAM_ERROR_SCALE = 2
CAM_FRAME_HEAD = 0xFF
NO_TARGET_MARKER = 0xFE
LINE_PACKET_TAG = 0xFC
CLASSIFY_PACKET_TAG = 0xFD
CAM_PACKET_TIMEOUT_MS = 200

FOLLOW_FORWARD_GAIN = 0.62
FOLLOW_LATERAL_GAIN = 0.55
FOLLOW_ORBIT_FORWARD_GAIN = 1.00
FOLLOW_ORBIT_LATERAL_GAIN = 0.76
FOLLOW_FORWARD_ERROR_SIGN = 1.0
FOLLOW_LATERAL_ERROR_SIGN = -1.0
FOLLOW_FORWARD_LIMIT = 70.0
FOLLOW_LATERAL_LIMIT = 44.0
FOLLOW_FORWARD_DEADBAND = 2
FOLLOW_LATERAL_DEADBAND = 2
FOLLOW_ORBIT_FORWARD_DEADBAND = 2
FOLLOW_ORBIT_LATERAL_DEADBAND = 2
FOLLOW_ORBIT_LATERAL_MIN_ERROR = 8
FOLLOW_ORBIT_LATERAL_MIN_VY = 8.5
FOLLOW_ORBIT_POSITION_X_ERROR = 4
FOLLOW_ORBIT_POSITION_Y_ERROR = 4
FOLLOW_DISTANCE_FAR_BOOST_ERROR = 6
FOLLOW_DISTANCE_FAR_BOOST_GAIN = 1.20
FOLLOW_DISTANCE_CLOSE_GAIN = 0.94
FOLLOW_DISTANCE_CLOSE_LIMIT = 42.0
FOLLOW_POSE_ANGLE_GAIN = -0.82
FOLLOW_POSE_ANGLE_LIMIT = 40.0
FOLLOW_NORMAL_POSE_ANGLE_DEADBAND = 14
FOLLOW_NORMAL_POSE_ANGLE_ACTIVE_ERROR = 18
FOLLOW_ANGLE_XY_MODE_ON_ERROR = 12
FOLLOW_ANGLE_XY_MODE_FULL_ERROR = 42
FOLLOW_ANGLE_XY_MIN_SCALE = 0.38
FOLLOW_NORMAL_VISUAL_FORWARD_SCALE = 0.98
FOLLOW_NORMAL_VISUAL_LATERAL_SCALE = 1.04
FOLLOW_POSE_WHEEL_TARGET_LIMIT = 46.0
FOLLOW_COMMAND_RAMP_VX = 12.0
FOLLOW_COMMAND_RAMP_VY = 10.0
FOLLOW_COMMAND_RAMP_WZ = 11.0
FOLLOW_ORBIT_COMMAND_RAMP_VX = 22.0
FOLLOW_ORBIT_COMMAND_RAMP_VY = 18.0
FOLLOW_ORBIT_COMMAND_RAMP_WZ = 12.0

WHEEL_TARGET_STOP_EPS = 0.05
WHEEL_TARGET_IDLE_EPS = 0.35
FOLLOW_START_PWM = 6200
FOLLOW_START_PWM_MID = 3600
FOLLOW_START_PWM_LOW = 0
FOLLOW_START_PWM_LOW_TARGET = 1.2
FOLLOW_START_PWM_MID_TARGET = 2.8
FOLLOW_RUN_PWM_LIMIT = 24000
MOTOR_DUTY_MAX = cfg.MOTOR_DUTY_MAX
PWM_SMOOTH_FACTOR = cfg.PWM_SMOOTH_FACTOR
MAX_PWM_CHANGE = cfg.MAX_PWM_CHANGE


cam_error_x = 0
cam_error_y = 0
cam_error_angle = 0
cam_last_rx_ms = 0
cam_has_target = False
cam_parse_state = 0
cam_parse_b1 = 0
cam_parse_b2 = 0

last_cmd_vx = 0.0
last_cmd_vy = 0.0
last_cmd_wz = 0.0
last_pwm_fl = 0
last_pwm_fr = 0
last_pwm_b = 0


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def ramp_value(target, current, step):
    delta = target - current
    if delta > step:
        return current + step
    if delta < -step:
        return current - step
    return target


def update_cam_target(err_x, err_y, err_angle):
    global cam_error_x, cam_error_y, cam_error_angle, cam_last_rx_ms
    cam_error_x = int(err_x)
    cam_error_y = int(err_y)
    cam_error_angle = int(err_angle)
    cam_last_rx_ms = utime.ticks_ms()


def cam_target_seen():
    return cam_has_target and utime.ticks_diff(utime.ticks_ms(), cam_last_rx_ms) <= CAM_PACKET_TIMEOUT_MS


def angle_abs_value(error_angle):
    if error_angle < 0:
        return -error_angle
    return error_angle


def angle_pose_mode_needed(error_angle):
    return error_angle >= FOLLOW_ANGLE_XY_MODE_ON_ERROR or error_angle <= -FOLLOW_ANGLE_XY_MODE_ON_ERROR


def angle_xy_lock_scale(error_angle):
    angle_abs = angle_abs_value(error_angle)
    if angle_abs <= FOLLOW_ANGLE_XY_MODE_ON_ERROR:
        return 1.0
    if angle_abs >= FOLLOW_ANGLE_XY_MODE_FULL_ERROR:
        return FOLLOW_ANGLE_XY_MIN_SCALE
    span = FOLLOW_ANGLE_XY_MODE_FULL_ERROR - FOLLOW_ANGLE_XY_MODE_ON_ERROR
    return 1.0 - ((angle_abs - FOLLOW_ANGLE_XY_MODE_ON_ERROR) * (1.0 - FOLLOW_ANGLE_XY_MIN_SCALE) / span)


def position_priority_needed(error_x, error_y, angle_active):
    return (
        angle_active
        or error_x >= FOLLOW_ORBIT_POSITION_X_ERROR
        or error_x <= -FOLLOW_ORBIT_POSITION_X_ERROR
        or error_y >= FOLLOW_ORBIT_POSITION_Y_ERROR
        or error_y <= -FOLLOW_ORBIT_POSITION_Y_ERROR
    )


def calc_follow_forward(error_y, position_priority):
    deadband = FOLLOW_ORBIT_FORWARD_DEADBAND if position_priority else FOLLOW_FORWARD_DEADBAND
    if -deadband <= error_y <= deadband:
        return 0.0
    if error_y > 0:
        gain = FOLLOW_ORBIT_FORWARD_GAIN if position_priority else FOLLOW_FORWARD_GAIN
        out = error_y * gain
        if error_y > FOLLOW_DISTANCE_FAR_BOOST_ERROR:
            out += (error_y - FOLLOW_DISTANCE_FAR_BOOST_ERROR) * FOLLOW_DISTANCE_FAR_BOOST_GAIN
        out *= FOLLOW_FORWARD_ERROR_SIGN
        if FOLLOW_FORWARD_ERROR_SIGN >= 0:
            return clamp(out, 0.0, FOLLOW_FORWARD_LIMIT)
        return clamp(out, -FOLLOW_FORWARD_LIMIT, 0.0)
    out = error_y * FOLLOW_DISTANCE_CLOSE_GAIN * FOLLOW_FORWARD_ERROR_SIGN
    if FOLLOW_FORWARD_ERROR_SIGN >= 0:
        return clamp(out, -FOLLOW_DISTANCE_CLOSE_LIMIT, 0.0)
    return clamp(out, 0.0, FOLLOW_DISTANCE_CLOSE_LIMIT)


def calc_follow_lateral(error_x, position_priority):
    deadband = FOLLOW_ORBIT_LATERAL_DEADBAND if position_priority else FOLLOW_LATERAL_DEADBAND
    if -deadband <= error_x <= deadband:
        return 0.0
    gain = FOLLOW_ORBIT_LATERAL_GAIN if position_priority else FOLLOW_LATERAL_GAIN
    out = -error_x * gain * FOLLOW_LATERAL_ERROR_SIGN
    if position_priority and (error_x >= FOLLOW_ORBIT_LATERAL_MIN_ERROR or error_x <= -FOLLOW_ORBIT_LATERAL_MIN_ERROR):
        if 0.0 < out < FOLLOW_ORBIT_LATERAL_MIN_VY:
            out = FOLLOW_ORBIT_LATERAL_MIN_VY
        elif -FOLLOW_ORBIT_LATERAL_MIN_VY < out < 0.0:
            out = -FOLLOW_ORBIT_LATERAL_MIN_VY
    return clamp(out, -FOLLOW_LATERAL_LIMIT, FOLLOW_LATERAL_LIMIT)


def calc_follow_angle(error_angle):
    if -FOLLOW_NORMAL_POSE_ANGLE_DEADBAND <= error_angle <= FOLLOW_NORMAL_POSE_ANGLE_DEADBAND:
        return 0.0
    return clamp(error_angle * FOLLOW_POSE_ANGLE_GAIN, -FOLLOW_POSE_ANGLE_LIMIT, FOLLOW_POSE_ANGLE_LIMIT)


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


def limit_pose_twist_for_wheels(vx, vy, vz, preserve_pose_ratio, preserve_turn):
    if preserve_turn:
        limit = FOLLOW_POSE_WHEEL_TARGET_LIMIT
        vz = clamp(vz, -limit, limit)
        wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(vx, vy, 0.0)
        scale = 1.0
        if wheel_fr > 0.001 and wheel_fr + vz > limit:
            scale = (limit - vz) / wheel_fr
        elif wheel_fr < -0.001 and wheel_fr + vz < -limit:
            scale = (-limit - vz) / wheel_fr
        if wheel_fl > 0.001 and wheel_fl + vz > limit:
            tmp = (limit - vz) / wheel_fl
            if tmp < scale:
                scale = tmp
        elif wheel_fl < -0.001 and wheel_fl + vz < -limit:
            tmp = (-limit - vz) / wheel_fl
            if tmp < scale:
                scale = tmp
        if wheel_b > 0.001 and wheel_b + vz > limit:
            tmp = (limit - vz) / wheel_b
            if tmp < scale:
                scale = tmp
        elif wheel_b < -0.001 and wheel_b + vz < -limit:
            tmp = (-limit - vz) / wheel_b
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
        if target_max > FOLLOW_POSE_WHEEL_TARGET_LIMIT:
            scale = FOLLOW_POSE_WHEEL_TARGET_LIMIT / target_max
            vx *= scale
            vy *= scale
            vz *= scale
        return vx, vy, vz

    wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(vx, vy, 0.0)
    pos_max = max_wheel_abs(wheel_fr, wheel_fl, wheel_b)
    if pos_max > FOLLOW_POSE_WHEEL_TARGET_LIMIT:
        pos_scale = FOLLOW_POSE_WHEEL_TARGET_LIMIT / pos_max
        vx *= pos_scale
        vy *= pos_scale
        wheel_fr, wheel_fl, wheel_b = pose_wheel_targets(vx, vy, 0.0)
    if -0.001 < vz < 0.001:
        return vx, vy, 0.0
    if vz > 0.0:
        remain = FOLLOW_POSE_WHEEL_TARGET_LIMIT - wheel_fr
        tmp = FOLLOW_POSE_WHEEL_TARGET_LIMIT - wheel_fl
        if tmp < remain:
            remain = tmp
        tmp = FOLLOW_POSE_WHEEL_TARGET_LIMIT - wheel_b
        if tmp < remain:
            remain = tmp
        if remain < 0.0:
            remain = 0.0
        if vz > remain:
            vz = remain
    else:
        remain = FOLLOW_POSE_WHEEL_TARGET_LIMIT + wheel_fr
        tmp = FOLLOW_POSE_WHEEL_TARGET_LIMIT + wheel_fl
        if tmp < remain:
            remain = tmp
        tmp = FOLLOW_POSE_WHEEL_TARGET_LIMIT + wheel_b
        if tmp < remain:
            remain = tmp
        if remain < 0.0:
            remain = 0.0
        if -vz > remain:
            vz = -remain
    return vx, vy, vz


def calc_probe_twist():
    global last_cmd_vx, last_cmd_vy, last_cmd_wz
    seen = cam_target_seen()
    if not seen:
        last_cmd_vx = 0.0
        last_cmd_vy = 0.0
        last_cmd_wz = 0.0
        return 0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False

    vision_wz = calc_follow_angle(cam_error_angle)
    angle_active = cam_error_angle >= FOLLOW_NORMAL_POSE_ANGLE_ACTIVE_ERROR or cam_error_angle <= -FOLLOW_NORMAL_POSE_ANGLE_ACTIVE_ERROR
    position_priority = position_priority_needed(cam_error_x, cam_error_y, angle_active)
    cam_vx = calc_follow_forward(cam_error_y, position_priority)
    cam_vy = calc_follow_lateral(cam_error_x, position_priority)
    body_vx = cam_vx * FOLLOW_NORMAL_VISUAL_FORWARD_SCALE
    body_vy = cam_vy * FOLLOW_NORMAL_VISUAL_LATERAL_SCALE
    vx = body_vx
    vy = body_vy
    vz = vision_wz
    angle_pose = angle_pose_mode_needed(cam_error_angle)
    if angle_pose:
        scale = angle_xy_lock_scale(cam_error_angle)
        vx *= scale
        vy *= scale
        position_priority = True
    vx = clamp(vx, -FOLLOW_FORWARD_LIMIT, FOLLOW_FORWARD_LIMIT)
    vy = clamp(vy, -FOLLOW_LATERAL_LIMIT, FOLLOW_LATERAL_LIMIT)
    if position_priority:
        vx_ramp = FOLLOW_ORBIT_COMMAND_RAMP_VX
        vy_ramp = FOLLOW_ORBIT_COMMAND_RAMP_VY
        wz_ramp = FOLLOW_ORBIT_COMMAND_RAMP_WZ
    else:
        vx_ramp = FOLLOW_COMMAND_RAMP_VX
        vy_ramp = FOLLOW_COMMAND_RAMP_VY
        wz_ramp = FOLLOW_COMMAND_RAMP_WZ
    vx = ramp_value(vx, last_cmd_vx, vx_ramp)
    vy = ramp_value(vy, last_cmd_vy, vy_ramp)
    vz = ramp_value(vz, last_cmd_wz, wz_ramp)
    last_cmd_vx = vx
    last_cmd_vy = vy
    last_cmd_wz = vz
    vx, vy, vz = limit_pose_twist_for_wheels(vx, vy, vz, position_priority, position_priority)
    return 1, vx, vy, vz, body_vx, body_vy, angle_active, position_priority


def wheel_target_idle(target):
    return -WHEEL_TARGET_IDLE_EPS <= target <= WHEEL_TARGET_IDLE_EPS


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


def follow_start_pwm_for_target(target):
    target_abs = abs(target)
    if target_abs <= WHEEL_TARGET_IDLE_EPS:
        return 0
    if target_abs < FOLLOW_START_PWM_LOW_TARGET:
        return FOLLOW_START_PWM_LOW
    if target_abs < FOLLOW_START_PWM_MID_TARGET:
        return FOLLOW_START_PWM_MID
    return FOLLOW_START_PWM


def follow_channel_pwm(cmd, target, last_pwm):
    min_pwm = follow_start_pwm_for_target(target)
    if min_pwm <= 0:
        return 0
    return smooth_value(apply_start_pwm(cmd, min_pwm), last_pwm)


def direct_pwm_for_target(target):
    min_pwm = follow_start_pwm_for_target(target)
    if min_pwm <= 0:
        return 0
    if target > 0:
        return min_pwm
    return -min_pwm


def speed_ctrl_probe(pid, target):
    if wheel_target_idle(target):
        speed_reset(pid)
        return 0.0
    return speed_ctrl(pid, 0, target)


def update_probe_pwm(t_fl, t_fr, t_b):
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    u_fl = speed_ctrl_probe(pid_fl, t_fl)
    u_fr = speed_ctrl_probe(pid_fr, t_fr)
    u_b = speed_ctrl_probe(pid_b, t_b)
    last_pwm_fl = follow_channel_pwm(u_fl, t_fl, last_pwm_fl)
    last_pwm_fr = follow_channel_pwm(u_fr, t_fr, last_pwm_fr)
    last_pwm_b = follow_channel_pwm(u_b, t_b, last_pwm_b)
    return last_pwm_fl, last_pwm_fr, last_pwm_b, u_fl, u_fr, u_b


def poll_art_uart():
    global cam_parse_state, cam_parse_b1, cam_parse_b2
    global cam_has_target
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
            if b == CAM_FRAME_HEAD:
                cam_parse_state = 1
        elif cam_parse_state == 1:
            if b != CAM_FRAME_HEAD:
                cam_parse_b1 = b
                cam_parse_state = 2
        elif cam_parse_state == 2:
            cam_parse_b2 = b
            if cam_parse_b1 == NO_TARGET_MARKER and b == NO_TARGET_MARKER:
                cam_has_target = False
                cam_parse_state = 0
            elif cam_parse_b1 == LINE_PACKET_TAG or cam_parse_b1 == CLASSIFY_PACKET_TAG:
                cam_parse_state = 0
            else:
                cam_parse_state = 3
        else:
            if b == CAM_FRAME_HEAD:
                err_angle = 0
                cam_parse_state = 1
            else:
                err_angle = (b - CAM_ERROR_OFFSET) * CAM_ERROR_SCALE
                cam_parse_state = 0
            cam_has_target = True
            update_cam_target(
                (cam_parse_b1 - CAM_ERROR_OFFSET) * CAM_ERROR_SCALE,
                (cam_parse_b2 - CAM_ERROR_OFFSET) * CAM_ERROR_SCALE,
                err_angle,
            )


def send_line(text):
    print(text)
    if wireless is not None:
        try:
            wireless.send_str(text)
            wireless.send_str("\r\n")
        except Exception:
            pass


def log_probe():
    seen, vx, vy, vz, body_vx, body_vy, angle_active, position_priority = calc_probe_twist()
    calc_wheel_spd(move_cmd, vx, vy, vz)
    t_fl = move_cmd.speed_fl
    t_fr = move_cmd.speed_fr
    t_b = move_cmd.speed_b
    p_fl, p_fr, p_b, u_fl, u_fr, u_b = update_probe_pwm(t_fl, t_fr, t_b)
    d_fl = direct_pwm_for_target(t_fl)
    d_fr = direct_pwm_for_target(t_fr)
    d_b = direct_pwm_for_target(t_b)
    send_line("V %d %d %d %d" % (seen, cam_error_x, cam_error_y, cam_error_angle))
    send_line("C %d %d %d %d %d %d" % (int(vx), int(vy), int(vz), int(body_vx), int(body_vy), 1 if position_priority else 0))
    send_line("T %d %d %d" % (int(t_fl), int(t_fr), int(t_b)))
    send_line("P %d %d %d %d %d %d" % (p_fl, p_fr, p_b, int(u_fl), int(u_fr), int(u_b)))
    send_line("D %d %d %d" % (d_fl, d_fr, d_b))


cam_uart = UART(cfg.CAM_UART_ID, cfg.CAM_UART_BAUD)
cam_uart.init(cfg.CAM_UART_BAUD, timeout_char=100)
cam_uart_buf = bytearray(32)
cam_uart.write(ART_MODE_TRACK_CMD)

try:
    wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
except Exception:
    wireless = None
move_cmd = MoveBase()
pid_fl = SpeedPID()
pid_fr = SpeedPID()
pid_b = SpeedPID()
pid_fl.init_c()
pid_fr.init_c()
pid_b.init_c()

last_log_ms = utime.ticks_ms()
send_line("PROBE vision_pwm")

while True:
    poll_art_uart()
    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_log_ms) >= LOG_PERIOD_MS:
        last_log_ms = now
        log_probe()
    gc.collect()
    utime.sleep_ms(1)
