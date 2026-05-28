from machine import Pin, UART
from array import array
import gc
import utime
from smartcar import ticker, encoder
from imu_runtime import IMUYawRuntime
from models import AnglePID, MoveBase, SpeedPID
from move_base import calc_wheel_spd, get_car_spd
import pid as _pid_mod
import config as cfg
from hardware import Motor
from seekfree import WIRELESS_UART
from coop_protocol import (
    H1,
    H2,
    MASTER_MOTION_FLAG_CLOSED_LOOP,
    MASTER_MOTION_FLAG_STARTED,
    MASTER_MOTION_FLAG_TARGET,
    MSG_MASTER_MOTION,
)

DEBUG_WIRELESS_LOG_ENABLE = True
DEBUG_WIRELESS_BAUD = 460800
MASTER_MOTION_BROADCAST_ENABLE = False

# 设置 PID 最大 PWM 值
_pid_mod.PWM_MAX = cfg.PWM_MAX
speed_ctrl = _pid_mod.speed_ctrl
gyro_ctrl = _pid_mod.gyro_ctrl
turn_ctrl = _pid_mod.turn_ctrl

# ====================== 基础配置 ======================
# 系统控制周期
TICK_PERIOD_MS = cfg.TICK_PERIOD_MS
# 电机最大/最小有效占空比
MOTOR_DUTY_MAX = cfg.MOTOR_DUTY_MAX
MOTOR_DUTY_MIN = cfg.MOTOR_DUTY_MIN
# PWM 平滑滤波系数
PWM_SMOOTH_FACTOR = cfg.PWM_SMOOTH_FACTOR
# PWM 单次最大变化量（防冲击）
MAX_PWM_CHANGE = cfg.MAX_PWM_CHANGE

# 默认开启速度环；需要方向控制时再叠加陀螺仪环
ENABLE_GYRO_LOOP = True

GYRO_SIGN = 1.0

# 陀螺仪 Z 轴标定参数
GYRO_OFFSET_Z = 3.16
GYRO_SCALE = -1.0 / 16.54052
GYRO_DEADBAND_DPS = 0.8  # 陀螺仪死区阈值
GYRO_KP = 0.22
GYRO_KI = 0.004
GYRO_OUTPUT_LIMIT = 5.0
AUTO_CALIBRATE_GYRO_ON_LAUNCH = True   #是否在启动时自动进行陀螺仪标定
GYRO_CALIBRATE_SAMPLES = 1000
GYRO_CALIBRATE_DELAY_MS = 2

# IMU 使能条件
ENABLE_IMU = ENABLE_GYRO_LOOP

# 调试与退出配置
GC_DIV = 50
FORCE_MOTOR_OFF = False  # 调试开关：True 时程序继续运行，但三个电机始终断输出
DEBUG_LOG_ENABLE = True
DEBUG_LOG_PERIOD_MS = 500
MASTER_MOTION_TX_PERIOD_MS = cfg.MASTER_MOTION_TX_PERIOD_MS
MASTER_MOTION_FRAME_LEN = 15

Cam_Error_Offset = 120
Cam_Error_Scale = 2
Cam_Packet_Timeout_Ms = 200
Cam_Frame_Head = 0xFF
Line_Packet_Tag = 0xFC
No_Target_Marker = 0xFE
Classify_Packet_Tag = 0xFD
NAV_STATE_SEARCH = "SEARCH"
NAV_STATE_SEARCH_TURN = "SEARCH_TURN_45"
NAV_STATE_COARSE = "COARSE_APPROACH"
NAV_STATE_FINE = "FINE_ALIGN"
NAV_STATE_PUSH_CLASSIFY = "PUSH_CLASSIFY"
NAV_STATE_PUSH_ORIENT = "PUSH_ORBIT"
NAV_STATE_PUSH_PREPARE = "PUSH_PREPARE"
NAV_STATE_PUSH = "PUSH_EXECUTE"
NAV_STATE_PUSH_BACK = "PUSH_FINISH_BACK"
NAV_STATE_PUSH_TURN = "PUSH_FINISH_TURN"
NAV_STATE_POST_TURN_FORWARD = "POST_TURN_FORWARD"
ART_MODE_SEARCH_CMD = b"SEARCH\n"
ART_MODE_COARSE_CMD = b"COARSE\n"
ART_MODE_FINE_CMD = b"FINE\n"
ART_MODE_CLASSIFY_CMD = b"CLASSIFY\n"
ART_MODE_LINE_CMD = b"LINE\n"
ART_MODE_IDLE_CMD = b"IDLE\n"
Push_Dir_None = 0
Push_Dir_Right = 1
Push_Dir_Up = 2
Push_Dir_Left = 3
Push_Dir_Down = 4
Nav_Detect_Ms = 200
Nav_Target_Lost_Ms = 500
Nav_Search_Turn_Yaw = 30.0
Nav_Search_Turn_Ok_Yaw = 5.0
Nav_Search_Turn_Ok_Ms = 120
Nav_Search_Turn_Gyro_Th = 3.0
Nav_Search_Turn_Max_Rate = 8.0
Nav_Search_Turn_Gyro_Limit = 2.5
Nav_Search_Turn_Open_Vz = 2.2
Nav_Coarse_Exit_Y = 230
Nav_Coarse_Ok_Ms = 10
Nav_Fine_Classify_Ok_X = 30
Nav_Fine_Classify_Ok_Y_Min = -35
Nav_Fine_Classify_Ok_Y_Max = 30
Nav_Fine_Classify_Ok_Ms = 40
Nav_Fine_Push_Ok_X = 14
Nav_Fine_Push_Ok_Y_Min = -12
Nav_Fine_Push_Ok_Y_Max = 24
Nav_Fine_Push_Ok_Ms = 40
Nav_Transition_Grace_Ms = 200
Nav_Low_Speed_Th = 30
Nav_Coarse_Forward_Gain = 0.075
Nav_Coarse_Lateral_Gain = 0.035		#COARSE 横移系数
Nav_Coarse_Forward_Limit = 8.0
Nav_Coarse_Lateral_Limit = 5.0
Nav_Fine_Forward_Gain = 0.035
Nav_Fine_Classify_Lateral_Gain = 0.08
Nav_Fine_Lateral_Gain = 0.12        #FINE 横移系数
Nav_Fine_Forward_Limit = 4.2
Nav_Fine_Lateral_Limit = 4.0    
Nav_Fine_Forward_Brake_Enable = True
Nav_Fine_Forward_Brake_Y = -8
Nav_Fine_Forward_Brake_Ms = 110
Nav_Fine_Forward_Brake_Speed = 4.5
Nav_Fine_Lateral_Brake_Enable = True
Nav_Fine_Lateral_Brake_Window_X = 55    #当摄像头误差在这个范围内且车速较高时启用横移制动
Nav_Fine_Lateral_Brake_Ms = 110
Nav_Fine_Lateral_Brake_Speed = 7.0
Nav_Forward_Deadband = 4
Nav_Lateral_Deadband = 3    #横移死区，单位像素；如果摄像头误差在这个范围内则认为不需要横移
Nav_Classify_Timeout_Ms = 1500

Nav_Push_Orient_Ok_Yaw = 5.0    #orbit 目标角度误差小于该值即认为定向完成
Nav_Push_Orbit_Skip_Yaw = 15.0
Nav_Push_Orient_Max_Ms = 15000
Nav_Push_Orbit_Slow_Yaw = 40.0
Nav_Push_Orbit_Fast_Vy = 4.5
Nav_Push_Orbit_Slow_Vy = 2.5
Nav_Push_Orbit_Fast_Rate = 75.0
Nav_Push_Orbit_Slow_Rate = 35.0
Nav_Push_Orbit_Gyro_Limit = 10.0
Nav_Push_Orbit_Radius_Base = 1.45   #orbit 基础半径系数，实际轨迹半径=该系数 * 车轮轴距；如果轨迹过大或过小可以调整该值
Nav_Push_Orbit_Radius_Gain = 0.01
Nav_Push_Orbit_Stop_Gyro_Th = 3.0   #orbit 过程中如果陀螺仪读数小于该值则认为已经接近目标角度，可以停止转向加速前进
Nav_Push_Orbit_Brake_Max_Ms = 1000
Nav_Push_Orbit_Vy_Sign_Right = -1
Nav_Push_Orbit_Vy_Sign_Up = 0
Nav_Push_Orbit_Vy_Sign_Left = 1
Nav_Push_Prepare_Reorient_Yaw = 10.0
Nav_Push_Prepare_Forward_Gain = 0.038
Nav_Push_Prepare_Lateral_Gain = 0.115
Nav_Push_Prepare_Forward_Limit = 3.6
Nav_Push_Prepare_Lateral_Limit = 5.2
Nav_Push_Prepare_Min_Vx = 1.5
Nav_Push_Prepare_Min_Vy = 3.0
Nav_Push_Prepare_Kick_X = 18
Nav_Push_Prepare_Kick_Vy = 5.0
Nav_Push_Prepare_Lateral_Brake_Ms = 70
Nav_Push_Prepare_Lateral_Brake_Speed = 5.0
Nav_Push_Prepare_Back_Ms = 90
Nav_Push_Prepare_Back_Speed = 4.0
Nav_Push_Prepare_Ok_X = 4    #准备阶段前进误差小于该值即认为横移准备就绪
Nav_Push_Prepare_Ok_Y_Min = -8
Nav_Push_Prepare_Ok_Y_Max = 8  #准备阶段横移误差小于该值即认为前进准备就绪
Nav_Push_Prepare_Ok_Yaw = 4   #准备阶段定向误差小于该值即认为定向准备就绪
Nav_Push_Prepare_Ok_Ms = 70
Nav_Push_Execute_Forward_Speed = 6.5    #执行阶段前进速度
Nav_Push_Execute_Gyro_Limit = 12.0
Nav_Push_Line_Lost_Ms = 150
Nav_Push_Line_Extra_Ms = 180
Nav_Push_Back_Speed = 20
Nav_Push_Back_Ms = 800
Nav_Push_Turn_Slow_Yaw = 35.0
Nav_Push_Turn_Fast_Rate = 170.0
Nav_Push_Turn_Slow_Rate = 55.0
Nav_Push_Turn_Gyro_Limit = 16.0
Nav_Push_Turn_Ok_Yaw = 6.0
Nav_Push_Turn_Ok_Ms = 150
Nav_Post_Turn_No_Target_Ms = 200
Nav_Post_Turn_Forward_Ms = 1500
Nav_Post_Turn_Forward_Speed = 9

# ====================== 全局状态变量 ======================
# 小车启动标志：False=上电静止，True=已启动
car_started = False
auto_start_done = False
# 上一次 C9 发车键状态（用于消抖）
last_c9_state = 1
# 上一次 C8 退出键状态（用于消抖）
last_c8_state = 1

# 视觉误差与导航状态
cam_error_x = 0
cam_error_y = 0
cam_target_vx = 0.0
cam_target_vy = 0.0
cam_last_rx_ms = utime.ticks_ms()
cam_rx_buf = bytearray()
cam_has_target = False
cam_rx_started = False
cam_valid_target_since_ms = 0
nav_state = NAV_STATE_SEARCH
nav_detect_since_ms = 0
nav_target_lost_since_ms = 0
nav_search_turn_ok_since_ms = 0
nav_coarse_ok_since_ms = 0
nav_fine_ok_since_ms = 0
nav_fine_last_x_sign = 0
nav_fine_last_y_sign = 0
nav_fine_brake_since_ms = 0
nav_fine_brake_vy = 0.0
nav_fine_forward_brake_since_ms = 0
nav_fine_forward_brake_armed = True
nav_transition_ms = 0
nav_push_prepare_ok_since_ms = 0
nav_push_prepare_back_since_ms = 0
nav_push_turn_ok_since_ms = 0
nav_ready_for_push = False
field_up_yaw = 0.0
field_right_yaw = 90.0
field_left_yaw = 270.0
field_down_yaw = 180.0
launch_yaw = 0.0
field_reference_valid = False
gyro_pid = None
search_turn_yaw_target = 45.0
push_dir_code = Push_Dir_None
push_dir_name = "NONE"
push_yaw_target = 0.0
push_return_yaw_target = 0.0
push_face_obj_yaw = 0.0
push_orbit_dir = 0
push_orbit_vy_sign = 0
push_orbit_blocked = False
push_orbit_done = False
push_orbit_reached = False
push_orbit_progress_deg = 0.0
push_orbit_target_delta = 0.0
push_orbit_last_ms = 0
push_orbit_brake_since_ms = 0
push_orbit_radius_ratio = Nav_Push_Orbit_Radius_Base
line_crossed = False
push_line_seen_once = False
push_line_lost_since_ms = 0
push_line_extra_since_ms = 0

def wrapped_yaw_error(ref_deg, now_deg):
    err = now_deg - ref_deg
    if err > 180.0:
        err -= 360.0
    elif err < -180.0:
        err += 360.0
    return err


def normalize_yaw_deg(yaw_deg):
    while yaw_deg >= 360.0:
        yaw_deg -= 360.0
    while yaw_deg < 0.0:
        yaw_deg += 360.0
    return yaw_deg


def push_dir_label(dir_code):
    if dir_code == Push_Dir_Right:
        return "RIGHT"
    if dir_code == Push_Dir_Up:
        return "UP"
    if dir_code == Push_Dir_Left:
        return "LEFT"
    if dir_code == Push_Dir_Down:
        return "DOWN"
    return "NONE"


def yaw_from_field_dir(dir_code):
    if dir_code == Push_Dir_Right:
        return field_right_yaw
    if dir_code == Push_Dir_Up:
        return field_up_yaw
    if dir_code == Push_Dir_Left:
        return field_left_yaw
    if dir_code == Push_Dir_Down:
        return field_down_yaw
    return field_up_yaw


def refresh_field_reference(force=False):
    global field_up_yaw, field_right_yaw, field_left_yaw, field_down_yaw, launch_yaw
    global field_reference_valid

    if field_reference_valid and (not force):
        log(
            "[FIELD] keep up=%.2f right=%.2f left=%.2f down=%.2f launch=%.2f"
            % (field_up_yaw, field_right_yaw, field_left_yaw, field_down_yaw, launch_yaw)
        )
        return False

    if ENABLE_IMU and imu_runtime is not None:
        launch_yaw = normalize_yaw_deg(imu_runtime.read_yaw())
    else:
        launch_yaw = 0.0

    field_up_yaw = launch_yaw
    # Measured yaw decreases when the car physically turns left.
    field_right_yaw = normalize_yaw_deg(field_up_yaw + 90.0)
    field_left_yaw = normalize_yaw_deg(field_up_yaw - 90.0)
    field_down_yaw = normalize_yaw_deg(field_up_yaw + 180.0)
    log(
        "[FIELD] up=%.2f right=%.2f left=%.2f down=%.2f launch=%.2f"
        % (field_up_yaw, field_right_yaw, field_left_yaw, field_down_yaw, launch_yaw)
    )
    field_reference_valid = True
    return True


def push_yaw_error_deg(yaw_deg):
    if not ENABLE_IMU:
        return 0.0
    return -wrapped_yaw_error(push_yaw_target, yaw_deg)


def orbit_turn_dir_from_dir(dir_code):
    if dir_code == Push_Dir_Right:
        return 1
    if dir_code == Push_Dir_Left:
        return -1
    return 0


def orbit_vy_sign_from_dir(dir_code):
    if dir_code == Push_Dir_Right:
        return Nav_Push_Orbit_Vy_Sign_Right
    if dir_code == Push_Dir_Up:
        return Nav_Push_Orbit_Vy_Sign_Up
    if dir_code == Push_Dir_Left:
        return Nav_Push_Orbit_Vy_Sign_Left
    return 0


def reset_gyro_pid_state():
    pid = gyro_pid
    if pid is not None:
        pid.output = 0.0
        pid.err = 0.0
        pid.err_last = 0.0


def get_push_orbit_motion(yaw_err_abs):
    if yaw_err_abs <= Nav_Push_Orient_Ok_Yaw:
        return 0.0, 0.0
    if yaw_err_abs > Nav_Push_Orbit_Slow_Yaw:
        vy_base = Nav_Push_Orbit_Fast_Vy
        turn_rate_mag = Nav_Push_Orbit_Fast_Rate
    else:
        span = Nav_Push_Orbit_Slow_Yaw - Nav_Push_Orient_Ok_Yaw
        ratio = (yaw_err_abs - Nav_Push_Orient_Ok_Yaw) / span
        vy_base = Nav_Push_Orbit_Fast_Vy * ratio
        turn_rate_mag = Nav_Push_Orbit_Slow_Rate + (Nav_Push_Orbit_Fast_Rate - Nav_Push_Orbit_Slow_Rate) * ratio
    return vy_base * push_orbit_radius_ratio, turn_rate_mag


def get_push_turn_rate(yaw_err_abs):
    if yaw_err_abs <= Nav_Push_Turn_Ok_Yaw:
        return 0.0
    if yaw_err_abs > Nav_Push_Turn_Slow_Yaw:
        return Nav_Push_Turn_Fast_Rate
    span = Nav_Push_Turn_Slow_Yaw - Nav_Push_Turn_Ok_Yaw
    ratio = (yaw_err_abs - Nav_Push_Turn_Ok_Yaw) / span
    return Nav_Push_Turn_Slow_Rate + (Nav_Push_Turn_Fast_Rate - Nav_Push_Turn_Slow_Rate) * ratio


def update_push_orbit_radius(err_y):
    global push_orbit_radius_ratio

    push_orbit_radius_ratio = Nav_Push_Orbit_Radius_Base + err_y * Nav_Push_Orbit_Radius_Gain
    if push_orbit_radius_ratio < 0.8:
        push_orbit_radius_ratio = 0.8
    elif push_orbit_radius_ratio > 2.5:
        push_orbit_radius_ratio = 2.5


def update_cam_target(err_x, err_y):
    global cam_error_x, cam_error_y, cam_last_rx_ms

    cam_error_x = int(err_x)
    cam_error_y = int(err_y)
    cam_last_rx_ms = utime.ticks_ms()


def clear_cam_target_state():
    global cam_error_x, cam_error_y, cam_last_rx_ms, cam_rx_buf
    global cam_has_target, cam_valid_target_since_ms

    cam_error_x = 0
    cam_error_y = 0
    cam_has_target = False
    cam_valid_target_since_ms = 0
    cam_last_rx_ms = 0
    cam_rx_buf = bytearray()


def cam_packet_fresh():
    return utime.ticks_diff(utime.ticks_ms(), cam_last_rx_ms) <= Cam_Packet_Timeout_Ms


def cam_target_seen():
    if not cam_packet_fresh():
        return False
    return cam_has_target


def send_art_mode_command(new_state):
    if new_state in (NAV_STATE_SEARCH, NAV_STATE_SEARCH_TURN, NAV_STATE_POST_TURN_FORWARD):
        cam_uart.write(ART_MODE_SEARCH_CMD)
    elif new_state == NAV_STATE_COARSE:
        cam_uart.write(ART_MODE_COARSE_CMD)
    elif new_state == NAV_STATE_FINE:
        cam_uart.write(ART_MODE_FINE_CMD)
    elif new_state == NAV_STATE_PUSH_CLASSIFY:
        cam_uart.write(ART_MODE_CLASSIFY_CMD)
    elif new_state == NAV_STATE_PUSH_ORIENT:
        cam_uart.write(ART_MODE_IDLE_CMD)
    elif new_state == NAV_STATE_PUSH_PREPARE:
        cam_uart.write(ART_MODE_FINE_CMD)
    elif new_state == NAV_STATE_PUSH:
        cam_uart.write(ART_MODE_LINE_CMD)
    elif new_state in (NAV_STATE_PUSH_BACK, NAV_STATE_PUSH_TURN):
        cam_uart.write(ART_MODE_IDLE_CMD)


def nav_state_code(state):
    if state == NAV_STATE_SEARCH:
        return 0
    if state == NAV_STATE_SEARCH_TURN:
        return 1
    if state == NAV_STATE_COARSE:
        return 2
    if state == NAV_STATE_FINE:
        return 3
    if state == NAV_STATE_PUSH_CLASSIFY:
        return 4
    if state == NAV_STATE_PUSH_ORIENT:
        return 5
    if state == NAV_STATE_PUSH_PREPARE:
        return 6
    if state == NAV_STATE_PUSH:
        return 7
    if state == NAV_STATE_PUSH_BACK:
        return 8
    if state == NAV_STATE_PUSH_TURN:
        return 9
    if state == NAV_STATE_POST_TURN_FORWARD:
        return 10
    return -1


def nav_set_state(new_state, reason="", force=False):
    global nav_state, nav_detect_since_ms, nav_target_lost_since_ms, nav_search_turn_ok_since_ms
    global nav_coarse_ok_since_ms, nav_fine_ok_since_ms, nav_transition_ms
    global nav_fine_last_x_sign, nav_fine_last_y_sign
    global nav_fine_brake_since_ms, nav_fine_brake_vy, nav_fine_forward_brake_since_ms
    global nav_fine_forward_brake_armed
    global nav_push_prepare_ok_since_ms, nav_push_prepare_back_since_ms, nav_push_turn_ok_since_ms
    global nav_ready_for_push, cam_target_vx, cam_target_vy
    global yaw_ref_deg, cam_rx_started, push_dir_code, push_dir_name
    global line_crossed, push_line_seen_once, push_line_lost_since_ms, push_line_extra_since_ms
    global push_return_yaw_target, search_turn_yaw_target
    global push_orbit_dir, push_orbit_vy_sign, push_orbit_done
    global push_orbit_reached, push_orbit_progress_deg, push_orbit_last_ms
    global push_orbit_brake_since_ms
    global last_pwm_fl, last_pwm_fr, last_pwm_b

    if nav_state == new_state and (not force):
        return

    now = utime.ticks_ms()
    nav_state = new_state
    nav_detect_since_ms = 0
    nav_target_lost_since_ms = 0
    nav_search_turn_ok_since_ms = 0
    nav_coarse_ok_since_ms = 0
    nav_fine_ok_since_ms = 0
    nav_fine_last_x_sign = 0
    nav_fine_last_y_sign = 0
    nav_fine_brake_since_ms = 0
    nav_fine_brake_vy = 0.0
    nav_fine_forward_brake_since_ms = 0
    nav_fine_forward_brake_armed = True
    nav_push_prepare_ok_since_ms = 0
    nav_push_prepare_back_since_ms = 0
    nav_push_turn_ok_since_ms = 0
    nav_transition_ms = now
    nav_ready_for_push = False
    cam_target_vx = 0.0
    cam_target_vy = 0.0

    if new_state in (
        NAV_STATE_SEARCH,
        NAV_STATE_SEARCH_TURN,
        NAV_STATE_POST_TURN_FORWARD,
        NAV_STATE_COARSE,
        NAV_STATE_FINE,
        NAV_STATE_PUSH_CLASSIFY,
        NAV_STATE_PUSH_PREPARE,
        NAV_STATE_PUSH,
    ):
        clear_cam_target_state()

    if new_state in (NAV_STATE_SEARCH, NAV_STATE_SEARCH_TURN, NAV_STATE_POST_TURN_FORWARD, NAV_STATE_PUSH, NAV_STATE_PUSH_CLASSIFY):
        line_crossed = False
        push_line_seen_once = False
        push_line_lost_since_ms = 0
        push_line_extra_since_ms = 0

    if new_state in (NAV_STATE_SEARCH, NAV_STATE_SEARCH_TURN, NAV_STATE_POST_TURN_FORWARD, NAV_STATE_COARSE, NAV_STATE_PUSH_CLASSIFY):
        push_orbit_done = False

    if new_state == NAV_STATE_PUSH_CLASSIFY:
        push_dir_code = Push_Dir_None
        push_dir_name = "NONE"
        push_orbit_dir = 0
        push_orbit_vy_sign = 0
        push_orbit_reached = False
        push_orbit_progress_deg = 0.0
        push_orbit_last_ms = 0
        push_orbit_brake_since_ms = 0
    elif new_state == NAV_STATE_PUSH_BACK:
        push_return_yaw_target = normalize_yaw_deg(push_yaw_target + 180.0)

    if new_state == NAV_STATE_FINE:
        if cam_error_x > Nav_Lateral_Deadband:
            nav_fine_last_x_sign = 1
        elif cam_error_x < -Nav_Lateral_Deadband:
            nav_fine_last_x_sign = -1
        if cam_error_y > Nav_Forward_Deadband:
            nav_fine_last_y_sign = 1
        elif cam_error_y < -Nav_Forward_Deadband:
            nav_fine_last_y_sign = -1

    if new_state in (NAV_STATE_PUSH_PREPARE, NAV_STATE_PUSH):
        reset_gyro_pid_state()
    if new_state == NAV_STATE_PUSH:
        pid_fl.output = pid_fr.output = pid_b.output = 0.0
        pid_fl.tar_spd_last = pid_fr.tar_spd_last = pid_b.tar_spd_last = 0.0
        pid_fl.delta_ud = pid_fr.delta_ud = pid_b.delta_ud = 0.0
        last_pwm_fl = last_pwm_fr = last_pwm_b = 0

    if ENABLE_IMU and imu_runtime is not None:
        if new_state == NAV_STATE_SEARCH_TURN:
            search_turn_yaw_target = normalize_yaw_deg(field_up_yaw + Nav_Search_Turn_Yaw)
            yaw_ref_deg = search_turn_yaw_target
        elif new_state == NAV_STATE_FINE:
            if push_orbit_done:
                yaw_ref_deg = push_yaw_target
            else:
                yaw_ref_deg = imu_runtime.read_yaw()
        elif new_state in (NAV_STATE_COARSE, NAV_STATE_PUSH_CLASSIFY):
            yaw_ref_deg = imu_runtime.read_yaw()
        elif new_state in (NAV_STATE_PUSH_ORIENT, NAV_STATE_PUSH_PREPARE, NAV_STATE_PUSH, NAV_STATE_PUSH_BACK):
            yaw_ref_deg = push_yaw_target
        elif new_state == NAV_STATE_PUSH_TURN:
            yaw_ref_deg = push_return_yaw_target
        elif new_state == NAV_STATE_POST_TURN_FORWARD:
            yaw_ref_deg = push_return_yaw_target

    update_nav_led_display()
    send_art_mode_command(new_state)

    if cam_rx_started:
        if reason:
            log("[NAV] -> %s (%s)" % (new_state, reason))
        else:
            log("[NAV] -> %s" % new_state)


def apply_nav_targets(vx, vy, vx_limit, vy_limit):
    global cam_target_vx, cam_target_vy

    if -Nav_Forward_Deadband <= cam_error_y <= Nav_Forward_Deadband:
        vx = 0.0
    if -Nav_Lateral_Deadband <= cam_error_x <= Nav_Lateral_Deadband:
        vy = 0.0

    if vx > vx_limit:
        vx = vx_limit
    elif vx < -vx_limit:
        vx = -vx_limit

    if vy > vy_limit:
        vy = vy_limit
    elif vy < -vy_limit:
        vy = -vy_limit

    cam_target_vx = vx
    cam_target_vy = vy


def update_nav_state_and_targets(yaw_deg, low_speed, gyro_z):
    global nav_detect_since_ms, nav_target_lost_since_ms, nav_search_turn_ok_since_ms
    global nav_coarse_ok_since_ms, nav_fine_ok_since_ms
    global nav_fine_last_x_sign, nav_fine_last_y_sign
    global nav_fine_brake_since_ms, nav_fine_brake_vy, nav_fine_forward_brake_since_ms
    global nav_fine_forward_brake_armed
    global nav_push_prepare_ok_since_ms, nav_push_prepare_back_since_ms, nav_push_turn_ok_since_ms
    global nav_ready_for_push
    global cam_target_vx, cam_target_vy, yaw_ref_deg, push_yaw_target
    global line_crossed, push_line_seen_once, push_line_lost_since_ms, push_line_extra_since_ms
    global push_return_yaw_target, push_face_obj_yaw
    global push_orbit_dir, push_orbit_vy_sign, push_orbit_blocked, push_orbit_done
    global push_orbit_reached, push_orbit_progress_deg, push_orbit_target_delta, push_orbit_last_ms
    global push_orbit_brake_since_ms

    now = utime.ticks_ms()
    track_target_states = (
        NAV_STATE_SEARCH,
        NAV_STATE_SEARCH_TURN,
        NAV_STATE_POST_TURN_FORWARD,
        NAV_STATE_COARSE,
        NAV_STATE_FINE,
        NAV_STATE_PUSH_PREPARE,
    )
    seen = cam_target_seen() if nav_state in track_target_states else False

    if nav_state in track_target_states:
        if seen:
            nav_target_lost_since_ms = 0
            if nav_detect_since_ms == 0:
                nav_detect_since_ms = now
        else:
            nav_detect_since_ms = 0
            if nav_target_lost_since_ms == 0:
                nav_target_lost_since_ms = now
    else:
        nav_detect_since_ms = 0
        nav_target_lost_since_ms = 0

    if nav_state == NAV_STATE_SEARCH_TURN:
        nav_ready_for_push = False
        cam_target_vx = 0.0
        cam_target_vy = 0.0
        if seen:
            if ENABLE_IMU:
                yaw_ref_deg = yaw_deg
            nav_set_state(NAV_STATE_COARSE, "target_seen_during_search_turn")
            return
        yaw_ref_deg = search_turn_yaw_target
        yaw_search_err_abs = abs(-wrapped_yaw_error(yaw_ref_deg, yaw_deg)) if ENABLE_IMU else 0.0
        search_turn_stable = (
            (not ENABLE_IMU)
            or (
                yaw_search_err_abs <= Nav_Search_Turn_Ok_Yaw
                and abs(gyro_z) <= Nav_Search_Turn_Gyro_Th
            )
        )
        if search_turn_stable:
            if nav_search_turn_ok_since_ms == 0:
                nav_search_turn_ok_since_ms = now
            elif utime.ticks_diff(now, nav_search_turn_ok_since_ms) >= Nav_Search_Turn_Ok_Ms:
                nav_set_state(NAV_STATE_SEARCH, "search_turn_done")
        else:
            nav_search_turn_ok_since_ms = 0
        return

    if nav_state == NAV_STATE_SEARCH:
        cam_target_vx = 0.0
        cam_target_vy = 0.0
        nav_ready_for_push = False
        if push_orbit_blocked:
            if nav_target_lost_since_ms > 0:
                if utime.ticks_diff(now, nav_target_lost_since_ms) >= Nav_Target_Lost_Ms:
                    push_orbit_blocked = False
                    log("[NAV] orbit retry unlocked after target lost")
            return
        if seen and nav_detect_since_ms > 0:
            if utime.ticks_diff(now, nav_detect_since_ms) >= Nav_Detect_Ms:
                if ENABLE_IMU:
                    yaw_ref_deg = yaw_deg
                nav_set_state(NAV_STATE_COARSE, "target_detected")
        return

    if nav_state == NAV_STATE_POST_TURN_FORWARD:
        nav_ready_for_push = False
        yaw_ref_deg = push_return_yaw_target
        cam_target_vy = 0.0
        if seen:
            cam_target_vx = 0.0
            if nav_detect_since_ms > 0:
                if utime.ticks_diff(now, nav_detect_since_ms) >= Nav_Detect_Ms:
                    if ENABLE_IMU:
                        yaw_ref_deg = yaw_deg
                    nav_set_state(NAV_STATE_COARSE, "post_turn_target_seen")
            return

        post_turn_elapsed = utime.ticks_diff(now, nav_transition_ms)
        if post_turn_elapsed < Nav_Post_Turn_No_Target_Ms:
            cam_target_vx = 0.0
        elif post_turn_elapsed < Nav_Post_Turn_No_Target_Ms + Nav_Post_Turn_Forward_Ms:
            cam_target_vx = Nav_Post_Turn_Forward_Speed
        else:
            cam_target_vx = 0.0
            nav_set_state(NAV_STATE_SEARCH, "post_turn_forward_done")
        return

    if nav_state in (NAV_STATE_COARSE, NAV_STATE_FINE, NAV_STATE_PUSH_PREPARE):
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Transition_Grace_Ms:
            if nav_target_lost_since_ms > 0:
                if utime.ticks_diff(now, nav_target_lost_since_ms) >= Nav_Target_Lost_Ms:
                    if nav_state == NAV_STATE_PUSH_PREPARE:
                        nav_set_state(NAV_STATE_FINE, "%s_target_lost" % nav_state.lower())
                    else:
                        nav_set_state(NAV_STATE_SEARCH, "target_lost")
                    return

    if nav_state == NAV_STATE_COARSE:
        nav_ready_for_push = False
        blend_start = Nav_Coarse_Exit_Y + 40
        if cam_error_y < blend_start:
            t = float(cam_error_y - Nav_Coarse_Exit_Y) / 40.0
            t = max(0.0, min(1.0, t))
            fwd_gain = Nav_Fine_Forward_Gain + t * (Nav_Coarse_Forward_Gain - Nav_Fine_Forward_Gain)
            lat_gain = Nav_Fine_Lateral_Gain + t * (Nav_Coarse_Lateral_Gain - Nav_Fine_Lateral_Gain)
        else:
            fwd_gain = Nav_Coarse_Forward_Gain
            lat_gain = Nav_Coarse_Lateral_Gain
        apply_nav_targets(
            cam_error_y * fwd_gain,
            -cam_error_x * lat_gain,
            Nav_Coarse_Forward_Limit,
            Nav_Coarse_Lateral_Limit,
        )
        if cam_error_y <= Nav_Coarse_Exit_Y:
            if nav_coarse_ok_since_ms == 0:
                nav_coarse_ok_since_ms = now
            elif utime.ticks_diff(now, nav_coarse_ok_since_ms) >= Nav_Coarse_Ok_Ms:
                nav_set_state(NAV_STATE_FINE, "err_y<=%d for %dms" % (Nav_Coarse_Exit_Y, Nav_Coarse_Ok_Ms))
        else:
            nav_coarse_ok_since_ms = 0
        return

    if nav_state == NAV_STATE_FINE:
        nav_ready_for_push = False
        if push_orbit_done:
            fine_lateral_gain = Nav_Fine_Lateral_Gain
        else:
            fine_lateral_gain = Nav_Fine_Classify_Lateral_Gain
        if not seen:
            cam_target_vx = 0.0
            cam_target_vy = 0.0
            nav_fine_ok_since_ms = 0
            nav_fine_last_x_sign = 0
            nav_fine_last_y_sign = 0
            nav_fine_brake_since_ms = 0
            nav_fine_brake_vy = 0.0
            nav_fine_forward_brake_since_ms = 0
            nav_fine_forward_brake_armed = True
            return
        current_x_sign = 0
        if cam_error_x > Nav_Lateral_Deadband:
            current_x_sign = 1
        elif cam_error_x < -Nav_Lateral_Deadband:
            current_x_sign = -1
        current_y_sign = 0
        if cam_error_y > Nav_Forward_Deadband:
            current_y_sign = 1
        elif cam_error_y < -Nav_Forward_Deadband:
            current_y_sign = -1
        fine_braking = False
        if current_y_sign > 0:
            nav_fine_forward_brake_armed = True
        if Nav_Fine_Forward_Brake_Enable and current_y_sign != 0:
            if (
                nav_fine_forward_brake_armed
                and (
                    cam_error_y < Nav_Fine_Forward_Brake_Y
                    or (
                        nav_fine_last_y_sign > 0
                        and current_y_sign < 0
                    )
                )
            ):
                nav_fine_forward_brake_since_ms = now
                nav_fine_forward_brake_armed = False
            nav_fine_last_y_sign = current_y_sign
        if Nav_Fine_Lateral_Brake_Enable and current_x_sign != 0:
            if (
                nav_fine_last_x_sign != 0
                and current_x_sign != nav_fine_last_x_sign
                and abs(cam_error_x) <= Nav_Fine_Lateral_Brake_Window_X
            ):
                nav_fine_brake_since_ms = now
                nav_fine_brake_vy = -current_x_sign * Nav_Fine_Lateral_Brake_Speed
            nav_fine_last_x_sign = current_x_sign
        if (
            Nav_Fine_Forward_Brake_Enable
            and nav_fine_forward_brake_since_ms != 0
            and utime.ticks_diff(now, nav_fine_forward_brake_since_ms) < Nav_Fine_Forward_Brake_Ms
        ):
            fine_braking = True
            cam_target_vx = -Nav_Fine_Forward_Brake_Speed
            cam_target_vy = 0.0
        elif Nav_Fine_Forward_Brake_Enable and nav_fine_forward_brake_since_ms != 0:
            nav_fine_forward_brake_since_ms = 0
            apply_nav_targets(
                cam_error_y * Nav_Fine_Forward_Gain,
                -cam_error_x * fine_lateral_gain,
                Nav_Fine_Forward_Limit,
                Nav_Fine_Lateral_Limit,
            )
        elif (not Nav_Fine_Lateral_Brake_Enable) or nav_fine_brake_since_ms == 0:
            apply_nav_targets(
                cam_error_y * Nav_Fine_Forward_Gain,
                -cam_error_x * fine_lateral_gain,
                Nav_Fine_Forward_Limit,
                Nav_Fine_Lateral_Limit,
            )
        elif utime.ticks_diff(now, nav_fine_brake_since_ms) < Nav_Fine_Lateral_Brake_Ms:
            fine_braking = True
            cam_target_vx = 0.0
            if nav_fine_brake_vy > Nav_Fine_Lateral_Limit:
                cam_target_vy = Nav_Fine_Lateral_Limit
            elif nav_fine_brake_vy < -Nav_Fine_Lateral_Limit:
                cam_target_vy = -Nav_Fine_Lateral_Limit
            else:
                cam_target_vy = nav_fine_brake_vy
        else:
            nav_fine_brake_since_ms = 0
            nav_fine_brake_vy = 0.0
            apply_nav_targets(
                cam_error_y * Nav_Fine_Forward_Gain,
                -cam_error_x * fine_lateral_gain,
                Nav_Fine_Forward_Limit,
                Nav_Fine_Lateral_Limit,
            )
        fine_yaw_ok = (
            (not push_orbit_done)
            or (not ENABLE_IMU)
            or (abs(push_yaw_error_deg(yaw_deg)) <= Nav_Push_Prepare_Reorient_Yaw)
        )
        if push_orbit_done:
            fine_ok_x = Nav_Fine_Push_Ok_X
            fine_ok_y_min = Nav_Fine_Push_Ok_Y_Min
            fine_ok_y_max = Nav_Fine_Push_Ok_Y_Max
            fine_ok_ms = Nav_Fine_Push_Ok_Ms
        else:
            fine_ok_x = Nav_Fine_Classify_Ok_X
            fine_ok_y_min = Nav_Fine_Classify_Ok_Y_Min
            fine_ok_y_max = Nav_Fine_Classify_Ok_Y_Max
            fine_ok_ms = Nav_Fine_Classify_Ok_Ms
        if (
            abs(cam_error_x) <= fine_ok_x
            and cam_error_y >= fine_ok_y_min
            and cam_error_y <= fine_ok_y_max
            and fine_yaw_ok
            and low_speed
            and (not fine_braking)
        ):
            if nav_fine_ok_since_ms == 0:
                nav_fine_ok_since_ms = now
            elif utime.ticks_diff(now, nav_fine_ok_since_ms) >= fine_ok_ms:
                cam_target_vx = 0.0
                cam_target_vy = 0.0
                if push_orbit_done:
                    nav_set_state(
                        NAV_STATE_PUSH_PREPARE,
                        "orbit_refine_locked seen=%d age=%d err=(%d,%d)"
                        % (
                            1 if seen else 0,
                            utime.ticks_diff(now, cam_last_rx_ms),
                            cam_error_x,
                            cam_error_y,
                        ),
                    )
                else:
                    update_push_orbit_radius(cam_error_y)
                    push_face_obj_yaw = yaw_deg
                    nav_set_state(
                        NAV_STATE_PUSH_CLASSIFY,
                        "fine_locked seen=%d age=%d err=(%d,%d)"
                        % (
                            1 if seen else 0,
                            utime.ticks_diff(now, cam_last_rx_ms),
                            cam_error_x,
                            cam_error_y,
                        ),
                    )
        else:
            nav_fine_ok_since_ms = 0
        return

    if nav_state == NAV_STATE_PUSH_CLASSIFY:
        nav_ready_for_push = False
        cam_target_vx = 0.0
        cam_target_vy = 0.0
        if push_dir_code in (Push_Dir_Right, Push_Dir_Up, Push_Dir_Left, Push_Dir_Down):
            push_yaw_target = yaw_from_field_dir(push_dir_code)
            push_orbit_dir = orbit_turn_dir_from_dir(push_dir_code)
            push_orbit_vy_sign = orbit_vy_sign_from_dir(push_dir_code)
            orbit_yaw_err = -wrapped_yaw_error(push_yaw_target, push_face_obj_yaw)
            push_orbit_target_delta = abs(orbit_yaw_err)
            if push_orbit_target_delta <= Nav_Push_Orbit_Skip_Yaw:
                push_orbit_dir = 0
                push_orbit_vy_sign = 0
            else:
                if orbit_yaw_err >= 0.0:
                    push_orbit_dir = 1
                else:
                    push_orbit_dir = -1
                push_orbit_vy_sign = -push_orbit_dir
            push_orbit_progress_deg = 0.0
            push_orbit_reached = False
            push_orbit_last_ms = now
            push_orbit_brake_since_ms = 0
            if push_orbit_dir == 0:
                push_orbit_done = True
                nav_set_state(
                    NAV_STATE_FINE,
                    "class=%s orbit_skip_refine" % push_dir_label(push_dir_code),
                )
            else:
                nav_set_state(
                    NAV_STATE_PUSH_ORIENT,
                    "class=%s face=%.1f yaw=%.1f orbit_dir=%d vy_sign=%d"
                    % (
                        push_dir_label(push_dir_code),
                        push_face_obj_yaw,
                        push_yaw_target,
                        push_orbit_dir,
                        push_orbit_vy_sign,
                    ),
                )
        elif utime.ticks_diff(now, nav_transition_ms) >= Nav_Classify_Timeout_Ms:
            nav_set_state(NAV_STATE_FINE, "classify_timeout")
        return

    if nav_state == NAV_STATE_PUSH_ORIENT:
        nav_ready_for_push = False
        yaw_ref_deg = push_yaw_target
        yaw_err_abs = abs(-wrapped_yaw_error(yaw_ref_deg, yaw_deg)) if ENABLE_IMU else 0.0
        orbit_stop_delta = max(0.0, push_orbit_target_delta - Nav_Push_Orient_Ok_Yaw)
        if push_orbit_last_ms == 0:
            push_orbit_last_ms = now
        elif not push_orbit_reached:
            orbit_step = push_orbit_dir * gyro_z * utime.ticks_diff(now, push_orbit_last_ms) * 0.001
            if orbit_step > 0.0:
                push_orbit_progress_deg += orbit_step
            push_orbit_last_ms = now
        if push_orbit_progress_deg >= orbit_stop_delta or yaw_err_abs <= Nav_Push_Orient_Ok_Yaw:
            if push_orbit_progress_deg > orbit_stop_delta:
                push_orbit_progress_deg = orbit_stop_delta
            push_orbit_reached = True
            if push_orbit_brake_since_ms == 0:
                push_orbit_brake_since_ms = now
        orbit_vy_mag, _orbit_turn_mag = get_push_orbit_motion(max(0.0, push_orbit_target_delta - push_orbit_progress_deg))
        cam_target_vx = 0.0
        if push_orbit_reached:
            cam_target_vy = 0.0
        else:
            cam_target_vy = push_orbit_vy_sign * orbit_vy_mag
        if (not ENABLE_IMU) or push_orbit_reached:
            brake_elapsed = 0
            if push_orbit_brake_since_ms > 0:
                brake_elapsed = utime.ticks_diff(now, push_orbit_brake_since_ms)
            if (not ENABLE_IMU) or abs(gyro_z) <= Nav_Push_Orbit_Stop_Gyro_Th or brake_elapsed >= Nav_Push_Orbit_Brake_Max_Ms:
                push_orbit_done = True
                nav_set_state(NAV_STATE_FINE, "orbit_done_refine")
        else:
            push_orbit_brake_since_ms = 0
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Push_Orient_Max_Ms:
            cam_target_vx = 0.0
            cam_target_vy = 0.0
            push_orbit_blocked = True
            nav_set_state(NAV_STATE_SEARCH, "orbit_timeout_blocked")
        return

    if nav_state == NAV_STATE_PUSH_PREPARE:
        nav_ready_for_push = False
        yaw_ref_deg = push_yaw_target
        yaw_prepare_err_abs = abs(-wrapped_yaw_error(yaw_ref_deg, yaw_deg)) if ENABLE_IMU else 0.0
        if ENABLE_IMU and yaw_prepare_err_abs > Nav_Push_Prepare_Reorient_Yaw:
            cam_target_vx = 0.0
            cam_target_vy = 0.0
            nav_push_prepare_ok_since_ms = 0
            nav_push_prepare_back_since_ms = 0
            return
        if not seen:
            cam_target_vx = 0.0
            cam_target_vy = 0.0
            nav_push_prepare_ok_since_ms = 0
            nav_push_prepare_back_since_ms = 0
            return
        prepare_braking = False
        current_prepare_x_sign = 0
        if cam_error_x > Nav_Push_Prepare_Ok_X:
            current_prepare_x_sign = 1
        elif cam_error_x < -Nav_Push_Prepare_Ok_X:
            current_prepare_x_sign = -1
        if current_prepare_x_sign != 0:
            if (
                nav_fine_last_x_sign != 0
                and current_prepare_x_sign != nav_fine_last_x_sign
                and abs(cam_error_x) <= Nav_Push_Prepare_Kick_X
            ):
                nav_fine_brake_since_ms = now
                nav_fine_brake_vy = -current_prepare_x_sign * Nav_Push_Prepare_Lateral_Brake_Speed
            nav_fine_last_x_sign = current_prepare_x_sign
        if cam_error_y < Nav_Push_Prepare_Ok_Y_Min:
            if nav_push_prepare_back_since_ms == 0:
                nav_push_prepare_back_since_ms = now
        else:
            nav_push_prepare_back_since_ms = 0
        if nav_push_prepare_back_since_ms != 0 and utime.ticks_diff(now, nav_push_prepare_back_since_ms) < Nav_Push_Prepare_Back_Ms:
            prepare_braking = True
            cam_target_vx = -Nav_Push_Prepare_Back_Speed
            cam_target_vy = 0.0
        elif nav_fine_brake_since_ms != 0 and utime.ticks_diff(now, nav_fine_brake_since_ms) < Nav_Push_Prepare_Lateral_Brake_Ms:
            prepare_braking = True
            cam_target_vx = 0.0
            if nav_fine_brake_vy > Nav_Push_Prepare_Lateral_Limit:
                cam_target_vy = Nav_Push_Prepare_Lateral_Limit
            elif nav_fine_brake_vy < -Nav_Push_Prepare_Lateral_Limit:
                cam_target_vy = -Nav_Push_Prepare_Lateral_Limit
            else:
                cam_target_vy = nav_fine_brake_vy
        else:
            if nav_fine_brake_since_ms != 0:
                nav_fine_brake_since_ms = 0
                nav_fine_brake_vy = 0.0
            if Nav_Push_Prepare_Ok_Y_Min <= cam_error_y <= Nav_Push_Prepare_Ok_Y_Max:
                vx_cmd = 0.0
            else:
                vx_cmd = cam_error_y * Nav_Push_Prepare_Forward_Gain
                if 0.0 < vx_cmd < Nav_Push_Prepare_Min_Vx:
                    vx_cmd = Nav_Push_Prepare_Min_Vx
                elif -Nav_Push_Prepare_Min_Vx < vx_cmd < 0.0:
                    vx_cmd = -Nav_Push_Prepare_Min_Vx
            if -Nav_Push_Prepare_Ok_X <= cam_error_x <= Nav_Push_Prepare_Ok_X:
                vy_cmd = 0.0
            else:
                vy_cmd = -cam_error_x * Nav_Push_Prepare_Lateral_Gain
                min_vy = Nav_Push_Prepare_Min_Vy
                if abs(cam_error_x) <= Nav_Push_Prepare_Kick_X:
                    min_vy = Nav_Push_Prepare_Kick_Vy
                if 0.0 < vy_cmd < min_vy:
                    vy_cmd = min_vy
                elif -min_vy < vy_cmd < 0.0:
                    vy_cmd = -min_vy
            apply_nav_targets(vx_cmd, vy_cmd, Nav_Push_Prepare_Forward_Limit, Nav_Push_Prepare_Lateral_Limit)
        if (
            abs(cam_error_x) <= Nav_Push_Prepare_Ok_X
            and cam_error_y >= Nav_Push_Prepare_Ok_Y_Min
            and cam_error_y <= Nav_Push_Prepare_Ok_Y_Max
            and ((not ENABLE_IMU) or (yaw_prepare_err_abs <= Nav_Push_Prepare_Ok_Yaw))
            and low_speed
            and (not prepare_braking)
        ):
            if nav_push_prepare_ok_since_ms == 0:
                nav_push_prepare_ok_since_ms = now
            elif utime.ticks_diff(now, nav_push_prepare_ok_since_ms) >= Nav_Push_Prepare_Ok_Ms:
                cam_target_vx = 0.0
                cam_target_vy = 0.0
                nav_set_state(NAV_STATE_PUSH, "push_pose_locked")
        else:
            nav_push_prepare_ok_since_ms = 0
        return

    if nav_state == NAV_STATE_PUSH:
        nav_ready_for_push = True
        cam_target_vx = Nav_Push_Execute_Forward_Speed
        cam_target_vy = 0.0
        yaw_ref_deg = push_yaw_target
        if line_crossed:
            push_line_seen_once = True
            push_line_lost_since_ms = 0
            push_line_extra_since_ms = 0
        elif push_line_seen_once:
            if push_line_lost_since_ms == 0:
                push_line_lost_since_ms = now
            elif utime.ticks_diff(now, push_line_lost_since_ms) >= Nav_Push_Line_Lost_Ms:
                if push_line_extra_since_ms == 0:
                    push_line_extra_since_ms = now
                elif utime.ticks_diff(now, push_line_extra_since_ms) >= Nav_Push_Line_Extra_Ms:
                    nav_set_state(NAV_STATE_PUSH_BACK, "line_extra_after_cross")
        return

    if nav_state == NAV_STATE_PUSH_BACK:
        nav_ready_for_push = False
        cam_target_vx = -Nav_Push_Back_Speed
        cam_target_vy = 0.0
        yaw_ref_deg = push_yaw_target
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Push_Back_Ms:
            nav_set_state(NAV_STATE_PUSH_TURN, "push_back_done")
        return

    if nav_state == NAV_STATE_PUSH_TURN:
        nav_ready_for_push = False
        yaw_ref_deg = push_return_yaw_target
        cam_target_vx = 0.0
        cam_target_vy = 0.0
        yaw_err_abs = abs(-wrapped_yaw_error(yaw_ref_deg, yaw_deg)) if ENABLE_IMU else 0.0
        if (not ENABLE_IMU) or (yaw_err_abs <= Nav_Push_Turn_Ok_Yaw and low_speed):
            if nav_push_turn_ok_since_ms == 0:
                nav_push_turn_ok_since_ms = now
            elif utime.ticks_diff(now, nav_push_turn_ok_since_ms) >= Nav_Push_Turn_Ok_Ms:
                if ENABLE_IMU and imu_runtime is not None:
                    imu_runtime.reset_yaw(push_return_yaw_target)
                nav_set_state(NAV_STATE_POST_TURN_FORWARD, "push_finish_wait_target")
        else:
            nav_push_turn_ok_since_ms = 0
        return

    cam_target_vx = 0.0
    cam_target_vy = 0.0


def poll_art_uart():
    global cam_rx_buf, cam_error_x, cam_error_y, cam_target_vx, cam_target_vy
    global cam_has_target, cam_last_rx_ms, cam_rx_started, cam_valid_target_since_ms
    global push_dir_code, push_dir_name, push_yaw_target, line_crossed

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
                if cam_rx_buf[1] == Classify_Packet_Tag:
                    new_dir = int(cam_rx_buf[2])
                    if new_dir != push_dir_code:
                        push_dir_code = new_dir
                        push_dir_name = push_dir_label(new_dir)
                        log("[NAV] classify dir=%s" % push_dir_name)
                    cam_last_rx_ms = utime.ticks_ms()
                elif cam_rx_buf[1] == Line_Packet_Tag:
                    new_line_crossed = int(cam_rx_buf[2]) != 0
                    if new_line_crossed != line_crossed:
                        line_crossed = new_line_crossed
                        log("[NAV] line_crossed=%d" % (1 if line_crossed else 0))
                    cam_last_rx_ms = utime.ticks_ms()
                elif cam_rx_buf[1] == No_Target_Marker and cam_rx_buf[2] == No_Target_Marker:
                    cam_has_target = False
                    cam_valid_target_since_ms = 0
                    cam_last_rx_ms = utime.ticks_ms()
                else:
                    cam_has_target = True
                    if cam_valid_target_since_ms == 0:
                        cam_valid_target_since_ms = utime.ticks_ms()
                    update_cam_target(
                        (int(cam_rx_buf[1]) - Cam_Error_Offset) * Cam_Error_Scale,
                        (int(cam_rx_buf[2]) - Cam_Error_Offset) * Cam_Error_Scale,
                    )
                cam_rx_buf = cam_rx_buf[3:]

    if len(cam_rx_buf) > 20:
        cam_rx_buf = bytearray()


key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)

# ====================== 按键硬件初始化 ======================
# 发车启动按键
key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)
# 状态 LED
led_straight  = Pin(cfg.LED_STRAIGHT_PIN,  Pin.OUT, value=0)
led_translate = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
led_rotate    = Pin(cfg.LED_ROTATE_PIN,    Pin.OUT, value=0)

# ---------------------- Demo-style hardware init ----------------------
utime.sleep_ms(100)

# 心跳 LED
led = Pin(cfg.LED_HB_PIN, Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

# ====================== 硬件对象创建 ======================
motor_fl = Motor(cfg.MOTOR_FL_PH, cfg.MOTOR_FL_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FL_INVERT)
motor_fr = Motor(cfg.MOTOR_FR_PH, cfg.MOTOR_FR_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FR_INVERT)
motor_b  = Motor(cfg.MOTOR_B_PH,  cfg.MOTOR_B_PWM,  freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_B_INVERT)
enc_fl = encoder(cfg.ENC_FL_A, cfg.ENC_FL_B, cfg.ENC_FL_INVERT)
enc_fr = encoder(cfg.ENC_FR_A, cfg.ENC_FR_B, cfg.ENC_FR_INVERT)
enc_b  = encoder(cfg.ENC_B_A,  cfg.ENC_B_B,  cfg.ENC_B_INVERT)

debug_wireless = None
if DEBUG_WIRELESS_LOG_ENABLE:
    try:
        debug_wireless = WIRELESS_UART(DEBUG_WIRELESS_BAUD)
    except Exception:
        debug_wireless = None
wireless = None
if MASTER_MOTION_BROADCAST_ENABLE:
    try:
        wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
    except Exception:
        wireless = None
motion_tx_buf = array('b', [0] * 16)
cam_uart = UART(cfg.CAM_UART_ID, cfg.CAM_UART_BAUD)
cam_uart.init(cfg.CAM_UART_BAUD, timeout_char=100)

# IMU 运行时初始化
imu_runtime = None
if ENABLE_IMU:
    imu_runtime = IMUYawRuntime(
        sign=GYRO_SIGN,
        offset_z=GYRO_OFFSET_Z,
        scale=GYRO_SCALE,
        deadband_dps=GYRO_DEADBAND_DPS,
        tick_period_ms=TICK_PERIOD_MS,
    )

# ====================== LED 导航显示辅助 ======================
def update_nav_led_display():
    straight_value = 0
    translate_value = 0
    rotate_value = 0
    if nav_state == NAV_STATE_COARSE:
        straight_value = 1
    elif nav_state == NAV_STATE_POST_TURN_FORWARD:
        straight_value = 1
    elif nav_state in (NAV_STATE_FINE, NAV_STATE_PUSH_CLASSIFY, NAV_STATE_PUSH_PREPARE):
        translate_value = 1
    elif nav_state in (NAV_STATE_SEARCH_TURN, NAV_STATE_PUSH_ORIENT, NAV_STATE_PUSH, NAV_STATE_PUSH_BACK, NAV_STATE_PUSH_TURN):
        rotate_value = 1

    led_straight.value(straight_value)
    led_translate.value(translate_value)
    led_rotate.value(rotate_value)


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


# ====================== C9 发车检查 ======================
def check_c9_start():
    """处理发车按键，带消抖和延时发车。"""
    global last_c9_state, car_started, auto_start_done, start_time, yaw_ref_deg
    current_c9 = key_start.value()
    if current_c9 == 0 and last_c9_state == 1:
        utime.sleep_ms(10)
        if key_start.value() == 0:
            if not car_started:
                log("[C9] launch in 0.5s")
                utime.sleep_ms(500)
                calibrate_gyro_before_launch()
                if ENABLE_IMU and imu_runtime is not None:
                    yaw_ref_deg = imu_runtime.read_yaw()
                else:
                    yaw_ref_deg = 0.0
                refresh_field_reference()
                car_started = True
                auto_start_done = True
                start_time = utime.ticks_ms()
                nav_set_state(NAV_STATE_SEARCH_TURN, "launch_search_turn", force=True)
                log("[C9] launched, vision loop on")
    last_c9_state = current_c9


# ====================== C8 退出按键检查 ======================
def check_c8_exit():
    """处理退出按键，带消抖。"""
    global last_c8_state
    current_c8 = key_exit.value()
    if current_c8 == 0 and last_c8_state == 1:
        utime.sleep_ms(10)
        if key_exit.value() == 0:
            log("[C8] exit requested")
            raise KeyboardInterrupt
    last_c8_state = current_c8

# ====================== 通用辅助函数 ======================
# 日志输出
def log(msg):
    print(msg)
    if debug_wireless is not None:
        try:
            debug_wireless.send_str(msg)
            debug_wireless.send_str("\r\n")
        except Exception:
            pass


# 停止所有电机
def stop_all():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)
    log("[STOP] all motor duty cleared")

# 占空比限幅
def clamp_duty(value):
    value = int(value)
    if value > MOTOR_DUTY_MAX:
        return MOTOR_DUTY_MAX
    if value < -MOTOR_DUTY_MAX:
        return -MOTOR_DUTY_MAX
    return value

# PWM 平滑处理
def smooth_value(target, last):
    delta = target - last
    if abs(delta) > MAX_PWM_CHANGE:
        target = last + MAX_PWM_CHANGE * (1 if delta > 0 else -1)
    target = int(last * (1.0 - PWM_SMOOTH_FACTOR) + target * PWM_SMOOTH_FACTOR)
    return clamp_duty(target)

# 应用电机占空比（含最小占空比处理）
def apply_motor_duty(cmd, motor):
    cmd = int(cmd)
    if 0 < abs(cmd) < MOTOR_DUTY_MIN:
        cmd = MOTOR_DUTY_MIN if cmd > 0 else -MOTOR_DUTY_MIN
    motor.duty(cmd)

# 三路电机 PWM 平滑设置
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

# ---------------------- CH7 exit calibration ----------------------
ch7_init_value = 0.0
log("Wireless debug log enabled; master motion broadcast disabled")

motion_seq = 0
motion_last_tx_ms = 0
master_actual_vx = 0.0
master_actual_vy = 0.0
master_actual_wz = 0.0


def motion_next_seq():
    global motion_seq
    motion_seq = (motion_seq + 1) & 0xFF
    if motion_seq == 0:
        motion_seq = 1
    return motion_seq


def put_u8(buf, idx, value):
    value = int(value) & 0xFF
    if value >= 128:
        value -= 256
    buf[idx] = value


def put_i16(buf, idx, value):
    value = int(value)
    if value > 32767:
        value = 32767
    elif value < -32768:
        value = -32768
    if value < 0:
        value += 65536
    put_u8(buf, idx, value)
    put_u8(buf, idx + 1, value >> 8)


def send_motion_frame(seq, vx, vy, wz, yaw_deg, flags):
    put_u8(motion_tx_buf, 0, H1)
    put_u8(motion_tx_buf, 1, H2)
    put_u8(motion_tx_buf, 2, 11)
    put_u8(motion_tx_buf, 3, MSG_MASTER_MOTION)
    put_u8(motion_tx_buf, 4, seq)
    put_i16(motion_tx_buf, 5, vx * 10)
    put_i16(motion_tx_buf, 7, vy * 10)
    put_i16(motion_tx_buf, 9, wz * 10)
    put_i16(motion_tx_buf, 11, yaw_deg * 10)
    put_u8(motion_tx_buf, 13, flags)
    checksum = 0
    i = 2
    while i < 14:
        checksum = (checksum + (motion_tx_buf[i] & 0xFF)) & 0xFF
        i += 1
    put_u8(motion_tx_buf, 14, checksum)
    try:
        wireless.send_bytearray(motion_tx_buf, MASTER_MOTION_FRAME_LEN)
        return True
    except Exception:
        return False


def send_master_motion(now):
    global motion_last_tx_ms

    if wireless is None:
        return
    if utime.ticks_diff(now, motion_last_tx_ms) < MASTER_MOTION_TX_PERIOD_MS:
        return
    flags = 0
    vx = 0.0
    vy = 0.0
    wz = 0.0
    yaw_deg = 0.0
    if car_started:
        flags |= MASTER_MOTION_FLAG_STARTED | MASTER_MOTION_FLAG_CLOSED_LOOP
        vx = master_actual_vx
        vy = master_actual_vy
        wz = master_actual_wz
    if cam_target_seen():
        flags |= MASTER_MOTION_FLAG_TARGET
    if ENABLE_IMU and imu_runtime is not None:
        yaw_deg = imu_runtime.read_yaw()
    if send_motion_frame(motion_next_seq(), vx, vy, wz, yaw_deg, flags):
        motion_last_tx_ms = now

# ====================== 初始化 LED 显示 ======================
update_nav_led_display()
log("[INIT] boot, vision loop waiting")
log("[INFO] C9=start C8=exit")

# 陀螺仪偏移设置
if ENABLE_IMU:
    log("IMU offset preset %.2f" % GYRO_OFFSET_Z)
else:
    log("IMU off, speed loop only")

# ---------------------- Ticker ----------------------
pit_flag = False
pit_count = 0

# 定时中断回调：置位标志，通知主循环执行控制
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
motion_feedback = MoveBase()

pid_fl = SpeedPID()
pid_fr = SpeedPID()
pid_b = SpeedPID()
pid_fl.init_c()
pid_fr.init_c()
pid_b.init_c()

if ENABLE_GYRO_LOOP:
    gyro_pid = AnglePID()
    gyro_pid.output = 0.0
    gyro_pid.err = 0.0
    gyro_pid.err_last = 0.0
    gyro_pid.gyro_kp = GYRO_KP
    gyro_pid.gyro_ki = GYRO_KI
    gyro_pid.gyro_output_limit = GYRO_OUTPUT_LIMIT

turn_pid = AnglePID()
turn_pid.output = 0.0
turn_pid.err = 0.0
turn_pid.err_last = 0.0

last_pwm_fl = 0
last_pwm_fr = 0
last_pwm_b = 0

start_time = utime.ticks_ms()
last_status_ms = start_time
loop_count = 0
last_vz_cmd = 0.0
last_turn_rate_cmd = 0.0
yaw_ref_deg = 0.0
debug_log_last_ms = start_time


def update_master_actual_motion(e_fl, e_fr, e_b, gyro_z):
    global master_actual_vx, master_actual_vy, master_actual_wz

    get_car_spd(motion_feedback, e_fr, e_fl, e_b)
    master_actual_vx = motion_feedback.speed_x
    master_actual_vy = motion_feedback.speed_y
    master_actual_wz = gyro_z


def clear_master_actual_motion():
    global master_actual_vx, master_actual_vy, master_actual_wz

    master_actual_vx = 0.0
    master_actual_vy = 0.0
    master_actual_wz = 0.0

# ====================== 速度闭环主函数（含发车判断） ======================
def calc_speed_closed_loop():
    global last_vz_cmd, last_turn_rate_cmd
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    global debug_log_last_ms
    global cam_target_vx, cam_target_vy

    # 未发车：直接输出 0，占空比清零
    if not car_started:
        set_three_pwm_smooth(0, 0, 0)
        clear_master_actual_motion()
        return None

    # 已发车：执行闭环逻辑
    # 读取陀螺仪数据
    if ENABLE_IMU:
        gyro_z = imu_runtime.read_gyro_z()
        raw_gyro_z = imu_runtime.raw_gyro_z
        yaw_deg = imu_runtime.read_yaw()
    else:
        gyro_z = 0.0
        raw_gyro_z = 0.0
        yaw_deg = 0.0
    e_fl = enc_fl.get()
    e_fr = enc_fr.get()
    e_b = enc_b.get()
    low_speed = abs(e_fl) <= Nav_Low_Speed_Th and abs(e_fr) <= Nav_Low_Speed_Th and abs(e_b) <= Nav_Low_Speed_Th
    update_nav_state_and_targets(yaw_deg, low_speed, gyro_z)

    if nav_state in (NAV_STATE_SEARCH, NAV_STATE_PUSH_CLASSIFY):
        move_cmd.tar_spd_x = 0.0
        move_cmd.tar_spd_y = 0.0
        move_cmd.tar_spd_z = 0.0
        move_cmd.speed_fl = 0.0
        move_cmd.speed_fr = 0.0
        move_cmd.speed_b = 0.0
        last_turn_rate_cmd = 0.0
        last_vz_cmd = 0.0
        turn_pid.output = 0.0
        turn_pid.err = 0.0
        turn_pid.err_last = 0.0
        pid_fl.output = 0.0
        pid_fl.err = 0.0
        pid_fl.err_last = 0.0
        pid_fl.tar_spd_last = 0.0
        pid_fl.delta_tar = 0.0
        pid_fl.delta_tar_last = 0.0
        pid_fl.delta_ud = 0.0
        pid_fr.output = 0.0
        pid_fr.err = 0.0
        pid_fr.err_last = 0.0
        pid_fr.tar_spd_last = 0.0
        pid_fr.delta_tar = 0.0
        pid_fr.delta_tar_last = 0.0
        pid_fr.delta_ud = 0.0
        pid_b.output = 0.0
        pid_b.err = 0.0
        pid_b.err_last = 0.0
        pid_b.tar_spd_last = 0.0
        pid_b.delta_tar = 0.0
        pid_b.delta_tar_last = 0.0
        pid_b.delta_ud = 0.0
        if gyro_pid is not None:
            gyro_pid.output = 0.0
            gyro_pid.err = 0.0
            gyro_pid.err_last = 0.0
            gyro_pid.gyro_output_limit = GYRO_OUTPUT_LIMIT
        last_pwm_fl = 0
        last_pwm_fr = 0
        last_pwm_b = 0
        motor_fl.duty(0)
        motor_fr.duty(0)
        motor_b.duty(0)
        update_master_actual_motion(e_fl, e_fr, e_b, gyro_z)
        return None

    yaw_err_deg = -wrapped_yaw_error(yaw_ref_deg, yaw_deg) if ENABLE_IMU else 0.0
    if nav_state == NAV_STATE_SEARCH_TURN:
        if yaw_err_deg > Nav_Search_Turn_Ok_Yaw:
            turn_rate_cmd = Nav_Search_Turn_Open_Vz
            vz_cmd = Nav_Search_Turn_Open_Vz
        elif yaw_err_deg < -Nav_Search_Turn_Ok_Yaw:
            turn_rate_cmd = -Nav_Search_Turn_Open_Vz
            vz_cmd = -Nav_Search_Turn_Open_Vz
        else:
            turn_rate_cmd = 0.0
            vz_cmd = 0.0
        move_cmd.tar_spd_x = cam_target_vx
        move_cmd.tar_spd_y = cam_target_vy
        move_cmd.tar_spd_z = vz_cmd
        last_turn_rate_cmd = turn_rate_cmd
        last_vz_cmd = vz_cmd
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
            set_three_pwm_smooth(0, 0, 0)
            s_fl = 0
            s_fr = 0
            s_b = 0
        else:
            s_fl, s_fr, s_b = set_three_pwm_smooth(u_fl, u_fr, u_b)
        update_master_actual_motion(e_fl, e_fr, e_b, gyro_z)
        now_log = utime.ticks_ms()
        if DEBUG_LOG_ENABLE and utime.ticks_diff(now_log, debug_log_last_ms) >= DEBUG_LOG_PERIOD_MS:
            debug_log_last_ms = now_log
            log(
                "D0 st=%d cam=%d,%d age=%d wz=%d gz=%d vz=%d"
                % (
                    nav_state_code(nav_state),
                    cam_error_x,
                    cam_error_y,
                    utime.ticks_diff(now_log, cam_last_rx_ms),
                    int(turn_rate_cmd),
                    int(gyro_z),
                    int(vz_cmd),
                )
            )
            log("D1 enc=%d,%d,%d tgt=%d,%d,%d" % (e_fl, e_fr, e_b, int(t_fl), int(t_fr), int(t_b)))
            log(
                "D2 pwm=%d,%d,%d out=%d,%d,%d"
                % (s_fl, s_fr, s_b, int(u_fl), int(u_fr), int(u_b))
            )
        return None

    gyro_rate_mode = False
    if nav_state == NAV_STATE_PUSH_ORIENT:
        orbit_remaining = max(0.0, push_orbit_target_delta - push_orbit_progress_deg)
        if push_orbit_reached:
            orbit_vy_mag = 0.0
            orbit_turn_mag = 0.0
        else:
            orbit_vy_mag, orbit_turn_mag = get_push_orbit_motion(orbit_remaining)
        cam_target_vx = 0.0
        cam_target_vy = push_orbit_vy_sign * orbit_vy_mag
        if orbit_turn_mag > 0.0 and push_orbit_dir != 0:
            turn_rate_cmd = push_orbit_dir * orbit_turn_mag
        else:
            turn_rate_cmd = 0.0
        gyro_rate_mode = True
    elif nav_state == NAV_STATE_PUSH_TURN:
        push_turn_rate_mag = get_push_turn_rate(abs(yaw_err_deg))
        if push_turn_rate_mag > 0.0:
            if yaw_err_deg > 0.0:
                turn_rate_cmd = push_turn_rate_mag
            else:
                turn_rate_cmd = -push_turn_rate_mag
        else:
            turn_rate_cmd = 0.0
        gyro_rate_mode = True

    if gyro_rate_mode:
        turn_pid.output = 0.0
        turn_pid.err = 0.0
        turn_pid.err_last = 0.0
    elif ENABLE_IMU:
        turn_rate_cmd = turn_ctrl(turn_pid, yaw_err_deg, 0)
        if nav_state == NAV_STATE_SEARCH_TURN:
            if turn_rate_cmd > Nav_Search_Turn_Max_Rate:
                turn_rate_cmd = Nav_Search_Turn_Max_Rate
            elif turn_rate_cmd < -Nav_Search_Turn_Max_Rate:
                turn_rate_cmd = -Nav_Search_Turn_Max_Rate
    else:
        turn_pid.output = 0.0
        turn_pid.err = 0.0
        turn_pid.err_last = 0.0
        turn_rate_cmd = 0.0

    # 陀螺仪内环（方向控制）
    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        if nav_state == NAV_STATE_SEARCH_TURN:
            gyro_pid.gyro_output_limit = Nav_Search_Turn_Gyro_Limit
        elif nav_state == NAV_STATE_PUSH_ORIENT:
            gyro_pid.gyro_output_limit = Nav_Push_Orbit_Gyro_Limit
        elif nav_state == NAV_STATE_PUSH:
            gyro_pid.gyro_output_limit = Nav_Push_Execute_Gyro_Limit
        elif nav_state == NAV_STATE_PUSH_TURN:
            gyro_pid.gyro_output_limit = Nav_Push_Turn_Gyro_Limit
        else:
            gyro_pid.gyro_output_limit = GYRO_OUTPUT_LIMIT
        vz_cmd = gyro_ctrl(gyro_pid, turn_rate_cmd - gyro_z)
    else:
        if gyro_pid is not None:
            gyro_pid.output = 0.0
            gyro_pid.err = 0.0
            gyro_pid.err_last = 0.0
        vz_cmd = turn_rate_cmd
    move_cmd.tar_spd_x = cam_target_vx
    move_cmd.tar_spd_y = cam_target_vy
    move_cmd.tar_spd_z = vz_cmd
    last_turn_rate_cmd = turn_rate_cmd
    last_vz_cmd = vz_cmd

    # 车体运动学分解：输出三个轮子的目标转速
    calc_wheel_spd(move_cmd, cam_target_vx, cam_target_vy, vz_cmd)

    # 读取当前编码器速度
    e_fl = enc_fl.get()
    e_fr = enc_fr.get()
    e_b = enc_b.get()

    # 目标速度
    t_fl = move_cmd.speed_fl
    t_fr = move_cmd.speed_fr
    t_b = move_cmd.speed_b

    # 速度 PID 输出
    u_fl = speed_ctrl(pid_fl, e_fl, t_fl)
    u_fr = speed_ctrl(pid_fr, e_fr, t_fr)
    u_b = speed_ctrl(pid_b, e_b, t_b)

    # PWM 平滑输出
    if FORCE_MOTOR_OFF:
        set_three_pwm_smooth(0, 0, 0)
        s_fl = 0
        s_fr = 0
        s_b = 0
    else:
        s_fl, s_fr, s_b = set_three_pwm_smooth(u_fl, u_fr, u_b)
    update_master_actual_motion(e_fl, e_fr, e_b, gyro_z)
    now_log = utime.ticks_ms()
    if DEBUG_LOG_ENABLE and utime.ticks_diff(now_log, debug_log_last_ms) >= DEBUG_LOG_PERIOD_MS:
        debug_log_last_ms = now_log
        log(
            "D0 st=%d cam=%d,%d age=%d wz=%d gz=%d vz=%d"
            % (
                nav_state_code(nav_state),
                cam_error_x,
                cam_error_y,
                utime.ticks_diff(now_log, cam_last_rx_ms),
                int(turn_rate_cmd),
                int(gyro_z),
                int(vz_cmd),
            )
        )
        log("D1 enc=%d,%d,%d tgt=%d,%d,%d" % (e_fl, e_fr, e_b, int(t_fl), int(t_fr), int(t_b)))
        log(
            "D2 pwm=%d,%d,%d out=%d,%d,%d"
            % (s_fl, s_fr, s_b, int(u_fl), int(u_fr), int(u_b))
        )

    return None

log("speed loop start")
if FORCE_MOTOR_OFF:
    log("FORCE_MOTOR_OFF=1")

try:
    while True:
        loop_count += 1
        now = utime.ticks_ms()

        # ====================== 按键检查（主循环最前面） ======================
        check_c8_exit()
        check_c9_start()
        poll_art_uart()
        update_nav_led_display()
        vision_ready = (
            (not car_started)
            and cam_rx_started
            and cam_packet_fresh()
        )
        if vision_ready:
            calibrate_gyro_before_launch()
            if ENABLE_IMU and imu_runtime is not None:
                yaw_ref_deg = imu_runtime.read_yaw()
            else:
                yaw_ref_deg = 0.0
            refresh_field_reference()
            car_started = True
            auto_start_done = True
            start_time = now
            nav_set_state(NAV_STATE_SEARCH_TURN, "vision_launch_search_turn", force=True)
            log("[VISION] first valid target, car_started=1")

        if pit_flag:
            pit_flag = False
            calc_speed_closed_loop()
        send_master_motion(now)

        if utime.ticks_diff(now, last_status_ms) >= 1000:
            led.toggle()
            last_status_ms = now

        if loop_count % GC_DIV == 0:
            gc.collect()

        utime.sleep_ms(1)

finally:
    pit1.stop()
    stop_all()
    led.value(True)
    # 退出时熄灭所有 LED
    led_straight.value(0)
    led_translate.value(0)
    led_rotate.value(0)
    log("=== program stopped ===")

