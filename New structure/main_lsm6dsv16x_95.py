from machine import Pin, UART
import gc
import utime
from smartcar import ticker, encoder
from lsm6dsv16x_gyro_runtime import LSM6DSV16XYawRuntime
from models import AnglePID, MoveBase, SpeedPID
from move_base import calc_wheel_spd
import pid as _pid_mod
import config as cfg
from hardware import Motor
import coop_master

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

GYRO_SIGN = 1.0

# 陀螺仪 Z 轴标定参数
GYRO_OFFSET_Z = 0.0
GYRO_SCALE = -1.0
GYRO_DEADBAND_DPS = 0.8  # 陀螺仪死区阈值
GYRO_KP = 0.22
GYRO_KI = 0.004
GYRO_OUTPUT_LIMIT = 5.0
Nav_Track_Gyro_Limit = 6.0
AUTO_CALIBRATE_GYRO_ON_LAUNCH = True   #是否在启动时自动进行陀螺仪标定
GYRO_CALIBRATE_SAMPLES = 1000
GYRO_CALIBRATE_DELAY_MS = 2


# 退出与回收配置
GC_DIV = 50

Cam_Error_Offset = 120
Cam_Error_Scale = 2
Cam_Packet_Timeout_Ms = 200
Cam_Frame_Head = 0xFF
Line_Packet_Tag = 0xFC
No_Target_Marker = 0xFE
Classify_Packet_Tag = 0xFD
NAV_STATE_SEARCH = "SEARCH"
NAV_STATE_SEARCH_TURN = "SEARCH_TURN_45"
NAV_STATE_SEARCH_SPIN = "SEARCH_SPIN_360"
NAV_STATE_COARSE = "COARSE_APPROACH"
NAV_STATE_FINE = "FINE_ALIGN"
NAV_STATE_PUSH_CLASSIFY = "PUSH_CLASSIFY"
NAV_STATE_PUSH_ORIENT = "PUSH_ORBIT"
NAV_STATE_PUSH_PREPARE = "PUSH_PREPARE"
NAV_STATE_PUSH = "PUSH_EXECUTE"
NAV_STATE_PUSH_BACK = "PUSH_FINISH_BACK"
NAV_STATE_PUSH_TURN = "PUSH_FINISH_TURN"
NAV_STATE_POST_TURN_FORWARD = "POST_TURN_FORWARD"
NAV_STATE_RETURN_LEFT = "RETURN_LEFT"
NAV_STATE_RETURN_BACK = "RETURN_BACK"
NAV_STATE_RETURN_TURN = "RETURN_TURN"
NAV_STATE_RETURN_FINAL = "RETURN_FINAL"
NAV_STATE_RETURN_DONE = "RETURN_DONE"
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
Nav_Search_Forward_Ms = 1000
Nav_Coarse_Exit_Y = 190
Nav_Coarse_Ok_Ms = 80
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
Nav_Coarse_Forward_Gain = 0.090
Nav_Coarse_Lateral_Gain = 0.0425		#COARSE 横移系数
Nav_Coarse_Forward_Limit = 11.0
Nav_Coarse_Lateral_Limit = 6.5
Nav_Fine_Forward_Gain = 0.0475
Nav_Fine_Classify_Lateral_Gain = 0.0775
Nav_Fine_Lateral_Gain = 0.125        #FINE 横移系数
Nav_Fine_Push_Lateral_Gain = 0.080
Nav_Fine_Forward_Limit = 7.0
Nav_Fine_Lateral_Limit = 7.25
Nav_Fine_Push_Lateral_Limit = 4.25
Nav_Fine_Forward_Brake_Enable = True
Nav_Fine_Forward_Brake_Y = -8
Nav_Fine_Forward_Brake_Ms = 110
Nav_Fine_Forward_Brake_Speed = 2.25
Nav_Fine_Lateral_Brake_Enable = True
Nav_Fine_Lateral_Brake_Window_X = 100    #当摄像头误差在这个范围内且车速较高时启用横移制动
Nav_Fine_Lateral_Brake_Ms = 150
Nav_Fine_Lateral_Brake_Speed = 4.0
Nav_Fine_Push_Lateral_Brake_Ms = 70
Nav_Fine_Push_Lateral_Brake_Speed = 1.5
Nav_Forward_Deadband = 4
Nav_Lateral_Deadband = 3    #横移死区，单位像素；如果摄像头误差在这个范围内则认为不需要横移
Nav_Classify_Timeout_Ms = 1500

Nav_Push_Orient_Ok_Yaw = 5.0    #orbit 目标角度误差小于该值即认为定向完成
Nav_Push_Orbit_Skip_Yaw = 15.0
Nav_Push_Orbit_Opposite_Yaw = 165.0
Nav_Push_Orient_Max_Ms = 15000
Nav_Push_Orbit_Slow_Yaw = 40.0
Nav_Push_Orbit_Fast_Vy = 4.2
Nav_Push_Orbit_Slow_Vy = 3.4
Nav_Push_Orbit_Fast_Rate = 62.5
Nav_Push_Orbit_Gyro_Limit = 10.0
Nav_Push_Orbit_Radius_Base = 2.0   #orbit 基础半径系数，实际轨迹半径=该系数 * 车轮轴距；如果轨迹过大或过小可以调整该值
Nav_Push_Orbit_Radius_Gain = 0.010
Nav_Push_Orbit_Stop_Gyro_Th = 3.0   #orbit 过程中如果陀螺仪读数小于该值则认为已经接近目标角度，可以停止转向加速前进
Nav_Push_Orbit_Brake_Max_Ms = 1000
Nav_Push_Prepare_Reorient_Yaw = 10.0
Nav_Push_Prepare_Hard_Reorient_Yaw = 24.0
Nav_Push_Prepare_Soft_Scale = 0.45
Nav_Push_Prepare_Forward_Gain = 0.019
Nav_Push_Prepare_Lateral_Gain = 0.0575
Nav_Push_Prepare_Forward_Limit = 2.9
Nav_Push_Prepare_Lateral_Limit = 4.0
Nav_Push_Prepare_Min_Vx = 1.2
Nav_Push_Prepare_Min_Vy = 2.4
Nav_Push_Prepare_Kick_X = 18
Nav_Push_Prepare_Kick_Vy = 3.75
Nav_Push_Prepare_Lateral_Brake_Ms = 70
Nav_Push_Prepare_Lateral_Brake_Speed = 2.5
Nav_Push_Prepare_Back_Ms = 90
Nav_Push_Prepare_Back_Speed = 3.1
Nav_Push_Prepare_Ok_X = 4    #准备阶段前进误差小于该值即认为横移准备就绪
Nav_Push_Prepare_Ok_Y_Min = -8
Nav_Push_Prepare_Ok_Y_Max = 8  #准备阶段横移误差小于该值即认为前进准备就绪
Nav_Push_Prepare_Ok_Yaw = 6   #准备阶段定向误差小于该值即认为定向准备就绪
Nav_Push_Prepare_Ok_Ms = 70
Nav_Ball_Push_Yaw_Offset = 30.0
Nav_Ball_Field_Vx_Scale = 0.8660254
Nav_Ball_Field_Vy_Scale = 0.5
Nav_Push_Execute_Forward_Speed = 10.0    #执行阶段前进速度
Nav_Push_Execute_Gyro_Limit = 16.0
Nav_Push_Line_Lost_Ms = 150
Nav_Push_Line_Extra_Ms = 100
Nav_Push_Back_Speed = 10.0
Nav_Push_Back_Ms = 3500
Nav_Push_Turn_Slow_Yaw = 95.0
Nav_Push_Turn_Fast_Rate = 60.0
Nav_Push_Turn_Slow_Rate = 26.0
Nav_Push_Turn_Gyro_Limit = 10.0
Nav_Push_Turn_Gyro_Kp = 0.17
Nav_Push_Turn_Gyro_Ki = 0.002
Nav_Push_Turn_Ok_Yaw = 6.0
Nav_Push_Turn_Recover_Yaw = 12.0
Nav_Push_Turn_Ok_Ms = 150
Nav_Push_Turn_Forced_Dir = 1
Nav_Push_Turn_Force_Window_Yaw = 170.0  # 接近180度时保留指定首转方向
Nav_Push_Turn_Correct_Rate = 12.0       # 越过目标后仅做低速最短路修正
Nav_Push_Turn_Max_Ms = 6000             # 推后回转超时直接停车
Nav_Post_Turn_No_Target_Ms = 100
Nav_Post_Turn_Forward_Ms = 1500
Nav_Post_Turn_Forward_Speed = 6.5
Nav_Object_Total = 2
Nav_Return_Left_Speed = 11.0
Nav_Return_Left_Start_Yaw = 10.0
Nav_Return_Left_Max_Ms = 10000
Nav_Return_Back_Speed = 8.0
Nav_Return_Back_Ms = 1300
Nav_Return_Turn_Dir = 1
Nav_Return_Final_Back_Speed = 7.0
Nav_Return_Final_Line_Extra_Ms = 60
Nav_Return_Max_Ms = 6000

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
push_turn_settle = False
push_turn_reached_once = False
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
pushed_object_count = 0
final_return_mode = 0  # 0=normal 1=ball 2=bag

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

    launch_yaw = normalize_yaw_deg(imu_runtime.read_yaw())

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
    return -wrapped_yaw_error(push_yaw_target, yaw_deg)


def reset_gyro_pid_state():
    pid = gyro_pid
    if pid is not None:
        pid.output = 0.0
        pid.err = 0.0
        pid.err_last = 0.0


def reset_speed_pid_state():
    global last_pwm_fl, last_pwm_fr, last_pwm_b

    move_cmd.tar_spd_x = 0.0
    move_cmd.tar_spd_y = 0.0
    move_cmd.tar_spd_z = 0.0
    move_cmd.speed_fl = 0.0
    move_cmd.speed_fr = 0.0
    move_cmd.speed_b = 0.0

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

    last_pwm_fl = 0
    last_pwm_fr = 0
    last_pwm_b = 0


def get_push_orbit_motion(yaw_err_abs):
    if yaw_err_abs <= Nav_Push_Orient_Ok_Yaw:
        return 0.0, 0.0
    if yaw_err_abs > Nav_Push_Orbit_Slow_Yaw:
        vy_base = Nav_Push_Orbit_Fast_Vy
    else:
        span = Nav_Push_Orbit_Slow_Yaw - Nav_Push_Orient_Ok_Yaw
        ratio = (yaw_err_abs - Nav_Push_Orient_Ok_Yaw) / span
        vy_base = Nav_Push_Orbit_Slow_Vy + (Nav_Push_Orbit_Fast_Vy - Nav_Push_Orbit_Slow_Vy) * ratio
    # 横移速度与角速度保持固定比例，使 radius_ratio 只控制公转半径。
    turn_rate_mag = Nav_Push_Orbit_Fast_Rate * vy_base / Nav_Push_Orbit_Fast_Vy
    return vy_base * push_orbit_radius_ratio, turn_rate_mag


def get_push_turn_rate(yaw_err_abs):
    if yaw_err_abs <= Nav_Push_Turn_Ok_Yaw:
        return 0.0
    if yaw_err_abs > Nav_Push_Turn_Slow_Yaw:
        return Nav_Push_Turn_Fast_Rate
    span = Nav_Push_Turn_Slow_Yaw - Nav_Push_Turn_Ok_Yaw
    ratio = (yaw_err_abs - Nav_Push_Turn_Ok_Yaw) / span
    return Nav_Push_Turn_Slow_Rate + (Nav_Push_Turn_Fast_Rate - Nav_Push_Turn_Slow_Rate) * ratio


def get_turn_rate_command(target_yaw, now_yaw, preferred_dir, correction_mode):
    yaw_err = -wrapped_yaw_error(target_yaw, now_yaw)
    yaw_err_abs = abs(yaw_err)
    if yaw_err_abs <= Nav_Push_Turn_Ok_Yaw:
        return 0.0

    # 180度附近的最短路符号容易受微小误差影响，首次转向仍使用指定方向。
    if (not correction_mode) and yaw_err_abs >= Nav_Push_Turn_Force_Window_Yaw:
        turn_dir = 1 if preferred_dir > 0 else -1
    elif yaw_err > 0.0:
        turn_dir = 1
    else:
        turn_dir = -1

    turn_rate_mag = get_push_turn_rate(yaw_err_abs)
    # 已经过目标后只按最短路低速回修，避免再次执行接近一整圈的转动。
    if correction_mode and turn_rate_mag > Nav_Push_Turn_Correct_Rate:
        turn_rate_mag = Nav_Push_Turn_Correct_Rate
    return turn_dir * turn_rate_mag


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
    if new_state in (NAV_STATE_SEARCH, NAV_STATE_SEARCH_TURN, NAV_STATE_SEARCH_SPIN, NAV_STATE_POST_TURN_FORWARD):
        cam_uart.write(ART_MODE_SEARCH_CMD)
    elif new_state in (NAV_STATE_RETURN_LEFT, NAV_STATE_RETURN_FINAL):
        cam_uart.write(ART_MODE_LINE_CMD)
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
    elif new_state == NAV_STATE_PUSH_BACK:
        if final_return_mode == 2:
            cam_uart.write(ART_MODE_LINE_CMD)
        else:
            cam_uart.write(ART_MODE_IDLE_CMD)
    elif new_state in (NAV_STATE_PUSH_TURN, NAV_STATE_RETURN_BACK, NAV_STATE_RETURN_TURN, NAV_STATE_RETURN_DONE):
        cam_uart.write(ART_MODE_IDLE_CMD)


def nav_state_code(state):
    if state == NAV_STATE_SEARCH:
        return 0
    if state == NAV_STATE_SEARCH_TURN:
        return 1
    if state == NAV_STATE_SEARCH_SPIN:
        return 16
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
    if state == NAV_STATE_RETURN_LEFT:
        return 11
    if state == NAV_STATE_RETURN_BACK:
        return 12
    if state == NAV_STATE_RETURN_TURN:
        return 13
    if state == NAV_STATE_RETURN_FINAL:
        return 14
    if state == NAV_STATE_RETURN_DONE:
        return 15
    return -1


def nav_set_state(new_state, reason="", force=False):
    global nav_state, nav_detect_since_ms, nav_target_lost_since_ms
    global nav_coarse_ok_since_ms, nav_fine_ok_since_ms, nav_transition_ms
    global nav_fine_last_x_sign, nav_fine_last_y_sign
    global nav_fine_brake_since_ms, nav_fine_brake_vy, nav_fine_forward_brake_since_ms
    global nav_fine_forward_brake_armed
    global nav_push_prepare_ok_since_ms, nav_push_prepare_back_since_ms, nav_push_turn_ok_since_ms
    global push_turn_settle, push_turn_reached_once
    global nav_ready_for_push, cam_target_vx, cam_target_vy
    global yaw_ref_deg, cam_rx_started, push_dir_code, push_dir_name
    global line_crossed, push_line_seen_once, push_line_lost_since_ms, push_line_extra_since_ms
    global push_return_yaw_target, search_turn_yaw_target
    global push_orbit_dir, push_orbit_vy_sign, push_orbit_done
    global push_orbit_reached, push_orbit_progress_deg, push_orbit_last_ms
    global push_orbit_brake_since_ms
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    global pushed_object_count, final_return_mode

    if nav_state == new_state and (not force):
        return

    now = utime.ticks_ms()
    nav_state = new_state
    nav_detect_since_ms = 0
    nav_target_lost_since_ms = 0
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
    push_turn_settle = False
    push_turn_reached_once = False
    nav_transition_ms = now
    nav_ready_for_push = False
    cam_target_vx = 0.0
    cam_target_vy = 0.0

    if new_state in (
        NAV_STATE_SEARCH,
        NAV_STATE_SEARCH_TURN,
        NAV_STATE_SEARCH_SPIN,
        NAV_STATE_POST_TURN_FORWARD,
        NAV_STATE_COARSE,
        NAV_STATE_FINE,
        NAV_STATE_PUSH_CLASSIFY,
        NAV_STATE_PUSH_PREPARE,
        NAV_STATE_PUSH,
        NAV_STATE_RETURN_LEFT,
        NAV_STATE_RETURN_FINAL,
    ):
        clear_cam_target_state()

    if new_state in (
        NAV_STATE_SEARCH,
        NAV_STATE_SEARCH_TURN,
        NAV_STATE_SEARCH_SPIN,
        NAV_STATE_POST_TURN_FORWARD,
        NAV_STATE_PUSH,
        NAV_STATE_PUSH_CLASSIFY,
        NAV_STATE_RETURN_LEFT,
        NAV_STATE_RETURN_FINAL,
    ):
        line_crossed = False
        push_line_seen_once = False
        push_line_lost_since_ms = 0
        push_line_extra_since_ms = 0

    if new_state in (NAV_STATE_SEARCH, NAV_STATE_SEARCH_TURN, NAV_STATE_SEARCH_SPIN, NAV_STATE_POST_TURN_FORWARD, NAV_STATE_COARSE, NAV_STATE_PUSH_CLASSIFY):
        push_orbit_done = False

    if new_state == NAV_STATE_SEARCH_SPIN:
        push_orbit_reached = False
        push_orbit_progress_deg = 0.0
        push_orbit_last_ms = now

    if new_state == NAV_STATE_PUSH_CLASSIFY:
        push_dir_code = Push_Dir_None
        push_dir_name = "NONE"
        push_orbit_dir = 0
        push_orbit_vy_sign = 0
        push_orbit_reached = False
        push_orbit_progress_deg = 0.0
        push_orbit_last_ms = 0
        push_orbit_brake_since_ms = 0
        final_return_mode = 0
    elif new_state == NAV_STATE_PUSH_BACK:
        if final_return_mode == 1:
            push_return_yaw_target = field_left_yaw
        elif push_dir_code == Push_Dir_Up:
            push_return_yaw_target = field_down_yaw
        else:
            push_return_yaw_target = normalize_yaw_deg(push_yaw_target + 180.0)
        if final_return_mode == 2:
            line_crossed = False

    if new_state == NAV_STATE_FINE:
        if cam_error_x > Nav_Lateral_Deadband:
            nav_fine_last_x_sign = 1
        elif cam_error_x < -Nav_Lateral_Deadband:
            nav_fine_last_x_sign = -1
        if cam_error_y > Nav_Forward_Deadband:
            nav_fine_last_y_sign = 1
        elif cam_error_y < -Nav_Forward_Deadband:
            nav_fine_last_y_sign = -1

    if new_state == NAV_STATE_PUSH_BACK:
        pushed_object_count += 1

    if new_state in (
        NAV_STATE_PUSH_PREPARE,
        NAV_STATE_PUSH,
        NAV_STATE_PUSH_TURN,
        NAV_STATE_SEARCH_SPIN,
        NAV_STATE_POST_TURN_FORWARD,
        NAV_STATE_RETURN_LEFT,
        NAV_STATE_RETURN_BACK,
        NAV_STATE_RETURN_TURN,
        NAV_STATE_RETURN_FINAL,
        NAV_STATE_RETURN_DONE,
    ):
        reset_gyro_pid_state()
    if (
        new_state in (
            NAV_STATE_PUSH_CLASSIFY,
            NAV_STATE_PUSH_ORIENT,
            NAV_STATE_PUSH_PREPARE,
            NAV_STATE_PUSH,
            NAV_STATE_PUSH_TURN,
            NAV_STATE_SEARCH_SPIN,
            NAV_STATE_POST_TURN_FORWARD,
            NAV_STATE_RETURN_LEFT,
            NAV_STATE_RETURN_BACK,
            NAV_STATE_RETURN_TURN,
            NAV_STATE_RETURN_FINAL,
            NAV_STATE_RETURN_DONE,
        )
        or (new_state == NAV_STATE_FINE and push_orbit_done)
    ):
        reset_speed_pid_state()
    if new_state in (NAV_STATE_POST_TURN_FORWARD, NAV_STATE_RETURN_DONE):
        motor_fl.duty(0)
        motor_fr.duty(0)
        motor_b.duty(0)

    if new_state == NAV_STATE_SEARCH_TURN:
        search_turn_yaw_target = field_up_yaw
        yaw_ref_deg = search_turn_yaw_target
    elif new_state == NAV_STATE_SEARCH_SPIN:
        yaw_ref_deg = imu_runtime.read_yaw()
    elif new_state == NAV_STATE_FINE:
        if push_orbit_done:
            yaw_ref_deg = push_yaw_target
    elif new_state == NAV_STATE_COARSE:
        yaw_ref_deg = imu_runtime.read_yaw()
    elif new_state == NAV_STATE_PUSH_CLASSIFY:
        pass
    elif new_state in (NAV_STATE_PUSH_ORIENT, NAV_STATE_PUSH_PREPARE, NAV_STATE_PUSH, NAV_STATE_PUSH_BACK):
        yaw_ref_deg = push_yaw_target
    elif new_state == NAV_STATE_PUSH_TURN:
        yaw_ref_deg = push_return_yaw_target
    elif new_state == NAV_STATE_POST_TURN_FORWARD:
        yaw_ref_deg = push_return_yaw_target
    elif new_state in (NAV_STATE_RETURN_LEFT, NAV_STATE_RETURN_BACK):
        yaw_ref_deg = field_left_yaw
    elif new_state in (NAV_STATE_RETURN_TURN, NAV_STATE_RETURN_FINAL, NAV_STATE_RETURN_DONE):
        yaw_ref_deg = field_up_yaw

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


def update_return_home(now, yaw_deg, low_speed, gyro_z):
    global nav_push_turn_ok_since_ms, nav_ready_for_push, push_turn_settle
    global push_turn_reached_once
    global cam_target_vx, cam_target_vy, yaw_ref_deg
    global push_line_seen_once, push_line_lost_since_ms, push_line_extra_since_ms

    if nav_state == NAV_STATE_RETURN_LEFT:
        nav_ready_for_push = False
        yaw_ref_deg = field_left_yaw
        cam_target_vy = 0.0
        if abs(-wrapped_yaw_error(yaw_ref_deg, yaw_deg)) > Nav_Return_Left_Start_Yaw:
            cam_target_vx = 0.0
        else:
            cam_target_vx = Nav_Return_Left_Speed
            if line_crossed:
                nav_set_state(NAV_STATE_RETURN_BACK)
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Return_Left_Max_Ms:
            nav_set_state(NAV_STATE_RETURN_DONE)
        return

    if nav_state == NAV_STATE_RETURN_BACK:
        nav_ready_for_push = False
        yaw_ref_deg = field_left_yaw
        cam_target_vx = -Nav_Return_Back_Speed
        cam_target_vy = 0.0
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Return_Back_Ms:
            nav_set_state(NAV_STATE_RETURN_TURN)
        return

    if nav_state == NAV_STATE_RETURN_TURN:
        nav_ready_for_push = False
        yaw_ref_deg = field_up_yaw
        cam_target_vx = 0.0
        cam_target_vy = 0.0
        yaw_err_abs = abs(-wrapped_yaw_error(yaw_ref_deg, yaw_deg))
        if yaw_err_abs <= Nav_Push_Turn_Ok_Yaw:
            if not push_turn_settle:
                reset_gyro_pid_state()
            push_turn_settle = True
            push_turn_reached_once = True
        elif push_turn_settle and yaw_err_abs > Nav_Push_Turn_Recover_Yaw:
            push_turn_settle = False
            nav_push_turn_ok_since_ms = 0
        if push_turn_settle and low_speed and abs(gyro_z) <= 8.0:
            if nav_push_turn_ok_since_ms == 0:
                nav_push_turn_ok_since_ms = now
            elif utime.ticks_diff(now, nav_push_turn_ok_since_ms) >= Nav_Push_Turn_Ok_Ms:
                imu_runtime.reset_yaw(field_up_yaw)
                nav_set_state(NAV_STATE_RETURN_FINAL)
        else:
            nav_push_turn_ok_since_ms = 0
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Return_Max_Ms:
            nav_set_state(NAV_STATE_RETURN_DONE)
        return

    if nav_state == NAV_STATE_RETURN_FINAL:
        nav_ready_for_push = False
        yaw_ref_deg = field_up_yaw
        cam_target_vx = -Nav_Return_Final_Back_Speed
        cam_target_vy = 0.0
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
                elif utime.ticks_diff(now, push_line_extra_since_ms) >= Nav_Return_Final_Line_Extra_Ms:
                    nav_set_state(NAV_STATE_RETURN_DONE)
        return

    nav_ready_for_push = False
    cam_target_vx = 0.0
    cam_target_vy = 0.0
    move_cmd.tar_spd_x = 0.0
    move_cmd.tar_spd_y = 0.0
    move_cmd.tar_spd_z = 0.0


def update_nav_state_and_targets(yaw_deg, low_speed, gyro_z):
    global nav_detect_since_ms, nav_target_lost_since_ms
    global nav_coarse_ok_since_ms, nav_fine_ok_since_ms
    global nav_fine_last_x_sign, nav_fine_last_y_sign
    global nav_fine_brake_since_ms, nav_fine_brake_vy, nav_fine_forward_brake_since_ms
    global nav_fine_forward_brake_armed
    global nav_push_prepare_ok_since_ms, nav_push_prepare_back_since_ms, nav_push_turn_ok_since_ms
    global nav_ready_for_push
    global push_turn_settle, push_turn_reached_once
    global cam_target_vx, cam_target_vy, yaw_ref_deg, push_yaw_target
    global line_crossed, push_line_seen_once, push_line_lost_since_ms, push_line_extra_since_ms
    global push_return_yaw_target, push_face_obj_yaw
    global push_orbit_dir, push_orbit_vy_sign, push_orbit_blocked, push_orbit_done
    global push_orbit_reached, push_orbit_progress_deg, push_orbit_target_delta, push_orbit_last_ms
    global push_orbit_brake_since_ms
    global final_return_mode

    now = utime.ticks_ms()
    if nav_state in (
        NAV_STATE_RETURN_LEFT,
        NAV_STATE_RETURN_BACK,
        NAV_STATE_RETURN_TURN,
        NAV_STATE_RETURN_FINAL,
        NAV_STATE_RETURN_DONE,
    ):
        update_return_home(now, yaw_deg, low_speed, gyro_z)
        return

    track_target_states = (
        NAV_STATE_SEARCH,
        NAV_STATE_SEARCH_TURN,
        NAV_STATE_SEARCH_SPIN,
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
        cam_target_vx = Nav_Coarse_Forward_Limit
        cam_target_vy = 0.0
        if seen:
            yaw_ref_deg = yaw_deg
            nav_set_state(NAV_STATE_COARSE, "target_seen_during_search_turn")
            return
        yaw_ref_deg = search_turn_yaw_target
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Search_Forward_Ms:
            nav_set_state(NAV_STATE_SEARCH_SPIN, "search_forward_no_target")
        return

    if nav_state == NAV_STATE_SEARCH_SPIN:
        nav_ready_for_push = False
        cam_target_vx = 0.0
        cam_target_vy = 0.0
        if seen and nav_detect_since_ms > 0:
            if utime.ticks_diff(now, nav_detect_since_ms) >= Nav_Detect_Ms:
                yaw_ref_deg = yaw_deg
                nav_set_state(NAV_STATE_COARSE, "target_seen_during_search_spin")
                return

        spin_dt_ms = utime.ticks_diff(now, push_orbit_last_ms)
        if spin_dt_ms > 0:
            spin_step = Nav_Return_Turn_Dir * gyro_z * spin_dt_ms * 0.001
            if spin_step > 0.0:
                push_orbit_progress_deg += spin_step
            push_orbit_last_ms = now

        if push_orbit_progress_deg >= 360.0 - Nav_Push_Turn_Ok_Yaw:
            nav_set_state(NAV_STATE_SEARCH, "search_spin_complete")
        elif utime.ticks_diff(now, nav_transition_ms) >= Nav_Return_Max_Ms * 2:
            nav_set_state(NAV_STATE_SEARCH, "search_spin_timeout")
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
                yaw_ref_deg = yaw_deg
                nav_set_state(NAV_STATE_COARSE, "target_detected")
        return

    if nav_state == NAV_STATE_POST_TURN_FORWARD:
        nav_ready_for_push = False
        yaw_ref_deg = push_return_yaw_target
        cam_target_vy = 0.0
        post_turn_elapsed = utime.ticks_diff(now, nav_transition_ms)
        if post_turn_elapsed < Nav_Post_Turn_No_Target_Ms:
            nav_detect_since_ms = 0
            cam_target_vx = 0.0
            return
        if seen:
            cam_target_vx = 0.0
            if nav_detect_since_ms > 0:
                if utime.ticks_diff(now, nav_detect_since_ms) >= Nav_Detect_Ms:
                    yaw_ref_deg = yaw_deg
                    nav_set_state(NAV_STATE_COARSE, "post_turn_target_seen")
            return

        if post_turn_elapsed < Nav_Post_Turn_No_Target_Ms + Nav_Post_Turn_Forward_Ms:
            cam_target_vx = Nav_Post_Turn_Forward_Speed
        else:
            cam_target_vx = 0.0
            if pushed_object_count < Nav_Object_Total:
                nav_set_state(NAV_STATE_SEARCH_SPIN, "post_turn_forward_no_target")
            else:
                nav_set_state(NAV_STATE_SEARCH, "post_turn_forward_done")
        return

    if nav_state in (NAV_STATE_COARSE, NAV_STATE_FINE, NAV_STATE_PUSH_PREPARE):
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Transition_Grace_Ms:
            if nav_target_lost_since_ms > 0:
                if utime.ticks_diff(now, nav_target_lost_since_ms) >= Nav_Target_Lost_Ms:
                    if nav_state == NAV_STATE_PUSH_PREPARE:
                        nav_set_state(NAV_STATE_FINE, "%s_target_lost" % nav_state.lower())
                    else:
                        if pushed_object_count < Nav_Object_Total:
                            nav_set_state(NAV_STATE_SEARCH_SPIN, "target_lost_search_spin")
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
        if seen and cam_error_y <= Nav_Coarse_Exit_Y:
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
            fine_lateral_gain = Nav_Fine_Push_Lateral_Gain
            fine_lateral_limit = Nav_Fine_Push_Lateral_Limit
            fine_lateral_brake_ms = Nav_Fine_Push_Lateral_Brake_Ms
            fine_lateral_brake_speed = Nav_Fine_Push_Lateral_Brake_Speed
        else:
            fine_lateral_gain = Nav_Fine_Classify_Lateral_Gain
            fine_lateral_limit = Nav_Fine_Lateral_Limit
            fine_lateral_brake_ms = Nav_Fine_Lateral_Brake_Ms
            fine_lateral_brake_speed = Nav_Fine_Lateral_Brake_Speed
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
                nav_fine_brake_vy = -current_x_sign * fine_lateral_brake_speed
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
                fine_lateral_limit,
            )
        elif (not Nav_Fine_Lateral_Brake_Enable) or nav_fine_brake_since_ms == 0:
            apply_nav_targets(
                cam_error_y * Nav_Fine_Forward_Gain,
                -cam_error_x * fine_lateral_gain,
                Nav_Fine_Forward_Limit,
                fine_lateral_limit,
            )
        elif utime.ticks_diff(now, nav_fine_brake_since_ms) < fine_lateral_brake_ms:
            fine_braking = True
            cam_target_vx = 0.0
            if nav_fine_brake_vy > fine_lateral_limit:
                cam_target_vy = fine_lateral_limit
            elif nav_fine_brake_vy < -fine_lateral_limit:
                cam_target_vy = -fine_lateral_limit
            else:
                cam_target_vy = nav_fine_brake_vy
        else:
            nav_fine_brake_since_ms = 0
            nav_fine_brake_vy = 0.0
            apply_nav_targets(
                cam_error_y * Nav_Fine_Forward_Gain,
                -cam_error_x * fine_lateral_gain,
                Nav_Fine_Forward_Limit,
                fine_lateral_limit,
            )
        fine_yaw_ok = (
            (not push_orbit_done)
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
            final_return_mode = 0
            if pushed_object_count + 1 >= Nav_Object_Total:
                if push_dir_code == Push_Dir_Up:
                    final_return_mode = 1
                elif push_dir_code == Push_Dir_Left:
                    final_return_mode = 2
            push_yaw_target = yaw_from_field_dir(push_dir_code)
            if push_dir_code == Push_Dir_Up:
                push_yaw_target = normalize_yaw_deg(
                    field_up_yaw - Nav_Ball_Push_Yaw_Offset
                )
            orbit_yaw_err = -wrapped_yaw_error(push_yaw_target, push_face_obj_yaw)
            push_orbit_target_delta = abs(orbit_yaw_err)
            if push_orbit_target_delta <= Nav_Push_Orbit_Skip_Yaw:
                push_orbit_dir = 0
                push_orbit_vy_sign = 0
            else:
                push_orbit_dir = -1
                if push_orbit_target_delta < Nav_Push_Orbit_Opposite_Yaw and orbit_yaw_err >= 0.0:
                    push_orbit_dir = 1
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
        yaw_err_abs = abs(-wrapped_yaw_error(yaw_ref_deg, yaw_deg))
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
        if push_orbit_reached:
            brake_elapsed = 0
            if push_orbit_brake_since_ms > 0:
                brake_elapsed = utime.ticks_diff(now, push_orbit_brake_since_ms)
            if abs(gyro_z) <= Nav_Push_Orbit_Stop_Gyro_Th or brake_elapsed >= Nav_Push_Orbit_Brake_Max_Ms:
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
        yaw_prepare_err_abs = abs(-wrapped_yaw_error(yaw_ref_deg, yaw_deg))
        if yaw_prepare_err_abs > Nav_Push_Prepare_Hard_Reorient_Yaw:
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
        prepare_error_x = cam_error_x
        prepare_braking = False
        current_prepare_x_sign = 0
        if prepare_error_x > Nav_Push_Prepare_Ok_X:
            current_prepare_x_sign = 1
        elif prepare_error_x < -Nav_Push_Prepare_Ok_X:
            current_prepare_x_sign = -1
        if current_prepare_x_sign != 0:
            if (
                nav_fine_last_x_sign != 0
                and current_prepare_x_sign != nav_fine_last_x_sign
                and abs(prepare_error_x) <= Nav_Push_Prepare_Kick_X
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
            if -Nav_Push_Prepare_Ok_X <= prepare_error_x <= Nav_Push_Prepare_Ok_X:
                vy_cmd = 0.0
            else:
                vy_cmd = -prepare_error_x * Nav_Push_Prepare_Lateral_Gain
                min_vy = Nav_Push_Prepare_Min_Vy
                if abs(prepare_error_x) <= Nav_Push_Prepare_Kick_X:
                    min_vy = Nav_Push_Prepare_Kick_Vy
                if 0.0 < vy_cmd < min_vy:
                    vy_cmd = min_vy
                elif -min_vy < vy_cmd < 0.0:
                    vy_cmd = -min_vy
            vx_limit = Nav_Push_Prepare_Forward_Limit
            vy_limit = Nav_Push_Prepare_Lateral_Limit
            if yaw_prepare_err_abs > Nav_Push_Prepare_Reorient_Yaw:
                vx_cmd = vx_cmd * Nav_Push_Prepare_Soft_Scale
                vy_cmd = vy_cmd * Nav_Push_Prepare_Soft_Scale
                vx_limit = vx_limit * Nav_Push_Prepare_Soft_Scale
                vy_limit = vy_limit * Nav_Push_Prepare_Soft_Scale
            apply_nav_targets(vx_cmd, vy_cmd, vx_limit, vy_limit)
        if (
            seen
            and abs(prepare_error_x) <= Nav_Push_Prepare_Ok_X
            and cam_error_y >= Nav_Push_Prepare_Ok_Y_Min
            and cam_error_y <= Nav_Push_Prepare_Ok_Y_Max
            and yaw_prepare_err_abs <= Nav_Push_Prepare_Ok_Yaw
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
        if push_dir_code == Push_Dir_Up:
            cam_target_vx = (
                Nav_Push_Execute_Forward_Speed * Nav_Ball_Field_Vx_Scale
            )
            cam_target_vy = (
                Nav_Push_Execute_Forward_Speed * Nav_Ball_Field_Vy_Scale
            )
        else:
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
        if push_dir_code == Push_Dir_Up:
            cam_target_vx = (
                -Nav_Push_Back_Speed * Nav_Ball_Field_Vx_Scale
            )
            cam_target_vy = (
                -Nav_Push_Back_Speed * Nav_Ball_Field_Vy_Scale
            )
        else:
            cam_target_vx = -Nav_Push_Back_Speed
            cam_target_vy = 0.0
        yaw_ref_deg = push_yaw_target
        if final_return_mode == 2 and line_crossed:
            nav_set_state(NAV_STATE_RETURN_BACK)
            return
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Push_Back_Ms:
            nav_set_state(NAV_STATE_PUSH_TURN, "push_back_done")
        return

    if nav_state == NAV_STATE_PUSH_TURN:
        nav_ready_for_push = False
        yaw_ref_deg = push_return_yaw_target
        cam_target_vx = 0.0
        cam_target_vy = 0.0
        yaw_err_abs = abs(-wrapped_yaw_error(yaw_ref_deg, yaw_deg))
        if yaw_err_abs <= Nav_Push_Turn_Ok_Yaw:
            if not push_turn_settle:
                reset_gyro_pid_state()
            push_turn_settle = True
            push_turn_reached_once = True
        elif push_turn_settle and yaw_err_abs > Nav_Push_Turn_Recover_Yaw:
            push_turn_settle = False
            nav_push_turn_ok_since_ms = 0
        if push_turn_settle and low_speed and abs(gyro_z) <= 8.0:
            if nav_push_turn_ok_since_ms == 0:
                nav_push_turn_ok_since_ms = now
            elif utime.ticks_diff(now, nav_push_turn_ok_since_ms) >= Nav_Push_Turn_Ok_Ms:
                imu_runtime.reset_yaw(push_return_yaw_target)
                if pushed_object_count >= Nav_Object_Total:
                    nav_set_state(NAV_STATE_RETURN_LEFT)
                else:
                    nav_set_state(NAV_STATE_POST_TURN_FORWARD, "push_finish_wait_target")
        else:
            nav_push_turn_ok_since_ms = 0
        if utime.ticks_diff(now, nav_transition_ms) >= Nav_Push_Turn_Max_Ms:
            nav_set_state(NAV_STATE_RETURN_DONE, "push_turn_timeout")
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

cam_uart = UART(cfg.CAM_UART_ID, cfg.CAM_UART_BAUD)
cam_uart.init(cfg.CAM_UART_BAUD, timeout_char=100)

# IMU 运行时初始化
imu_runtime = LSM6DSV16XYawRuntime(
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
    elif nav_state in (
        NAV_STATE_FINE,
        NAV_STATE_PUSH_CLASSIFY,
        NAV_STATE_PUSH_PREPARE,
        NAV_STATE_RETURN_LEFT,
        NAV_STATE_RETURN_BACK,
        NAV_STATE_RETURN_FINAL,
    ):
        translate_value = 1
    elif nav_state in (
        NAV_STATE_SEARCH_TURN,
        NAV_STATE_SEARCH_SPIN,
        NAV_STATE_PUSH_ORIENT,
        NAV_STATE_PUSH,
        NAV_STATE_PUSH_BACK,
        NAV_STATE_PUSH_TURN,
        NAV_STATE_RETURN_TURN,
    ):
        rotate_value = 1

    led_straight.value(straight_value)
    led_translate.value(translate_value)
    led_rotate.value(rotate_value)


def calibrate_gyro_before_launch():
    if AUTO_CALIBRATE_GYRO_ON_LAUNCH:
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
    global pushed_object_count, final_return_mode
    current_c9 = key_start.value()
    if current_c9 == 0 and last_c9_state == 1:
        utime.sleep_ms(10)
        if key_start.value() == 0:
            if not car_started:
                log("[C9] launch in 0.5s")
                utime.sleep_ms(500)
                calibrate_gyro_before_launch()
                yaw_ref_deg = imu_runtime.read_yaw()
                refresh_field_reference()
                car_started = True
                auto_start_done = True
                pushed_object_count = 0
                final_return_mode = 0
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

# ====================== 初始化 LED 显示 ======================
update_nav_led_display()
coop_master.init()
log("[INIT] boot, vision loop waiting")
log("[INFO] C9=start C8=exit")

# 陀螺仪偏移设置
log("IMU offset preset %.2f" % GYRO_OFFSET_Z)

# ---------------------- Ticker ----------------------
pit_flag = False
pit_count = 0

# 定时中断回调：置位标志，通知主循环执行控制
def time_pit_handler(_):
    global pit_flag, pit_count
    pit_flag = True
    pit_count += 1

pit1 = ticker(1)
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


# ====================== 速度闭环主函数（含发车判断） ======================
def calc_speed_closed_loop():
    global last_vz_cmd, last_turn_rate_cmd
    global last_pwm_fl, last_pwm_fr, last_pwm_b
    global cam_target_vx, cam_target_vy

    # 未发车：直接输出 0，占空比清零
    if not car_started:
        set_three_pwm_smooth(0, 0, 0)
        return None

    # 已发车：执行闭环逻辑
    # 读取陀螺仪数据
    gyro_z = imu_runtime.read_gyro_z()
    yaw_deg = imu_runtime.read_yaw()
    e_fl = enc_fl.get()
    e_fr = enc_fr.get()
    e_b = enc_b.get()
    coop_master.update_state(e_fl, e_fr, e_b, gyro_z)
    low_speed = abs(e_fl) <= Nav_Low_Speed_Th and abs(e_fr) <= Nav_Low_Speed_Th and abs(e_b) <= Nav_Low_Speed_Th
    update_nav_state_and_targets(yaw_deg, low_speed, gyro_z)

    if nav_state == NAV_STATE_SEARCH:
        reset_speed_pid_state()
        last_turn_rate_cmd = 0.0
        last_vz_cmd = 0.0
        turn_pid.output = 0.0
        turn_pid.err = 0.0
        turn_pid.err_last = 0.0
        gyro_pid.output = 0.0
        gyro_pid.err = 0.0
        gyro_pid.err_last = 0.0
        gyro_pid.gyro_output_limit = GYRO_OUTPUT_LIMIT
        motor_fl.duty(0)
        motor_fr.duty(0)
        motor_b.duty(0)
        return None

    if nav_state == NAV_STATE_RETURN_DONE:
        last_turn_rate_cmd = 0.0
        last_vz_cmd = 0.0
        motor_fl.duty(0)
        motor_fr.duty(0)
        motor_b.duty(0)
        last_pwm_fl = 0
        last_pwm_fr = 0
        last_pwm_b = 0
        return None

    yaw_err_deg = -wrapped_yaw_error(yaw_ref_deg, yaw_deg)
    gyro_rate_mode = False
    if nav_state == NAV_STATE_SEARCH_SPIN:
        spin_remaining = max(0.0, 360.0 - push_orbit_progress_deg)
        spin_rate_mag = get_push_turn_rate(spin_remaining)
        if spin_rate_mag > 0.0:
            turn_rate_cmd = Nav_Return_Turn_Dir * spin_rate_mag
        else:
            turn_rate_cmd = 0.0
        gyro_rate_mode = True
    elif nav_state == NAV_STATE_PUSH_ORIENT:
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
        if push_turn_settle:
            turn_rate_cmd = 0.0
        else:
            turn_dir = Nav_Push_Turn_Forced_Dir
            if final_return_mode == 1:
                turn_dir = -1
            turn_rate_cmd = get_turn_rate_command(
                push_return_yaw_target,
                yaw_deg,
                turn_dir,
                push_turn_reached_once,
            )
        gyro_rate_mode = True
    elif nav_state == NAV_STATE_RETURN_TURN:
        if push_turn_settle:
            turn_rate_cmd = 0.0
        else:
            turn_rate_cmd = get_turn_rate_command(
                field_up_yaw,
                yaw_deg,
                Nav_Return_Turn_Dir,
                push_turn_reached_once,
            )
        gyro_rate_mode = True

    if gyro_rate_mode:
        turn_pid.output = 0.0
        turn_pid.err = 0.0
        turn_pid.err_last = 0.0
    else:
        turn_rate_cmd = turn_ctrl(turn_pid, yaw_err_deg, 0)

    # 陀螺仪内环（方向控制）
    if nav_state in (NAV_STATE_SEARCH_SPIN, NAV_STATE_PUSH_TURN, NAV_STATE_RETURN_TURN):
        gyro_pid.gyro_kp = Nav_Push_Turn_Gyro_Kp
        gyro_pid.gyro_ki = Nav_Push_Turn_Gyro_Ki
    else:
        gyro_pid.gyro_kp = GYRO_KP
        gyro_pid.gyro_ki = GYRO_KI

    if nav_state == NAV_STATE_PUSH_ORIENT:
        gyro_pid.gyro_output_limit = Nav_Push_Orbit_Gyro_Limit
    elif nav_state == NAV_STATE_PUSH:
        gyro_pid.gyro_output_limit = Nav_Push_Execute_Gyro_Limit
    elif nav_state in (NAV_STATE_SEARCH_SPIN, NAV_STATE_PUSH_TURN, NAV_STATE_RETURN_TURN):
        gyro_pid.gyro_output_limit = Nav_Push_Turn_Gyro_Limit
    elif nav_state in (NAV_STATE_SEARCH_TURN, NAV_STATE_COARSE, NAV_STATE_FINE, NAV_STATE_PUSH_CLASSIFY, NAV_STATE_PUSH_PREPARE):
        gyro_pid.gyro_output_limit = Nav_Track_Gyro_Limit
    else:
        gyro_pid.gyro_output_limit = GYRO_OUTPUT_LIMIT
    vz_cmd = gyro_ctrl(gyro_pid, turn_rate_cmd - gyro_z)
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
    set_three_pwm_smooth(u_fl, u_fr, u_b)

    return None

log("speed loop start")

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
            yaw_ref_deg = imu_runtime.read_yaw()
            refresh_field_reference()
            car_started = True
            auto_start_done = True
            pushed_object_count = 0
            final_return_mode = 0
            start_time = now
            nav_set_state(NAV_STATE_SEARCH_TURN, "vision_launch_search_turn", force=True)
            log("[VISION] first valid target, car_started=1")

        if pit_flag:
            pit_flag = False
            calc_speed_closed_loop()
        coop_master.send_if_due(
            now,
            car_started,
            nav_state_code(nav_state),
            cam_target_seen(),
            imu_runtime.read_yaw(),
            cam_target_vx,
            cam_target_vy,
            last_turn_rate_cmd,
        )

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

