from machine import Pin,PWM
import gc
import utime
from smartcar import ticker, encoder
from seekfree import WIRELESS_UART, MOTOR_CONTROLLER
from imu_runtime import IMUYawRuntime
from models import AnglePID, MoveBase, SpeedPID
from move_base import calc_wheel_spd
import pid as _pid_mod

# 设置PID最大PWM值
_pid_mod.PWM_MAX = 10000.0
speed_ctrl = _pid_mod.speed_ctrl
gyro_ctrl = _pid_mod.gyro_ctrl

# ====================== 基础配置 ======================
# 系统控制周期
TICK_PERIOD_MS = 5
# 电机PWM频率
MOTOR_FREQ = 13000
# 电机最大/最小有效占空比
MOTOR_DUTY_MAX = 10000
MOTOR_DUTY_MIN = 300
# PWM平滑滤波系数
PWM_SMOOTH_FACTOR = 0.4
# PWM单次最大变化量（防冲击）
MAX_PWM_CHANGE = 800

# 默认只开启速度环，需要方向控制时再打开陀螺仪环
ENABLE_GYRO_LOOP = False
ENABLE_OSCILLOSCOPE = False
SHOW_DEVICE_INFO = False

# 运行模式
# wheel_dir_cal：仅校准轮子方向
# speed_loop：正常速度闭环测试
RUN_MODE = "speed_loop"
CAL_WHEEL = "fr"
CAL_PWM = 1200
CAL_SIGN = 1

# 车体运动模式
# straight：前后直行
# translate：左右平移
# rotate：原地旋转
BODY_TEST_MODE = "straight"
TEST_LINEAR_SPEED = 3.0
TEST_ROTATE_SPEED = 3.0
GYRO_SIGN = -1.0

# 陀螺仪Z轴校准参数
GYRO_OFFSET_Z = 1.0
AUTO_GYRO_OFFSET_CAL = False
GYRO_CALIB_SAMPLES = 2000
GYRO_CALIB_DELAY_MS = 1
GYRO_SCALE = -1.0 / 16.54052
GYRO_DEADBAND_DPS = 0.8
GYRO_KP = 0.4
GYRO_KI = 0.012
GYRO_OUTPUT_LIMIT = 8.0

# 根据运动模式设置目标速度
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
    raise ValueError("BODY_TEST_MODE 必须是 straight / translate / rotate")

# IMU使能条件
ENABLE_IMU = ENABLE_GYRO_LOOP or AUTO_GYRO_OFFSET_CAL or SHOW_DEVICE_INFO

# 编码器与IMU捕获配置
ENCODER_CAPTURE_DIV = 1
IMU_CAPTURE_DIV = 1
ENC_FL_INVERT = False
ENC_FR_INVERT = False
ENC_B_INVERT = False

# 调试与退出配置
DEBUG_DIV = 20
EXIT_CHECK_DIV = 5
GC_DIV = 50
TIMEOUT_SECOND = 10  # 【修改】发车后运行10秒自动退出

# 无线遥控器7通道作为退出触发
EXIT_TRIGGER_CHANNEL = 7
CH7_TOLERANCE = 1.0

# ====================== 全局状态变量 ======================
# 运动模式索引：0=直行 1=平移 2=旋转
motion_mode_idx = 0
# 运动模式名称列表
motion_mode_names = ["straight", "translate", "rotate"]
# 小车发车启动标志：默认False=上电静止不动，True=启动运行
car_started = False
# 上一次C14模式按键状态（消抖用）
last_c14_state = 1
# 上一次C9发车按键状态（消抖用）
last_c9_state = 1
# 上一次C8退出按键状态（消抖用）
last_c8_state = 1

# ====================== 标准 machine.PWM 电机驱动类 ======================
class Motor:
    def __init__(self, ph_pin, pwm_pin, freq=13000, invert=False):
        self.invert = invert
        self.ph = Pin(ph_pin, Pin.OUT, value=0)
        # 标准MicroPython PWM初始化：pin, freq, duty_u16
        self.pwm = PWM(pwm_pin, freq, duty_u16=0)

    def set_speed(self, speed):
        if speed == 0:
            self.pwm.duty_u16(0)
            return
        # 设置正反转方向
        dir_val = 1 if speed > 0 else 0
        if self.invert:
            dir_val = 1 - dir_val
        self.ph.value(dir_val)
        # 标准PWM占空比设置 [0,65535]
        self.pwm.duty_u16(abs(speed))

    def duty(self, value):
        # 对外提供duty接口，保持原有代码兼容
        if value == 0:
            self.pwm.duty_u16(0)
            return
        dir_val = 1 if value > 0 else 0
        if self.invert:
            dir_val = 1 - dir_val
        self.ph.value(dir_val)
        self.pwm.duty_u16(abs(value))

# 外部退出按键 C8
key_exit = Pin("C8", Pin.IN, Pin.PULL_UP)

# ====================== 按键硬件初始化 ======================
# 模式切换按键 C14
key_mode = Pin("C14", Pin.IN, Pin.PULL_UP)
# 发车启动按键 C9
key_start = Pin("C9", Pin.IN, Pin.PULL_UP)
# 状态LED C19 C20 C21
led_straight = Pin("C19", Pin.OUT, value=0)
led_translate = Pin("C20", Pin.OUT, value=0)
led_rotate = Pin("C21", Pin.OUT, value=0)

# ---------------------- Demo-style hardware init ----------------------
utime.sleep_ms(100)

# 状态LED
led = Pin("C4", Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

# 三个电机初始化（引脚完全不变，只换驱动底层）
motor_fl = Motor("C30", "C31", freq=MOTOR_FREQ, invert=False)
motor_fr = Motor("D4",  "D5",  freq=MOTOR_FREQ, invert=False)
motor_b = Motor("C28", "C29", freq=MOTOR_FREQ, invert=False)

# 三个编码器初始化
enc_fl = encoder("C2", "C3", ENC_FL_INVERT)
enc_fr = encoder("D13", "D14", ENC_FR_INVERT)
enc_b = encoder("C0", "C1", ENC_B_INVERT)

# 无线串口初始化
wireless = WIRELESS_UART(460800)

# IMU运行时初始化
imu_runtime = None
if ENABLE_IMU:
    imu_runtime = IMUYawRuntime(
        sign=GYRO_SIGN,
        offset_z=GYRO_OFFSET_Z,
        scale=GYRO_SCALE,
        deadband_dps=GYRO_DEADBAND_DPS,
        tick_period_ms=TICK_PERIOD_MS,
    )

# 打印设备信息
if SHOW_DEVICE_INFO:
    MOTOR_CONTROLLER.help()
    motor_fl.info()
    motor_fr.info()
    motor_b.info()
    if ENABLE_IMU:
        IMUYawRuntime.help()
        imu_runtime.info()

# ====================== LED模式显示函数 ======================
def update_led_display():
    """根据当前运动模式索引更新C19/C20/C21 LED状态"""
    led_straight.value(0)
    led_translate.value(0)
    led_rotate.value(0)
    if motion_mode_idx == 0:
        led_straight.value(1)
    elif motion_mode_idx == 1:
        led_translate.value(1)
    elif motion_mode_idx == 2:
        led_rotate.value(1)

# ====================== 运动模式切换函数（C14触发） ======================
def switch_motion_mode():
    """切换运动模式索引，更新全局目标速度变量，更新LED，打印日志，切换时自动停车"""
    global motion_mode_idx, STRAIGHT_VX, STRAIGHT_VY, STRAIGHT_VZ, BODY_TEST_MODE, car_started
    # 切换模式时先停车，禁止运动中切换
    car_started = False
    stop_all()
    # 索引循环+1
    motion_mode_idx = (motion_mode_idx + 1) % 3
    # 更新模式名称
    BODY_TEST_MODE = motion_mode_names[motion_mode_idx]
    # 更新目标速度
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
    # 更新LED
    update_led_display()
    # 打印日志
    log(f"[模式切换] 当前模式：{BODY_TEST_MODE}，已停车，请按C9发车")

# ====================== C9发车启动函数 ======================
def check_c9_start():
    """检查C9发车按键，按下后等待0.5秒，再启动小车，并重置计时起点"""
    global last_c9_state, car_started, start_time
    current_c9 = key_start.value()
    # 按键按下触发（下降沿）
    if current_c9 == 0 and last_c9_state == 1:
        utime.sleep_ms(10)
        if key_start.value() == 0:
            # 未启动的话，等待0.5秒发车
            if not car_started:
                log("[C9按键] 即将发车，等待0.5秒...")
                utime.sleep_ms(500)
                car_started = True
                # 【修改】发车时重置计时起点
                start_time = utime.ticks_ms()
                log(f"[C9按键] 发车成功！当前模式：{BODY_TEST_MODE}，将运行{TIMEOUT_SECOND}秒")
    last_c9_state = current_c9

# ====================== C14模式切换检查函数 ======================
def check_c14_switch():
    """检查C14模式切换按键（带消抖），按下则切换运动模式"""
    global last_c14_state
    current_c14 = key_mode.value()
    if current_c14 == 0 and last_c14_state == 1:
        utime.sleep_ms(10)
        if key_mode.value() == 0:
            switch_motion_mode()
    last_c14_state = current_c14

# ====================== C8按键退出检查函数 ======================
def check_c8_exit():
    """检查C8按键（带消抖），按下则抛出KeyboardInterrupt"""
    global last_c8_state
    current_c8 = key_exit.value()
    if current_c8 == 0 and last_c8_state == 1:
        utime.sleep_ms(10)
        if key_exit.value() == 0:
            log("[C8按键] 触发退出程序")
            raise KeyboardInterrupt
    last_c8_state = current_c8

# ====================== Helpers ======================
# 日志输出：串口+无线双发
def log(msg):
    print(msg)
    wireless.send_str(msg + "\r\n")

# 停止所有电机
def stop_all():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)
    log("[STOP] 所有电机占空比已清零")

# 占空比限幅函数
def clamp_duty(value):
    value = int(value)
    if value > MOTOR_DUTY_MAX:
        return MOTOR_DUTY_MAX
    if value < -MOTOR_DUTY_MAX:
        return -MOTOR_DUTY_MAX
    return value

# PWM平滑处理函数
def smooth_value(target, last):
    delta = target - last
    if abs(delta) > MAX_PWM_CHANGE:
        target = last + MAX_PWM_CHANGE * (1 if delta > 0 else -1)
    target = int(last * (1.0 - PWM_SMOOTH_FACTOR) + target * PWM_SMOOTH_FACTOR)
    return clamp_duty(target)

# 电机占空比应用函数（含最小占空比处理）
def apply_motor_duty(cmd, motor):
    cmd = int(cmd)
    if 0 < abs(cmd) < MOTOR_DUTY_MIN:
        cmd = MOTOR_DUTY_MIN if cmd > 0 else -MOTOR_DUTY_MIN
    motor.duty(cmd)

# 三路电机PWM平滑设置
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

# 根据角色获取电机对象
def get_role_motor(role):
    if role == "fl":
        return motor_fl
    if role == "fr":
        return motor_fr
    if role == "b":
        return motor_b
    return None

# 根据角色获取编码器值
def get_role_encoder_value(role, e_fl, e_fr, e_b):
    if role == "fl":
        return e_fl
    if role == "fr":
        return e_fr
    if role == "b":
        return e_b
    return 0

# 单轮驱动函数
def apply_single_wheel_duty(role, duty):
    apply_motor_duty(duty if role == "fl" else 0, motor_fl)
    apply_motor_duty(duty if role == "fr" else 0, motor_fr)
    apply_motor_duty(duty if role == "b" else 0, motor_b)

# 车轮方向分析函数
def analyze_wheel_dir(role, e_fl, e_fr, e_b):
    focus = get_role_encoder_value(role, e_fl, e_fr, e_b)
    other_1 = get_role_encoder_value("fl" if role != "fl" else "fr", e_fl, e_fr, e_b)
    other_2 = get_role_encoder_value("b" if role != "b" else "fr", e_fl, e_fr, e_b)
    other_peak = max(abs(other_1), abs(other_2))

    if abs(focus) <= 1 and other_peak > 1:
        return "目标编码器无反应，引脚映射可能错误"
    if focus < -1:
        return "正PWM产生负编码器值，需要反转该编码器"
    if focus > 1 and other_peak <= abs(focus):
        return "方向匹配正确"
    if focus > 1:
        return "方向正确，但其他编码器也有变化"
    return "编码器响应过小，请抬起轮子或加大PWM"

# 检查遥控器7通道是否触发退出
def check_upper_exit():
    wireless.data_analysis()
    ch7_current = wireless.get_data(EXIT_TRIGGER_CHANNEL)
    if abs(ch7_current - ch7_init_value) > CH7_TOLERANCE:
        log(f"CH7变化：基准={ch7_init_value:.1f} 当前={ch7_current:.1f}")
        return True
    return False

# 检查硬件退出按键（带消抖）
def check_exit_key():
    if key_exit.value() == 0:
        utime.sleep_ms(10)
        if key_exit.value() == 0:
            raise KeyboardInterrupt

# ---------------------- CH7 exit calibration ----------------------
log("=== 开始CH7通道基准校准 ===")

ch7_samples = []
for _ in range(5):
    wireless.data_analysis()
    utime.sleep_ms(10)
    ch7_samples.append(wireless.get_data(EXIT_TRIGGER_CHANNEL))

ch7_init_value = sum(ch7_samples) / len(ch7_samples)
log(f"CH7基准校准完成：{ch7_init_value:.1f}")
log(f"当CHZ变化超过±{CH7_TOLERANCE:.1f}时退出程序")

# ====================== 初始化LED显示 ======================
update_led_display()
log(f"[初始化] 程序启动，小车已静止，默认模式：{BODY_TEST_MODE}")
log("[操作说明] C14=切换模式 | C9=发车（运行{TIMEOUT_SECOND}秒自动退出） | C8=退出程序")

# 陀螺仪偏移量设置
gyro_offset_z = GYRO_OFFSET_Z
if RUN_MODE != "wheel_dir_cal":
    if ENABLE_IMU and AUTO_GYRO_OFFSET_CAL:
        gyro_offset_z = imu_runtime.calibrate_offset(
            samples=GYRO_CALIB_SAMPLES,
            delay_ms=GYRO_CALIB_DELAY_MS,
            logger=log,
        )
    elif ENABLE_IMU:
        log(f"陀螺仪偏移预设：z轴={gyro_offset_z:.2f}")
    else:
        log("IMU未使能，仅速度环运行")

# ---------------------- Ticker ----------------------
pit_flag = False
pit_count = 0

# 定时中断回调：设置标志位，通知主循环执行控制
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

# ====================== 速度闭环计算主函数（增加发车判断） ======================
def calc_speed_closed_loop():
    global last_vz_cmd

    # 未发车状态：直接输出0占空比，电机完全静止
    if not car_started:
        set_three_pwm_smooth(0, 0, 0)
        return {
            "enc_fl": 0, "enc_fr": 0, "enc_b": 0,
            "tar_fl": 0, "tar_fr": 0, "tar_b": 0,
            "out_fl": 0, "out_fr": 0, "out_b": 0,
            "pwm_fl": 0, "pwm_fr": 0, "pwm_b": 0,
            "raw_gyro_z": 0, "gyro_z": 0, "yaw_deg": 0,
            "vz_cmd": 0
        }

    # 已发车状态：执行原有闭环逻辑
    # 读取陀螺仪数据
    if ENABLE_IMU:
        gyro_z = imu_runtime.read_gyro_z()
        raw_gyro_z = imu_runtime.raw_gyro_z
        yaw_deg = imu_runtime.read_yaw()
    else:
        gyro_z = 0.0
        raw_gyro_z = 0.0
        yaw_deg = 0.0

    # 陀螺仪闭环控制（方向控制）
    if ENABLE_GYRO_LOOP and gyro_pid is not None:
        vz_cmd = gyro_ctrl(gyro_pid, STRAIGHT_VZ - gyro_z)
    else:
        if gyro_pid is not None:
            gyro_pid.output = 0.0
            gyro_pid.err = 0.0
            gyro_pid.err_last = 0.0
        vz_cmd = STRAIGHT_VZ
    last_vz_cmd = vz_cmd

    # 车体运动学解算：输出三个轮子目标转速
    calc_wheel_spd(move_cmd, STRAIGHT_VX, STRAIGHT_VY, vz_cmd)

    # 读取编码器当前速度
    e_fl = enc_fl.get()
    e_fr = enc_fr.get()
    e_b = enc_b.get()

    # 目标速度
    t_fl = move_cmd.speed_fl
    t_fr = move_cmd.speed_fr
    t_b = move_cmd.speed_b

    # 速度PID计算输出
    u_fl = speed_ctrl(pid_fl, e_fl, t_fl)
    u_fr = speed_ctrl(pid_fr, e_fr, t_fr)
    u_b = speed_ctrl(pid_b, e_b, t_b)

    # PWM平滑输出
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

# 轮子方向校准模式
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
    log("=== 轮子方向校准开始 ===")
    log(
        "tick=%dms mode=%s wheel=%s duty=%d sign=%+d"
        % (TICK_PERIOD_MS, RUN_MODE, CAL_WHEEL, CAL_PWM, CAL_SIGN)
    )
    log(
        "规则：正PWM应让对应轮子编码器正向计数，否则反转该编码器配置"
    )
else:
    log("=== 速度闭环启动 ===")
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
    "电机映射：fl=C28/C29 invert=False, fr=C30/C31 invert=False, b=D4/D5 invert=True"
)
log(
    "编码器映射：fl=D15/D16 inv=%s, fr=D13/D14 inv=%s, b=C2/C3 inv=%s, capture_div=%d"
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

        # ====================== 按键检查（主循环最前面） ======================
        check_c8_exit()
        check_c14_switch()
        check_c9_start()

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

        # 【修改】只有发车后才检查超时
        if car_started and elapsed_s >= TIMEOUT_SECOND:
            log("=== 运行10秒超时，程序停止 ===")
            break

        if loop_count % GC_DIV == 0:
            gc.collect()

        utime.sleep_ms(1)

finally:
    pit1.stop()
    stop_all()
    led.value(True)
    # 退出时熄灭所有LED
    led_straight.value(0)
    led_translate.value(0)
    led_rotate.value(0)
    log("=== program fully stopped ===")
