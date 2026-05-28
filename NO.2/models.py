class CarState:
    # 主状态机枚举（与原项目状态意义一致）
    LINE_FOLLOW_MODE = 0
    BLOCK_LOCATE_MODE = 1
    BLOCK_ROTATION_MODE = 2
    WAITING_IDENTIFY_MODE = 3
    MOVING_DIRECTION_MODE = 4
    VERIFY_TRACK_MODE = 5
    PUSHING_BLOCKS_MODE = 6
    BACKING_TRACK_MODE = 7
    COARSE_APPROACH_MODE = 8


class AnglePID:
    def __init__(self):
        self.err = 0.0
        self.output = 0.0
        self.err_last = 0.0
        # 分段转向控制使用的比例参数
        self.kp = [3.5, 2.5, 1.0]
        self.gyro_kp = 0.4
        self.gyro_ki = 0.012
        self.gyro_output_limit = 150.0


class SpeedPID:
    def __init__(self):
        # 前馈/滤波中间变量
        self.temp = 0.0
        self.c1 = 0.0
        self.c2 = 0.0
        self.c3 = 0.0

        # 闭环误差与输出
        self.err = 0.0
        self.err_last = 0.0
        self.tar_spd_last = 0.0
        self.output = 0.0

        # 目标变化量
        self.delta_tar_last = 0.0
        self.delta_tar = 0.0
        self.delta_ud = 0.0

        # 可调参数（建议调参时优先改 kp/ki）
        self.param_a = 0.0
        self.param_b = 0.0
        self.kd = 30.0
        self.gama = 0.4
        self.kp = 760.0
        self.ki = 0.0

    def init_c(self):
        # 根据 kd/gama/kp 计算离散控制系数
        self.temp = 1.0 / (self.gama * self.kd + self.kp)
        self.c3 = self.kd * self.temp
        self.c2 = (self.kd + self.kp) * self.temp
        self.c1 = self.c3 * self.gama


class MoveBase:
    def __init__(self):
        # 三轮目标速度与车体速度分量
        self.speed_fl = 0.0
        self.speed_fr = 0.0
        self.speed_b = 0.0
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0
        self.tar_spd_x = 0.0
        self.tar_spd_y = 0.0
        self.tar_spd_z = 0.0


class UartChannel:
    def __init__(self, name, maxlen=16):
        self.name = name
        self.maxlen = maxlen
        self._fifo = []

    def push(self, byte_val):
        if len(self._fifo) >= self.maxlen:
            self._fifo.pop(0)
        self._fifo.append(byte_val & 0xFF)

    def pop_all(self):
        out = self._fifo
        self._fifo = []
        return out

    def used(self):
        return len(self._fifo)


class VehicleContext:
    """
    控制上下文：
    1. 这里保存所有可调参数与运行时状态。
    2. 你主程序通常只需要创建一个 ctx 传给 VehicleController。
    """

    def __init__(self):
        self.trace_speed = 140.0
        self.slow_trace_speed = 40.0

        self.state = CarState.LINE_FOLLOW_MODE
        self.move = MoveBase()

        self.turn_loop = AnglePID()
        self.gyro_loop = AnglePID()
        self.pid_fl = SpeedPID()
        self.pid_fr = SpeedPID()
        self.pid_b = SpeedPID()

        self.art_detect = UartChannel("art_detect")
        self.art_model = UartChannel("art_model")

        self.start_flag = 1
        self.show_res_flag = 0

        self.page_num = 0
        self.pic_num = 0
        self.class_type = [200] * 40

        self.picture_x = 127
        self.picture_y = 127
        self.push_direction = 0

        self.rotate_target_yaw = 0.0
        self.rotate_init_yaw = 0.0
        self.errangle = 0.0

        self.back_time_count = 0
        self.cross_box_flag = 0
        self.motor_slow_flag = 0
        self.curve_angle_flag = 0
        self.turn_angle_flag = 0
        self.push_white_flag = 0
        self.rotation_finish_flag = 0
        self.correct_finish2_flag = 0

        self.ini_cross_flag = 0
        self.circle_flag = 0
        self.time_interval_flag = 0
        self.time_count = 0

