from math import fabs

from models import CarState, VehicleContext
from move_base import calc_wheel_spd
from pid import gyro_ctrl, speed_ctrl, turn_ctrl
from uart_protocol import (
    BLOCK_LEFT,
    BLOCK_RIGHT,
    clear_data_fifo,
    deal_art_class,
    deal_art_correct,
    deal_art_correct_y,
    deal_art_detect,
)


class VehicleController:
    """
    MicroPython 运行时的非阻塞控制器。\n    每个 5ms 周期建议调用顺序：\n    1) pit_5ms_step() -> 传感器更新 + 底层电机环\n    2) step()         -> 上层状态机推进
    """

    def __init__(self, hardware, ctx=None):
        if ctx is None:
            ctx = VehicleContext()
        self.ctx = ctx
        self.hw = hardware

        # 启动时初始化速度环参数。
        self.ctx.pid_fl.init_c()
        self.ctx.pid_fr.init_c()
        self.ctx.pid_b.init_c()

        # 保存最新编码器速度快照。
        self.enc_fl = 0.0
        self.enc_fr = 0.0
        self.enc_b = 0.0

        self.correct_num = 0
        self.enable_cross_flag = 1
        self.enable_circle_flag = 1

        self.curve_left_angle = 0.0
        self.curve_right_angle = 0.0
        self.curve_correction_angle = 0.0

        # 用 tick 状态变量替代原先阻塞流程。
        self.run_track_budget = 0
        self.slow_target_yaw = None
        self.slow_timeout = 0

        self.block_rotation_side_mode = 0
        self.push_settle_stage = 0
        self.backing_stage = 0

    def _wrapped_delta(self, target, now):
        # 将航向误差归一化到 [-180, 180]。
        delta = target - now
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        return delta

    def _shortest_abs_delta(self, a, b):
        d = abs(a - b)
        d2 = 360.0 - d
        if d2 < d:
            return d2
        return d

    def _update_encoder(self):
        self.enc_fl, self.enc_fr, self.enc_b = self.hw.read_encoder_spd()

    def _is_low_speed(self, th=30):
        return abs(self.enc_fl) <= th and abs(self.enc_fr) <= th and abs(self.enc_b) <= th

    def _apply_motor_ctrl(self):
        # 底层速度闭环控制（对应 C 中 ISR 的电机更新职责）。
        fl_pwm = speed_ctrl(self.ctx.pid_fl, self.enc_fl, self.ctx.move.speed_fl)
        fr_pwm = speed_ctrl(self.ctx.pid_fr, self.enc_fr, self.ctx.move.speed_fr)
        b_pwm = speed_ctrl(self.ctx.pid_b, self.enc_b, self.ctx.move.speed_b)
        self.hw.set_motor_pwm(fl_pwm, fr_pwm, b_pwm)

    def pit_5ms_step(self):
        """每次 5ms ticker 触发时执行一次。"""
        self._update_encoder()
        yaw_rate = self.hw.read_gyro_z()
        vz = gyro_ctrl(self.ctx.gyro_loop, self.ctx.move.tar_spd_z - yaw_rate)
        calc_wheel_spd(self.ctx.move, self.ctx.move.tar_spd_x, self.ctx.move.tar_spd_y, vz)
        self._apply_motor_ctrl()

        if self.ctx.time_interval_flag == 0:
            self.ctx.time_count += 1
            if self.ctx.time_count >= 175:
                self.ctx.time_count = 0
                self.ctx.time_interval_flag = 1

    def _tick_slow_speed(self):
        # “减速到稳定”为非阻塞 tick 版本。
        if self.slow_target_yaw is None:
            return True

        self.ctx.move.tar_spd_x = 0
        self.ctx.move.tar_spd_y = 0
        err = self._wrapped_delta(self.slow_target_yaw, self.hw.read_yaw())
        self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)

        self.slow_timeout += 1
        if self._is_low_speed(30) or self.slow_timeout >= 120:
            self.slow_target_yaw = None
            self.slow_timeout = 0
            return True
        return False

    def _start_slow_speed(self, target_yaw):
        self.slow_target_yaw = target_yaw
        self.slow_timeout = 0

    def run_track(self):
        # 仅在新图像帧到达时处理图像。
        if not self.hw.mt9v03x_frame_ready():
            return
        self.hw.clear_frame_ready()

        self.ctx.errangle = self.hw.image_solve()
        self.ctx.move.tar_spd_y = 0.0
        self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, self.ctx.errangle, self.ctx.motor_slow_flag)
        self.ctx.move.tar_spd_x = self.ctx.slow_trace_speed if self.ctx.motor_slow_flag else self.ctx.trace_speed

    def run_track_budget_step(self):
        # RunTrack_times(N) 的 tick 版本：消费 N 帧有效图像。
        if self.run_track_budget <= 0:
            return False

        if self.hw.mt9v03x_frame_ready():
            self.hw.clear_frame_ready()
            self.ctx.move.tar_spd_x = self.ctx.trace_speed
            self.ctx.errangle = self.hw.image_solve()
            self.run_track_budget -= 1

        self.ctx.move.tar_spd_y = 0.0
        self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, self.ctx.errangle, self.ctx.motor_slow_flag)
        return self.run_track_budget > 0

    def line_follow(self):
        # 先解析检测串口标记位（250/251/252）。
        deal_art_detect(self.ctx, self.ctx.art_detect)
        if self.ctx.curve_angle_flag:
            self.curve_left_angle, self.curve_right_angle = self.hw.curve_angle_get()
            if fabs(self.curve_left_angle) > fabs(self.curve_right_angle):
                self.curve_correction_angle = self.curve_right_angle
            else:
                self.curve_correction_angle = self.curve_left_angle

            self.ctx.rotate_init_yaw = self.hw.read_yaw()
            self.ctx.rotate_target_yaw = self.ctx.rotate_init_yaw - self.curve_correction_angle
            self.ctx.rotate_init_yaw = self.ctx.rotate_init_yaw - self.curve_correction_angle * 0.3

            if self.ctx.rotate_target_yaw >= 360:
                self.ctx.rotate_target_yaw -= 360.0
            elif self.ctx.rotate_target_yaw < 0:
                self.ctx.rotate_target_yaw += 360.0

            err = self._wrapped_delta(self.ctx.rotate_init_yaw, self.hw.read_yaw())
            self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)

            self.ctx.motor_slow_flag = 0
            self.ctx.curve_angle_flag = 0
            self.enable_cross_flag = self.ctx.ini_cross_flag
            self.enable_circle_flag = self.ctx.circle_flag
            self.hw.uart_write_str("art_detect", "FINE\n")
            self.ctx.state = CarState.BLOCK_LOCATE_MODE
        else:
            self.run_track()

    def block_locate(self):
        err = self._wrapped_delta(self.ctx.rotate_init_yaw, self.hw.read_yaw())
        self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)

        # 每 6 个 tick 处理一次矫正数据，作用等价于旧周期检查。
        self.correct_num += 1
        if self.correct_num >= 6:
            self.correct_num = 0
            if deal_art_correct(self.ctx, self.ctx.art_detect):
                if self._is_low_speed(30) and abs(self.ctx.picture_x) < 30 and abs(self.ctx.picture_y) < 19:
                    self.hw.uart_write_byte("art_detect", 0x03)
                    self.ctx.state = CarState.BLOCK_ROTATION_MODE

    def block_rotation(self):
        if self.ctx.rotation_finish_flag:
            err = self._wrapped_delta(self.ctx.rotate_target_yaw, self.hw.read_yaw())
            self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)
            self.correct_num += 1
            if self.correct_num >= 6:
                self.correct_num = 0
                if deal_art_correct(self.ctx, self.ctx.art_detect):
                    if self._is_low_speed(25) and abs(self.ctx.picture_x) < 25 and -18 < self.ctx.picture_y < 12:
                        self.hw.uart_write_byte("art_model", 0x01)
                        self.ctx.state = CarState.WAITING_IDENTIFY_MODE
                        self.ctx.rotation_finish_flag = 0
        else:
            err_now = self._shortest_abs_delta(self.ctx.rotate_target_yaw, self.hw.read_yaw())
            if err_now < 10:
                self.ctx.rotation_finish_flag = 1
                clear_data_fifo(self.ctx.art_detect)
                self.hw.uart_write_byte("art_detect", 0x02)
                self.block_rotation_side_mode = 0
            else:
                self.ctx.move.tar_spd_x = 0
                if self.curve_correction_angle > 0:
                    self.ctx.move.tar_spd_y = 44 * 1.3
                    self.ctx.move.tar_spd_z = -90 * 1.3
                else:
                    self.ctx.move.tar_spd_y = -44 * 1.3
                    self.ctx.move.tar_spd_z = 90 * 1.3

    def waiting_identify(self):
        err = self._wrapped_delta(self.ctx.rotate_target_yaw, self.hw.read_yaw())
        self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)
        self.ctx.move.tar_spd_x = 0
        self.ctx.move.tar_spd_y = 0

        got_class = deal_art_class(self.ctx, self.ctx.art_model, self.hw.push_left_or_right)
        if got_class:
            self.hw.uart_write_byte("art_detect", 0x03)
            if self.ctx.push_direction == BLOCK_RIGHT:
                self.ctx.move.tar_spd_y = 44 * 1.5
                self.ctx.move.tar_spd_z = -90 * 1.5
            else:
                self.ctx.move.tar_spd_y = -44 * 1.5
                self.ctx.move.tar_spd_z = 90 * 1.5
            self.ctx.move.tar_spd_x = 0
            self.ctx.state = CarState.MOVING_DIRECTION_MODE

    def moving_direction(self):
        # 两阶段过渡：先减速稳定，再切换到推块模式。
        delta = self._shortest_abs_delta(self.ctx.rotate_target_yaw, self.hw.read_yaw())
        if delta >= 80.0:
            if self.ctx.push_direction == BLOCK_RIGHT:
                target = self.ctx.rotate_target_yaw - 90
            else:
                target = self.ctx.rotate_target_yaw + 90

            if self.slow_target_yaw is None and self.push_settle_stage == 0:
                self._start_slow_speed(target)
                return

            if self.push_settle_stage == 0:
                if not self._tick_slow_speed():
                    return
                self.push_settle_stage = 1

            if self.push_settle_stage == 1:
                self.ctx.move.tar_spd_x = 0
                self.ctx.move.tar_spd_y = 0
                err = self._wrapped_delta(target, self.hw.read_yaw())
                self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)
                clear_data_fifo(self.ctx.art_detect)
                clear_data_fifo(self.ctx.art_model)
                self.hw.uart_write_byte("art_detect", 0x02)
                self.ctx.state = CarState.PUSHING_BLOCKS_MODE
                self.push_settle_stage = 0

    def pushing_blocks(self):
        if self.ctx.push_direction == BLOCK_RIGHT:
            target = self.ctx.rotate_target_yaw - 90
        else:
            target = self.ctx.rotate_target_yaw + 90

        err = self._wrapped_delta(target, self.hw.read_yaw())
        self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)

        self.correct_num += 1
        if self.correct_num >= 6:
            self.correct_num = 0
            if deal_art_correct_y(self.ctx, self.ctx.art_detect):
                if self.ctx.correct_finish2_flag == 0:
                    self.ctx.move.tar_spd_x = 0
                    if self._is_low_speed(30) and abs(self.ctx.picture_x) < 30:
                        self.ctx.move.tar_spd_x = 80
                        self.ctx.correct_finish2_flag = 1

        if self.ctx.correct_finish2_flag:
            c15 = self.hw.gpio_get_level("C15")
            if self.ctx.push_white_flag == 0 and c15 == 1:
                self.ctx.back_time_count += 1
                if self.ctx.back_time_count > 10:
                    self.ctx.push_white_flag = 1
                    self.ctx.back_time_count = 0
            elif self.ctx.push_white_flag and c15 == 0:
                self.hw.uart_write_byte("art_detect", 0x03)
                if self.slow_target_yaw is None:
                    self._start_slow_speed(target)
                    return
                if not self._tick_slow_speed():
                    return

                self.ctx.state = CarState.BACKING_TRACK_MODE
                self.ctx.push_white_flag = 0
                self.ctx.correct_finish2_flag = 0
                self.backing_stage = 0

    def backing_track(self):
        # 多阶段回赛道流程，每个阶段都按 tick 推进。
        if self.backing_stage == 0:
            self.ctx.move.tar_spd_x = -70
            self.ctx.move.tar_spd_y = 0

            if self.ctx.push_direction == BLOCK_RIGHT:
                side_target = self.ctx.rotate_target_yaw - 90
            else:
                side_target = self.ctx.rotate_target_yaw + 90

            err = self._wrapped_delta(side_target, self.hw.read_yaw())
            self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)

            if self.hw.gpio_get_level("C15") == 1:
                self.ctx.back_time_count += 1
                if self.ctx.back_time_count > 20:
                    self.ctx.back_time_count = 0
                    self._start_slow_speed(self.hw.read_yaw())
                    self.backing_stage = 1

        elif self.backing_stage == 1:
            if not self._tick_slow_speed():
                return
            self.backing_stage = 2

        elif self.backing_stage == 2:
            err = self._wrapped_delta(self.ctx.rotate_target_yaw, self.hw.read_yaw())
            self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)
            delta = self._shortest_abs_delta(self.ctx.rotate_target_yaw, self.hw.read_yaw())
            if delta <= 6.0:
                self._start_slow_speed(self.ctx.rotate_target_yaw)
                self.backing_stage = 3

        elif self.backing_stage == 3:
            if not self._tick_slow_speed():
                return
            self.run_track_budget = 70
            self.backing_stage = 4

        elif self.backing_stage == 4:
            if self.run_track_budget > 0:
                self.run_track_budget_step()
                return
            clear_data_fifo(self.ctx.art_detect)
            clear_data_fifo(self.ctx.art_model)
            self.hw.uart_write_byte("art_detect", 0x01)
            self.ctx.state = CarState.LINE_FOLLOW_MODE
            self.backing_stage = 0

    def coarse_approach(self):
        self.correct_num += 1
        if self.correct_num >= 6:
            self.correct_num = 0
            deal_art_correct(self.ctx, self.ctx.art_detect)

        moving = self.ctx.move.tar_spd_x != 0.0 or self.ctx.move.tar_spd_y != 0.0
        if moving and self._is_low_speed(30):
            self.hw.uart_write_str("art_detect", "FINE\n")
            self.ctx.state = CarState.BLOCK_LOCATE_MODE

    def set_speed_pid_all(self, kp=None, ki=None, kd=None, gama=None):
        """
        统一设置三路轮速 PID。
        常用：ctrl.set_speed_pid_all(kp=80, ki=12)
        如果改了 kp/kd/gama，会自动重算离散系数。
        """
        pids = [self.ctx.pid_fl, self.ctx.pid_fr, self.ctx.pid_b]
        for pid in pids:
            if kp is not None:
                pid.kp = float(kp)
            if ki is not None:
                pid.ki = float(ki)
            need_reinit = False
            if kd is not None:
                pid.kd = float(kd)
                need_reinit = True
            if gama is not None:
                pid.gama = float(gama)
                need_reinit = True
            if kp is not None:
                need_reinit = True
            if need_reinit:
                pid.init_c()

    def set_gyro_pid(self, kp=None, ki=None, output_limit=None):
        if kp is not None:
            self.ctx.gyro_loop.gyro_kp = float(kp)
        if ki is not None:
            self.ctx.gyro_loop.gyro_ki = float(ki)
        if output_limit is not None:
            self.ctx.gyro_loop.gyro_output_limit = float(output_limit)

    def reset_gyro_pid_output(self):
        self.ctx.gyro_loop.output = 0.0
        self.ctx.gyro_loop.err = 0.0
        self.ctx.gyro_loop.err_last = 0.0

    def reset_speed_pid_output(self):
        """调参阶段可调用：清空历史积分/输出，避免突变。"""
        for pid in (self.ctx.pid_fl, self.ctx.pid_fr, self.ctx.pid_b):
            pid.output = 0.0
            pid.err = 0.0
            pid.err_last = 0.0
            pid.delta_ud = 0.0
            pid.delta_tar = 0.0
            pid.delta_tar_last = 0.0
            pid.tar_spd_last = 0.0

    def get_debug_snapshot(self):
        """返回当前调试快照，便于你串口打印/示波。"""
        return {
            "state": self.ctx.state,
            "enc_fl": self.enc_fl,
            "enc_fr": self.enc_fr,
            "enc_b": self.enc_b,
            "tar_x": self.ctx.move.tar_spd_x,
            "tar_y": self.ctx.move.tar_spd_y,
            "tar_z": self.ctx.move.tar_spd_z,
            "wheel_fl": self.ctx.move.speed_fl,
            "wheel_fr": self.ctx.move.speed_fr,
            "wheel_b": self.ctx.move.speed_b,
            "pid_fl_out": self.ctx.pid_fl.output,
            "pid_fr_out": self.ctx.pid_fr.output,
            "pid_b_out": self.ctx.pid_b.output,
        }
    def step(self):
        # 上层调度入口：主循环每轮调用一次。
        if self.ctx.start_flag != 1:
            return

        if self.ctx.state == CarState.LINE_FOLLOW_MODE:
            self.line_follow()
        elif self.ctx.state == CarState.BLOCK_LOCATE_MODE:
            self.block_locate()
        elif self.ctx.state == CarState.BLOCK_ROTATION_MODE:
            self.block_rotation()
        elif self.ctx.state == CarState.WAITING_IDENTIFY_MODE:
            self.waiting_identify()
        elif self.ctx.state == CarState.MOVING_DIRECTION_MODE:
            self.moving_direction()
        elif self.ctx.state == CarState.PUSHING_BLOCKS_MODE:
            self.pushing_blocks()
        elif self.ctx.state == CarState.BACKING_TRACK_MODE:
            self.backing_track()
        elif self.ctx.state == CarState.COARSE_APPROACH_MODE:
            self.coarse_approach()




