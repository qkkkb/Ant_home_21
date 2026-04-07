from move_base import calc_wheel_spd
from pid import gyro_ctrl, speed_ctrl, turn_ctrl, speed_reset
from models import VehicleContext
from coop_protocol import CoopProtocol, get_target_profile, update_partner_flags


class CoopState:
    SEARCH_TARGET = 0
    ALIGN_TARGET = 1
    WAIT_PARTNER = 2
    COOP_ROTATE = 3
    COOP_PUSH = 4
    VERIFY_FINISH = 5
    RETURN_ROUTE = 6
    FAIL_SAFE = 7


class PartnerState:
    IDLE = CoopProtocol.IDLE
    SEARCHING = CoopProtocol.SEARCHING
    READY = CoopProtocol.READY
    ROTATING = CoopProtocol.ROTATING
    PUSHING = CoopProtocol.PUSHING
    DONE = CoopProtocol.DONE
    ERROR = CoopProtocol.ERROR


class CoopController:
    """
    两车协同任务骨架：
    1. 底层仍沿用 5ms 节拍的 gyro 环 + 轮速环
    2. 上层状态机改成“找目标 -> 对准 -> 等队友 -> 协同转向 -> 协同推动 -> 验证 -> 返回”
    3. 通信建议只改标志位，不要在串口接收处直接打电机
    """

    def __init__(self, hardware, ctx=None):
        if ctx is None:
            ctx = VehicleContext()
        self.ctx = ctx
        self.hw = hardware

        self.ctx.pid_fl.init_c()
        self.ctx.pid_fr.init_c()
        self.ctx.pid_b.init_c()

        self.enc_fl = 0.0
        self.enc_fr = 0.0
        self.enc_b = 0.0

        self._ensure_coop_fields()

    def _ensure_coop_fields(self):
        ctx = self.ctx
        if not hasattr(ctx, "coop_state"):
            ctx.coop_state = CoopState.SEARCH_TARGET
        if not hasattr(ctx, "target_label"):
            ctx.target_label = -1
        if not hasattr(ctx, "target_found"):
            ctx.target_found = 0
        if not hasattr(ctx, "target_x"):
            ctx.target_x = 0.0
        if not hasattr(ctx, "target_y"):
            ctx.target_y = 0.0
        if not hasattr(ctx, "partner_state"):
            ctx.partner_state = PartnerState.IDLE
        if not hasattr(ctx, "partner_ready"):
            ctx.partner_ready = 0
        if not hasattr(ctx, "partner_payload"):
            ctx.partner_payload = 0
        if not hasattr(ctx, "task_done"):
            ctx.task_done = 0
        if not hasattr(ctx, "task_failed"):
            ctx.task_failed = 0
        if not hasattr(ctx, "rotate_active"):
            ctx.rotate_active = 0
        if not hasattr(ctx, "rotate_target_yaw"):
            ctx.rotate_target_yaw = 0.0
        if not hasattr(ctx, "state_ticks"):
            ctx.state_ticks = 0
        if not hasattr(ctx, "push_ticks"):
            ctx.push_ticks = 0
        if not hasattr(ctx, "verify_ticks"):
            ctx.verify_ticks = 0
        if not hasattr(ctx, "return_ticks"):
            ctx.return_ticks = 0
        if not hasattr(ctx, "target_push_yaw"):
            ctx.target_push_yaw = 0.0
        if not hasattr(ctx, "search_speed"):
            ctx.search_speed = 70.0
        if not hasattr(ctx, "align_gain_x"):
            ctx.align_gain_x = 0.28
        if not hasattr(ctx, "align_gain_y"):
            ctx.align_gain_y = 0.20
        if not hasattr(ctx, "push_speed"):
            ctx.push_speed = 90.0
        if not hasattr(ctx, "return_speed"):
            ctx.return_speed = -70.0
        if not hasattr(ctx, "align_tol"):
            ctx.align_tol = 12.0
        if not hasattr(ctx, "rotate_plan_deg"):
            ctx.rotate_plan_deg = 90
        if not hasattr(ctx, "push_timeout"):
            ctx.push_timeout = 240

    def _load_target_profile(self):
        profile = get_target_profile(self.ctx.target_label)
        self.ctx.align_tol = float(profile["align_tol"])
        self.ctx.push_speed = float(profile["push_speed"])
        self.ctx.rotate_plan_deg = int(profile["rotate_deg"])
        self.ctx.push_timeout = int(profile["push_timeout"])
        return profile

    def _wrapped_delta(self, target, now):
        delta = target - now
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        return delta

    def _shortest_abs_delta(self, a, b):
        d = abs(a - b)
        d2 = 360.0 - d
        return d2 if d2 < d else d

    def _update_encoder(self):
        self.enc_fl, self.enc_fr, self.enc_b = self.hw.read_encoder_spd()

    def _is_low_speed(self, th=30):
        return abs(self.enc_fl) <= th and abs(self.enc_fr) <= th and abs(self.enc_b) <= th

    def _apply_motor_ctrl(self):
        fl_pwm = speed_ctrl(self.ctx.pid_fl, self.enc_fl, self.ctx.move.speed_fl)
        fr_pwm = speed_ctrl(self.ctx.pid_fr, self.enc_fr, self.ctx.move.speed_fr)
        b_pwm = speed_ctrl(self.ctx.pid_b, self.enc_b, self.ctx.move.speed_b)
        self.hw.set_motor_pwm(fl_pwm, fr_pwm, b_pwm)

    def _stop_motion(self):
        self.ctx.move.tar_spd_x = 0.0
        self.ctx.move.tar_spd_y = 0.0
        self.ctx.move.tar_spd_z = 0.0

    def reset_speed_pid_output(self):
        for pid in (self.ctx.pid_fl, self.ctx.pid_fr, self.ctx.pid_b):
            speed_reset(pid)
            pid.delta_ud = 0.0
            pid.delta_tar = 0.0
            pid.delta_tar_last = 0.0
            pid.tar_spd_last = 0.0

    # ---------------- 通信/标志位入口 ----------------
    def feed_art_pose(self, target_found, target_x, target_y, target_label=-1):
        self.ctx.target_found = 1 if target_found else 0
        self.ctx.target_x = float(target_x)
        self.ctx.target_y = float(target_y)
        if target_label >= 0:
            self.ctx.target_label = int(target_label)
            self._load_target_profile()

    def feed_partner_state(self, partner_state, payload=0):
        update_partner_flags(self.ctx, int(partner_state), int(payload))

    # ---------------- 动作层：独立能力函数 ----------------
    def start_rotate_by(self, degrees):
        now_yaw = self.hw.read_yaw()
        target = now_yaw - degrees
        if target >= 360:
            target -= 360.0
        elif target < 0:
            target += 360.0
        self.ctx.rotate_target_yaw = target
        self.ctx.rotate_active = 1

    def tick_rotate_to_target(self, tol=5.0):
        if not self.ctx.rotate_active:
            return True

        err = self._wrapped_delta(self.ctx.rotate_target_yaw, self.hw.read_yaw())
        self.ctx.move.tar_spd_x = 0.0
        self.ctx.move.tar_spd_y = 0.0
        self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)

        if self._shortest_abs_delta(self.ctx.rotate_target_yaw, self.hw.read_yaw()) <= tol and self._is_low_speed(35):
            self.ctx.rotate_active = 0
            self.ctx.move.tar_spd_z = 0.0
            return True
        return False

    # ---------------- 底层 5ms 节拍 ----------------
    def pit_5ms_step(self):
        self._update_encoder()
        yaw_rate = self.hw.read_gyro_z()
        vz = gyro_ctrl(self.ctx.gyro_loop, self.ctx.move.tar_spd_z - yaw_rate)
        calc_wheel_spd(self.ctx.move, self.ctx.move.tar_spd_x, self.ctx.move.tar_spd_y, vz)
        self._apply_motor_ctrl()
        self.ctx.state_ticks += 1

    # ---------------- 协同状态机 ----------------
    def search_target(self):
        self.ctx.move.tar_spd_x = self.ctx.search_speed
        self.ctx.move.tar_spd_y = 0.0
        self.ctx.move.tar_spd_z = 0.0

        if self.ctx.target_found:
            self._load_target_profile()
            self._stop_motion()
            self.ctx.state_ticks = 0
            self.ctx.coop_state = CoopState.ALIGN_TARGET

    def align_target(self):
        if not self.ctx.target_found:
            self.ctx.coop_state = CoopState.SEARCH_TARGET
            return

        self.ctx.move.tar_spd_x = self.ctx.target_y * self.ctx.align_gain_x
        self.ctx.move.tar_spd_y = -self.ctx.target_x * self.ctx.align_gain_y
        self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, self.ctx.target_x, self.ctx.motor_slow_flag)

        if abs(self.ctx.target_x) < self.ctx.align_tol and abs(self.ctx.target_y) < self.ctx.align_tol and self._is_low_speed(45):
            self._stop_motion()
            self.ctx.state_ticks = 0
            self.ctx.coop_state = CoopState.WAIT_PARTNER

    def wait_partner(self):
        self._stop_motion()

        # 这里可以向队友发“我已就位”标志位
        self.hw.uart_write_byte("art_detect", 0x31)

        if self.ctx.partner_ready:
            self.ctx.target_push_yaw = self.hw.read_yaw()
            self.ctx.state_ticks = 0
            self.ctx.coop_state = CoopState.COOP_ROTATE
        elif self.ctx.state_ticks > 600:
            self.ctx.task_failed = 1
            self.ctx.coop_state = CoopState.FAIL_SAFE

    def coop_rotate(self):
        if not self.ctx.rotate_active:
            self.start_rotate_by(self.ctx.rotate_plan_deg)
            return

        if self.tick_rotate_to_target(tol=6.0):
            self.ctx.state_ticks = 0
            self.ctx.push_ticks = 0
            self.ctx.coop_state = CoopState.COOP_PUSH

    def coop_push(self):
        self.ctx.push_ticks += 1

        err = self._wrapped_delta(self.ctx.target_push_yaw, self.hw.read_yaw())
        self.ctx.move.tar_spd_x = self.ctx.push_speed
        self.ctx.move.tar_spd_y = 0.0
        self.ctx.move.tar_spd_z = turn_ctrl(self.ctx.turn_loop, err, self.ctx.motor_slow_flag)

        if self.ctx.partner_state == PartnerState.ERROR:
            self.ctx.task_failed = 1
            self.ctx.coop_state = CoopState.FAIL_SAFE
            return

        if self.ctx.partner_state == PartnerState.DONE or self.ctx.push_ticks > self.ctx.push_timeout:
            self._stop_motion()
            self.ctx.verify_ticks = 0
            self.ctx.coop_state = CoopState.VERIFY_FINISH

    def verify_finish(self):
        self.ctx.verify_ticks += 1
        self._stop_motion()

        if self.ctx.verify_ticks > 30:
            self.ctx.task_done = 1
            self.ctx.return_ticks = 0
            self.ctx.coop_state = CoopState.RETURN_ROUTE

    def return_route(self):
        self.ctx.return_ticks += 1
        self.ctx.move.tar_spd_x = self.ctx.return_speed
        self.ctx.move.tar_spd_y = 0.0
        self.ctx.move.tar_spd_z = 0.0

        if self.ctx.return_ticks > 100:
            self._stop_motion()

    def fail_safe(self):
        self._stop_motion()
        self.reset_speed_pid_output()

    def step(self):
        if self.ctx.coop_state == CoopState.SEARCH_TARGET:
            self.search_target()
        elif self.ctx.coop_state == CoopState.ALIGN_TARGET:
            self.align_target()
        elif self.ctx.coop_state == CoopState.WAIT_PARTNER:
            self.wait_partner()
        elif self.ctx.coop_state == CoopState.COOP_ROTATE:
            self.coop_rotate()
        elif self.ctx.coop_state == CoopState.COOP_PUSH:
            self.coop_push()
        elif self.ctx.coop_state == CoopState.VERIFY_FINISH:
            self.verify_finish()
        elif self.ctx.coop_state == CoopState.RETURN_ROUTE:
            self.return_route()
        elif self.ctx.coop_state == CoopState.FAIL_SAFE:
            self.fail_safe()

    def get_debug_snapshot(self):
        return {
            "coop_state": self.ctx.coop_state,
            "target_found": self.ctx.target_found,
            "target_x": self.ctx.target_x,
            "target_y": self.ctx.target_y,
            "target_label": self.ctx.target_label,
            "partner_state": self.ctx.partner_state,
            "enc_fl": self.enc_fl,
            "enc_fr": self.enc_fr,
            "enc_b": self.enc_b,
            "tar_x": self.ctx.move.tar_spd_x,
            "tar_y": self.ctx.move.tar_spd_y,
            "tar_z": self.ctx.move.tar_spd_z,
            "wheel_fl": self.ctx.move.speed_fl,
            "wheel_fr": self.ctx.move.speed_fr,
            "wheel_b": self.ctx.move.speed_b,
            "task_done": self.ctx.task_done,
            "task_failed": self.ctx.task_failed,
            "rotate_plan_deg": self.ctx.rotate_plan_deg,
            "push_timeout": self.ctx.push_timeout,
        }
