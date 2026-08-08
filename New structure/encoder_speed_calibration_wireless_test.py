from machine import Pin
from micropython import const
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker
import gc
import utime

import config as cfg
from hardware import Motor
from models import MoveBase, SpeedPID
from move_base import calc_wheel_spd
import pid as pid_mod


_MODE_OPEN = const(0)
_MODE_CLOSED = const(1)
_MODE_GROUND = const(2)
_MODE_COUNT = const(3)

_WHEEL_FL = const(0)
_WHEEL_FR = const(1)
_WHEEL_B = const(2)

_DIR_SETTLE_MS = const(20)
_STOP_MS = const(300)
_OPEN_RUN_MS = const(800)
_OPEN_WARMUP_MS = const(200)
_CLOSED_RUN_MS = const(1000)
_CLOSED_LOG_MS = const(100)
_GROUND_RUN_MS = const(1000)
_GROUND_STOP_MS = const(1000)
_GROUND_LOG_MS = const(100)
_GROUND_BASE_VX = 22.0
_CLOSED_KP = 300.0
_CLOSED_KI = 2.0
_GROUND_KP = 1200.0
_GROUND_KI = 3.0
_GROUND_BASE_PWM = const(30000)
_LOG_BUF_SIZE = const(192)

_OPEN_PWM_LEVELS = (6000, 12000, 18000, 24000, 30000)
_CLOSED_TARGETS = (10, 20, 30, 45, 60)
_GROUND_DIVISORS = (64,)

_pit_flag = False


def _pit_handler(_):
    global _pit_flag
    _pit_flag = True


def _put_int(buf, pos, value):
    value = int(value)
    if value < 0:
        buf[pos] = 45
        pos += 1
        value = -value

    start = pos
    if value == 0:
        buf[pos] = 48
        pos += 1
    else:
        while value:
            buf[pos] = 48 + value % 10
            value //= 10
            pos += 1
        end = pos - 1
        while start < end:
            temp = buf[start]
            buf[start] = buf[end]
            buf[end] = temp
            start += 1
            end -= 1

    buf[pos] = 32
    return pos + 1


def _mean(total, count):
    if count <= 0:
        return 0
    half = count >> 1
    if total >= 0:
        return (total + half) // count
    return -((-total + half) // count)


def _normalize(raw_value, dt_ms):
    value = int(raw_value) * cfg.TICK_PERIOD_MS
    half = dt_ms >> 1
    if value >= 0:
        return (value + half) // dt_ms
    return -((-value + half) // dt_ms)


def _reset_pid(pid):
    pid.output = 0.0
    pid.err = 0.0
    pid.err_last = 0.0
    pid.tar_spd_last = 0.0
    pid.delta_tar = 0.0
    pid.delta_tar_last = 0.0
    pid.delta_ud = 0.0


def _smooth_pwm(target, last):
    delta = target - last
    if delta > cfg.MAX_PWM_CHANGE:
        target = last + cfg.MAX_PWM_CHANGE
    elif delta < -cfg.MAX_PWM_CHANGE:
        target = last - cfg.MAX_PWM_CHANGE
    value = int(
        last * (1.0 - cfg.PWM_SMOOTH_FACTOR)
        + target * cfg.PWM_SMOOTH_FACTOR
    )
    if value > cfg.MOTOR_DUTY_MAX:
        return cfg.MOTOR_DUTY_MAX
    if value < -cfg.MOTOR_DUTY_MAX:
        return -cfg.MOTOR_DUTY_MAX
    return value


def _apply_min_duty(value):
    if 0 < value < cfg.MOTOR_DUTY_MIN:
        return cfg.MOTOR_DUTY_MIN
    if -cfg.MOTOR_DUTY_MIN < value < 0:
        return -cfg.MOTOR_DUTY_MIN
    return value


class EncoderSpeedCalibration:
    def __init__(self):
        gc.collect()
        pid_mod.PWM_MAX = cfg.PWM_MAX

        self.wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
        self.log_buf = bytearray(_LOG_BUF_SIZE)

        self.motor_fl = Motor(
            cfg.MOTOR_FL_PH,
            cfg.MOTOR_FL_PWM,
            freq=cfg.MOTOR_FREQ,
            invert=cfg.MOTOR_FL_INVERT,
        )
        self.motor_fr = Motor(
            cfg.MOTOR_FR_PH,
            cfg.MOTOR_FR_PWM,
            freq=cfg.MOTOR_FREQ,
            invert=cfg.MOTOR_FR_INVERT,
        )
        self.motor_b = Motor(
            cfg.MOTOR_B_PH,
            cfg.MOTOR_B_PWM,
            freq=cfg.MOTOR_FREQ,
            invert=cfg.MOTOR_B_INVERT,
        )
        self.motors = (self.motor_fl, self.motor_fr, self.motor_b)

        self.enc_fl = encoder(
            cfg.ENC_FL_DIR, cfg.ENC_FL_PULSE, cfg.ENC_FL_INVERT
        )
        self.enc_fr = encoder(
            cfg.ENC_FR_DIR, cfg.ENC_FR_PULSE, cfg.ENC_FR_INVERT
        )
        self.enc_b = encoder(
            cfg.ENC_B_DIR, cfg.ENC_B_PULSE, cfg.ENC_B_INVERT
        )

        self.pid_fl = SpeedPID()
        self.pid_fr = SpeedPID()
        self.pid_b = SpeedPID()
        self.pid_fl.init_c()
        self.pid_fr.init_c()
        self.pid_b.init_c()
        self.pids = (self.pid_fl, self.pid_fr, self.pid_b)
        self.move = MoveBase()

        self.key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
        self.key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)
        self.key_mode = Pin(cfg.BTN_MODE_PIN, Pin.IN, Pin.PULL_UP)
        self.led = Pin(
            cfg.LED_HB_PIN,
            Pin.OUT,
            pull=Pin.PULL_UP_47K,
            value=True,
        )
        self.led_open = Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
        self.led_closed = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
        self.led_ground = Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)

        self.pit = ticker(1)
        self.pit.capture_list(self.enc_fl, self.enc_fr, self.enc_b)
        self.pit.callback(_pit_handler)
        self.pit.start(cfg.TICK_PERIOD_MS)

        self.mode = _MODE_OPEN
        self.last_start = 1
        self.last_mode = 1
        self.last_encoder_ms = 0
        self.encoder_dt_ms = cfg.TICK_PERIOD_MS
        self.e_fl = 0
        self.e_fr = 0
        self.e_b = 0
        self.exit_requested = False
        self.running = False
        self.update_mode_leds()

    def send_static(self, text):
        try:
            self.wireless.send_str(text)
            self.wireless.send_str("\r\n")
        except Exception:
            pass

    def send_mode(self):
        buf = self.log_buf
        buf[0] = 77
        buf[1] = 32
        pos = _put_int(buf, 2, self.mode)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

    def send_open(self, wheel, pwm, direction, count, total, minimum, maximum, zeros, dt_max):
        buf = self.log_buf
        buf[0] = 79
        buf[1] = 32
        pos = _put_int(buf, 2, wheel)
        pos = _put_int(buf, pos, pwm)
        pos = _put_int(buf, pos, direction)
        pos = _put_int(buf, pos, count)
        pos = _put_int(buf, pos, _mean(total, count))
        pos = _put_int(buf, pos, minimum)
        pos = _put_int(buf, pos, maximum)
        pos = _put_int(buf, pos, zeros)
        pos = _put_int(buf, pos, dt_max)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

    def send_closed(self, elapsed, wheel, target, actual, pwm, reverse, dt_ms):
        buf = self.log_buf
        buf[0] = 67
        buf[1] = 32
        pos = _put_int(buf, 2, elapsed)
        pos = _put_int(buf, pos, wheel)
        pos = _put_int(buf, pos, target)
        pos = _put_int(buf, pos, actual)
        pos = _put_int(buf, pos, pwm)
        pos = _put_int(buf, pos, target - actual)
        pos = _put_int(buf, pos, reverse)
        pos = _put_int(buf, pos, dt_ms)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

    def send_summary(self, wheel, target, count, enc_total, pwm_total, enc_min, enc_max, zeros, reverses):
        buf = self.log_buf
        buf[0] = 83
        buf[1] = 32
        pos = _put_int(buf, 2, wheel)
        pos = _put_int(buf, pos, target)
        pos = _put_int(buf, pos, count)
        pos = _put_int(buf, pos, _mean(enc_total, count))
        pos = _put_int(buf, pos, _mean(pwm_total, count))
        pos = _put_int(buf, pos, enc_min)
        pos = _put_int(buf, pos, enc_max)
        pos = _put_int(buf, pos, zeros)
        pos = _put_int(buf, pos, reverses)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

    def send_ground(
        self,
        elapsed,
        divisor,
        t_fl,
        t_fr,
        t_b,
        a_fl,
        a_fr,
        a_b,
        pwm_fl,
        pwm_fr,
        pwm_b,
    ):
        buf = self.log_buf
        buf[0] = 70
        buf[1] = 32
        pos = _put_int(buf, 2, elapsed)
        pos = _put_int(buf, pos, divisor)
        pos = _put_int(buf, pos, t_fl)
        pos = _put_int(buf, pos, t_fr)
        pos = _put_int(buf, pos, t_b)
        pos = _put_int(buf, pos, self.e_fl)
        pos = _put_int(buf, pos, self.e_fr)
        pos = _put_int(buf, pos, self.e_b)
        pos = _put_int(buf, pos, a_fl * 10.0)
        pos = _put_int(buf, pos, a_fr * 10.0)
        pos = _put_int(buf, pos, a_b * 10.0)
        pos = _put_int(buf, pos, pwm_fl)
        pos = _put_int(buf, pos, pwm_fr)
        pos = _put_int(buf, pos, pwm_b)
        pos = _put_int(buf, pos, self.encoder_dt_ms)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

    def update_mode_leds(self):
        self.led_open.value(1 if self.mode == _MODE_OPEN else 0)
        self.led_closed.value(1 if self.mode == _MODE_CLOSED else 0)
        self.led_ground.value(1 if self.mode == _MODE_GROUND else 0)

    def check_exit(self):
        if self.key_exit.value() == 0:
            self.exit_requested = True
            raise KeyboardInterrupt

    def stop_all(self):
        self.motor_fl.duty(0)
        self.motor_fr.duty(0)
        self.motor_b.duty(0)

    def reset_pids(self):
        _reset_pid(self.pid_fl)
        _reset_pid(self.pid_fr)
        _reset_pid(self.pid_b)

    def set_pid_gains(self, kp, ki):
        index = 0
        while index < 3:
            speed_pid = self.pids[index]
            speed_pid.kp = kp
            speed_pid.ki = ki
            speed_pid.init_c()
            _reset_pid(speed_pid)
            index += 1

    def prepare_motor_direction(self, motor, value):
        motor.pwm.duty_u16(0)
        if value == 0:
            return
        direction = 1 if value > 0 else 0
        if motor.invert:
            direction = 1 - direction
        motor.ph.value(direction)

    def prepare_three_directions(self, t_fl, t_fr, t_b):
        self.stop_all()
        self.prepare_motor_direction(self.motor_fl, t_fl)
        self.prepare_motor_direction(self.motor_fr, t_fr)
        self.prepare_motor_direction(self.motor_b, t_b)
        utime.sleep_ms(_DIR_SETTLE_MS)

    def read_encoder_tick(self):
        now = utime.ticks_ms()
        raw_fl = self.enc_fl.get()
        raw_fr = self.enc_fr.get()
        raw_b = self.enc_b.get()
        if self.last_encoder_ms:
            dt_ms = utime.ticks_diff(now, self.last_encoder_ms)
            if dt_ms <= 0:
                dt_ms = cfg.TICK_PERIOD_MS
        else:
            dt_ms = cfg.TICK_PERIOD_MS
        self.last_encoder_ms = now
        self.encoder_dt_ms = dt_ms
        self.e_fl = _normalize(raw_fl, dt_ms)
        self.e_fr = _normalize(raw_fr, dt_ms)
        self.e_b = _normalize(raw_b, dt_ms)

    def wait_tick(self):
        global _pit_flag
        while not _pit_flag:
            self.check_exit()
            utime.sleep_ms(1)
        _pit_flag = False
        self.read_encoder_tick()

    def wait_stopped(self, duration_ms):
        self.stop_all()
        self.last_encoder_ms = 0
        end_ms = utime.ticks_add(utime.ticks_ms(), duration_ms)
        while utime.ticks_diff(end_ms, utime.ticks_ms()) > 0:
            self.wait_tick()
        gc.collect()

    def encoder_value(self, wheel):
        if wheel == _WHEEL_FL:
            return self.e_fl
        if wheel == _WHEEL_FR:
            return self.e_fr
        return self.e_b

    def run_open_phase(self, wheel, pwm):
        motor = self.motors[wheel]
        self.stop_all()
        self.prepare_motor_direction(motor, pwm)
        utime.sleep_ms(_DIR_SETTLE_MS)
        motor.pwm.duty_u16(abs(int(pwm)))

        start_ms = utime.ticks_ms()
        self.last_encoder_ms = 0
        count = 0
        total = 0
        minimum = 0
        maximum = 0
        zeros = 0
        dt_max = 0

        while utime.ticks_diff(utime.ticks_ms(), start_ms) < _OPEN_RUN_MS:
            self.wait_tick()
            elapsed = utime.ticks_diff(utime.ticks_ms(), start_ms)
            if elapsed < _OPEN_WARMUP_MS:
                continue
            value = self.encoder_value(wheel)
            if count == 0:
                minimum = value
                maximum = value
            elif value < minimum:
                minimum = value
            elif value > maximum:
                maximum = value
            count += 1
            total += value
            if value == 0:
                zeros += 1
            if self.encoder_dt_ms > dt_max:
                dt_max = self.encoder_dt_ms

        self.stop_all()
        self.send_open(
            wheel, abs(pwm), 1 if pwm > 0 else -1,
            count, total, minimum, maximum, zeros, dt_max,
        )

    def run_open_test(self):
        self.send_static("OPEN LIFT WHEELS")
        wheel = 0
        while wheel < 3:
            direction = 1
            while direction >= -1:
                for level in _OPEN_PWM_LEVELS:
                    self.run_open_phase(wheel, level * direction)
                    self.wait_stopped(_STOP_MS)
                direction -= 2
            wheel += 1
        self.send_static("OPEN DONE")

    def run_pid_channel(self, wheel, target, actual, last_pwm):
        pid = self.pids[wheel]
        cmd = pid_mod.speed_ctrl(pid, actual, target)
        reverse = 0
        if target > 0 and cmd < 0:
            cmd = 0
            reverse = 1
        elif target < 0 and cmd > 0:
            cmd = 0
            reverse = 1
        pwm = _smooth_pwm(cmd, last_pwm)
        pwm = _apply_min_duty(pwm)
        self.motors[wheel].duty(pwm)
        return pwm, reverse

    def run_closed_phase(self, wheel, target):
        self.stop_all()
        _reset_pid(self.pids[wheel])
        self.prepare_motor_direction(self.motors[wheel], target)
        utime.sleep_ms(_DIR_SETTLE_MS)
        self.last_encoder_ms = 0

        start_ms = utime.ticks_ms()
        next_log_ms = start_ms
        last_pwm = 0
        count = 0
        enc_total = 0
        pwm_total = 0
        enc_min = 0
        enc_max = 0
        zeros = 0
        reverses = 0

        while utime.ticks_diff(utime.ticks_ms(), start_ms) < _CLOSED_RUN_MS:
            self.wait_tick()
            actual = self.encoder_value(wheel)
            last_pwm, reverse = self.run_pid_channel(
                wheel, target, actual, last_pwm
            )
            count += 1
            enc_total += actual
            pwm_total += last_pwm
            reverses += reverse
            if count == 1:
                enc_min = actual
                enc_max = actual
            elif actual < enc_min:
                enc_min = actual
            elif actual > enc_max:
                enc_max = actual
            if actual == 0:
                zeros += 1

            now_ms = utime.ticks_ms()
            if utime.ticks_diff(now_ms, next_log_ms) >= 0:
                next_log_ms = utime.ticks_add(now_ms, _CLOSED_LOG_MS)
                self.send_closed(
                    utime.ticks_diff(now_ms, start_ms), wheel, target,
                    actual, last_pwm, reverse, self.encoder_dt_ms,
                )

        self.stop_all()
        self.send_summary(
            wheel, target, count, enc_total, pwm_total,
            enc_min, enc_max, zeros, reverses,
        )

    def run_closed_test(self):
        self.set_pid_gains(_CLOSED_KP, _CLOSED_KI)
        self.send_static("CLOSED LIFT WHEELS")
        wheel = 0
        while wheel < 3:
            direction = 1
            while direction >= -1:
                for target_abs in _CLOSED_TARGETS:
                    self.run_closed_phase(wheel, target_abs * direction)
                    self.wait_stopped(_STOP_MS)
                direction -= 2
            wheel += 1
        self.send_static("CLOSED DONE")

    def run_three_pid(
        self,
        t_fl,
        t_fr,
        t_b,
        a_fl,
        a_fr,
        a_b,
        last_fl,
        last_fr,
        last_b,
    ):
        u_fl = pid_mod.speed_ctrl(self.pid_fl, a_fl, t_fl)
        u_fr = pid_mod.speed_ctrl(self.pid_fr, a_fr, t_fr)
        u_b = pid_mod.speed_ctrl(self.pid_b, a_b, t_b)

        if t_fl > 0:
            u_fl += _GROUND_BASE_PWM
        elif t_fl < 0:
            u_fl -= _GROUND_BASE_PWM
        if t_fr > 0:
            u_fr += _GROUND_BASE_PWM
        elif t_fr < 0:
            u_fr -= _GROUND_BASE_PWM
        if t_b > 0:
            u_b += _GROUND_BASE_PWM
        elif t_b < 0:
            u_b -= _GROUND_BASE_PWM

        if t_fl > 0 and u_fl < 0 or t_fl < 0 and u_fl > 0:
            u_fl = 0
        if t_fr > 0 and u_fr < 0 or t_fr < 0 and u_fr > 0:
            u_fr = 0
        if t_b > 0 and u_b < 0 or t_b < 0 and u_b > 0:
            u_b = 0

        last_fl = _apply_min_duty(_smooth_pwm(u_fl, last_fl))
        last_fr = _apply_min_duty(_smooth_pwm(u_fr, last_fr))
        last_b = _apply_min_duty(_smooth_pwm(u_b, last_b))
        self.motor_fl.duty(last_fl)
        self.motor_fr.duty(last_fr)
        self.motor_b.duty(last_b)
        return last_fl, last_fr, last_b

    def run_ground_phase(self, divisor):
        calc_wheel_spd(self.move, _GROUND_BASE_VX, 0.0, 0.0)
        t_fl = self.move.speed_fl
        t_fr = self.move.speed_fr
        t_b = self.move.speed_b
        inv_divisor = 1.0 / divisor

        self.reset_pids()
        self.prepare_three_directions(t_fl, t_fr, t_b)
        self.last_encoder_ms = 0
        last_fl = 0
        last_fr = 0
        last_b = 0
        start_ms = utime.ticks_ms()
        next_log_ms = start_ms

        while utime.ticks_diff(utime.ticks_ms(), start_ms) < _GROUND_RUN_MS:
            self.wait_tick()
            a_fl = self.e_fl * inv_divisor
            a_fr = self.e_fr * inv_divisor
            a_b = self.e_b * inv_divisor
            last_fl, last_fr, last_b = self.run_three_pid(
                t_fl,
                t_fr,
                t_b,
                a_fl,
                a_fr,
                a_b,
                last_fl,
                last_fr,
                last_b,
            )
            now_ms = utime.ticks_ms()
            if utime.ticks_diff(now_ms, next_log_ms) >= 0:
                next_log_ms = utime.ticks_add(now_ms, _GROUND_LOG_MS)
                self.send_ground(
                    utime.ticks_diff(now_ms, start_ms),
                    divisor,
                    t_fl,
                    t_fr,
                    t_b,
                    a_fl,
                    a_fr,
                    a_b,
                    last_fl,
                    last_fr,
                    last_b,
                )

        self.stop_all()

    def run_ground_test(self):
        self.set_pid_gains(_GROUND_KP, _GROUND_KI)
        self.send_static("GROUND CLEAR LONG PATH")
        self.send_static("GROUND DIV64 BASE30000")
        self.send_static("F MS DIV T3 RAW3 E10_3 PWM3 DT")
        try:
            for divisor in _GROUND_DIVISORS:
                self.run_ground_phase(divisor)
                self.wait_stopped(_GROUND_STOP_MS)
            self.send_static("GROUND DONE")
        finally:
            self.set_pid_gains(_CLOSED_KP, _CLOSED_KI)

    def run_selected(self):
        self.running = True
        try:
            if self.mode == _MODE_OPEN:
                self.run_open_test()
            elif self.mode == _MODE_CLOSED:
                self.run_closed_test()
            else:
                self.run_ground_test()
        finally:
            self.stop_all()
            self.running = False
            gc.collect()

    def poll_keys(self):
        start = self.key_start.value()
        mode = self.key_mode.value()

        if self.last_mode == 1 and mode == 0 and not self.running:
            utime.sleep_ms(20)
            if self.key_mode.value() == 0:
                self.mode = (self.mode + 1) % _MODE_COUNT
                self.update_mode_leds()
                self.send_mode()

        if self.last_start == 1 and start == 0 and not self.running:
            utime.sleep_ms(20)
            if self.key_start.value() == 0:
                self.run_selected()

        self.last_start = start
        self.last_mode = mode

    def run(self):
        self.send_static("ENC SPEED CAL")
        self.send_static("C14 MODE 0=OPEN 1=CLOSED 2=GROUND")
        self.send_static("C9 RUN C8 STOP")
        self.send_static("KC 300 2 KG 1200 3 T 5")
        self.send_mode()

        while not self.exit_requested:
            self.check_exit()
            self.poll_keys()
            self.led.toggle()
            utime.sleep_ms(100)


tester = None

try:
    tester = EncoderSpeedCalibration()
    tester.run()
except KeyboardInterrupt:
    pass
finally:
    if tester is not None:
        tester.stop_all()
        try:
            tester.pit.stop()
        except Exception:
            pass
        tester.send_static("EXIT")
