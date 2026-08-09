from machine import Pin
from micropython import const
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker
import gc
import utime

import config as cfg
from hardware import Motor
from lsm6dsv16x_gyro_runtime import LSM6DSV16XYawRuntime
from models import MoveBase, SpeedPID
from move_base import calc_wheel_spd
import pid as pid_mod


_MODE_KICK_HOLD = const(0)
_MODE_KICK_HOLD_GYRO = const(1)
_MODE_COUNT = const(2)

_PROFILE_COUNT = const(6)
_RUN_MS = const(1500)
_SETTLE_MS = const(800)
_GAP_MS = const(500)
_LOG_MS = const(20)
_LOG_BUF_SIZE = const(192)

_RATE_EMA_ALPHA = 0.5
_RATE_STOP_KP = 0.03
_RATE_STOP_LIMIT = 5.0
_RATE_STOP_DEADBAND = 8.0

_DIRECT_START_SPEED = 0.3
_DIRECT_START_CONFIRM_TICKS = const(2)
_DIRECT_KICK_TIMEOUT_MS = const(250)
_DIRECT_RATE_FF_POS = const(90)
_DIRECT_RATE_FF_NEG = const(150)
_DIRECT_RATE_KP = const(10)
_DIRECT_RATE_KI = 0.1
_DIRECT_RATE_I_LIMIT = const(1500)
_DIRECT_RATE_LIMIT = const(2000)
_HOLD_FL_POS = const(3800)
_HOLD_FL_NEG = const(4000)
_HOLD_FR_POS = const(4000)
_HOLD_FR_NEG = const(4400)
_HOLD_B_POS = const(5000)
_HOLD_B_NEG = const(4800)
_KICK_FL_POS = const(4500)
_KICK_FL_NEG = const(6900)
_KICK_FR_POS = const(6700)
_KICK_FR_NEG = const(7900)
_KICK_B_POS = const(8100)
_KICK_B_NEG = const(6700)

_GYRO_SIGN = 1.0
_GYRO_SCALE = -1.0
_GYRO_DEADBAND = 0.8
_CAL_SAMPLES = const(1000)
_CAL_DELAY_MS = const(2)
_CAL_SETTLE_MS = const(700)

_pit_flag = False


def _pit_handler(_):
    global _pit_flag
    _pit_flag = True


def _profile_rate(profile):
    if profile & 1:
        return -35.0
    return 35.0


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


def _reset_speed_pid(pid):
    pid.output = 0.0
    pid.err = 0.0
    pid.err_last = 0.0
    pid.tar_spd_last = 0.0
    pid.delta_tar = 0
    pid.delta_tar_last = 0
    pid.delta_ud = 0
    pid.param_a = 0
    pid.param_b = 0.0


def _clamp_pwm(value):
    if value > cfg.MOTOR_DUTY_MAX:
        return cfg.MOTOR_DUTY_MAX
    if value < -cfg.MOTOR_DUTY_MAX:
        return -cfg.MOTOR_DUTY_MAX
    return int(value)


def _smooth_pwm(target, last):
    if target == 0:
        return 0
    delta = target - last
    if delta > cfg.MAX_PWM_CHANGE:
        target = last + cfg.MAX_PWM_CHANGE
    elif delta < -cfg.MAX_PWM_CHANGE:
        target = last - cfg.MAX_PWM_CHANGE
    value = int(
        last * (1.0 - cfg.PWM_SMOOTH_FACTOR)
        + target * cfg.PWM_SMOOTH_FACTOR
    )
    value = _clamp_pwm(value)
    if 0 < value < cfg.MOTOR_DUTY_MIN:
        return cfg.MOTOR_DUTY_MIN
    if -cfg.MOTOR_DUTY_MIN < value < 0:
        return -cfg.MOTOR_DUTY_MIN
    return value


def _direct_pwm_base(wheel, positive, kick):
    if wheel == 0:
        if positive:
            return _KICK_FL_POS if kick else _HOLD_FL_POS
        return _KICK_FL_NEG if kick else _HOLD_FL_NEG
    if wheel == 1:
        if positive:
            return _KICK_FR_POS if kick else _HOLD_FR_POS
        return _KICK_FR_NEG if kick else _HOLD_FR_NEG
    if positive:
        return _KICK_B_POS if kick else _HOLD_B_POS
    return _KICK_B_NEG if kick else _HOLD_B_NEG


class GyroRateLoopTest:
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

        self.enc_fl = encoder(
            cfg.ENC_FL_DIR, cfg.ENC_FL_PULSE, cfg.ENC_FL_INVERT
        )
        self.enc_fr = encoder(
            cfg.ENC_FR_DIR, cfg.ENC_FR_PULSE, cfg.ENC_FR_INVERT
        )
        self.enc_b = encoder(
            cfg.ENC_B_DIR, cfg.ENC_B_PULSE, cfg.ENC_B_INVERT
        )

        self.imu = LSM6DSV16XYawRuntime(
            sign=_GYRO_SIGN,
            offset_z=0.0,
            scale=_GYRO_SCALE,
            deadband_dps=_GYRO_DEADBAND,
            tick_period_ms=cfg.TICK_PERIOD_MS,
        )

        self.pid_fl = SpeedPID()
        self.pid_fr = SpeedPID()
        self.pid_b = SpeedPID()
        self.pid_fl.init_c()
        self.pid_fr.init_c()
        self.pid_b.init_c()
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
        self.led_0 = Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
        self.led_1 = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
        self.led_2 = Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)

        self.pit = ticker(1)
        self.pit.capture_list(self.enc_fl, self.enc_fr, self.enc_b)
        self.pit.callback(_pit_handler)
        self.pit.start(cfg.TICK_PERIOD_MS)

        self.mode = _MODE_KICK_HOLD
        self.last_start = 1
        self.last_mode = 1
        self.last_encoder_ms = 0
        self.encoder_dt_ms = cfg.TICK_PERIOD_MS
        self.e_fl = 0.0
        self.e_fr = 0.0
        self.e_b = 0.0
        self.last_pwm_fl = 0
        self.last_pwm_fr = 0
        self.last_pwm_b = 0
        self.gyro_raw = 0.0
        self.gyro_filt = 0.0
        self.filter_ready = False
        self.vz_cmd = 0.0
        self.rate_integral = 0.0
        self.direct_was_active = False
        self.running = False
        self.exit_requested = False
        self.update_leds()

    def send_static(self, value):
        try:
            self.wireless.send_str(value)
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

    def send_profile(self, profile, rate):
        buf = self.log_buf
        buf[0] = 80
        buf[1] = 32
        pos = _put_int(buf, 2, profile)
        pos = _put_int(buf, pos, rate)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

    def send_log(self, profile, elapsed, rate_cmd):
        buf = self.log_buf
        buf[0] = 82
        buf[1] = 32
        pos = _put_int(buf, 2, self.mode)
        pos = _put_int(buf, pos, profile)
        pos = _put_int(buf, pos, elapsed)
        pos = _put_int(buf, pos, rate_cmd * 10.0)
        pos = _put_int(buf, pos, self.gyro_raw * 10.0)
        pos = _put_int(buf, pos, self.gyro_filt * 10.0)
        pos = _put_int(buf, pos, self.vz_cmd)
        pos = _put_int(buf, pos, self.move.speed_fl * 10.0)
        pos = _put_int(buf, pos, self.move.speed_fr * 10.0)
        pos = _put_int(buf, pos, self.move.speed_b * 10.0)
        pos = _put_int(buf, pos, self.e_fl * 10.0)
        pos = _put_int(buf, pos, self.e_fr * 10.0)
        pos = _put_int(buf, pos, self.e_b * 10.0)
        pos = _put_int(buf, pos, self.last_pwm_fl)
        pos = _put_int(buf, pos, self.last_pwm_fr)
        pos = _put_int(buf, pos, self.last_pwm_b)
        pos = _put_int(buf, pos, self.pid_fl.param_a)
        pos = _put_int(buf, pos, self.pid_fr.param_a)
        pos = _put_int(buf, pos, self.pid_b.param_a)
        pos = _put_int(buf, pos, self.pid_fl.delta_tar)
        pos = _put_int(buf, pos, self.pid_fr.delta_tar)
        pos = _put_int(buf, pos, self.pid_b.delta_tar)
        pos = _put_int(buf, pos, self.encoder_dt_ms)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

    def update_leds(self):
        self.led_0.value(1 if self.mode == _MODE_KICK_HOLD else 0)
        self.led_1.value(1 if self.mode == _MODE_KICK_HOLD_GYRO else 0)
        self.led_2.value(0)

    def stop_all(self):
        self.motor_fl.duty(0)
        self.motor_fr.duty(0)
        self.motor_b.duty(0)
        self.last_pwm_fl = 0
        self.last_pwm_fr = 0
        self.last_pwm_b = 0

    def reset_controllers(self):
        _reset_speed_pid(self.pid_fl)
        _reset_speed_pid(self.pid_fr)
        _reset_speed_pid(self.pid_b)
        self.filter_ready = False
        self.gyro_filt = 0.0
        self.vz_cmd = 0.0
        self.rate_integral = 0.0
        self.direct_was_active = False

    def check_exit(self):
        if self.key_exit.value() == 0:
            self.exit_requested = True
            raise KeyboardInterrupt

    def wait_tick(self):
        global _pit_flag
        while not _pit_flag:
            self.check_exit()
            utime.sleep_ms(1)
        _pit_flag = False
        self.read_encoders()

    def read_encoders(self):
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
        scale = cfg.ENCODER_SPEED_SCALE
        self.e_fl = int(raw_fl) * scale
        self.e_fr = int(raw_fr) * scale
        self.e_b = int(raw_b) * scale

    def update_gyro(self):
        self.gyro_raw = self.imu.read_gyro_z()
        if not self.filter_ready:
            self.gyro_filt = self.gyro_raw
            self.filter_ready = True
        else:
            self.gyro_filt += _RATE_EMA_ALPHA * (
                self.gyro_raw - self.gyro_filt
            )

    def stop_rate_ctrl(self):
        self.rate_integral = 0.0
        if -_RATE_STOP_DEADBAND <= self.gyro_filt <= _RATE_STOP_DEADBAND:
            return 0.0
        command = -_RATE_STOP_KP * self.gyro_filt
        if command > _RATE_STOP_LIMIT:
            return _RATE_STOP_LIMIT
        if command < -_RATE_STOP_LIMIT:
            return -_RATE_STOP_LIMIT
        return command

    def direct_rate_ctrl(self, rate_cmd):
        if rate_cmd > 0.0:
            aligned_rate = self.gyro_filt
            target_rate = rate_cmd
            feedforward = _DIRECT_RATE_FF_POS
        else:
            aligned_rate = -self.gyro_filt
            target_rate = -rate_cmd
            feedforward = _DIRECT_RATE_FF_NEG
        error = target_rate - aligned_rate
        integral_last = self.rate_integral
        integral = integral_last + _DIRECT_RATE_KI * error
        if integral > _DIRECT_RATE_I_LIMIT:
            integral = _DIRECT_RATE_I_LIMIT
        elif integral < 0.0:
            integral = 0.0

        command = feedforward + _DIRECT_RATE_KP * error + integral
        if command > _DIRECT_RATE_LIMIT:
            command = _DIRECT_RATE_LIMIT
            if error > 0.0:
                integral = integral_last
        elif command < 0.0:
            command = 0.0

        self.rate_integral = integral
        return command

    def update_stop_motors(self):
        t_fl = self.move.speed_fl
        t_fr = self.move.speed_fr
        t_b = self.move.speed_b
        u_fl = pid_mod.speed_ctrl(self.pid_fl, self.e_fl, t_fl)
        u_fr = pid_mod.speed_ctrl(self.pid_fr, self.e_fr, t_fr)
        u_b = pid_mod.speed_ctrl(self.pid_b, self.e_b, t_b)
        self.last_pwm_fl = _smooth_pwm(u_fl, self.last_pwm_fl)
        self.last_pwm_fr = _smooth_pwm(u_fr, self.last_pwm_fr)
        self.last_pwm_b = _smooth_pwm(u_b, self.last_pwm_b)
        self.motor_fl.duty(self.last_pwm_fl)
        self.motor_fr.duty(self.last_pwm_fr)
        self.motor_b.duty(self.last_pwm_b)

    def direct_wheel_base(self, pid, actual, wheel, positive, elapsed):
        hold = _direct_pwm_base(wheel, positive, False)
        if pid.delta_tar:
            return hold

        aligned_actual = actual if positive else -actual
        if aligned_actual >= _DIRECT_START_SPEED:
            pid.delta_tar_last += 1
            if pid.delta_tar_last >= _DIRECT_START_CONFIRM_TICKS:
                pid.delta_tar = 1
                pid.delta_tar_last = 0
                return hold
        else:
            pid.delta_tar_last = 0

        if elapsed >= _DIRECT_KICK_TIMEOUT_MS:
            return hold
        return _direct_pwm_base(wheel, positive, True)

    def update_direct_motors(self, rate_cmd, elapsed):
        positive = rate_cmd > 0.0
        base_fl = self.direct_wheel_base(
            self.pid_fl, self.e_fl, 0, positive, elapsed
        )
        base_fr = self.direct_wheel_base(
            self.pid_fr, self.e_fr, 1, positive, elapsed
        )
        base_b = self.direct_wheel_base(
            self.pid_b, self.e_b, 2, positive, elapsed
        )

        correction = 0
        if (
            self.mode == _MODE_KICK_HOLD_GYRO
            and self.pid_fl.delta_tar
            and self.pid_fr.delta_tar
            and self.pid_b.delta_tar
        ):
            correction = int(self.direct_rate_ctrl(rate_cmd))
        else:
            self.rate_integral = 0.0
        self.vz_cmd = correction

        sign = 1 if positive else -1
        u_fl = sign * (base_fl + correction)
        u_fr = sign * (base_fr + correction)
        u_b = sign * (base_b + correction)

        self.move.speed_fl = 0.0
        self.move.speed_fr = 0.0
        self.move.speed_b = 0.0
        self.pid_fl.param_a = base_fl
        self.pid_fr.param_a = base_fr
        self.pid_b.param_a = base_b

        self.last_pwm_fl = _smooth_pwm(u_fl, self.last_pwm_fl)
        self.last_pwm_fr = _smooth_pwm(u_fr, self.last_pwm_fr)
        self.last_pwm_b = _smooth_pwm(u_b, self.last_pwm_b)
        self.motor_fl.duty(self.last_pwm_fl)
        self.motor_fr.duty(self.last_pwm_fr)
        self.motor_b.duty(self.last_pwm_b)

    def control_tick(self, rate_cmd, elapsed):
        self.update_gyro()
        if rate_cmd != 0.0:
            self.direct_was_active = True
            self.update_direct_motors(rate_cmd, elapsed)
            return

        if self.direct_was_active:
            _reset_speed_pid(self.pid_fl)
            _reset_speed_pid(self.pid_fr)
            _reset_speed_pid(self.pid_b)
            self.rate_integral = 0.0
            self.direct_was_active = False
        self.vz_cmd = self.stop_rate_ctrl()
        calc_wheel_spd(self.move, 0.0, 0.0, self.vz_cmd)
        self.update_stop_motors()

    def calibrate(self):
        global _pit_flag
        self.stop_all()
        self.send_static("CAL")
        self.pit.stop()
        try:
            self.imu.calibrate_offset(
                samples=_CAL_SAMPLES,
                delay_ms=_CAL_DELAY_MS,
                logger=None,
            )
            self.imu.reset_yaw(0.0)
        finally:
            self.pit.start(cfg.TICK_PERIOD_MS)
        _pit_flag = False
        self.last_encoder_ms = 0
        self.send_static("CAL OK")

    def wait_gap(self):
        self.stop_all()
        self.reset_controllers()
        end_ms = utime.ticks_add(utime.ticks_ms(), _GAP_MS)
        while utime.ticks_diff(end_ms, utime.ticks_ms()) > 0:
            self.wait_tick()
        gc.collect()

    def run_profile(self, profile):
        rate = _profile_rate(profile)
        self.stop_all()
        self.reset_controllers()
        self.last_encoder_ms = 0
        self.imu.reset_yaw(0.0)
        self.send_profile(profile, rate)

        start_ms = utime.ticks_ms()
        next_log_ms = start_ms
        total_ms = _RUN_MS + _SETTLE_MS
        while utime.ticks_diff(utime.ticks_ms(), start_ms) < total_ms:
            self.wait_tick()
            now = utime.ticks_ms()
            elapsed = utime.ticks_diff(now, start_ms)
            rate_cmd = rate if elapsed < _RUN_MS else 0.0
            self.control_tick(rate_cmd, elapsed)
            if utime.ticks_diff(now, next_log_ms) >= 0:
                next_log_ms = utime.ticks_add(now, _LOG_MS)
                self.send_log(profile, elapsed, rate_cmd)

        self.stop_all()

    def run_selected(self):
        self.running = True
        self.send_static("KEEP STILL")
        try:
            utime.sleep_ms(_CAL_SETTLE_MS)
            self.calibrate()
            gc.collect()
            self.send_static("RUN")
            profile = 0
            while profile < _PROFILE_COUNT:
                self.run_profile(profile)
                self.wait_gap()
                profile += 1
            self.send_static("DONE")
        finally:
            self.stop_all()
            self.reset_controllers()
            self.running = False
            gc.collect()

    def poll_keys(self):
        start = self.key_start.value()
        mode = self.key_mode.value()

        if self.last_mode == 1 and mode == 0 and not self.running:
            utime.sleep_ms(20)
            if self.key_mode.value() == 0:
                self.mode = (self.mode + 1) % _MODE_COUNT
                self.update_leds()
                self.send_mode()

        if self.last_start == 1 and start == 0 and not self.running:
            utime.sleep_ms(20)
            if self.key_start.value() == 0:
                self.run_selected()

        self.last_start = start
        self.last_mode = mode

    def run(self):
        self.send_static("RATE FLOOR AB")
        self.send_static("C14 0=HOLD 1=HOLD+GYRO")
        self.send_static("C9 RUN C8 EXIT GROUND")
        self.send_static("P +35 -35 +35 -35 +35 -35")
        self.send_static("H 3800/4000 4000/4400 5000/4800")
        self.send_static("K 4500/6900 6700/7900 8100/6700 V.3 N2 TO250")
        self.send_static("G FF90/150 KP10 KI.1 I1500 U2000")
        self.send_static("R M P MS C10 G10 F10 U T10_3 E10_3 PWM3 B3 S3 DT")
        self.send_mode()

        while not self.exit_requested:
            self.check_exit()
            self.poll_keys()
            self.led.toggle()
            utime.sleep_ms(100)


def main():
    tester = None
    try:
        tester = GyroRateLoopTest()
        gc.collect()
        try:
            gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
        except Exception:
            pass
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


main()
