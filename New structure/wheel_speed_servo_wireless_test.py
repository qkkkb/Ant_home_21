from machine import Pin
from micropython import const
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker
import gc
import utime

import config as cfg
from hardware import Motor
from models import SpeedPID
import pid as pid_mod


_MODE_PID = const(0)
_MODE_FF_PI = const(1)
_MODE_FF_ADAPTIVE = const(2)
_MODE_COUNT = const(3)

_PROFILE_COUNT = const(7)
_RUN_MS = const(1000)
_STOP_MS = const(600)
_LOG_MS = const(20)
_LOG_BUF_SIZE = const(192)

# Candidate controller values. The test compares these against the production PID.
_FF_GAIN = const(1450)
_FB_KP = const(300)
_FB_KI = const(8)
_FB_KI_LOW = const(20)
_LOW_TARGET_MAX = const(7)
_I_LIMIT = const(18000)
_BRAKE_KP = const(1200)
_BRAKE_STOP_SPEED = 1.0

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


def _reset_pid(pid):
    pid.output = 0.0
    pid.err = 0.0
    pid.err_last = 0.0
    pid.tar_spd_last = 0.0
    pid.delta_tar = 0.0
    pid.delta_tar_last = 0.0
    pid.delta_ud = 0.0
    pid.param_a = 0.0
    pid.param_b = 0.0


def _normalize_encoder(raw_value, dt_ms):
    value = int(raw_value) * cfg.TICK_PERIOD_MS
    half_dt = dt_ms >> 1
    if value >= 0:
        return (value + half_dt) // dt_ms
    return -((-value + half_dt) // dt_ms)


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


def _production_ctrl(pid, actual, target):
    pid.param_b = 0.0
    return pid_mod.speed_ctrl(pid, actual, target)


def _candidate_ctrl(pid, actual, target, adaptive_ki):
    if target == 0:
        if adaptive_ki and abs(actual) > _BRAKE_STOP_SPEED:
            pid.output = 0.0
            pid.err = -actual
            pid.err_last = pid.err
            pid.tar_spd_last = 0.0
            pid.param_a = 0.0
            pid.param_b = 0.0
            return _clamp_pwm(-_BRAKE_KP * actual)
        _reset_pid(pid)
        return 0

    if target * pid.tar_spd_last < 0:
        pid.output = 0.0
        pid.param_a = 0.0

    error = target - actual
    integral_last = pid.output
    ki = _FB_KI
    if adaptive_ki and abs(target) <= _LOW_TARGET_MAX:
        ki = _FB_KI_LOW
    integral = integral_last + ki * error
    if integral > _I_LIMIT:
        integral = _I_LIMIT
    elif integral < -_I_LIMIT:
        integral = -_I_LIMIT

    feedforward = _FF_GAIN * target
    command = feedforward + _FB_KP * error + integral

    if command > cfg.MOTOR_DUTY_MAX:
        command = cfg.MOTOR_DUTY_MAX
        if error > 0:
            integral = integral_last
    elif command < -cfg.MOTOR_DUTY_MAX:
        command = -cfg.MOTOR_DUTY_MAX
        if error < 0:
            integral = integral_last

    if target > 0 and command < 0:
        command = 0
        integral = 0.0
    elif target < 0 and command > 0:
        command = 0
        integral = 0.0

    pid.err = error
    pid.err_last = error
    pid.tar_spd_last = target
    pid.output = integral
    pid.param_b = integral
    return command


class WheelSpeedServoTest:
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

        self.pid_fl = SpeedPID()
        self.pid_fr = SpeedPID()
        self.pid_b = SpeedPID()
        self.pid_fl.init_c()
        self.pid_fr.init_c()
        self.pid_b.init_c()

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

        self.mode = _MODE_PID
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
        self.target_fl = 0
        self.target_fr = 0
        self.target_b = 0
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

    def send_log(self, profile, elapsed, t_fl, t_fr, t_b):
        buf = self.log_buf
        buf[0] = 84
        buf[1] = 32
        pos = _put_int(buf, 2, self.mode)
        pos = _put_int(buf, pos, profile)
        pos = _put_int(buf, pos, elapsed)
        pos = _put_int(buf, pos, t_fl)
        pos = _put_int(buf, pos, t_fr)
        pos = _put_int(buf, pos, t_b)
        pos = _put_int(buf, pos, self.e_fl * 10.0)
        pos = _put_int(buf, pos, self.e_fr * 10.0)
        pos = _put_int(buf, pos, self.e_b * 10.0)
        pos = _put_int(buf, pos, self.last_pwm_fl)
        pos = _put_int(buf, pos, self.last_pwm_fr)
        pos = _put_int(buf, pos, self.last_pwm_b)
        pos = _put_int(buf, pos, self.pid_fl.param_b)
        pos = _put_int(buf, pos, self.pid_fr.param_b)
        pos = _put_int(buf, pos, self.pid_b.param_b)
        pos = _put_int(buf, pos, self.encoder_dt_ms)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

    def update_leds(self):
        self.led_0.value(1 if self.mode == _MODE_PID else 0)
        self.led_1.value(1 if self.mode == _MODE_FF_PI else 0)
        self.led_2.value(1 if self.mode == _MODE_FF_ADAPTIVE else 0)

    def stop_all(self):
        self.motor_fl.duty(0)
        self.motor_fr.duty(0)
        self.motor_b.duty(0)
        self.last_pwm_fl = 0
        self.last_pwm_fr = 0
        self.last_pwm_b = 0

    def reset_controllers(self):
        _reset_pid(self.pid_fl)
        _reset_pid(self.pid_fr)
        _reset_pid(self.pid_b)

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
        self.e_fl = _normalize_encoder(raw_fl, dt_ms) * scale
        self.e_fr = _normalize_encoder(raw_fr, dt_ms) * scale
        self.e_b = _normalize_encoder(raw_b, dt_ms) * scale

    def controller_output(self, pid, actual, target):
        if self.mode == _MODE_PID:
            return _production_ctrl(pid, actual, target)
        return _candidate_ctrl(
            pid,
            actual,
            target,
            self.mode == _MODE_FF_ADAPTIVE,
        )

    def update_motors(self, t_fl, t_fr, t_b):
        u_fl = self.controller_output(self.pid_fl, self.e_fl, t_fl)
        u_fr = self.controller_output(self.pid_fr, self.e_fr, t_fr)
        u_b = self.controller_output(self.pid_b, self.e_b, t_b)

        self.last_pwm_fl = _smooth_pwm(u_fl, self.last_pwm_fl)
        self.last_pwm_fr = _smooth_pwm(u_fr, self.last_pwm_fr)
        self.last_pwm_b = _smooth_pwm(u_b, self.last_pwm_b)
        self.motor_fl.duty(self.last_pwm_fl)
        self.motor_fr.duty(self.last_pwm_fr)
        self.motor_b.duty(self.last_pwm_b)

    def wait_stopped(self):
        self.stop_all()
        self.reset_controllers()
        self.last_encoder_ms = 0
        end_ms = utime.ticks_add(utime.ticks_ms(), _STOP_MS)
        while utime.ticks_diff(end_ms, utime.ticks_ms()) > 0:
            self.wait_tick()
        gc.collect()

    def set_targets(self, profile, elapsed):
        if profile == 0:
            self.target_fl = 19
            self.target_fr = -19
            self.target_b = 0
        elif profile == 1:
            self.target_fl = 10
            self.target_fr = 10
            self.target_b = 10
        elif profile == 2:
            self.target_fl = -4
            self.target_fr = -4
            self.target_b = -33
        elif profile == 3:
            self.target_fl = 3
            self.target_fr = 3
            self.target_b = -25
        elif profile == 4:
            self.target_fl = 7
            self.target_fr = 7
            self.target_b = -20
        elif profile == 5 and elapsed < (_RUN_MS >> 1):
            self.target_fl = 19
            self.target_fr = -19
            self.target_b = 0
        elif profile == 5:
            self.target_fl = -19
            self.target_fr = 19
            self.target_b = 0
        elif elapsed < (_RUN_MS >> 1):
            self.target_fl = 19
            self.target_fr = -19
            self.target_b = 0
        else:
            self.target_fl = 0
            self.target_fr = 0
            self.target_b = 0

    def run_profile(self, profile):
        self.stop_all()
        self.reset_controllers()
        self.last_encoder_ms = 0
        start_ms = utime.ticks_ms()
        next_log_ms = start_ms

        while utime.ticks_diff(utime.ticks_ms(), start_ms) < _RUN_MS:
            self.wait_tick()
            now = utime.ticks_ms()
            elapsed = utime.ticks_diff(now, start_ms)
            self.set_targets(profile, elapsed)
            self.update_motors(
                self.target_fl,
                self.target_fr,
                self.target_b,
            )
            if utime.ticks_diff(now, next_log_ms) >= 0:
                next_log_ms = utime.ticks_add(now, _LOG_MS)
                self.send_log(
                    profile,
                    elapsed,
                    self.target_fl,
                    self.target_fr,
                    self.target_b,
                )

        self.stop_all()

    def run_selected(self):
        self.running = True
        self.send_static("RUN")
        self.send_static("CLEAR GROUND")
        try:
            profile = 0
            while profile < _PROFILE_COUNT:
                self.run_profile(profile)
                self.wait_stopped()
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
        self.send_static("WHEEL SERVO AB")
        self.send_static("C14 0=PID 1=FFPI 2=FFPI+LOWI+BRAKE")
        self.send_static("C9 RUN C8 EXIT")
        self.send_static("P 0=F 1=SPIN 2=O1 3=O2 4=O3 5=REV 6=STOP")
        self.send_static("K 1450 300 8 20 7 18000")
        self.send_static("B 1200 10")
        self.send_static("T M P MS T3 E10_3 PWM3 I3 DT")
        self.send_mode()

        while not self.exit_requested:
            self.check_exit()
            self.poll_keys()
            self.led.toggle()
            utime.sleep_ms(100)


def main():
    tester = None
    try:
        tester = WheelSpeedServoTest()
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
