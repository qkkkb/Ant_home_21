from machine import Pin
from micropython import const
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker
import gc
import utime

import config as cfg
from hardware import Motor
from lsm6dsv16x_gyro_runtime import LSM6DSV16XYawRuntime
from models import AnglePID, MoveBase, SpeedPID
from move_base import calc_wheel_spd
import pid as pid_mod


_MODE_BASELINE = const(0)
_MODE_FF_RAW = const(1)
_MODE_FF_EMA = const(2)
_MODE_COUNT = const(3)

_PROFILE_COUNT = const(6)
_RUN_MS = const(1500)
_SETTLE_MS = const(800)
_GAP_MS = const(500)
_LOG_MS = const(20)
_LOG_BUF_SIZE = const(176)

_BASE_KP = 0.08
_BASE_KI = 0.005
_BASE_EMA_ALPHA = 0.25

_RATE_FF_GAIN = 0.031
_RATE_FB_KP = 0.02
_RATE_FB_KI = 0.0005
_RATE_FB_I_LIMIT = 2.0
_RATE_EMA_ALPHA = 0.5
_RATE_STOP_KP = 0.03
_RATE_STOP_LIMIT = 5.0
_RATE_STOP_DEADBAND = 8.0
_RATE_START_CMD_MIN = 10.0
_RATE_START_CMD_MAX = 30.0
_RATE_START_GYRO_MAX = 5.0
_RATE_START_VZ = 2.0
_GYRO_LIMIT = 18.0

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
    if profile == 0:
        return 15.0
    if profile == 1:
        return -15.0
    if profile == 2:
        return 60.0
    if profile == 3:
        return -60.0
    if profile == 4:
        return 145.0
    return -145.0


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
        self.gyro_pid = AnglePID()
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

        self.mode = _MODE_BASELINE
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
        self.rate_start_done = False
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
        pos = _put_int(buf, pos, self.vz_cmd * 10.0)
        pos = _put_int(buf, pos, self.imu.read_yaw() * 10.0)
        pos = _put_int(buf, pos, self.e_fl * 10.0)
        pos = _put_int(buf, pos, self.e_fr * 10.0)
        pos = _put_int(buf, pos, self.e_b * 10.0)
        pos = _put_int(buf, pos, self.last_pwm_fl)
        pos = _put_int(buf, pos, self.last_pwm_fr)
        pos = _put_int(buf, pos, self.last_pwm_b)
        pos = _put_int(buf, pos, self.encoder_dt_ms)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

    def update_leds(self):
        self.led_0.value(1 if self.mode == _MODE_BASELINE else 0)
        self.led_1.value(1 if self.mode == _MODE_FF_RAW else 0)
        self.led_2.value(1 if self.mode == _MODE_FF_EMA else 0)

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
        self.gyro_pid.output = 0.0
        self.gyro_pid.err = 0.0
        self.gyro_pid.err_last = 0.0
        self.gyro_pid.gyro_kp = _BASE_KP
        self.gyro_pid.gyro_ki = _BASE_KI
        self.gyro_pid.gyro_output_limit = _GYRO_LIMIT
        self.filter_ready = False
        self.gyro_filt = 0.0
        self.vz_cmd = 0.0
        self.rate_integral = 0.0
        self.rate_start_done = False

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
        if self.mode == _MODE_FF_RAW:
            self.gyro_filt = self.gyro_raw
            self.filter_ready = True
        elif not self.filter_ready:
            self.gyro_filt = self.gyro_raw
            self.filter_ready = True
        else:
            alpha = (
                _BASE_EMA_ALPHA
                if self.mode == _MODE_BASELINE
                else _RATE_EMA_ALPHA
            )
            self.gyro_filt += alpha * (
                self.gyro_raw - self.gyro_filt
            )

    def feedforward_rate_ctrl(self, rate_cmd):
        if rate_cmd == 0.0:
            self.rate_integral = 0.0
            self.rate_start_done = False
            if (
                -_RATE_STOP_DEADBAND
                <= self.gyro_filt
                <= _RATE_STOP_DEADBAND
            ):
                return 0.0
            command = -_RATE_STOP_KP * self.gyro_filt
            if command > _RATE_STOP_LIMIT:
                return _RATE_STOP_LIMIT
            if command < -_RATE_STOP_LIMIT:
                return -_RATE_STOP_LIMIT
            return command

        rate_abs = abs(rate_cmd)
        start_active = False
        if (
            _RATE_START_CMD_MIN <= rate_abs <= _RATE_START_CMD_MAX
            and not self.rate_start_done
        ):
            if abs(self.gyro_filt) >= _RATE_START_GYRO_MAX:
                self.rate_start_done = True
            else:
                start_active = True

        error = rate_cmd - self.gyro_filt
        integral_last = 0.0 if start_active else self.rate_integral
        if start_active:
            integral = 0.0
        else:
            integral = integral_last + _RATE_FB_KI * error
        if integral > _RATE_FB_I_LIMIT:
            integral = _RATE_FB_I_LIMIT
        elif integral < -_RATE_FB_I_LIMIT:
            integral = -_RATE_FB_I_LIMIT

        command = (
            _RATE_FF_GAIN * rate_cmd
            + _RATE_FB_KP * error
            + integral
        )
        if command > _GYRO_LIMIT:
            command = _GYRO_LIMIT
            if error > 0.0:
                integral = integral_last
        elif command < -_GYRO_LIMIT:
            command = -_GYRO_LIMIT
            if error < 0.0:
                integral = integral_last

        if start_active:
            if rate_cmd > 0.0 and command < _RATE_START_VZ:
                command = _RATE_START_VZ
            elif rate_cmd < 0.0 and command > -_RATE_START_VZ:
                command = -_RATE_START_VZ

        self.rate_integral = integral
        return command

    def update_motors(self):
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

    def control_tick(self, rate_cmd):
        self.update_gyro()
        if self.mode == _MODE_BASELINE:
            self.vz_cmd = pid_mod.gyro_ctrl(
                self.gyro_pid,
                rate_cmd - self.gyro_filt,
            )
        else:
            self.vz_cmd = self.feedforward_rate_ctrl(rate_cmd)
        calc_wheel_spd(self.move, 0.0, 0.0, self.vz_cmd)
        self.update_motors()

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
            self.control_tick(rate_cmd)
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
        self.send_static("GYRO RATE AB")
        self.send_static("C14 0=BASE 1=FFRAW 2=FFEMA")
        self.send_static("C9 RUN C8 EXIT GROUND")
        self.send_static("P +15 -15 +60 -60 +145 -145")
        self.send_static("BASE A.25 KP.08 KI.005")
        self.send_static("FF .031 KP.02 KI.0005 I2 A.5")
        self.send_static("STOP KP.03 LIM5 DB8")
        self.send_static("START ONCE C10-30 G5 V2")
        self.send_static("R M P MS C10 G10 F10 Z10 Y10 E10_3 PWM3 DT")
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
