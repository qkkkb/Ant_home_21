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


_PROFILE_COUNT = const(8)
_RUN_MS = const(1400)
_BRAKE_MS = const(800)
_GAP_MS = const(500)
_LOG_MS = const(20)
_LOG_BUF_SIZE = const(192)

_GYRO_EMA_ALPHA = 0.25
_GYRO_DEADBAND = 0.8
_GYRO_STOP_RATE = 4.0
_GYRO_SIGN = 1.0
_GYRO_SCALE = -1.0
_CAL_SAMPLES = const(1000)
_CAL_DELAY_MS = const(2)
_CAL_SETTLE_MS = const(700)

_GYRO_KP = 0.04
_GYRO_KI = 0.004
_TURN_GYRO_KI = 0.002
_NORMAL_LIMIT = 12.0
_PUSH_LIMIT = 16.0
_ORBIT_LIMIT = 28.0
_SPIN_LIMIT = 18.0

_PWM_LIMIT = const(60000)
_PWM_STEP = const(4800)

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


def _reset_speed_pid(pid):
    pid.output = 0.0
    pid.err = 0.0
    pid.tar_spd_last = 0.0


def _smooth_pwm(target, last):
    if target == 0:
        return 0
    delta = target - last
    if delta > _PWM_STEP:
        target = last + _PWM_STEP
    elif delta < -_PWM_STEP:
        target = last - _PWM_STEP
    value = int(last * 0.6 + target * 0.4)
    if value > _PWM_LIMIT:
        value = _PWM_LIMIT
    elif value < -_PWM_LIMIT:
        value = -_PWM_LIMIT
    if 0 < value < cfg.MOTOR_DUTY_MIN:
        return cfg.MOTOR_DUTY_MIN
    if -cfg.MOTOR_DUTY_MIN < value < 0:
        return -cfg.MOTOR_DUTY_MIN
    return value


def _profile_mode(profile):
    return profile >> 1


def _profile_rate(profile):
    mode = _profile_mode(profile)
    if mode == 0:
        rate = 15.0
    elif mode == 1:
        rate = 40.0
    elif mode == 2:
        rate = 95.0
    else:
        rate = 120.0
    return -rate if profile & 1 else rate


class GyroRateLoopTest:
    def __init__(self):
        gc.collect()
        pid_mod.PWM_MAX = _PWM_LIMIT
        self.wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
        self.log_buf = bytearray(_LOG_BUF_SIZE)
        self.send_static("BOOT 1 WIRELESS")

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
        self.send_static("BOOT 2 MOTORS")

        self.enc_fl = encoder(
            cfg.ENC_FL_DIR, cfg.ENC_FL_PULSE, cfg.ENC_FL_INVERT
        )
        self.enc_fr = encoder(
            cfg.ENC_FR_DIR, cfg.ENC_FR_PULSE, cfg.ENC_FR_INVERT
        )
        self.enc_b = encoder(
            cfg.ENC_B_DIR, cfg.ENC_B_PULSE, cfg.ENC_B_INVERT
        )
        self.send_static("BOOT 3 ENCODERS")

        self.imu = LSM6DSV16XYawRuntime(
            sign=_GYRO_SIGN,
            offset_z=0.0,
            scale=_GYRO_SCALE,
            deadband_dps=_GYRO_DEADBAND,
            tick_period_ms=cfg.TICK_PERIOD_MS,
        )
        self.send_static("BOOT 4 GYRO")

        self.pid_fl = SpeedPID()
        self.pid_fr = SpeedPID()
        self.pid_b = SpeedPID()
        self.gyro_pid = AnglePID()
        self.move = MoveBase()

        self.pit = ticker(1)
        self.pit.capture_list(self.enc_fl, self.enc_fr, self.enc_b)
        self.pit.callback(_pit_handler)
        self.pit.start(cfg.TICK_PERIOD_MS)
        self.send_static("BOOT 5 TICKER")

        self.e_fl = 0.0
        self.e_fr = 0.0
        self.e_b = 0.0
        self.gyro_raw = 0.0
        self.gyro_filt = 0.0
        self.filter_ready = False
        self.vz_cmd = 0.0
        self.mode = 0
        self.brake_done = False
        self.last_pwm_fl = 0
        self.last_pwm_fr = 0
        self.last_pwm_b = 0

    def send_static(self, value):
        try:
            self.wireless.send_str(value)
            self.wireless.send_str("\r\n")
        except Exception:
            pass

    def send_profile(self, profile, rate):
        buf = self.log_buf
        buf[0] = 80
        buf[1] = 32
        pos = _put_int(buf, 2, profile)
        pos = _put_int(buf, pos, _profile_mode(profile))
        pos = _put_int(buf, pos, rate * 10.0)
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
        pos = _put_int(buf, 2, profile)
        pos = _put_int(buf, pos, elapsed)
        pos = _put_int(buf, pos, rate_cmd * 10.0)
        pos = _put_int(buf, pos, self.gyro_raw * 10.0)
        pos = _put_int(buf, pos, self.gyro_filt * 10.0)
        pos = _put_int(buf, pos, self.vz_cmd * 100.0)
        pos = _put_int(buf, pos, self.move.speed_fl * 10.0)
        pos = _put_int(buf, pos, self.move.speed_fr * 10.0)
        pos = _put_int(buf, pos, self.move.speed_b * 10.0)
        pos = _put_int(buf, pos, self.e_fl * 10.0)
        pos = _put_int(buf, pos, self.e_fr * 10.0)
        pos = _put_int(buf, pos, self.e_b * 10.0)
        pos = _put_int(buf, pos, self.last_pwm_fl)
        pos = _put_int(buf, pos, self.last_pwm_fr)
        pos = _put_int(buf, pos, self.last_pwm_b)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            self.wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass

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
        self.vz_cmd = 0.0

    def set_gyro_mode(self, mode):
        self.mode = mode
        self.gyro_pid.gyro_kp = _GYRO_KP
        if mode == 0:
            self.gyro_pid.gyro_ki = _GYRO_KI
            self.gyro_pid.gyro_output_limit = _NORMAL_LIMIT
        elif mode == 1:
            self.gyro_pid.gyro_ki = _GYRO_KI
            self.gyro_pid.gyro_output_limit = _PUSH_LIMIT
        elif mode == 2:
            self.gyro_pid.gyro_ki = _TURN_GYRO_KI
            self.gyro_pid.gyro_output_limit = _ORBIT_LIMIT
        else:
            self.gyro_pid.gyro_ki = _TURN_GYRO_KI
            self.gyro_pid.gyro_output_limit = _SPIN_LIMIT

    def wait_tick(self):
        global _pit_flag
        while not _pit_flag:
            utime.sleep_ms(1)
        _pit_flag = False
        scale = cfg.ENC_SCALE
        self.e_fl = self.enc_fl.get() * scale
        self.e_fr = self.enc_fr.get() * scale
        self.e_b = self.enc_b.get() * scale

    def update_gyro(self):
        self.gyro_raw = self.imu.read_gyro_z()
        if self.filter_ready:
            self.gyro_filt += _GYRO_EMA_ALPHA * (
                self.gyro_raw - self.gyro_filt
            )
        else:
            self.gyro_filt = self.gyro_raw
            self.filter_ready = True

    def update_motors(self, rate_cmd):
        if rate_cmd == 0.0:
            if self.brake_done:
                return
            if (
                self.mode == 0
                or -_GYRO_STOP_RATE <= self.gyro_filt <= _GYRO_STOP_RATE
            ):
                self.reset_controllers()
                self.move.speed_fl = 0.0
                self.move.speed_fr = 0.0
                self.move.speed_b = 0.0
                self.stop_all()
                self.brake_done = True
                return
        self.vz_cmd = pid_mod.gyro_ctrl(
            self.gyro_pid,
            rate_cmd - self.gyro_filt,
        )
        calc_wheel_spd(self.move, 0.0, 0.0, self.vz_cmd)
        u_fl = pid_mod.speed_ctrl(
            self.pid_fl, self.e_fl, self.move.speed_fl
        )
        u_fr = pid_mod.speed_ctrl(
            self.pid_fr, self.e_fr, self.move.speed_fr
        )
        u_b = pid_mod.speed_ctrl(
            self.pid_b, self.e_b, self.move.speed_b
        )
        self.last_pwm_fl = _smooth_pwm(u_fl, self.last_pwm_fl)
        self.last_pwm_fr = _smooth_pwm(u_fr, self.last_pwm_fr)
        self.last_pwm_b = _smooth_pwm(u_b, self.last_pwm_b)
        self.motor_fl.duty(self.last_pwm_fl)
        self.motor_fr.duty(self.last_pwm_fr)
        self.motor_b.duty(self.last_pwm_b)

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
        self.brake_done = False
        self.set_gyro_mode(_profile_mode(profile))
        self.send_profile(profile, rate)
        start_ms = utime.ticks_ms()
        next_log_ms = start_ms
        total_ms = _RUN_MS + _BRAKE_MS
        while utime.ticks_diff(utime.ticks_ms(), start_ms) < total_ms:
            self.wait_tick()
            self.update_gyro()
            now = utime.ticks_ms()
            elapsed = utime.ticks_diff(now, start_ms)
            rate_cmd = rate if elapsed < _RUN_MS else 0.0
            self.update_motors(rate_cmd)
            if utime.ticks_diff(now, next_log_ms) >= 0:
                next_log_ms = utime.ticks_add(now, _LOG_MS)
                self.send_log(profile, elapsed, rate_cmd)
        self.stop_all()

    def run(self):
        self.send_static("FOLLOWER GYRO RATE 5MS")
        self.send_static("AUTO RUN CLEAR GROUND CTRL+C STOP")
        self.send_static("M 0=N15 1=P40 2=O95 3=S120 BOTH DIR")
        self.send_static("K .04/.004/12 .04/.004/16 .04/.002/28 .04/.002/18")
        self.send_static("STOP N=NOW P/O/S=PI UNTIL 4DPS THEN RESET")
        self.send_static("R P MS T10 RAW10 FILT10 V100 WT10_3 E10_3 PWM3")
        self.send_static("KEEP STILL")
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


def main():
    tester = None
    print("BOOT 0 START")
    try:
        tester = GyroRateLoopTest()
        gc.collect()
        try:
            gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
        except Exception:
            pass
        tester.run()
    except KeyboardInterrupt:
        print("CTRL+C STOP")
    except Exception as exc:
        print("ERROR", exc)
        if tester is not None:
            tester.send_static("ERROR")
    finally:
        if tester is not None:
            tester.stop_all()
            try:
                tester.pit.stop()
            except Exception:
                pass
            tester.send_static("EXIT")
        print("MOTORS STOPPED")


main()
