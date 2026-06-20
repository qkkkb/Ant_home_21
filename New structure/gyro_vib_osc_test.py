from array import array
from machine import Pin
from seekfree import WIRELESS_UART
from smartcar import ticker, encoder
import gc
import utime

import config as cfg
from hardware import Motor
from lsm6dsv16x_gyro_runtime import LSM6DSV16XYawRuntime
from models import AnglePID, MoveBase
from move_base import calc_wheel_spd
import pid as _pid_mod


GYRO_SIGN = 1.0
GYRO_OFFSET_Z = 0.0
GYRO_SCALE = -1.0
GYRO_DEADBAND_DPS = 0.8
GYRO_KP = 0.22
GYRO_KI = 0.004
NAV_TRACK_GYRO_LIMIT = 6.0
GYRO_CAL_SAMPLES = 1000
GYRO_CAL_DELAY_MS = 2

SAMPLE_MS = 5
CAP_N = 320
WARMUP_MS = 300
PWM_PER_SPEED = 2400
FILTER_ALPHA_NUM = 1
FILTER_ALPHA_DEN = 4

MODE_STATIC = 0
MODE_LIFT = 1
MODE_STRAIGHT = 2
MODE_COARSE = 3
MODE_COUNT = 4
MODE_NAMES = ("STATIC", "LIFT", "STRAIGHT", "COARSE")
MODE_LED = ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0))

turn_ctrl = _pid_mod.turn_ctrl
gyro_ctrl = _pid_mod.gyro_ctrl

_pit_flag = False


def pit_cb(_):
    global _pit_flag
    _pit_flag = True


def clamp_duty(value):
    value = int(value)
    if value > cfg.MOTOR_DUTY_MAX:
        return cfg.MOTOR_DUTY_MAX
    if value < -cfg.MOTOR_DUTY_MAX:
        return -cfg.MOTOR_DUTY_MAX
    return value


def apply_min_duty(value):
    value = clamp_duty(value)
    if value == 0:
        return 0
    if 0 < value < cfg.MOTOR_DUTY_MIN:
        return cfg.MOTOR_DUTY_MIN
    if -cfg.MOTOR_DUTY_MIN < value < 0:
        return -cfg.MOTOR_DUTY_MIN
    return value


def yaw_delta(start_deg, end_deg):
    delta = end_deg - start_deg
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def median3(a, b, c):
    if a > b:
        if b > c:
            return b
        if a > c:
            return c
        return a
    if a > c:
        return a
    if b > c:
        return c
    return b


def scale10(value):
    if value >= 0:
        return int(value * 10.0 + 0.5)
    return int(value * 10.0 - 0.5)


class MotorRig:
    def __init__(self):
        self.fl = Motor(
            cfg.MOTOR_FL_PH,
            cfg.MOTOR_FL_PWM,
            freq=cfg.MOTOR_FREQ,
            invert=cfg.MOTOR_FL_INVERT,
        )
        self.fr = Motor(
            cfg.MOTOR_FR_PH,
            cfg.MOTOR_FR_PWM,
            freq=cfg.MOTOR_FREQ,
            invert=cfg.MOTOR_FR_INVERT,
        )
        self.b = Motor(
            cfg.MOTOR_B_PH,
            cfg.MOTOR_B_PWM,
            freq=cfg.MOTOR_FREQ,
            invert=cfg.MOTOR_B_INVERT,
        )
        self.pwm_fl = 0
        self.pwm_fr = 0
        self.pwm_b = 0

    def set_pwm(self, pwm_fl, pwm_fr, pwm_b):
        self.pwm_fl = apply_min_duty(pwm_fl)
        self.pwm_fr = apply_min_duty(pwm_fr)
        self.pwm_b = apply_min_duty(pwm_b)
        self.fl.duty(self.pwm_fl)
        self.fr.duty(self.pwm_fr)
        self.b.duty(self.pwm_b)

    def stop(self):
        self.fl.duty(0)
        self.fr.duty(0)
        self.b.duty(0)
        self.pwm_fl = 0
        self.pwm_fr = 0
        self.pwm_b = 0


class CaptureBuffer:
    def __init__(self):
        self.t = array("H", [0] * CAP_N)
        self.raw = array("h", [0] * CAP_N)
        self.gyro = array("h", [0] * CAP_N)
        self.filt = array("h", [0] * CAP_N)
        self.yaw = array("h", [0] * CAP_N)
        self.efl = array("h", [0] * CAP_N)
        self.efr = array("h", [0] * CAP_N)
        self.eb = array("h", [0] * CAP_N)
        self.tfl = array("h", [0] * CAP_N)
        self.tfr = array("h", [0] * CAP_N)
        self.tb = array("h", [0] * CAP_N)
        self.vz = array("h", [0] * CAP_N)
        self.pfl = array("h", [0] * CAP_N)
        self.pfr = array("h", [0] * CAP_N)
        self.pb = array("h", [0] * CAP_N)


class GyroVibOscTest:
    def __init__(self):
        gc.collect()
        self.wireless = None
        try:
            self.wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
        except Exception:
            self.wireless = None

        self.key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
        self.key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)
        self.key_mode = Pin(cfg.BTN_MODE_PIN, Pin.IN, Pin.PULL_UP)
        self.key_last = [1, 1, 1]

        self.led = Pin(cfg.LED_HB_PIN, Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
        self.led_straight = Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
        self.led_translate = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
        self.led_rotate = Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)

        self.motors = MotorRig()
        self.move = MoveBase()
        self.enc_fl = encoder(cfg.ENC_FL_A, cfg.ENC_FL_B, cfg.ENC_FL_INVERT)
        self.enc_fr = encoder(cfg.ENC_FR_A, cfg.ENC_FR_B, cfg.ENC_FR_INVERT)
        self.enc_b = encoder(cfg.ENC_B_A, cfg.ENC_B_B, cfg.ENC_B_INVERT)
        self.pit = ticker(1)
        self.pit.capture_list(self.enc_fl, self.enc_fr, self.enc_b)
        self.pit.callback(pit_cb)
        self.turn_pid = AnglePID()
        self.gyro_pid = AnglePID()

        self.imu = None
        self.buf = CaptureBuffer()
        self.mode = MODE_STATIC
        self.calibrated = False
        self.exit_requested = False
        self.last_status_ms = utime.ticks_ms()
        self.last_dyaw10 = 0

    def log(self, msg):
        print(msg)
        if self.wireless is not None:
            try:
                self.wireless.send_str(msg)
                self.wireless.send_str("\r\n")
            except Exception:
                pass

    def set_led_mode(self):
        led = MODE_LED[self.mode]
        self.led_straight.value(led[0])
        self.led_translate.value(led[1])
        self.led_rotate.value(led[2])

    def show_mode(self):
        self.set_led_mode()
        self.log("MODE %s" % MODE_NAMES[self.mode])
        if self.mode == MODE_LIFT:
            self.log("LIFT WHEELS VZ0")
        elif self.mode == MODE_STRAIGHT:
            self.log("GROUND STRAIGHT VZ0")
        elif self.mode == MODE_COARSE:
            self.log("COARSE GYRO VZ")

    def print_help(self):
        if self.wireless is None:
            self.log("WIRE FAIL")
        else:
            self.log("WIRE OK")
        self.log("GYRO VIB OSC")
        self.log("C14 MODE C9 CAL/RUN C8 EXIT")
        self.log("FMT D i t r g f y vz e1 e2 e3 tf tr tb p1 p2 p3")
        self.log("SCALE r/g/f/y/vz/tar x10")
        self.show_mode()

    def calibrate(self):
        self.log("KEEP STILL")
        self.log("CAL START")
        self.imu = LSM6DSV16XYawRuntime(
            sign=GYRO_SIGN,
            offset_z=GYRO_OFFSET_Z,
            scale=GYRO_SCALE,
            deadband_dps=GYRO_DEADBAND_DPS,
            tick_period_ms=SAMPLE_MS,
        )
        self.imu.calibrate_offset(
            samples=GYRO_CAL_SAMPLES,
            delay_ms=GYRO_CAL_DELAY_MS,
            logger=self.log,
        )
        self.imu.reset_yaw(0.0)
        self.calibrated = True
        gc.collect()
        self.log("CAL OK")

    def mode_body_cmd(self):
        if self.mode == MODE_LIFT:
            return 6.5, 4.0, 0.0
        if self.mode == MODE_STRAIGHT:
            return 6.5, 0.0, 0.0
        if self.mode == MODE_COARSE:
            return 6.5, 4.0, 0.0
        return 0.0, 0.0, 0.0

    def reset_heading_loop(self):
        self.turn_pid.output = 0.0
        self.turn_pid.err = 0.0
        self.turn_pid.err_last = 0.0
        self.gyro_pid.output = 0.0
        self.gyro_pid.err = 0.0
        self.gyro_pid.err_last = 0.0
        self.gyro_pid.gyro_kp = GYRO_KP
        self.gyro_pid.gyro_ki = GYRO_KI
        self.gyro_pid.gyro_output_limit = NAV_TRACK_GYRO_LIMIT

    def calc_vz_cmd(self, yaw, gyro_z):
        if self.mode != MODE_COARSE:
            return 0.0
        yaw_err = -yaw_delta(0.0, yaw)
        turn_rate = turn_ctrl(self.turn_pid, yaw_err, 0)
        self.gyro_pid.gyro_kp = GYRO_KP
        self.gyro_pid.gyro_ki = GYRO_KI
        self.gyro_pid.gyro_output_limit = NAV_TRACK_GYRO_LIMIT
        return gyro_ctrl(self.gyro_pid, turn_rate - gyro_z)

    def prepare_motion(self):
        vx, vy, vz = self.mode_body_cmd()
        calc_wheel_spd(self.move, vx, vy, vz)
        pwm_fl = self.move.speed_fl * PWM_PER_SPEED
        pwm_fr = self.move.speed_fr * PWM_PER_SPEED
        pwm_b = self.move.speed_b * PWM_PER_SPEED
        if self.mode == MODE_STATIC:
            pwm_fl = 0
            pwm_fr = 0
            pwm_b = 0
        self.motors.set_pwm(pwm_fl, pwm_fr, pwm_b)
        return vx, vy, vz

    def warmup(self):
        start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start) < WARMUP_MS:
            if self.key_exit.value() == 0:
                raise KeyboardInterrupt
            self.imu.read_gyro_z()
            utime.sleep_ms(SAMPLE_MS)

    def capture(self):
        buf = self.buf
        self.imu.reset_yaw(0.0)
        self.reset_heading_loop()
        start_yaw = self.imu.read_yaw()
        start_ms = utime.ticks_ms()
        next_ms = start_ms
        g0 = 0.0
        g1 = 0.0
        filt = 0.0
        for i in range(CAP_N):
            while utime.ticks_diff(utime.ticks_ms(), next_ms) < 0:
                utime.sleep_ms(1)
            now = utime.ticks_ms()
            next_ms = utime.ticks_add(next_ms, SAMPLE_MS)
            gyro_z = self.imu.read_gyro_z()
            raw_z = self.imu.raw_gyro_z
            yaw = self.imu.read_yaw()
            vx, vy, _ = self.mode_body_cmd()
            vz_cmd = self.calc_vz_cmd(yaw, gyro_z)
            calc_wheel_spd(self.move, vx, vy, vz_cmd)
            self.motors.set_pwm(
                self.move.speed_fl * PWM_PER_SPEED,
                self.move.speed_fr * PWM_PER_SPEED,
                self.move.speed_b * PWM_PER_SPEED,
            )
            med = median3(g0, g1, gyro_z)
            filt += (med - filt) * FILTER_ALPHA_NUM / FILTER_ALPHA_DEN
            g0 = g1
            g1 = gyro_z

            buf.t[i] = utime.ticks_diff(now, start_ms)
            buf.raw[i] = scale10(raw_z)
            buf.gyro[i] = scale10(gyro_z)
            buf.filt[i] = scale10(filt)
            buf.yaw[i] = scale10(yaw)
            buf.efl[i] = int(self.enc_fl.get())
            buf.efr[i] = int(self.enc_fr.get())
            buf.eb[i] = int(self.enc_b.get())
            buf.tfl[i] = scale10(self.move.speed_fl)
            buf.tfr[i] = scale10(self.move.speed_fr)
            buf.tb[i] = scale10(self.move.speed_b)
            buf.vz[i] = scale10(vz_cmd)
            buf.pfl[i] = self.motors.pwm_fl
            buf.pfr[i] = self.motors.pwm_fr
            buf.pb[i] = self.motors.pwm_b

        self.last_dyaw10 = scale10(yaw_delta(start_yaw, self.imu.read_yaw()))

    def dump_capture(self):
        b = self.buf
        max_g = 0
        max_f = 0
        max_e = 0
        i = 0
        while i < CAP_N:
            ag = abs(b.gyro[i])
            af = abs(b.filt[i])
            ae = max(abs(b.efl[i]), abs(b.efr[i]), abs(b.eb[i]))
            if ag > max_g:
                max_g = ag
            if af > max_f:
                max_f = af
            if ae > max_e:
                max_e = ae
            i += 1

        self.log(
            "SUM %s N=%d DY10=%d GMAX10=%d FMAX10=%d EMAX=%d"
            % (MODE_NAMES[self.mode], CAP_N, self.last_dyaw10, max_g, max_f, max_e)
        )
        self.log("DUMP BEGIN")
        i = 0
        while i < CAP_N:
            self.log(
                "D %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d"
                % (
                    i,
                    b.t[i],
                    b.raw[i],
                    b.gyro[i],
                    b.filt[i],
                    b.yaw[i],
                    b.vz[i],
                    b.efl[i],
                    b.efr[i],
                    b.eb[i],
                    b.tfl[i],
                    b.tfr[i],
                    b.tb[i],
                    b.pfl[i],
                    b.pfr[i],
                    b.pb[i],
                )
            )
            i += 1
            if (i & 31) == 0:
                gc.collect()
        self.log("DUMP END")

    def run_selected(self):
        if not self.calibrated:
            self.calibrate()
            return
        self.log("RUN %s" % MODE_NAMES[self.mode])
        self.pit.start(cfg.TICK_PERIOD_MS)
        try:
            vx, vy, vz = self.prepare_motion()
            self.log("BODY10 %d %d %d" % (scale10(vx), scale10(vy), scale10(vz)))
            self.warmup()
            self.capture()
        finally:
            self.motors.stop()
            self.pit.stop()
        gc.collect()
        self.dump_capture()
        gc.collect()
        self.log("READY")

    def on_mode(self):
        self.mode = (self.mode + 1) % MODE_COUNT
        self.show_mode()

    def on_exit(self):
        self.exit_requested = True
        self.log("EXIT")

    def poll_keys(self):
        keys = (self.key_mode, self.key_start, self.key_exit)
        i = 0
        while i < 3:
            cur = keys[i].value()
            if self.key_last[i] == 1 and cur == 0:
                utime.sleep_ms(20)
                if keys[i].value() == 0:
                    if i == 0:
                        self.on_mode()
                    elif i == 1:
                        self.run_selected()
                    else:
                        self.on_exit()
            self.key_last[i] = cur
            i += 1

    def run(self):
        self.print_help()
        while not self.exit_requested:
            self.poll_keys()
            now = utime.ticks_ms()
            if utime.ticks_diff(now, self.last_status_ms) >= 1000:
                self.led.toggle()
                self.last_status_ms = now
            utime.sleep_ms(5)


tester = None

try:
    tester = GyroVibOscTest()
    tester.run()
except KeyboardInterrupt:
    if tester is not None:
        tester.log("EXIT")
finally:
    if tester is not None:
        tester.motors.stop()
        try:
            tester.pit.stop()
        except Exception:
            pass
    try:
        Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
        Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
        Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)
    except Exception:
        pass
