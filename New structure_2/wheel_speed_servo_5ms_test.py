import gc
import utime

from machine import Pin
from micropython import const
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker

import config as cfg
from hardware import Motor


_TICK_MS = const(5)
_DIR_WAIT_MS = const(20)
_RUN_MS = const(1200)
_MEASURE_START_MS = const(300)
_STOP_TEST_MS = const(600)
_TRACE_MS = const(200)
_SETTLE_SAMPLES = const(10)

_PWM_LIMIT = const(50000)
_PWM_STEP = const(4800)
_FF_GAIN = const(1450)
_FB_KP = const(300)
_FB_KI = const(8)
_FB_KI_LOW = const(20)
_LOW_TARGET_MAX = const(7)
_I_LIMIT = const(18000)

# Direct wheel targets: FL, FR, B.  Profiles 15..17 are the compound
# targets that verified the master's orbit speed servo.
_TARGETS = (
    (5, 5, 5),
    (-5, -5, -5),
    (10, 10, 10),
    (-10, -10, -10),
    (19, 19, 19),
    (-19, -19, -19),
    (30, 30, 30),
    (-30, -30, -30),
    (35, 35, 35),
    (-35, -35, -35),
    (-19, 19, 0),
    (19, -19, 0),
    (10, 10, -20),
    (-10, -10, 20),
    (10, 10, 10),
    (-4, -4, -33),
    (3, 3, -25),
    (7, 7, -20),
)

_pit_pending = 0
wireless = None


def _pit_handler(_):
    global _pit_pending
    if _pit_pending < 100:
        _pit_pending += 1


def _take_pending():
    global _pit_pending
    count = _pit_pending
    _pit_pending = 0
    return count


def _log(message):
    print(message)
    if wireless is not None:
        try:
            wireless.send_str(message)
            wireless.send_str("\r\n")
        except Exception:
            pass


class _SpeedPI:
    def __init__(self):
        self.integral = 0.0
        self.last_target = 0.0
        self.error = 0.0
        self.saturated = 0

    def reset(self):
        self.integral = 0.0
        self.last_target = 0.0
        self.error = 0.0
        self.saturated = 0

    def update(self, actual, target):
        if target == 0:
            self.reset()
            return 0

        if target * self.last_target < 0:
            self.integral = 0.0

        error = target - actual
        integral_last = self.integral
        ki = _FB_KI_LOW if -_LOW_TARGET_MAX <= target <= _LOW_TARGET_MAX else _FB_KI
        integral = integral_last + ki * error
        if integral > _I_LIMIT:
            integral = _I_LIMIT
        elif integral < -_I_LIMIT:
            integral = -_I_LIMIT

        command = _FF_GAIN * target + _FB_KP * error + integral
        self.saturated = 0
        if command > _PWM_LIMIT:
            command = _PWM_LIMIT
            self.saturated = 1
            if error > 0:
                integral = integral_last
        elif command < -_PWM_LIMIT:
            command = -_PWM_LIMIT
            self.saturated = 1
            if error < 0:
                integral = integral_last

        if target > 0 and command < 0:
            command = 0
            integral = 0.0
        elif target < 0 and command > 0:
            command = 0
            integral = 0.0

        self.error = error
        self.last_target = target
        self.integral = integral
        return int(command)


def _smooth_pwm(target, last):
    if target == 0:
        return 0
    delta = target - last
    if delta > _PWM_STEP:
        target = last + _PWM_STEP
    elif delta < -_PWM_STEP:
        target = last - _PWM_STEP
    mixed = last * 3 + target * 2
    if mixed >= 0:
        value = mixed // 5
    else:
        value = -((-mixed) // 5)
    if value > _PWM_LIMIT:
        return _PWM_LIMIT
    if value < -_PWM_LIMIT:
        return -_PWM_LIMIT
    if 0 < value < cfg.MOTOR_DUTY_MIN:
        return cfg.MOTOR_DUTY_MIN
    if -cfg.MOTOR_DUTY_MIN < value < 0:
        return -cfg.MOTOR_DUTY_MIN
    return value


class WheelServo5msTest:
    def __init__(self):
        global wireless
        gc.collect()
        wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)

        self.key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
        self.key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)
        self.led = Pin(cfg.LED_HB_PIN, Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

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

        self.enc_fl = encoder(cfg.ENC_FL_DIR, cfg.ENC_FL_PULSE, cfg.ENC_FL_INVERT)
        self.enc_fr = encoder(cfg.ENC_FR_DIR, cfg.ENC_FR_PULSE, cfg.ENC_FR_INVERT)
        self.enc_b = encoder(cfg.ENC_B_DIR, cfg.ENC_B_PULSE, cfg.ENC_B_INVERT)

        self.pid_fl = _SpeedPI()
        self.pid_fr = _SpeedPI()
        self.pid_b = _SpeedPI()
        self.pids = (self.pid_fl, self.pid_fr, self.pid_b)

        self.actual = [0.0, 0.0, 0.0]
        self.pwm = [0, 0, 0]
        self.sum_x100 = [0, 0, 0]
        self.mae_x100 = [0, 0, 0]
        self.min_x100 = [32767, 32767, 32767]
        self.max_x100 = [-32768, -32768, -32768]
        self.zero_count = [0, 0, 0]
        self.wrong_count = [0, 0, 0]
        self.sat_count = [0, 0, 0]
        self.settle_count = [0, 0, 0]
        self.settle_ms = [-1, -1, -1]

        self.pit = ticker(1)
        self.pit.capture_list(self.enc_fl, self.enc_fr, self.enc_b)
        self.pit.callback(_pit_handler)
        self.pit.start(_TICK_MS)

    def check_exit(self):
        if self.key_exit.value() == 0:
            raise KeyboardInterrupt

    def wait_ms(self, delay_ms):
        end = utime.ticks_add(utime.ticks_ms(), delay_ms)
        while utime.ticks_diff(end, utime.ticks_ms()) > 0:
            self.check_exit()
            utime.sleep_ms(1)

    def wait_tick(self):
        while _pit_pending == 0:
            self.check_exit()
            utime.sleep_ms(1)
        return _take_pending()

    def stop_motors(self):
        self.motor_fl.pwm.duty_u16(0)
        self.motor_fr.pwm.duty_u16(0)
        self.motor_b.pwm.duty_u16(0)
        self.pwm[0] = 0
        self.pwm[1] = 0
        self.pwm[2] = 0

    def reset_controllers(self):
        self.pid_fl.reset()
        self.pid_fr.reset()
        self.pid_b.reset()

    def clear_encoder_counts(self):
        self.enc_fl.get()
        self.enc_fr.get()
        self.enc_b.get()
        _take_pending()

    def read_encoders(self):
        self.actual[0] = int(self.enc_fl.get()) * 0.015625
        self.actual[1] = int(self.enc_fr.get()) * 0.015625
        self.actual[2] = int(self.enc_b.get()) * 0.015625

    def prepare_directions(self, targets):
        self.stop_motors()
        index = 0
        while index < 3:
            target = targets[index]
            if target:
                direction = 1 if target > 0 else 0
                motor = self.motors[index]
                if motor.invert:
                    direction = 1 - direction
                motor.ph.value(direction)
            index += 1
        self.wait_ms(_DIR_WAIT_MS)
        self.clear_encoder_counts()

    def apply_pwm(self, targets):
        index = 0
        while index < 3:
            target = targets[index]
            if target == 0:
                value = 0
                self.pids[index].reset()
            else:
                value = self.pids[index].update(self.actual[index], target)
                value = _smooth_pwm(value, self.pwm[index])
            self.pwm[index] = value
            self.motors[index].pwm.duty_u16(abs(value))
            index += 1

    def reset_stats(self):
        index = 0
        while index < 3:
            self.sum_x100[index] = 0
            self.mae_x100[index] = 0
            self.min_x100[index] = 32767
            self.max_x100[index] = -32768
            self.zero_count[index] = 0
            self.wrong_count[index] = 0
            self.sat_count[index] = 0
            self.settle_count[index] = 0
            self.settle_ms[index] = -1
            index += 1

    def update_stats(self, targets, elapsed):
        index = 0
        while index < 3:
            target = targets[index]
            actual = self.actual[index]
            actual_x100 = int(actual * 100)
            error_x100 = int(abs(target - actual) * 100)
            self.sum_x100[index] += actual_x100
            self.mae_x100[index] += error_x100
            if actual_x100 < self.min_x100[index]:
                self.min_x100[index] = actual_x100
            if actual_x100 > self.max_x100[index]:
                self.max_x100[index] = actual_x100
            if actual_x100 == 0:
                self.zero_count[index] += 1
            elif target and actual * target < 0:
                self.wrong_count[index] += 1
            if self.pids[index].saturated:
                self.sat_count[index] += 1

            tolerance = 80
            target_tol = abs(target) * 5
            if target_tol > tolerance:
                tolerance = target_tol
            if error_x100 <= tolerance:
                self.settle_count[index] += 1
                if (
                    self.settle_ms[index] < 0
                    and self.settle_count[index] >= _SETTLE_SAMPLES
                ):
                    self.settle_ms[index] = elapsed - (_SETTLE_SAMPLES - 1) * _TICK_MS
            else:
                self.settle_count[index] = 0
            index += 1

    def trace(self, profile, elapsed, targets):
        _log(
            "S P=%d MS=%d T=%d,%d,%d E_X100=%d,%d,%d PWM=%d,%d,%d"
            % (
                profile,
                elapsed,
                targets[0],
                targets[1],
                targets[2],
                int(self.actual[0] * 100),
                int(self.actual[1] * 100),
                int(self.actual[2] * 100),
                self.pwm[0],
                self.pwm[1],
                self.pwm[2],
            )
        )

    def report(self, profile, targets, samples, late_events, late_ticks):
        index = 0
        while index < 3:
            average = self.sum_x100[index] // samples if samples else 0
            mae = self.mae_x100[index] // samples if samples else 0
            _log(
                "R P=%d W=%d T=%d N=%d AVG_X100=%d MAE_X100=%d MIN_X100=%d MAX_X100=%d SET_MS=%d SAT=%d ZERO=%d WRONG=%d LATE=%d/%d"
                % (
                    profile,
                    index,
                    targets[index],
                    samples,
                    average,
                    mae,
                    self.min_x100[index],
                    self.max_x100[index],
                    self.settle_ms[index],
                    self.sat_count[index],
                    self.zero_count[index],
                    self.wrong_count[index],
                    late_events,
                    late_ticks,
                )
            )
            index += 1

    def stop_test(self, profile):
        self.stop_motors()
        self.reset_controllers()
        start = utime.ticks_ms()
        stable = 0
        stopped_ms = -1
        late_events = 0
        late_ticks = 0
        while utime.ticks_diff(utime.ticks_ms(), start) < _STOP_TEST_MS:
            pending = self.wait_tick()
            if pending > 1:
                late_events += 1
                late_ticks += pending - 1
            self.read_encoders()
            elapsed = utime.ticks_diff(utime.ticks_ms(), start)
            if (
                -0.5 <= self.actual[0] <= 0.5
                and -0.5 <= self.actual[1] <= 0.5
                and -0.5 <= self.actual[2] <= 0.5
            ):
                stable += 1
                if stopped_ms < 0 and stable >= 5:
                    stopped_ms = elapsed - 20
            else:
                stable = 0
        _log(
            "Z P=%d STOP_MS=%d RES_X100=%d,%d,%d LATE=%d/%d"
            % (
                profile,
                stopped_ms,
                int(self.actual[0] * 100),
                int(self.actual[1] * 100),
                int(self.actual[2] * 100),
                late_events,
                late_ticks,
            )
        )

    def run_profile(self, profile, targets):
        self.prepare_directions(targets)
        self.reset_controllers()
        self.reset_stats()
        start = utime.ticks_ms()
        next_trace = start
        samples = 0
        late_events = 0
        late_ticks = 0

        while utime.ticks_diff(utime.ticks_ms(), start) < _RUN_MS:
            pending = self.wait_tick()
            if pending > 1:
                late_events += 1
                late_ticks += pending - 1
            self.read_encoders()
            now = utime.ticks_ms()
            elapsed = utime.ticks_diff(now, start)
            self.apply_pwm(targets)
            if elapsed >= _MEASURE_START_MS:
                self.update_stats(targets, elapsed)
                samples += 1
            if utime.ticks_diff(now, next_trace) >= 0:
                next_trace = utime.ticks_add(now, _TRACE_MS)
                self.trace(profile, elapsed, targets)

        self.report(profile, targets, samples, late_events, late_ticks)
        self.stop_test(profile)
        gc.collect()

    def run_all(self):
        _log("=== FOLLOWER WHEEL SERVO 5MS TEST ===")
        _log("ENC=RAW/64 CTRL=FF1450+P300+I8/20 PWM_LIMIT=50000")
        _log("SMOOTH=0.4 STEP=4800 DIR_WAIT_MS=20 TICK_MS=5")
        _log("W=0 FL,1 FR,2 B; C8=EMERGENCY STOP")
        _log("P0..9=ALL +/-5,10,19,30,35 P10..17=COMPOUND")
        profile = 0
        while profile < len(_TARGETS):
            targets = _TARGETS[profile]
            _log("BEGIN P=%d T=%d,%d,%d" % (profile, targets[0], targets[1], targets[2]))
            self.run_profile(profile, targets)
            profile += 1
        _log("=== TEST COMPLETE ===")

    def wait_start(self):
        _log("C9=START C8=EXIT; RAISE WHEELS AND CLEAR THE CHASSIS")
        last = 1
        while True:
            self.check_exit()
            value = self.key_start.value()
            if last == 1 and value == 0:
                self.wait_ms(20)
                if self.key_start.value() == 0:
                    return
            last = value
            self.led.toggle()
            utime.sleep_ms(100)


def main():
    tester = None
    try:
        gc.collect()
        try:
            gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
        except Exception:
            pass
        tester = WheelServo5msTest()
        tester.wait_start()
        tester.run_all()
    except KeyboardInterrupt:
        _log("EMERGENCY STOP")
    finally:
        if tester is not None:
            tester.stop_motors()
            try:
                tester.pit.stop()
            except Exception:
                pass
        _log("MOTORS STOPPED")


main()
