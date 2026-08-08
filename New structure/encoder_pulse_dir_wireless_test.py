from machine import Pin
from micropython import const
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker
import gc
import utime

import config as cfg
from hardware import Motor


_WHEEL_FL = const(0)
_WHEEL_FR = const(1)
_WHEEL_B = const(2)
_WHEEL_COUNT = const(3)

_TEST_PWM = const(24000)
_RUN_MS = const(1500)
_SETTLE_MS = const(120)
_STOP_MS = const(400)
_DIR_SETTLE_MS = const(20)
_AUTO_START_DELAY_MS = const(1000)
_LOG_PERIOD_MS = const(100)
_LOOP_SLEEP_MS = const(5)

_WHEEL_NAMES = ("FL", "FR", "B")
_PULSE_PINS = (cfg.ENC_FL_PULSE, cfg.ENC_FR_PULSE, cfg.ENC_B_PULSE)
_DIR_PINS = (cfg.ENC_FL_DIR, cfg.ENC_FR_DIR, cfg.ENC_B_DIR)
_CURRENT_INVERT = (
    cfg.ENC_FL_INVERT,
    cfg.ENC_FR_INVERT,
    cfg.ENC_B_INVERT,
)

_pit_flag = False


def _pit_handler(_):
    global _pit_flag
    _pit_flag = True


def _sign_char(value):
    if value > 0:
        return "+"
    if value < 0:
        return "-"
    return "0"


def _phase_sign(positive_count, negative_count):
    if positive_count > negative_count:
        return 1
    if negative_count > positive_count:
        return -1
    return 0


class EncoderPulseDirTest:
    def __init__(self):
        gc.collect()

        self.wireless = None
        try:
            self.wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
        except Exception:
            pass

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

        # 测试时关闭软件反向，直接观察新编码器的原始方向极性。
        self.enc_fl = encoder(_DIR_PINS[0], _PULSE_PINS[0], False)
        self.enc_fr = encoder(_DIR_PINS[1], _PULSE_PINS[1], False)
        self.enc_b = encoder(_DIR_PINS[2], _PULSE_PINS[2], False)
        self.encoders = (self.enc_fl, self.enc_fr, self.enc_b)

        self.pit = ticker(1)
        self.pit.capture_list(self.enc_fl, self.enc_fr, self.enc_b)
        self.pit.callback(_pit_handler)
        self.pit.start(cfg.TICK_PERIOD_MS)

        self.key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
        self.led = Pin(
            cfg.LED_HB_PIN,
            Pin.OUT,
            pull=Pin.PULL_UP_47K,
            value=True,
        )
        self.led_fl = Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
        self.led_fr = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
        self.led_b = Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)
        self.wheel_leds = (self.led_fl, self.led_fr, self.led_b)

        self.exit_requested = False
        self.stop_requested = False

        try:
            gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
        except Exception:
            pass

    def log(self, message):
        if not isinstance(message, str):
            message = str(message)
        try:
            message = message.encode("ascii", "ignore").decode("ascii")
        except Exception:
            pass
        print(message)
        if self.wireless is not None:
            try:
                self.wireless.send_str(message)
                self.wireless.send_str("\r\n")
            except Exception:
                pass

    def log_config(self):
        self.log("ENCODER API ORDER=DIR,PULSE RAW_INV=0")
        self.log(
            "ENC FL PULSE=%s DIR=%s CURRENT_INV=%d"
            % (
                _PULSE_PINS[_WHEEL_FL],
                _DIR_PINS[_WHEEL_FL],
                1 if _CURRENT_INVERT[_WHEEL_FL] else 0,
            )
        )
        self.log(
            "ENC FR PULSE=%s DIR=%s CURRENT_INV=%d"
            % (
                _PULSE_PINS[_WHEEL_FR],
                _DIR_PINS[_WHEEL_FR],
                1 if _CURRENT_INVERT[_WHEEL_FR] else 0,
            )
        )
        self.log(
            "ENC B PULSE=%s DIR=%s CURRENT_INV=%d"
            % (
                _PULSE_PINS[_WHEEL_B],
                _DIR_PINS[_WHEEL_B],
                1 if _CURRENT_INVERT[_WHEEL_B] else 0,
            )
        )

    def _show_wheel(self, wheel):
        index = 0
        while index < _WHEEL_COUNT:
            self.wheel_leds[index].value(1 if index == wheel else 0)
            index += 1
        self.log("SELECT %s" % _WHEEL_NAMES[wheel])

    def _read_encoders(self):
        global _pit_flag
        _pit_flag = False
        return (
            int(self.enc_fl.get()),
            int(self.enc_fr.get()),
            int(self.enc_b.get()),
        )

    def _stop_motors(self):
        self.motor_fl.duty(0)
        self.motor_fr.duty(0)
        self.motor_b.duty(0)

    def _set_test_motor(self, wheel, pwm):
        self._stop_motors()
        motor = self.motors[wheel]
        direction = 1 if pwm > 0 else 0
        if motor.invert:
            direction = 1 - direction
        motor.pwm.duty_u16(0)
        motor.ph.value(direction)
        utime.sleep_ms(_DIR_SETTLE_MS)
        motor.pwm.duty_u16(abs(int(pwm)))
        return direction

    def _service_running_controls(self):
        if self.key_exit.value() == 0:
            self.exit_requested = True
            self.stop_requested = True
            self.log("C8 EMERGENCY STOP")

    def _wait_stopped(self, wait_ms):
        self._stop_motors()
        start_ms = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start_ms) < wait_ms:
            self._service_running_controls()
            if self.stop_requested:
                return False
            utime.sleep_ms(_LOOP_SLEEP_MS)
        return True

    def _run_phase(self, wheel, pwm, phase_name):
        direction = self._set_test_motor(wheel, pwm)
        start_ms = utime.ticks_ms()
        next_log_ms = start_ms
        positive_count = 0
        negative_count = 0
        nonzero_count = 0
        max_target = 0
        max_other_1 = 0
        max_other_2 = 0

        self.log(
            "PHASE %s %s PWM=%d DIR_OUT=%d MS=%d"
            % (_WHEEL_NAMES[wheel], phase_name, pwm, direction, _RUN_MS)
        )

        while utime.ticks_diff(utime.ticks_ms(), start_ms) < _RUN_MS:
            self._service_running_controls()
            if self.stop_requested:
                self._stop_motors()
                return None

            now_ms = utime.ticks_ms()
            if utime.ticks_diff(now_ms, next_log_ms) >= 0:
                next_log_ms = utime.ticks_add(now_ms, _LOG_PERIOD_MS)
                values = self._read_encoders()
                target_value = values[wheel]
                target_abs = abs(target_value)
                if target_abs > max_target:
                    max_target = target_abs

                other_1 = values[(wheel + 1) % _WHEEL_COUNT]
                other_2 = values[(wheel + 2) % _WHEEL_COUNT]
                other_1_abs = abs(other_1)
                other_2_abs = abs(other_2)
                if other_1_abs > max_other_1:
                    max_other_1 = other_1_abs
                if other_2_abs > max_other_2:
                    max_other_2 = other_2_abs

                elapsed_ms = utime.ticks_diff(now_ms, start_ms)
                if elapsed_ms >= _SETTLE_MS and target_value != 0:
                    nonzero_count += 1
                    if target_value > 0:
                        positive_count += 1
                    else:
                        negative_count += 1

                self.log(
                    "S %s %s T=%d PWM=%d DIR_OUT=%d ENC=%d,%d,%d SIGN=%s,%s,%s"
                    % (
                        _WHEEL_NAMES[wheel],
                        phase_name,
                        elapsed_ms,
                        pwm,
                        direction,
                        values[0],
                        values[1],
                        values[2],
                        _sign_char(values[0]),
                        _sign_char(values[1]),
                        _sign_char(values[2]),
                    )
                )

            utime.sleep_ms(_LOOP_SLEEP_MS)

        self._stop_motors()
        return (
            _phase_sign(positive_count, negative_count),
            nonzero_count,
            max_target,
            max_other_1,
            max_other_2,
        )

    def _log_result(self, wheel, forward, reverse):
        name = _WHEEL_NAMES[wheel]
        other_1_name = _WHEEL_NAMES[(wheel + 1) % _WHEEL_COUNT]
        other_2_name = _WHEEL_NAMES[(wheel + 2) % _WHEEL_COUNT]
        forward_sign = forward[0]
        reverse_sign = reverse[0]
        current_invert = 1 if _CURRENT_INVERT[wheel] else 0

        if forward[1] == 0 and reverse[1] == 0:
            result = "NO_PULSE CHECK_PULSE_PIN"
        elif forward_sign == 1 and reverse_sign == -1:
            result = "OK RECOMMEND_INV=0"
        elif forward_sign == -1 and reverse_sign == 1:
            result = "REVERSED RECOMMEND_INV=1"
        elif forward_sign == reverse_sign and forward_sign != 0:
            result = "DIR_STUCK CHECK_DIR_PIN"
        else:
            result = "UNSTABLE CHECK_WIRE_AND_PWM"

        self.log(
            "RESULT %s F=%s R=%s FMAX=%d RMAX=%d CURRENT_INV=%d %s"
            % (
                name,
                _sign_char(forward_sign),
                _sign_char(reverse_sign),
                forward[2],
                reverse[2],
                current_invert,
                result,
            )
        )
        self.log(
            "OTHER_MAX %s F_%s=%d F_%s=%d R_%s=%d R_%s=%d"
            % (
                name,
                other_1_name,
                forward[3],
                other_2_name,
                forward[4],
                other_1_name,
                reverse[3],
                other_2_name,
                reverse[4],
            )
        )

    def run_wheel_test(self, wheel):
        self.stop_requested = False
        self._show_wheel(wheel)
        self.log("TEST %s START" % _WHEEL_NAMES[wheel])
        gc.collect()

        if not self._wait_stopped(_STOP_MS):
            return
        forward = self._run_phase(wheel, _TEST_PWM, "FWD")
        if forward is None or not self._wait_stopped(_STOP_MS):
            return
        reverse = self._run_phase(wheel, -_TEST_PWM, "REV")
        if reverse is None:
            return
        self._wait_stopped(_STOP_MS)

        if not self.stop_requested:
            self._log_result(wheel, forward, reverse)
            self.log("TEST %s DONE" % _WHEEL_NAMES[wheel])
        gc.collect()

    def run_all_tests(self):
        wheel = _WHEEL_FL
        while wheel < _WHEEL_COUNT:
            self.run_wheel_test(wheel)
            if self.stop_requested or self.exit_requested:
                return
            wheel += 1
        self.log("ALL DONE")

    def run(self):
        self.log("=== PULSE DIR ENCODER TEST ===")
        self.log_config()
        self.log("AUTO RUN FL FR B; C8=EMERGENCY STOP")

        try:
            self.led.value(False)
            if self._wait_stopped(_AUTO_START_DELAY_MS):
                self.run_all_tests()
        finally:
            self._stop_motors()
            try:
                self.pit.stop()
            except Exception:
                pass
            self.led.value(True)
            self.led_fl.value(0)
            self.led_fr.value(0)
            self.led_b.value(0)
            gc.collect()
            self.log("=== TEST STOPPED ===")


def main():
    test = EncoderPulseDirTest()
    test.run()


main()
