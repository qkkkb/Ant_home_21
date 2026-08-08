import gc
import utime

from machine import Pin
from micropython import const
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker

import config as cfg
from hardware import Motor


_WHEEL_FL = const(0)
_WHEEL_FR = const(1)
_WHEEL_B = const(2)
_TICK_MS = const(5)
_DIR_SETTLE_MS = const(20)
_PHASE_MS = const(1000)
_SAMPLE_SETTLE_MS = const(200)
_STOP_MS = const(250)
_SKIP_TICKS = const(4)

_PWM_LEVELS = (12000, 18000, 24000, 30000)
_WHEEL_NAMES = ("FL", "FR", "B")

_pit_pending = 0


def _pit_handler(_):
    global _pit_pending
    if _pit_pending < 100:
        _pit_pending += 1


def _take_pending():
    global _pit_pending
    count = _pit_pending
    _pit_pending = 0
    return count


wireless = None


def _log(message):
    print(message)
    if wireless is not None:
        try:
            wireless.send_str(message)
            wireless.send_str("\r\n")
        except Exception:
            pass


key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)

motor_fl = Motor(
    cfg.MOTOR_FL_PH,
    cfg.MOTOR_FL_PWM,
    freq=cfg.MOTOR_FREQ,
    invert=cfg.MOTOR_FL_INVERT,
)
motor_fr = Motor(
    cfg.MOTOR_FR_PH,
    cfg.MOTOR_FR_PWM,
    freq=cfg.MOTOR_FREQ,
    invert=cfg.MOTOR_FR_INVERT,
)
motor_b = Motor(
    cfg.MOTOR_B_PH,
    cfg.MOTOR_B_PWM,
    freq=cfg.MOTOR_FREQ,
    invert=cfg.MOTOR_B_INVERT,
)
motors = (motor_fl, motor_fr, motor_b)

enc_fl = encoder(cfg.ENC_FL_DIR, cfg.ENC_FL_PULSE, False)
enc_fr = encoder(cfg.ENC_FR_DIR, cfg.ENC_FR_PULSE, False)
enc_b = encoder(cfg.ENC_B_DIR, cfg.ENC_B_PULSE, False)

pit = ticker(1)
pit.capture_list(enc_fl, enc_fr, enc_b)
pit.callback(_pit_handler)


def _check_exit():
    if key_exit.value() == 0:
        raise KeyboardInterrupt


def _stop_motors():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)


def _clear_encoder_counts():
    enc_fl.get()
    enc_fr.get()
    enc_b.get()
    _take_pending()


def _wait_ms(wait_ms):
    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < wait_ms:
        _check_exit()
        utime.sleep_ms(1)


def _set_test_motor(wheel, pwm):
    _stop_motors()
    motor = motors[wheel]
    direction = 1 if pwm > 0 else 0
    if motor.invert:
        direction = 1 - direction
    motor.pwm.duty_u16(0)
    motor.ph.value(direction)
    _wait_ms(_DIR_SETTLE_MS)
    motor.pwm.duty_u16(abs(pwm))


def _read_target(wheel):
    value_fl = int(enc_fl.get())
    value_fr = int(enc_fr.get())
    value_b = int(enc_b.get())
    if wheel == _WHEEL_FL:
        return value_fl, abs(value_fr), abs(value_b)
    if wheel == _WHEEL_FR:
        return value_fr, abs(value_fl), abs(value_b)
    return value_b, abs(value_fl), abs(value_fr)


def _wait_for_tick():
    while _pit_pending == 0:
        _check_exit()
        utime.sleep_ms(1)
    return _take_pending()


def _run_skip_probe():
    _log("SKIP BEGIN WHEEL=FL PWM=24000")
    _set_test_motor(_WHEEL_FL, 24000)
    _wait_ms(_SAMPLE_SETTLE_MS)
    _clear_encoder_counts()

    base_sum = 0
    base_count = 0
    while base_count < 12:
        pending = _wait_for_tick()
        value, _, _ = _read_target(_WHEEL_FL)
        base_sum += value
        base_count += 1

    base = base_sum // base_count
    _clear_encoder_counts()
    while _pit_pending < _SKIP_TICKS:
        _check_exit()
        utime.sleep_ms(1)
    pending = _take_pending()
    accumulated, _, _ = _read_target(_WHEEL_FL)
    ratio_x100 = 0
    if base:
        ratio_x100 = abs(accumulated) * 100 // abs(base)
    _stop_motors()
    _log(
        "SKIP BASE=%d TICKS=%d ACC=%d RATIO_X100=%d"
        % (base, pending, accumulated, ratio_x100)
    )
    _wait_ms(_STOP_MS)


def _run_phase(wheel, pwm):
    _set_test_motor(wheel, pwm)
    _wait_ms(_SAMPLE_SETTLE_MS)
    _clear_encoder_counts()

    sample_count = 0
    sample_sum = 0
    sample_min = 32767
    sample_max = -32768
    zero_count = 0
    wrong_sign = 0
    late_events = 0
    late_ticks = 0
    other_1_max = 0
    other_2_max = 0
    start = utime.ticks_ms()

    while utime.ticks_diff(utime.ticks_ms(), start) < _PHASE_MS:
        pending = _wait_for_tick()
        value, other_1, other_2 = _read_target(wheel)
        if pending > 1:
            late_events += 1
            late_ticks += pending - 1

        sample_count += 1
        sample_sum += value
        if value < sample_min:
            sample_min = value
        if value > sample_max:
            sample_max = value
        if value == 0:
            zero_count += 1
        elif (pwm > 0 and value < 0) or (pwm < 0 and value > 0):
            wrong_sign += 1
        if other_1 > other_1_max:
            other_1_max = other_1
        if other_2 > other_2_max:
            other_2_max = other_2

    _stop_motors()
    average_x100 = 0
    if sample_count:
        average_x100 = sample_sum * 100 // sample_count
    _log(
        "R W=%s PWM=%d N=%d AVG_X100=%d MIN=%d MAX=%d ZERO=%d WRONG=%d LATE=%d/%d NOISE=%d,%d"
        % (
            _WHEEL_NAMES[wheel],
            pwm,
            sample_count,
            average_x100,
            sample_min,
            sample_max,
            zero_count,
            wrong_sign,
            late_events,
            late_ticks,
            other_1_max,
            other_2_max,
        )
    )
    _wait_ms(_STOP_MS)
    gc.collect()


def _run_all():
    _log("=== FOLLOWER ENCODER 5MS TEST ===")
    _log("API=DIR,PULSE INV=0 TICK_MS=5 DIR_WAIT_MS=20")
    _log("AVG_X100=average pulses per 5ms multiplied by 100")
    _log("C8=EMERGENCY STOP")
    _run_skip_probe()

    wheel = 0
    while wheel < 3:
        level_index = 0
        while level_index < len(_PWM_LEVELS):
            pwm = _PWM_LEVELS[level_index]
            _run_phase(wheel, pwm)
            _run_phase(wheel, -pwm)
            level_index += 1
        wheel += 1
    _log("=== TEST COMPLETE ===")


try:
    gc.collect()
    try:
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
    except Exception:
        pass
    wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
    pit.start(_TICK_MS)
    _run_all()
except KeyboardInterrupt:
    _log("TEST ABORTED")
finally:
    pit.stop()
    _stop_motors()
    _log("MOTORS STOPPED")
