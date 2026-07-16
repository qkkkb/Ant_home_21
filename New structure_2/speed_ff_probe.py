import gc
import utime
from machine import Pin
from micropython import const
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker

import config as cfg
import pid as _pid_mod
from hardware import Motor
from models import SpeedPID


_LOG_PERIOD_MS = const(200)
_TEST_MS = const(1600)
_STEP_PHASE_MS = const(800)
_STOP_MS = const(700)
_START_DELAY_MS = const(600)
_STEADY_MS = const(400)
_PWM_FF_PER_SPEED = const(4000)
_PWM_LIMIT = const(40000)

_START_PWM = const(6200)
_START_PWM_MID = const(3600)
_START_PWM_LOW_TARGET_X10 = const(12)
_START_PWM_MID_TARGET_X10 = const(28)
_IDLE_TARGET_X10 = const(3)


_pid_mod.PWM_MAX = cfg.PWM_MAX
speed_ctrl = _pid_mod.speed_ctrl
speed_reset = _pid_mod.speed_reset

wireless = None
pit1 = None
key_exit = None
key_start = None
motor_fl = None
motor_fr = None
motor_b = None
enc_fl = None
enc_fr = None
enc_b = None
pid_fl = None
pid_fr = None
pid_b = None

pit_flag = False
actual_fl = 0
actual_fr = 0
actual_b = 0
target_fl = 0.0
target_fr = 0.0
target_b = 0.0
raw_fl = 0.0
raw_fr = 0.0
raw_b = 0.0
ff_fl = 0
ff_fr = 0
ff_b = 0
pwm_fl = 0
pwm_fr = 0
pwm_b = 0
sat_mask = 0
use_ff_mode = False


def send_line(text):
    print(text)
    if wireless is None:
        return
    try:
        wireless.send_str(text)
        wireless.send_str("\r\n")
    except Exception:
        pass


def tick_handler(_):
    global pit_flag
    pit_flag = True


def clamp_pwm(value):
    if value > _PWM_LIMIT:
        return _PWM_LIMIT
    if value < -_PWM_LIMIT:
        return -_PWM_LIMIT
    return int(value)


def smooth_pwm(target, last):
    delta = target - last
    if delta > cfg.MAX_PWM_CHANGE:
        target = last + cfg.MAX_PWM_CHANGE
    elif delta < -cfg.MAX_PWM_CHANGE:
        target = last - cfg.MAX_PWM_CHANGE
    target = int(last * (1.0 - cfg.PWM_SMOOTH_FACTOR) + target * cfg.PWM_SMOOTH_FACTOR)
    return clamp_pwm(target)


def target_idle(target):
    value_x10 = int(target * 10)
    return -_IDLE_TARGET_X10 <= value_x10 <= _IDLE_TARGET_X10


def start_pwm_for_target(target):
    value_x10 = int(abs(target) * 10)
    if value_x10 <= _IDLE_TARGET_X10:
        return 0
    if value_x10 < _START_PWM_LOW_TARGET_X10:
        return 0
    if value_x10 < _START_PWM_MID_TARGET_X10:
        return _START_PWM_MID
    return _START_PWM


def speed_output(pid, actual, target):
    if target_idle(target):
        speed_reset(pid)
        return 0.0
    return speed_ctrl(pid, actual, target)


def feedforward_pwm(target):
    if target_idle(target):
        return 0
    return int(target * _PWM_FF_PER_SPEED)


def apply_channel(raw, feedforward, target, last):
    command = raw
    if use_ff_mode:
        if target > 0.0 and command < feedforward:
            command = feedforward
        elif target < 0.0 and command > feedforward:
            command = feedforward
    minimum = start_pwm_for_target(target)
    if command > 0.0 and command < minimum:
        command = minimum
    elif command < 0.0 and command > -minimum:
        command = -minimum
    return smooth_pwm(clamp_pwm(command), last)


def update_control():
    global pit_flag, actual_fl, actual_fr, actual_b
    global raw_fl, raw_fr, raw_b, ff_fl, ff_fr, ff_b
    global pwm_fl, pwm_fr, pwm_b, sat_mask

    if not pit_flag:
        return False
    pit_flag = False
    actual_fl = enc_fl.get()
    actual_fr = enc_fr.get()
    actual_b = enc_b.get()

    raw_fl = speed_output(pid_fl, actual_fl, target_fl)
    raw_fr = speed_output(pid_fr, actual_fr, target_fr)
    raw_b = speed_output(pid_b, actual_b, target_b)
    ff_fl = feedforward_pwm(target_fl)
    ff_fr = feedforward_pwm(target_fr)
    ff_b = feedforward_pwm(target_b)

    command = raw_fl
    if use_ff_mode and (
        (target_fl > 0.0 and command < ff_fl)
        or (target_fl < 0.0 and command > ff_fl)
    ):
        command = ff_fl
    sat_mask = 0
    if abs(command) >= _PWM_LIMIT:
        sat_mask |= 1
    command = raw_fr
    if use_ff_mode and (
        (target_fr > 0.0 and command < ff_fr)
        or (target_fr < 0.0 and command > ff_fr)
    ):
        command = ff_fr
    if abs(command) >= _PWM_LIMIT:
        sat_mask |= 2
    command = raw_b
    if use_ff_mode and (
        (target_b > 0.0 and command < ff_b)
        or (target_b < 0.0 and command > ff_b)
    ):
        command = ff_b
    if abs(command) >= _PWM_LIMIT:
        sat_mask |= 4

    pwm_fl = apply_channel(raw_fl, ff_fl, target_fl, pwm_fl)
    pwm_fr = apply_channel(raw_fr, ff_fr, target_fr, pwm_fr)
    pwm_b = apply_channel(raw_b, ff_b, target_b, pwm_b)
    motor_fl.duty(pwm_fl)
    motor_fr.duty(pwm_fr)
    motor_b.duty(pwm_b)
    return True


def stop_motors():
    global pwm_fl, pwm_fr, pwm_b
    pwm_fl = 0
    pwm_fr = 0
    pwm_b = 0
    if motor_fl is not None:
        motor_fl.duty(0)
        motor_fr.duty(0)
        motor_b.duty(0)


def reset_speed_loop():
    speed_reset(pid_fl)
    speed_reset(pid_fr)
    speed_reset(pid_b)
    stop_motors()


def check_exit():
    if key_exit.value() == 0:
        raise KeyboardInterrupt


def wait_for_start():
    send_line("SPEED_FF_PROBE_READY press_C9_to_start C8_to_stop")
    while key_start.value() != 0:
        check_exit()
        utime.sleep_ms(20)
    utime.sleep_ms(40)
    while key_start.value() == 0:
        check_exit()
        utime.sleep_ms(20)
    send_line(
        "CFG tick=%d kff=%d pwm_limit=%d test_ms=%d"
        % (cfg.TICK_PERIOD_MS, _PWM_FF_PER_SPEED, _PWM_LIMIT, _TEST_MS)
    )
    utime.sleep_ms(_START_DELAY_MS)


def idle_ms(duration_ms):
    global target_fl, target_fr, target_b, pit_flag
    target_fl = 0.0
    target_fr = 0.0
    target_b = 0.0
    reset_speed_loop()
    end_ms = utime.ticks_add(utime.ticks_ms(), duration_ms)
    while utime.ticks_diff(end_ms, utime.ticks_ms()) > 0:
        check_exit()
        if pit_flag:
            pit_flag = False
            enc_fl.get()
            enc_fr.get()
            enc_b.get()
        utime.sleep_ms(1)
    gc.collect()


def target_reached(actual, target):
    if target > 0.0:
        return actual * 10 >= target * 8
    if target < 0.0:
        return actual * 10 <= target * 8
    return True


def set_stage_targets(axis_name, target):
    global target_fl, target_fr, target_b

    value = float(target)
    if axis_name == "FWD":
        target_fl = value
        target_fr = -value
        target_b = 0.0
    elif axis_name == "LAT":
        target_fl = value * 0.5
        target_fr = value * 0.5
        target_b = -value
    elif axis_name == "ROT":
        target_fl = value
        target_fr = value
        target_b = value
    else:
        target_fl = value * 0.75
        target_fr = -value * 0.85
        target_b = value


def set_step_targets(phase_b):
    global target_fl, target_fr, target_b

    if phase_b:
        target_fl = 10.0
        target_fr = -8.75
        target_b = 0.0
    else:
        target_fl = 0.5
        target_fr = -9.5
        target_b = 5.0


def send_sample(elapsed):
    send_line("S %d %d" % (elapsed, sat_mask))
    send_line(
        "E %d %d %d %d %d %d"
        % (
            actual_fl,
            actual_fr,
            actual_b,
            int(target_fl),
            int(target_fr),
            int(target_b),
        )
    )
    send_line(
        "P %d %d %d %d %d %d"
        % (pwm_fl, pwm_fr, pwm_b, int(raw_fl), int(raw_fr), int(raw_b))
    )
    send_line("F %d %d %d" % (ff_fl, ff_fr, ff_b))
    send_line(
        "V %d %d %d %d %d %d"
        % (
            int((target_fl - target_fr) * 10 / 1.73205),
            int((target_fl + target_fr - 2 * target_b) * 10 / 3),
            int((target_fl + target_fr + target_b) * 10 / 3),
            int((actual_fl - actual_fr) * 10 / 1.73205),
            int((actual_fl + actual_fr - 2 * actual_b) * 10 / 3),
            int((actual_fl + actual_fr + actual_b) * 10 / 3),
        )
    )


def run_stage(mode_name, ff_enabled, axis_name, target):
    global use_ff_mode

    set_stage_targets(axis_name, target)
    use_ff_mode = ff_enabled
    reset_speed_loop()

    send_line(
        "B %s %s %d %d %d %d"
        % (mode_name, axis_name, target, int(target_fl), int(target_fr), int(target_b))
    )
    start_ms = utime.ticks_ms()
    next_log_ms = start_ms
    rise_fl = -1
    rise_fr = -1
    rise_b = -1
    sum_fl = 0
    sum_fr = 0
    sum_b = 0
    steady_count = 0
    stage_sat_mask = 0

    while utime.ticks_diff(utime.ticks_ms(), start_ms) < _TEST_MS:
        check_exit()
        if update_control():
            elapsed = utime.ticks_diff(utime.ticks_ms(), start_ms)
            if rise_fl < 0 and target_reached(actual_fl, target_fl):
                rise_fl = elapsed
            if rise_fr < 0 and target_reached(actual_fr, target_fr):
                rise_fr = elapsed
            if rise_b < 0 and target_reached(actual_b, target_b):
                rise_b = elapsed
            if elapsed >= _TEST_MS - _STEADY_MS:
                sum_fl += actual_fl
                sum_fr += actual_fr
                sum_b += actual_b
                steady_count += 1
            stage_sat_mask |= sat_mask

        now_ms = utime.ticks_ms()
        if utime.ticks_diff(now_ms, next_log_ms) >= 0:
            next_log_ms = utime.ticks_add(now_ms, _LOG_PERIOD_MS)
            send_sample(utime.ticks_diff(now_ms, start_ms))
        utime.sleep_ms(1)

    if steady_count > 0:
        sum_fl //= steady_count
        sum_fr //= steady_count
        sum_b //= steady_count
    send_line(
        "R %s %s %d %d %d %d %d"
        % (mode_name, axis_name, target, sum_fl, sum_fr, sum_b, stage_sat_mask)
    )
    send_line("Q %d %d %d" % (rise_fl, rise_fr, rise_b))
    idle_ms(_STOP_MS)


def run_step_stage(mode_name, ff_enabled):
    global use_ff_mode

    set_step_targets(False)
    use_ff_mode = ff_enabled
    reset_speed_loop()
    send_line("B %s STEP A 5 -95 50" % mode_name)
    start_ms = utime.ticks_ms()
    next_log_ms = start_ms
    phase_b = False
    stage_sat_mask = 0

    while utime.ticks_diff(utime.ticks_ms(), start_ms) < _STEP_PHASE_MS * 2:
        check_exit()
        elapsed = utime.ticks_diff(utime.ticks_ms(), start_ms)
        if (not phase_b) and elapsed >= _STEP_PHASE_MS:
            set_step_targets(True)
            phase_b = True
            send_line("C %s STEP B 100 -87 0" % mode_name)
        if update_control():
            stage_sat_mask |= sat_mask

        now_ms = utime.ticks_ms()
        if utime.ticks_diff(now_ms, next_log_ms) >= 0:
            next_log_ms = utime.ticks_add(now_ms, _LOG_PERIOD_MS)
            send_sample(utime.ticks_diff(now_ms, start_ms))
        utime.sleep_ms(1)

    send_line("R %s STEP %d" % (mode_name, stage_sat_mask))
    idle_ms(_STOP_MS)


def run_suite():
    run_stage("PID", False, "FWD", 6)
    run_stage("PID", False, "LAT", 6)
    run_stage("PID", False, "ROT", 6)
    run_stage("PID", False, "MIX", 10)
    run_step_stage("PID", False)

    run_stage("FF4K", True, "FWD", 6)
    run_stage("FF4K", True, "LAT", 6)
    run_stage("FF4K", True, "ROT", 6)
    run_stage("FF4K", True, "MIX", 10)
    run_step_stage("FF4K", True)


def init_hardware():
    global wireless, pit1, key_exit, key_start
    global motor_fl, motor_fr, motor_b, enc_fl, enc_fr, enc_b
    global pid_fl, pid_fr, pid_b

    key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
    key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)
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
    enc_fl = encoder(cfg.ENC_FL_A, cfg.ENC_FL_B, cfg.ENC_FL_INVERT)
    enc_fr = encoder(cfg.ENC_FR_A, cfg.ENC_FR_B, cfg.ENC_FR_INVERT)
    enc_b = encoder(cfg.ENC_B_A, cfg.ENC_B_B, cfg.ENC_B_INVERT)
    pid_fl = SpeedPID()
    pid_fr = SpeedPID()
    pid_b = SpeedPID()
    pid_fl.init_c()
    pid_fr.init_c()
    pid_b.init_c()
    try:
        wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
    except Exception:
        wireless = None

    pit1 = ticker(1)
    pit1.capture_list(enc_fl, enc_fr, enc_b)
    pit1.callback(tick_handler)
    pit1.start(cfg.TICK_PERIOD_MS)


try:
    gc.collect()
    init_hardware()
    wait_for_start()
    idle_ms(_STOP_MS)
    send_line("SPEED_FF_PROBE_START")
    run_suite()
    send_line("SPEED_FF_PROBE_DONE")
except KeyboardInterrupt:
    send_line("SPEED_FF_PROBE_STOP")
finally:
    stop_motors()
    if pit1 is not None:
        try:
            pit1.stop()
        except Exception:
            pass
    gc.collect()
