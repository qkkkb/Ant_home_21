import gc
import utime
from machine import Pin
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker

import config as cfg
from hardware import Motor


LOG_PERIOD_MS = 200
TEST_MS = 1600
STOP_MS = 700
START_DELAY_MS = 600

PWM_1 = 12000
PWM_2 = 18000
PWM_3 = 24000
PWM_4 = 30000
SINGLE_WHEEL_PWM = 24000


wireless = None
pit1 = None
pit_flag = False
last_e_fl = 0
last_e_fr = 0
last_e_b = 0


def clamp_pwm(value):
    if value > cfg.MOTOR_DUTY_MAX:
        return cfg.MOTOR_DUTY_MAX
    if value < -cfg.MOTOR_DUTY_MAX:
        return -cfg.MOTOR_DUTY_MAX
    return int(value)


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


def update_encoder_cache():
    global pit_flag, last_e_fl, last_e_fr, last_e_b
    if not pit_flag:
        return
    pit_flag = False
    last_e_fl = enc_fl.get()
    last_e_fr = enc_fr.get()
    last_e_b = enc_b.get()


def set_pwm(fl_pwm, fr_pwm, b_pwm):
    motor_fl.duty(clamp_pwm(fl_pwm))
    motor_fr.duty(clamp_pwm(fr_pwm))
    motor_b.duty(clamp_pwm(b_pwm))


def stop_motors():
    try:
        set_pwm(0, 0, 0)
    except Exception:
        pass


def check_exit():
    if key_exit.value() == 0:
        raise KeyboardInterrupt


def wait_for_start():
    send_line("PWM_PROBE_READY press_C9_to_start C8_to_stop")
    while key_start.value() != 0:
        check_exit()
        utime.sleep_ms(20)
    utime.sleep_ms(40)
    while key_start.value() == 0:
        check_exit()
        utime.sleep_ms(20)
    send_line("PWM_PROBE_START")
    utime.sleep_ms(START_DELAY_MS)


def idle_ms(duration_ms):
    end_ms = utime.ticks_add(utime.ticks_ms(), duration_ms)
    set_pwm(0, 0, 0)
    while utime.ticks_diff(end_ms, utime.ticks_ms()) > 0:
        check_exit()
        update_encoder_cache()
        utime.sleep_ms(1)
    gc.collect()


def run_test(name, pwm, fl_pwm, fr_pwm, b_pwm):
    start_ms = utime.ticks_ms()
    next_log_ms = start_ms
    set_pwm(fl_pwm, fr_pwm, b_pwm)
    send_line("B %s %d %d %d %d" % (name, pwm, fl_pwm, fr_pwm, b_pwm))

    while utime.ticks_diff(utime.ticks_ms(), start_ms) < TEST_MS:
        check_exit()
        update_encoder_cache()
        now_ms = utime.ticks_ms()
        if utime.ticks_diff(now_ms, next_log_ms) >= 0:
            next_log_ms = utime.ticks_add(now_ms, LOG_PERIOD_MS)
            send_line(
                "T %s %d %d %d %d %d %d %d %d"
                % (
                    name,
                    pwm,
                    utime.ticks_diff(now_ms, start_ms),
                    fl_pwm,
                    fr_pwm,
                    b_pwm,
                    last_e_fl,
                    last_e_fr,
                    last_e_b,
                )
            )
        utime.sleep_ms(1)

    stop_motors()
    send_line("E %s %d %d %d %d" % (name, pwm, last_e_fl, last_e_fr, last_e_b))
    idle_ms(STOP_MS)


def run_chassis_tests(pwm):
    half_pwm = pwm // 2
    run_test("FWD", pwm, pwm, -pwm, 0)
    run_test("LAT", pwm, half_pwm, half_pwm, -pwm)
    run_test("SPIN", pwm, pwm, pwm, pwm)
    run_test("SPIN_NEG", pwm, -pwm, -pwm, -pwm)


def run_single_wheel_tests():
    pwm = SINGLE_WHEEL_PWM
    run_test("W_FL", pwm, pwm, 0, 0)
    run_test("W_FR", pwm, 0, pwm, 0)
    run_test("W_B", pwm, 0, 0, pwm)
    run_test("W_B_NEG", pwm, 0, 0, -pwm)


key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)

motor_fl = Motor(cfg.MOTOR_FL_PH, cfg.MOTOR_FL_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FL_INVERT)
motor_fr = Motor(cfg.MOTOR_FR_PH, cfg.MOTOR_FR_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FR_INVERT)
motor_b = Motor(cfg.MOTOR_B_PH, cfg.MOTOR_B_PWM, freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_B_INVERT)
enc_fl = encoder(cfg.ENC_FL_A, cfg.ENC_FL_B, cfg.ENC_FL_INVERT)
enc_fr = encoder(cfg.ENC_FR_A, cfg.ENC_FR_B, cfg.ENC_FR_INVERT)
enc_b = encoder(cfg.ENC_B_A, cfg.ENC_B_B, cfg.ENC_B_INVERT)

try:
    wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
except Exception:
    wireless = None

try:
    pit1 = ticker(1)
    pit1.capture_list(enc_fl, enc_fr, enc_b)
    pit1.callback(tick_handler)
    pit1.start(cfg.TICK_PERIOD_MS)

    gc.collect()
    wait_for_start()
    idle_ms(STOP_MS)

    run_chassis_tests(PWM_1)
    run_chassis_tests(PWM_2)
    run_chassis_tests(PWM_3)
    run_chassis_tests(PWM_4)
    run_single_wheel_tests()

    send_line("PWM_PROBE_DONE")
except KeyboardInterrupt:
    send_line("PWM_PROBE_STOP")
finally:
    stop_motors()
    if pit1 is not None:
        try:
            pit1.stop()
        except Exception:
            pass
    gc.collect()
