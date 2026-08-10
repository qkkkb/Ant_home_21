import gc
import utime

from machine import Pin, UART
from seekfree import WIRELESS_UART
from smartcar import encoder, ticker

import config as cfg
from hardware import Motor


_RUN_MS = 1000
_SETTLE_MS = 250
_STOP_MS = 400
_LOG_MS = 200
_PWM_LEVELS = (12000, 30000, 60000)

_pit_pending = 0
_wireless = None


def _pit_handler(_):
    global _pit_pending
    if _pit_pending < 100:
        _pit_pending += 1


def _send(text):
    print(text)
    if _wireless is not None:
        try:
            _wireless.send_str(text)
            _wireless.send_str("\r\n")
        except Exception:
            pass


def _check_exit():
    if key_exit.value() == 0:
        raise KeyboardInterrupt


def _wait_ms(duration):
    end = utime.ticks_add(utime.ticks_ms(), duration)
    while utime.ticks_diff(end, utime.ticks_ms()) > 0:
        _check_exit()
        utime.sleep_ms(1)


def _read_all():
    global _pit_pending
    raw_fl = int(enc_fl.get())
    raw_fr = int(enc_fr.get())
    raw_b = int(enc_b.get())
    _pit_pending = 0
    return raw_fl, raw_fr, raw_b


def _stop_all():
    motor_fl.duty(0)
    motor_fr.duty(0)
    motor_b.duty(0)


def _run_direction(pwm):
    _stop_all()
    _wait_ms(_STOP_MS)
    _read_all()

    applied_pwm = motor_fr.duty(pwm)
    _wait_ms(25)
    applied_pwm = motor_fr.duty(pwm)
    _wait_ms(_SETTLE_MS - 25)
    _read_all()

    sample_count = 0
    sum_raw = 0
    zero_count = 0
    wrong_count = 0
    min_raw = 32767
    max_raw = -32768
    next_log = utime.ticks_ms()
    end = utime.ticks_add(next_log, _RUN_MS)

    while utime.ticks_diff(end, utime.ticks_ms()) > 0:
        _check_exit()
        while _pit_pending == 0:
            _check_exit()
            utime.sleep_ms(1)
        raw_fl, raw_fr, raw_b = _read_all()
        sample_count += 1
        sum_raw += raw_fr
        if raw_fr == 0:
            zero_count += 1
        elif (pwm > 0 and raw_fr < 0) or (pwm < 0 and raw_fr > 0):
            wrong_count += 1
        if raw_fr < min_raw:
            min_raw = raw_fr
        if raw_fr > max_raw:
            max_raw = raw_fr

        now = utime.ticks_ms()
        if utime.ticks_diff(now, next_log) >= 0:
            avg_raw = 0
            if sample_count:
                avg_raw = sum_raw // sample_count
            _send(
                "T PWM=%d N=%d RAW_FR=%d RAW_FL=%d RAW_B=%d PH=%d"
                % (pwm, sample_count, raw_fr, raw_fl, raw_b, motor_fr.ph.value())
            )
            next_log = utime.ticks_add(next_log, _LOG_MS)

    _stop_all()
    avg_raw = 0
    avg_x100 = 0
    if sample_count:
        avg_raw = sum_raw // sample_count
        avg_x100 = int(avg_raw * cfg.ENC_SCALE * 100.0)
    _send(
        "R PWM=%d APPLIED=%d N=%d AVG_RAW=%d AVG_X100=%d MIN=%d MAX=%d ZERO=%d WRONG=%d PH=%d"
        % (
            pwm,
            applied_pwm,
            sample_count,
            avg_raw,
            avg_x100,
            min_raw,
            max_raw,
            zero_count,
            wrong_count,
            motor_fr.ph.value(),
        )
    )
    _wait_ms(_STOP_MS)
    gc.collect()


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

enc_fl = encoder(cfg.ENC_FL_DIR, cfg.ENC_FL_PULSE, cfg.ENC_FL_INVERT)
enc_fr = encoder(cfg.ENC_FR_DIR, cfg.ENC_FR_PULSE, cfg.ENC_FR_INVERT)
enc_b = encoder(cfg.ENC_B_DIR, cfg.ENC_B_PULSE, cfg.ENC_B_INVERT)

pit = ticker(1)
pit.capture_list(enc_fl, enc_fr, enc_b)
pit.callback(_pit_handler)

try:
    gc.collect()
    try:
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
    except Exception:
        pass
    try:
        _wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
    except Exception:
        _wireless = None
    try:
        cam_uart = UART(cfg.CAM_UART_ID, cfg.CAM_UART_BAUD)
        cam_uart.init(cfg.CAM_UART_BAUD, timeout_char=100)
        cam_uart.write(b"TRACK\n")
    except Exception:
        pass

    pit.start(cfg.TICK_PERIOD_MS)
    _send("=== FR RUNTIME CHAIN TEST ===")
    _send("CFG PWM=C30 PH=C31 ENC=PULSE:C2 DIR:C3 SCALE=%s" % cfg.ENC_SCALE)
    _send("C8=STOP WHEEL=FR ONLY OTHER MOTORS=0")
    for level in _PWM_LEVELS:
        _run_direction(level)
        _run_direction(-level)
    _send("=== TEST COMPLETE ===")
except KeyboardInterrupt:
    _send("TEST ABORTED")
finally:
    pit.stop()
    _stop_all()
    _send("MOTORS STOPPED")
