"""Standalone state-16 wheel-space spin test.

On a PC this prints the virtual leader and follower wheel targets. When this
file is copied to a board as main.py it runs the same wheel-space allocation
used by the follower during a leader search spin. C8 is an emergency stop.
"""

# Negative wheel rate is the clockwise direction used by the vehicle.
MASTER_WHEEL_RATE = -6.0
MASTER_FL = MASTER_WHEEL_RATE
MASTER_FR = MASTER_WHEEL_RATE
MASTER_B = MASTER_WHEEL_RATE

# Keep these values identical to the production state-16 allocator.
ANCHOR_GAIN = 1.05
DRIVE_GAIN = 2.0
TARGET_RAMP = 1.2

MASTER_WHEEL_MEAN = (MASTER_FL + MASTER_FR + MASTER_B) * 0.3333333
FOLLOW_ANCHOR_FR = MASTER_FL * ANCHOR_GAIN
FOLLOW_CENTER = (3.0 * MASTER_WHEEL_MEAN - FOLLOW_ANCHOR_FR) * 0.5
FOLLOW_DRIVE = MASTER_WHEEL_MEAN * DRIVE_GAIN
FOLLOW_FL_GOAL = FOLLOW_CENTER + FOLLOW_DRIVE
FOLLOW_FR_GOAL = FOLLOW_ANCHOR_FR
FOLLOW_B_GOAL = FOLLOW_CENTER - FOLLOW_DRIVE


try:
    from machine import Pin
    _DEVICE = True
except ImportError:
    _DEVICE = False


if _DEVICE:
    import gc
    import utime

    from micropython import const
    from seekfree import WIRELESS_UART
    from smartcar import encoder, ticker

    import config as cfg
    import pid as pid_mod
    from hardware import Motor

    _RUN_MS = const(4000)
    _START_DELAY_MS = const(500)
    _LOG_MS = const(100)
    _LOG_BUF_SIZE = const(224)
    _PWM_LIMIT = const(36000)
    _PWM_STEP = const(4800)

    wireless = None
    pit = None
    key_exit = None
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
    actual_fl = 0.0
    actual_fr = 0.0
    actual_b = 0.0
    target_fl = 0.0
    target_fr = 0.0
    target_b = 0.0
    pwm_fl = 0
    pwm_fr = 0
    pwm_b = 0
    log_buf = None


    def _pit_handler(_):
        global pit_flag
        pit_flag = True


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


    def _send_text(text):
        print(text)
        if wireless is None:
            return
        try:
            wireless.send_str(text)
            wireless.send_str("\r\n")
        except Exception:
            pass


    def _send_sample(elapsed):
        """Send one fixed-buffer sample; no formatted string in the hot loop."""
        buf = log_buf
        pos = 0
        buf[pos] = 84  # T
        pos += 1
        buf[pos] = 32
        pos += 1
        pos = _put_int(buf, pos, elapsed)
        pos = _put_int(buf, pos, elapsed * 360 // _RUN_MS)
        pos = _put_int(buf, pos, MASTER_FL * 100)
        pos = _put_int(buf, pos, MASTER_FR * 100)
        pos = _put_int(buf, pos, MASTER_B * 100)
        pos = _put_int(buf, pos, target_fl * 100)
        pos = _put_int(buf, pos, target_fr * 100)
        pos = _put_int(buf, pos, target_b * 100)
        pos = _put_int(buf, pos, actual_fl * 100)
        pos = _put_int(buf, pos, actual_fr * 100)
        pos = _put_int(buf, pos, actual_b * 100)
        pos = _put_int(buf, pos, pwm_fl)
        pos = _put_int(buf, pos, pwm_fr)
        pos = _put_int(buf, pos, pwm_b)
        pos = _put_int(buf, pos, (target_fr - MASTER_FL) * 100)
        buf[pos - 1] = 13
        buf[pos] = 10
        try:
            wireless.send_bytearray(buf, pos + 1)
        except Exception:
            pass


    def _check_exit():
        if key_exit.value() == 0:
            raise KeyboardInterrupt


    def _reset_pid(pid):
        pid_mod.speed_reset(pid)


    def _ramp_value(goal, value, step):
        delta = goal - value
        if delta > step:
            return value + step
        if delta < -step:
            return value - step
        return goal


    def _clamp_pwm(value):
        if value > _PWM_LIMIT:
            return _PWM_LIMIT
        if value < -_PWM_LIMIT:
            return -_PWM_LIMIT
        return int(value)


    def _smooth_pwm(target, last):
        if target == 0:
            return 0
        delta = target - last
        if delta > _PWM_STEP:
            target = last + _PWM_STEP
        elif delta < -_PWM_STEP:
            target = last - _PWM_STEP
        value = int(last * 0.6 + target * 0.4)
        value = _clamp_pwm(value)
        if 0 < value < cfg.MOTOR_DUTY_MIN:
            return cfg.MOTOR_DUTY_MIN
        if -cfg.MOTOR_DUTY_MIN < value < 0:
            return -cfg.MOTOR_DUTY_MIN
        return value


    def _minimum_pwm(target):
        target_abs = abs(target)
        if target_abs < 0.03:
            return 0
        if target_abs < 2.0:
            return 3600
        return 6200


    def _speed_output(pid, actual, target):
        if abs(target) < 0.03:
            _reset_pid(pid)
            return 0
        value = pid_mod.speed_ctrl(pid, actual, target)
        minimum = _minimum_pwm(target)
        if 0 < value < minimum:
            value = minimum
        elif -minimum < value < 0:
            value = -minimum
        return _clamp_pwm(value)


    def _read_encoders():
        global actual_fl, actual_fr, actual_b
        actual_fl = enc_fl.get() * cfg.ENC_SCALE
        actual_fr = enc_fr.get() * cfg.ENC_SCALE
        actual_b = enc_b.get() * cfg.ENC_SCALE


    def _update_control():
        global pit_flag, target_fl, target_fr, target_b
        global pwm_fl, pwm_fr, pwm_b
        if not pit_flag:
            return False
        pit_flag = False
        _read_encoders()
        target_fl = _ramp_value(FOLLOW_FL_GOAL, target_fl, TARGET_RAMP)
        target_fr = _ramp_value(FOLLOW_FR_GOAL, target_fr, TARGET_RAMP)
        target_b = _ramp_value(FOLLOW_B_GOAL, target_b, TARGET_RAMP)
        out_fl = _speed_output(pid_fl, actual_fl, target_fl)
        out_fr = _speed_output(pid_fr, actual_fr, target_fr)
        out_b = _speed_output(pid_b, actual_b, target_b)
        pwm_fl = _smooth_pwm(out_fl, pwm_fl)
        pwm_fr = _smooth_pwm(out_fr, pwm_fr)
        pwm_b = _smooth_pwm(out_b, pwm_b)
        pwm_fl = motor_fl.duty(pwm_fl)
        pwm_fr = motor_fr.duty(pwm_fr)
        pwm_b = motor_b.duty(pwm_b)
        return True


    def _stop_all():
        global pwm_fl, pwm_fr, pwm_b
        pwm_fl = 0
        pwm_fr = 0
        pwm_b = 0
        if motor_fl is not None:
            motor_fl.duty(0)
            motor_fr.duty(0)
            motor_b.duty(0)


    def _wait_tick():
        global pit_flag
        while not pit_flag:
            _check_exit()
            utime.sleep_ms(1)
        pit_flag = False
        _read_encoders()


    def _idle(duration_ms):
        global target_fl, target_fr, target_b
        end_ms = utime.ticks_add(utime.ticks_ms(), duration_ms)
        while utime.ticks_diff(end_ms, utime.ticks_ms()) > 0:
            _check_exit()
            _wait_tick()
        _stop_all()
        target_fl = target_fr = target_b = 0.0
        _reset_pid(pid_fl)
        _reset_pid(pid_fr)
        _reset_pid(pid_b)
        gc.collect()


    def _init_hardware():
        global wireless, pit, key_exit, log_buf
        global motor_fl, motor_fr, motor_b
        global enc_fl, enc_fr, enc_b
        global pid_fl, pid_fr, pid_b

        key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
        try:
            wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
        except Exception as exc:
            wireless = None
            print("WIRELESS ERROR", exc)
        log_buf = bytearray(_LOG_BUF_SIZE)
        _send_text("BOOT 1 WIRELESS BAUD=460800")

        motor_fl = Motor(
            cfg.MOTOR_FL_PH, cfg.MOTOR_FL_PWM,
            freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FL_INVERT,
        )
        motor_fr = Motor(
            cfg.MOTOR_FR_PH, cfg.MOTOR_FR_PWM,
            freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_FR_INVERT,
        )
        motor_b = Motor(
            cfg.MOTOR_B_PH, cfg.MOTOR_B_PWM,
            freq=cfg.MOTOR_FREQ, invert=cfg.MOTOR_B_INVERT,
        )
        _send_text("BOOT 2 MOTORS")

        enc_fl = encoder(cfg.ENC_FL_DIR, cfg.ENC_FL_PULSE, cfg.ENC_FL_INVERT)
        enc_fr = encoder(cfg.ENC_FR_DIR, cfg.ENC_FR_PULSE, cfg.ENC_FR_INVERT)
        enc_b = encoder(cfg.ENC_B_DIR, cfg.ENC_B_PULSE, cfg.ENC_B_INVERT)
        pid_fl = pid_mod.SpeedPID()
        pid_fr = pid_mod.SpeedPID()
        pid_b = pid_mod.SpeedPID()
        _send_text("BOOT 3 ENCODERS PID")

        pit = ticker(1)
        pit.capture_list(enc_fl, enc_fr, enc_b)
        pit.callback(_pit_handler)
        pit.start(cfg.TICK_PERIOD_MS)
        _send_text("BOOT 4 TICKER")


    def _run_device_test():
        _send_text("STATE16 VIRTUAL_MASTER DIR=CW")
        _send_text(
            "MASTER FL=%.3f FR=%.3f B=%.3f" %
            (MASTER_FL, MASTER_FR, MASTER_B)
        )
        _send_text(
            "FOLLOW_GOAL FL=%.3f FR=%.3f B=%.3f LOCK_ERR=%.3f" %
            (FOLLOW_FL_GOAL, FOLLOW_FR_GOAL, FOLLOW_B_GOAL,
             FOLLOW_FR_GOAL - MASTER_FL)
        )
        _send_text("READY AUTO_START C8=STOP")
        _idle(_START_DELAY_MS)
        _send_text("RUN NOMINAL_DEG=360")

        start_ms = utime.ticks_ms()
        next_log_ms = start_ms
        end_ms = utime.ticks_add(start_ms, _RUN_MS)
        while utime.ticks_diff(end_ms, utime.ticks_ms()) > 0:
            _check_exit()
            _update_control()
            now_ms = utime.ticks_ms()
            if utime.ticks_diff(now_ms, next_log_ms) >= 0:
                next_log_ms = utime.ticks_add(next_log_ms, _LOG_MS)
                _send_sample(utime.ticks_diff(now_ms, start_ms))
            utime.sleep_ms(1)

        _stop_all()
        _send_text("DONE")


    def _device_main():
        global pit
        print("BOOT 0 START")
        try:
            gc.collect()
            _init_hardware()
            try:
                gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
            except Exception:
                pass
            _run_device_test()
        except KeyboardInterrupt:
            _send_text("STOP KEYBOARD/C8")
        except Exception as exc:
            _send_text("ERROR %s" % exc)
        finally:
            _stop_all()
            if pit is not None:
                try:
                    pit.stop()
                except Exception:
                    pass
            _send_text("MOTORS STOPPED")


else:

    def _pc_main():
        print("Virtual state-16 spin test")
        print("MASTER FL=%.3f FR=%.3f B=%.3f" %
              (MASTER_FL, MASTER_FR, MASTER_B))
        print("FOLLOW FL=%.3f FR=%.3f B=%.3f LOCK_ERR=%.3f" %
              (FOLLOW_FL_GOAL, FOLLOW_FR_GOAL, FOLLOW_B_GOAL,
               FOLLOW_FR_GOAL - MASTER_FL))


if _DEVICE:
    _device_main()
else:
    _pc_main()
