"""前右轮延长轴公转测试。

电脑端执行时只打印几何计算结果；复制为设备上的 ``main.py`` 后，
会自动低速运行轮速闭环测试。C8 为急停键，日志通过无线 UART 发送。
"""

# 车体坐标：x 向前，y 向左。轮位和轴偏移单位为 mm。
WHEEL_FR_X = 70
WHEEL_FR_Y = -45
AXIS_FROM_FR_X = 0
AXIS_FROM_FR_Y = 100

AXIS_X = WHEEL_FR_X + AXIS_FROM_FR_X
AXIS_Y = WHEEL_FR_Y + AXIS_FROM_FR_Y

# 轮速单位沿用现有编码器速度闭环。修改符号可切换公转方向。
TEST_WZ = 0.10
BODY_VX = AXIS_Y * TEST_WZ
BODY_VY = -AXIS_X * TEST_WZ
TARGET_FR = -0.866025 * BODY_VX + 0.5 * BODY_VY + TEST_WZ
TARGET_FL = 0.866025 * BODY_VX + 0.5 * BODY_VY + TEST_WZ
TARGET_B = -BODY_VY + TEST_WZ


def wheel_targets(vx, vy, wz):
    """返回 (FR, FL, B)，使 AXIS_X/AXIS_Y 点成为瞬时旋转轴。"""
    return (
        -0.866025 * vx + 0.5 * vy + wz,
        0.866025 * vx + 0.5 * vy + wz,
        -vy + wz,
    )


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
    _LOG_BUF_SIZE = const(192)
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
        """使用固定 bytearray 发一行，避免格式化字符串反复上堆。"""
        buf = log_buf
        pos = 0
        buf[pos] = 84  # T
        pos += 1
        buf[pos] = 32
        pos += 1
        pos = _put_int(buf, pos, elapsed)
        pos = _put_int(buf, pos, elapsed * 360 // _RUN_MS)
        pos = _put_int(buf, pos, TARGET_FR * 100)
        pos = _put_int(buf, pos, TARGET_FL * 100)
        pos = _put_int(buf, pos, TARGET_B * 100)
        pos = _put_int(buf, pos, actual_fr * 100)
        pos = _put_int(buf, pos, actual_fl * 100)
        pos = _put_int(buf, pos, actual_b * 100)
        pos = _put_int(buf, pos, pwm_fr)
        pos = _put_int(buf, pos, pwm_fl)
        pos = _put_int(buf, pos, pwm_b)
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
        global pit_flag, pwm_fl, pwm_fr, pwm_b
        if not pit_flag:
            return False
        pit_flag = False
        _read_encoders()
        out_fl = _speed_output(pid_fl, actual_fl, TARGET_FL)
        out_fr = _speed_output(pid_fr, actual_fr, TARGET_FR)
        out_b = _speed_output(pid_b, actual_b, TARGET_B)
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
        end_ms = utime.ticks_add(utime.ticks_ms(), duration_ms)
        while utime.ticks_diff(end_ms, utime.ticks_ms()) > 0:
            _check_exit()
            _wait_tick()
        _stop_all()
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
        _send_text("A AXIS_X=%d AXIS_Y=%d" % (AXIS_X, AXIS_Y))
        _send_text(
            "CFG DIR=%s BODY_VX=%.3f BODY_VY=%.3f WZ=%.3f" %
            ("CCW" if TEST_WZ >= 0 else "CW", BODY_VX, BODY_VY, TEST_WZ)
        )
        _send_text(
            "TARGET FR=%.3f FL=%.3f B=%.3f RUN_MS=%d" %
            (TARGET_FR, TARGET_FL, TARGET_B, _RUN_MS)
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
        targets = wheel_targets(BODY_VX, BODY_VY, TEST_WZ)
        print("Follower front-right extension-axis geometry")
        print("axis=(%d,%d) body=(%.3f,%.3f,%.3f)" %
              (AXIS_X, AXIS_Y, BODY_VX, BODY_VY, TEST_WZ))
        print("targets FR=%.3f FL=%.3f B=%.3f" % targets)


if _DEVICE:
    _device_main()
else:
    _pc_main()
