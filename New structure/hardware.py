class VehicleHardware:
    def read_encoder_spd(self):
        ...

    def read_yaw(self):
        ...

    def read_gyro_z(self):
        ...

    def set_motor_pwm(self, pwm_fl, pwm_fr, pwm_b):
        ...

    def uart_write_byte(self, uart_name, value):
        ...

    def uart_write_str(self, uart_name, text):
        ...

    def gpio_get_level(self, pin_name):
        ...

    def image_solve(self):
        ...

    def curve_angle_get(self):
        ...

    def get_follow_line_state(self):
        ...

    def mt9v03x_frame_ready(self):
        ...

    def clear_frame_ready(self):
        ...

    def push_left_or_right(self, cls):
        ...

    def delay_ms(self, ms):
        ...


class NullHardware:
    def read_encoder_spd(self):
        return 0.0, 0.0, 0.0

    def read_yaw(self):
        return 0.0

    def read_gyro_z(self):
        return 0.0

    def set_motor_pwm(self, pwm_fl, pwm_fr, pwm_b):
        return None

    def uart_write_byte(self, uart_name, value):
        return None

    def uart_write_str(self, uart_name, text):
        return None

    def gpio_get_level(self, pin_name):
        return 0

    def image_solve(self):
        return 0.0

    def curve_angle_get(self):
        return 0.0, 0.0

    def get_follow_line_state(self):
        return "NORMAL"

    def mt9v03x_frame_ready(self):
        return False

    def clear_frame_ready(self):
        return None

    def push_left_or_right(self, cls):
        return 0

    def delay_ms(self, ms):
        return None


# ====================== 真实硬件层（MicroPython 运行时） ======================
# Motor 类从 main.py 迁入，原样保留，不修改任何逻辑
try:
    from machine import Pin, PWM, UART
    import config as _cfg
    from smartcar import encoder as _encoder

    class Motor:
        def __init__(self, ph_pin, pwm_pin, freq=13000, invert=False):
            self.invert = invert
            self.ph = Pin(ph_pin, Pin.OUT, value=0)
            self.pwm = PWM(pwm_pin, freq, duty_u16=0)

        def set_speed(self, speed):
            if speed == 0:
                self.pwm.duty_u16(0)
                return
            dir_val = 1 if speed > 0 else 0
            if self.invert:
                dir_val = 1 - dir_val
            self.ph.value(dir_val)
            self.pwm.duty_u16(abs(speed))

        def duty(self, value):
            if value == 0:
                self.pwm.duty_u16(0)
                return
            dir_val = 1 if value > 0 else 0
            if self.invert:
                dir_val = 1 - dir_val
            self.ph.value(dir_val)
            self.pwm.duty_u16(abs(value))

    class RealHardware:
        def __init__(self):
            self.motor_fl = Motor(_cfg.MOTOR_FL_PH, _cfg.MOTOR_FL_PWM,
                                  freq=_cfg.MOTOR_FREQ, invert=_cfg.MOTOR_FL_INVERT)
            self.motor_fr = Motor(_cfg.MOTOR_FR_PH, _cfg.MOTOR_FR_PWM,
                                  freq=_cfg.MOTOR_FREQ, invert=_cfg.MOTOR_FR_INVERT)
            self.motor_b  = Motor(_cfg.MOTOR_B_PH,  _cfg.MOTOR_B_PWM,
                                  freq=_cfg.MOTOR_FREQ, invert=_cfg.MOTOR_B_INVERT)
            self.enc_fl = _encoder(_cfg.ENC_FL_A, _cfg.ENC_FL_B, _cfg.ENC_FL_INVERT)
            self.enc_fr = _encoder(_cfg.ENC_FR_A, _cfg.ENC_FR_B, _cfg.ENC_FR_INVERT)
            self.enc_b  = _encoder(_cfg.ENC_B_A,  _cfg.ENC_B_B,  _cfg.ENC_B_INVERT)
            self.cam_uart = UART(_cfg.CAM_UART_ID, _cfg.CAM_UART_BAUD)
            self.cam_uart.init(_cfg.CAM_UART_BAUD, timeout_char=100)
            self._imu = None  # 如需 IMU 可在外部赋值

        def read_encoder_spd(self):
            return self.enc_fl.get(), self.enc_fr.get(), self.enc_b.get()

        def read_yaw(self):
            return self._imu.read_yaw() if self._imu else 0.0

        def read_gyro_z(self):
            return self._imu.read_gyro_z() if self._imu else 0.0

        def set_motor_pwm(self, pwm_fl, pwm_fr, pwm_b):
            def _apply(motor, v):
                v = int(v)
                if v > _cfg.PWM_MAX:
                    v = _cfg.PWM_MAX
                elif v < -_cfg.PWM_MAX:
                    v = -_cfg.PWM_MAX
                if 0 < abs(v) < _cfg.MOTOR_DUTY_MIN:
                    v = _cfg.MOTOR_DUTY_MIN if v > 0 else -_cfg.MOTOR_DUTY_MIN
                motor.duty(v)
            _apply(self.motor_fl, pwm_fl)
            _apply(self.motor_fr, pwm_fr)
            _apply(self.motor_b,  pwm_b)

        def uart_write_byte(self, uart_name, value):
            if uart_name == "art_detect":
                self.cam_uart.write(bytearray([value & 0xFF]))

        def uart_write_str(self, uart_name, text):
            if uart_name == "art_detect":
                self.cam_uart.write(text)

        def poll_cam_uart(self, channel):
            """非协议方法：主循环调用，把相机 UART 收到的字节压入 UartChannel"""
            if self.cam_uart.any():
                data = self.cam_uart.read()
                if data:
                    for b in data:
                        channel.push(b)

        def gpio_get_level(self, pin_name):
            return Pin(pin_name, Pin.IN).value()

        def image_solve(self):
            return 0.0

        def curve_angle_get(self):
            return 0.0, 0.0

        def get_follow_line_state(self):
            return "NORMAL"

        def mt9v03x_frame_ready(self):
            return False

        def clear_frame_ready(self):
            pass

        def push_left_or_right(self, cls):
            return 0

        def delay_ms(self, ms):
            import utime
            utime.sleep_ms(ms)

except ImportError:
    # PC 端 / 单元测试环境：machine/smartcar 不可用，跳过真实硬件类定义
    pass
