from machine import Pin, PWM
import utime


_DIR_WAIT_MS = 20


class Motor:
    def __init__(self, ph_pin, pwm_pin, freq=13000, invert=False):
        self.invert = invert
        self.ph = Pin(ph_pin, Pin.OUT, value=0)
        self.pwm = PWM(pwm_pin, freq, duty_u16=0)
        self.direction = 0
        self.dir_wait_until = 0

    def duty(self, value, min_duty=0):
        value = int(value)
        if 0 < abs(value) < min_duty:
            value = min_duty if value > 0 else -min_duty
        if value == 0:
            self.pwm.duty_u16(0)
            return 0
        dir_val = 1 if value > 0 else 0
        if self.invert:
            dir_val = 1 - dir_val
        if dir_val != self.direction:
            self.pwm.duty_u16(0)
            self.ph.value(dir_val)
            self.direction = dir_val
            self.dir_wait_until = utime.ticks_add(
                utime.ticks_ms(),
                _DIR_WAIT_MS,
            )
            return 0
        if self.dir_wait_until:
            if utime.ticks_diff(
                utime.ticks_ms(),
                self.dir_wait_until,
            ) < 0:
                self.pwm.duty_u16(0)
                return 0
            self.dir_wait_until = 0
        self.ph.value(dir_val)
        self.pwm.duty_u16(abs(value))
        return value
