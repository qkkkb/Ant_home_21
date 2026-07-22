from machine import Pin, PWM


class Motor:
    __slots__ = ("invert", "ph", "pwm")

    def __init__(self, ph_pin, pwm_pin, freq=13000, invert=False):
        self.invert = invert
        self.ph = Pin(ph_pin, Pin.OUT, value=0)
        self.pwm = PWM(pwm_pin, freq, duty_u16=0)

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
        self.ph.value(dir_val)
        self.pwm.duty_u16(abs(value))
        return value
