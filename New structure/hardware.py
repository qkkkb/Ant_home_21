from machine import Pin, PWM


class Motor:
    def __init__(self, ph_pin, pwm_pin, freq=13000, invert=False):
        self.invert = invert
        self.ph = Pin(ph_pin, Pin.OUT, value=0)
        self.pwm = PWM(pwm_pin, freq, duty_u16=0)

    def set_speed(self, speed):
        if speed == 0:
            self.pwm.duty_u16(0)
            return
        direction = 1 if speed > 0 else 0
        if self.invert:
            direction = 1 - direction
        self.ph.value(direction)
        self.pwm.duty_u16(abs(speed))

    def duty(self, value):
        if value == 0:
            self.pwm.duty_u16(0)
            return
        direction = 1 if value > 0 else 0
        if self.invert:
            direction = 1 - direction
        self.ph.value(direction)
        self.pwm.duty_u16(abs(value))
