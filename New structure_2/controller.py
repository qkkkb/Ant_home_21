# controller.py - PID 控制器
class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, out_max=65535, int_max=1000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.int_max = int_max
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self._first = True

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        if self.integral > self.int_max:
            self.integral = self.int_max
        elif self.integral < -self.int_max:
            self.integral = -self.int_max

        if self._first:
            derivative = 0.0
            self._first = False
        else:
            derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if output > self.out_max:
            output = self.out_max
        elif output < -self.out_max:
            output = -self.out_max
        return output