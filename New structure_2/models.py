class AnglePID:
    def __init__(self):
        self.err = 0.0
        self.output = 0.0
        self.err_last = 0.0
        self.gyro_kp = 0.4
        self.gyro_ki = 0.012
        self.gyro_output_limit = 150.0


class SpeedPID:
    def __init__(self):
        self.err = 0.0
        self.tar_spd_last = 0.0
        self.output = 0.0
        self.kp = 300.0
        self.ki = 8.0


class MoveBase:
    def __init__(self):
        self.speed_fl = 0.0
        self.speed_fr = 0.0
        self.speed_b = 0.0
