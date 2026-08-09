class AnglePID:
    def __init__(self):
        self.err = 0.0
        self.output = 0.0
        self.err_last = 0.0
        self.kp = (3.5, 2.5, 1.0)
        self.gyro_kp = 0.4
        self.gyro_ki = 0.012
        self.gyro_output_limit = 150.0


class SpeedPID:
    def __init__(self):
        self.temp = 0.0
        self.c1 = 0.0
        self.c2 = 0.0
        self.c3 = 0.0
        self.err = 0.0
        self.err_last = 0.0
        self.tar_spd_last = 0.0
        self.output = 0.0
        self.delta_tar_last = 0.0
        self.delta_tar = 0.0
        self.delta_ud = 0.0
        self.param_a = 0.0
        self.param_b = 0.0
        self.kd = 30.0
        self.gama = 0.25
        self.kp = 300.0
        self.ki = 8.0

    def init_c(self):
        self.temp = 1.0 / (self.gama * self.kd + self.kp)
        self.c3 = self.kd * self.temp
        self.c2 = (self.kd + self.kp) * self.temp
        self.c1 = self.c3 * self.gama


class MoveBase:
    def __init__(self):
        self.speed_fl = 0.0
        self.speed_fr = 0.0
        self.speed_b = 0.0
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0
        self.tar_spd_x = 0.0
        self.tar_spd_y = 0.0
        self.tar_spd_z = 0.0
