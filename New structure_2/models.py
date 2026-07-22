class AnglePID:
    __slots__ = (
        "err",
        "output",
        "err_last",
        "gyro_kp",
        "gyro_ki",
        "gyro_output_limit",
    )

    def __init__(self):
        self.err = 0.0
        self.output = 0.0
        self.err_last = 0.0
        self.gyro_kp = 0.4
        self.gyro_ki = 0.012
        self.gyro_output_limit = 150.0


class SpeedPID:
    __slots__ = (
        "temp",
        "c1",
        "c2",
        "c3",
        "err",
        "err_last",
        "tar_spd_last",
        "output",
        "delta_tar_last",
        "delta_tar",
        "delta_ud",
        "enc_samples",
        "enc_sum",
        "enc_index",
        "kd",
        "gama",
        "kp",
        "ki",
    )

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
        self.enc_samples = [0] * 4
        self.enc_sum = 0
        self.enc_index = 0
        self.kd = 30.0
        self.gama = 0.25
        self.kp = 3000.0
        self.ki = 6.0

    def init_c(self):
        self.temp = 1.0 / (self.gama * self.kd + self.kp)
        self.c3 = self.kd * self.temp
        self.c2 = (self.kd + self.kp) * self.temp
        self.c1 = self.c3 * self.gama


class MoveBase:
    __slots__ = ("speed_fl", "speed_fr", "speed_b")

    def __init__(self):
        self.speed_fl = 0.0
        self.speed_fr = 0.0
        self.speed_b = 0.0
