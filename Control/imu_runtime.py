import utime

from seekfree import IMU660RX


class IMUYawRuntime:
    """
    Reusable IMU runtime:
    1. Official demo style init: IMU660RX() + imu.get() + ticker.capture_list(imu)
    2. Old C style yaw update: gyro-z bias, scale, deadband, 5ms integration
    """

    def __init__(
        self,
        sign=1.0,
        offset_z=0.0,
        scale=-1.0 / 16.54052,
        deadband_dps=0.8,
        tick_period_ms=5,
    ):
        self.sign = float(sign)
        self.gyro_offset_z = float(offset_z)
        self.gyro_scale = float(scale)
        self.deadband_dps = float(deadband_dps)
        self.tick_period_ms = int(tick_period_ms)

        self.imu = IMU660RX()
        self.imu_data = self.imu.get()

        self.raw_gyro_z = 0.0
        self.gyro_z_deg = 0.0
        self.yaw_deg = 0.0
        self.last_update_ms = utime.ticks_ms()

    @staticmethod
    def help():
        IMU660RX.help()

    def info(self):
        self.imu.info()

    def capture_device(self):
        return self.imu

    def set_sign(self, sign):
        self.sign = float(sign)

    def set_offset_z(self, offset_z):
        self.gyro_offset_z = float(offset_z)

    def reset_yaw(self, yaw_deg=0.0):
        self.yaw_deg = float(yaw_deg) % 360.0
        self.last_update_ms = utime.ticks_ms()

    def calibrate_offset(self, samples=2000, delay_ms=1, logger=None):
        total = 0.0
        if logger is not None:
            logger(
                "gyro offset calibrating: axis=z samples=%d delay=%dms"
                % (samples, delay_ms)
            )

        for _ in range(samples):
            sample = self.imu.read()
            total += self.sign * float(sample[5])
            utime.sleep_ms(delay_ms)

        self.gyro_offset_z = total / samples

        if logger is not None:
            logger("gyro offset calibrated: axis=z offset_z=%.2f" % self.gyro_offset_z)
        return self.gyro_offset_z

    def update(self):
        now = utime.ticks_ms()
        dt_ms = utime.ticks_diff(now, self.last_update_ms)
        if dt_ms <= 0:
            dt_ms = self.tick_period_ms
        self.last_update_ms = now

        self.raw_gyro_z = self.sign * float(self.imu_data[5])
        self.gyro_z_deg = (self.raw_gyro_z - self.gyro_offset_z) * self.gyro_scale

        if -self.deadband_dps < self.gyro_z_deg < self.deadband_dps:
            self.gyro_z_deg = 0.0
        else:
            self.yaw_deg += self.gyro_z_deg * (dt_ms * 0.001)
            self.yaw_deg %= 360.0

        return self.gyro_z_deg

    def read_gyro_z(self):
        return self.update()

    def read_yaw(self):
        return self.yaw_deg
