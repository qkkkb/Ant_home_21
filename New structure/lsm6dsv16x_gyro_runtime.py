from machine import Pin, SPI
import gc
import utime


_LSM_FUNC_CFG_ACCESS = 0x01
_LSM_IF_CFG = 0x03
_LSM_CHIP_ID = 0x0F
_LSM_CTRL1 = 0x10
_LSM_CTRL2 = 0x11
_LSM_CTRL3 = 0x12
_LSM_CTRL6 = 0x15
_LSM_CTRL7 = 0x16
_LSM_CTRL8 = 0x17
_LSM_CTRL9 = 0x18
_LSM_OUTX_L_G = 0x22
_LSM_SPI_R = 0x80
_LSM_TIMEOUT_COUNT = 0xFF
_LSM_SW_RESET_VALUE = 0x04
_LSM_CTRL3_BDU_INC = 0x44


class LSM6DSV16XGyroOnly:
    """Small SPI gyro-only LSM6DSV16X driver."""

    def __init__(
        self,
        sck="C10",
        mosi="C12",
        miso="C13",
        cs="C11",
        gyro_fs=2000,
        accel_fs=8,
        baudrate=1000000,
        spi_mode=0,
    ):
        self.cs = Pin(cs, Pin.OUT)
        self.cs(1)
        self.spi = SPI(1)
        self.spi.init(
            baudrate=baudrate,
            polarity=1 if spi_mode == 3 else 0,
            phase=1 if spi_mode == 3 else 0,
        )
        self._tx2 = bytearray(2)
        self._rx2 = bytearray(2)
        self._tx7 = bytearray(7)
        self._rx7 = bytearray(7)

        if accel_fs == 2:
            acc_reg = 0x00
        elif accel_fs == 4:
            acc_reg = 0x01
        elif accel_fs == 8:
            acc_reg = 0x02
        elif accel_fs == 16:
            acc_reg = 0x03
        else:
            raise ValueError("bad accel_fs")

        if gyro_fs == 125:
            gyro_reg = 0x00
            self.gyro_factor = 228.5714
        elif gyro_fs == 250:
            gyro_reg = 0x01
            self.gyro_factor = 114.2857
        elif gyro_fs == 500:
            gyro_reg = 0x02
            self.gyro_factor = 57.1428
        elif gyro_fs == 1000:
            gyro_reg = 0x03
            self.gyro_factor = 28.5714
        elif gyro_fs == 2000:
            gyro_reg = 0x04
            self.gyro_factor = 14.2857
        elif gyro_fs == 4000:
            gyro_reg = 0x0C
            self.gyro_factor = 7.14285
        else:
            raise ValueError("bad gyro_fs")

        utime.sleep_ms(10)
        self._last_chip_id = 0xFF
        if self._self_check():
            raise RuntimeError("LSM6DSV16X WHO_AM_I error")

        self._write_reg(_LSM_FUNC_CFG_ACCESS, _LSM_SW_RESET_VALUE)
        utime.sleep_ms(30)
        self._write_reg(_LSM_FUNC_CFG_ACCESS, 0x00)
        self._write_reg(_LSM_IF_CFG, 0x01)
        self._write_reg(_LSM_CTRL3, _LSM_CTRL3_BDU_INC)
        self._write_reg(_LSM_CTRL8, acc_reg)
        self._write_reg(_LSM_CTRL6, gyro_reg)
        self._write_reg(_LSM_CTRL1, 0x15)
        self._write_reg(_LSM_CTRL2, 0x18)
        self._write_reg(_LSM_CTRL7, 0x01)
        self._write_reg(_LSM_CTRL9, 0x08)
        self.gyro_fs = gyro_fs
        self.accel_fs = accel_fs
        gc.collect()

    def _write_reg(self, reg, value):
        tx = self._tx2
        tx[0] = reg & 0x7F
        tx[1] = value & 0xFF
        self.cs(0)
        self.spi.write(tx)
        self.cs(1)

    def _read_reg(self, reg):
        tx = self._tx2
        rx = self._rx2
        tx[0] = reg | _LSM_SPI_R
        tx[1] = 0
        self.cs(0)
        self.spi.write_readinto(tx, rx)
        self.cs(1)
        return rx[1]

    def _self_check(self):
        for _ in range(_LSM_TIMEOUT_COUNT):
            self._last_chip_id = self._read_reg(_LSM_CHIP_ID)
            if self._last_chip_id == 0x70:
                return 0
            utime.sleep_ms(1)
        return 1

    @staticmethod
    def _i16(lo, hi):
        value = lo | (hi << 8)
        if value & 0x8000:
            value -= 65536
        return value

    def read_gyro_z(self):
        tx = self._tx7
        rx = self._rx7
        tx[0] = _LSM_OUTX_L_G | _LSM_SPI_R
        self.cs(0)
        self.spi.write_readinto(tx, rx)
        self.cs(1)
        return self._i16(rx[5], rx[6]) / self.gyro_factor

    def info(self):
        print("LSM6DSV16X gyro-only SPI mode")


class LSM6DSV16XYawRuntime:
    """Yaw runtime wrapper that keeps the old main.py interface."""

    def __init__(
        self,
        sign=1.0,
        offset_z=0.0,
        scale=-1.0,
        deadband_dps=0.8,
        tick_period_ms=5,
        gyro_fs=2000,
        accel_fs=8,
    ):
        self.sign = float(sign)
        self.gyro_offset_z = float(offset_z)
        self.gyro_scale = float(scale)
        self.deadband_dps = float(deadband_dps)
        self.tick_period_ms = int(tick_period_ms)

        self.imu = LSM6DSV16XGyroOnly(gyro_fs=gyro_fs, accel_fs=accel_fs)
        gc.collect()

        self.raw_gyro_z = 0.0
        self.gyro_z_deg = 0.0
        self.yaw_deg = 0.0
        self.last_update_ms = utime.ticks_ms()

    @staticmethod
    def help():
        print("LSM6DSV16X gyro-only runtime: gyro Z -> yaw-rate")

    def info(self):
        self.imu.info()

    def capture_device(self):
        return None

    def set_sign(self, sign):
        self.sign = float(sign)

    def set_offset_z(self, offset_z):
        self.gyro_offset_z = float(offset_z)

    def reset_yaw(self, yaw_deg=0.0):
        self.yaw_deg = float(yaw_deg) % 360.0
        self.last_update_ms = utime.ticks_ms()

    def _read_raw_gyro_z(self):
        return self.sign * float(self.imu.read_gyro_z())

    def calibrate_offset(self, samples=2000, delay_ms=1, logger=None):
        total = 0.0
        if logger is not None:
            logger(
                "gyro offset calibrating: axis=z samples=%d delay=%dms"
                % (samples, delay_ms)
            )

        for _ in range(samples):
            total += self._read_raw_gyro_z()
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

        self.raw_gyro_z = self._read_raw_gyro_z()
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
