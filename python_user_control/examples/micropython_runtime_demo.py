# RT1021 MicroPython 运行模板

# 接口风格与固件说明/官方示例一致：

# machine.Pin / machine.UART / smartcar.ticker / smartcar.encoder / seekfree.MOTOR_CONTROLLER / seekfree.IMU660RX



from machine import Pin, UART

from smartcar import ticker, encoder

from seekfree import IMU660RX













from seekfree import MOTOR_CONTROLLER



from models import VehicleContext

from controller import VehicleController

from isr_control import ISRControl

import utime


GYRO_SCALE = -1.0 / 16.54052
GYRO_OFFSET_SAMPLES = 2000
GYRO_DEADBAND_DPS = 0.8
GYRO_DT_S = 0.005
GYRO_DEBUG_DIV = 20





class RT1021Hardware:

    """

    硬件适配层。

    你移植时主要修改这里：引脚映射和 TODO 方法对接。

    """



    def __init__(self):

        # IMU：get() 返回由固件维护的数组/缓冲。

        self.imu = IMU660RX()

        self.imu_data = self.imu.get()



        # 编码器通道：按你实车接线修改。

        self.enc_fl = encoder("D15", "D16", True)

        self.enc_fr = encoder("D13", "D14", False)

        self.enc_b = encoder("C0", "C1", False)



        # 电机通道：按你驱动板接线调整 index 和 invert。

        self.motor_fl = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty=0, invert=False)

        self.motor_fr = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty=0, invert=True)

        self.motor_b = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty=0, invert=False)



        # UART 映射（说明书）：id=0 -> LPUART1, id=7 -> LPUART8

        self.uart_detect = UART(0)

        self.uart_model = UART(7)

        self.uart_detect.init(9600)

        self.uart_model.init(9600)



        self.gray_pin = Pin("C15", Pin.IN, pull=Pin.PULL_UP_47K)



        # TODO 对接项：替换成你的相机/姿态流程。

        self.frame_ready = False

        self.gyro_offset_z = 0.0

        self.gyro_z_deg = 0.0

        self.yaw_deg = 0.0

        self.gyro_tick = 0

        self._calibrate_gyro_offset()



    def refresh_imu_data(self):

        self.imu_data = self.imu.get()

        return self.imu_data


    def _debug_text(self, text):

        print(text)


    def _wrap_yaw(self):

        if self.yaw_deg >= 360.0 or self.yaw_deg < 0.0:

            self.yaw_deg = self.yaw_deg % 360.0


    def _calibrate_gyro_offset(self):

        total = 0.0

        self._debug_text("=== gyro offset calibration start (2000 samples) ===")

        for _ in range(GYRO_OFFSET_SAMPLES):

            imu_data = self.refresh_imu_data()

            total += float(imu_data[5])

            utime.sleep_ms(1)

        self.gyro_offset_z = total / GYRO_OFFSET_SAMPLES

        self._debug_text("gyro_offset_z=%.6f" % self.gyro_offset_z)


    def _update_gyro_state(self):

        imu_data = self.refresh_imu_data()

        raw_gyro_z = float(imu_data[5])

        gyro_z_deg = (raw_gyro_z - self.gyro_offset_z) * GYRO_SCALE

        if -GYRO_DEADBAND_DPS < gyro_z_deg < GYRO_DEADBAND_DPS:

            gyro_z_deg = 0.0

        else:

            self.yaw_deg += gyro_z_deg * GYRO_DT_S

            self._wrap_yaw()

        self.gyro_z_deg = gyro_z_deg

        self.gyro_tick += 1


    def read_encoder_spd(self):

        return self.enc_fl.get(), self.enc_fr.get(), self.enc_b.get()



    def read_yaw(self):

        # TODO：对接你自己的融合航向角。

        return self.yaw_deg



    def read_gyro_z(self):

        # imu_data: [accx, accy, accz, gyrox, gyroy, gyroz]

        imu_data = self.refresh_imu_data()

        return float(imu_data[5])


    def read_yaw(self):

        return self.yaw_deg


    def read_gyro_z(self):

        # Keep C updateYaw() equivalent timing: refresh IMU and update yaw in the 5ms tick.

        self._update_gyro_state()

        return self.gyro_z_deg



    def set_motor_pwm(self, pwm_fl, pwm_fr, pwm_b):

        # 示例固件 duty 常用范围是 [-10000, 10000]。

        if pwm_fl > 10000:

            pwm_fl = 10000

        elif pwm_fl < -10000:

            pwm_fl = -10000



        if pwm_fr > 10000:

            pwm_fr = 10000

        elif pwm_fr < -10000:

            pwm_fr = -10000



        if pwm_b > 10000:

            pwm_b = 10000

        elif pwm_b < -10000:

            pwm_b = -10000



        self.motor_fl.duty(int(pwm_fl))

        self.motor_fr.duty(int(pwm_fr))

        self.motor_b.duty(int(pwm_b))



    def uart_write_byte(self, uart_name, value):

        data = bytes([value & 0xFF])

        if uart_name == "art_detect":

            self.uart_detect.write(data)

        elif uart_name == "art_model":

            self.uart_model.write(data)



    def gpio_get_level(self, pin_name):

        if pin_name == "C15":

            return int(self.gray_pin.value())

        return 0



    def image_solve(self):

        # TODO：替换成你的巡线/图像求解角度误差。

        return 0.0



    def curve_angle_get(self):

        # TODO：返回 (左侧曲率角, 右侧曲率角)

        return 0.0, 0.0



    def get_follow_line_state(self):

        # 预留钩子：可接你的循迹子状态机。

        return "NORMAL"



    def mt9v03x_frame_ready(self):

        return self.frame_ready



    def clear_frame_ready(self):

        self.frame_ready = False



    def push_left_or_right(self, cls):

        # TODO：按你的类别规则映射左右推块。

        if cls % 2:

            return 0

        return 1



    def delay_ms(self, ms):

        # 兼容占位；当前控制器是非阻塞设计，不依赖 delay。

        import time



        time.sleep_ms(ms)





def main():

    hw = RT1021Hardware()

    ctx = VehicleContext()

    ctrl = VehicleController(hw, ctx)

    isr = ISRControl(ctrl)



    # EXTI 回调要尽量轻量：只做事件分发。

    key = Pin("B15", Pin.IN, pull=Pin.PULL_UP_47K)



    def key_handler(pin):

        isr.gpio_b15_rising_irq(bool(pin.value()))



    key.irq(key_handler, Pin.IRQ_RISING, False)



    # 5ms ticker：驱动底层控制节拍。

    pit_flag = False



    def time_pit_handler(_):

        nonlocal pit_flag

        pit_flag = True



    pit = ticker(1)

    pit.capture_list(hw.imu, hw.enc_fl, hw.enc_fr, hw.enc_b)

    pit.callback(time_pit_handler)

    pit.start(5)

    tick = 0



    while True:

        if pit_flag:

            pit_flag = False

            tick += 1

            isr.pit_ch1_irq()

            ctrl.step()

            if tick % GYRO_DEBUG_DIV == 0:

                print(
                    "gyro_offset_z=%.3f gyro_z_deg=%.3f yaw=%.2f"
                    % (hw.gyro_offset_z, hw.gyro_z_deg, hw.yaw_deg)
                )



        # UART 轮询：把字节喂给 ISR 适配层 FIFO。

        n = hw.uart_detect.any()

        if n:

            data = hw.uart_detect.read(n)

            for b in data:

                isr.lpuart1_irq(b)



        n = hw.uart_model.any()

        if n:

            data = hw.uart_model.read(n)

            for b in data:

                isr.lpuart8_irq(b)





if __name__ == "__main__":

    main()









