# 实车调试模板（按你提供的 demo 通道映射）

# 说明：本文件重点是让你快速验证通道、PWM方向、PID调参链路。



from machine import Pin

from smartcar import ticker, encoder

from seekfree import IMU660RX, WIRELESS_UART, MOTOR_CONTROLLER

import utime

from models import VehicleContext

from controller import VehicleController

from isr_control import ISRControl


GYRO_SCALE = -1.0 / 16.54052
GYRO_OFFSET_SAMPLES = 2000
GYRO_DEADBAND_DPS = 0.8
GYRO_DT_S = 0.005
GYRO_DEBUG_DIV = 20





class RealCarHardware:

    def __init__(self):

        # ?????????? demo ??

        self.motor_fl = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty=0, invert=False)

        self.motor_fr = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty=0, invert=True)

        self.motor_b = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty=0, invert=False)



        # 3 路编码器（与你 demo 一致）

        self.enc_fl = encoder("C0", "C1")

        self.enc_fr = encoder("C2", "C3")

        self.enc_b = encoder("D13", "D14")



        # IMU + 无线串口（与你 demo 一致）

        self.imu = IMU660RX()

        self.imu_data = self.imu.get()

        self.wireless = WIRELESS_UART(460800)



        # 你的 demo 里用于灰度/状态判定的管脚

        self.gray_pin = Pin("C15", Pin.IN, pull=Pin.PULL_UP_47K)



        # 预留：你自己的 yaw 与图像流程

        self.gyro_offset_z = 0.0

        self.gyro_z_deg = 0.0

        self.yaw_deg = 0.0

        self.gyro_tick = 0

        self.frame_ready = False

        self._calibrate_gyro_offset()



    def refresh_imu_data(self):

        self.imu_data = self.imu.get()

        return self.imu_data


    def _debug_text(self, text):

        print(text)

        self.wireless.send_str(text + "\r\n")


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

        # TODO: 替换为你真实的姿态融合 yaw

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

        # ??? demo ?????????? MOTOR_CONTROLLER duty???? ?10000

        self.motor_fl.duty(int(max(min(pwm_fl, 10000), -10000)))

        self.motor_fr.duty(int(max(min(pwm_fr, 10000), -10000)))

        self.motor_b.duty(int(max(min(pwm_b, 10000), -10000)))



    def uart_write_byte(self, uart_name, value):

        # 当前通过无线串口打日志（你也可以改成真实 UART 发包）

        self.wireless.send_str("[%s] 0x%02X\r\n" % (uart_name, value & 0xFF))



    def gpio_get_level(self, pin_name):

        if pin_name == "C15":

            return int(self.gray_pin.value())

        return 0



    def image_solve(self):

        # TODO: 替换成你的图像误差输出

        return 0.0



    def curve_angle_get(self):

        # TODO: 返回曲率角 (left, right)

        return 0.0, 0.0



    def get_follow_line_state(self):

        return "NORMAL"



    def mt9v03x_frame_ready(self):

        return self.frame_ready



    def clear_frame_ready(self):

        self.frame_ready = False



    def push_left_or_right(self, cls):

        # TODO: 用你的规则替换

        return 0 if (cls % 2) else 1





def main():

    led = Pin("C4", Pin.OUT, pull=Pin.PULL_UP_47K, value=True)



    hw = RealCarHardware()

    ctx = VehicleContext()

    ctrl = VehicleController(hw, ctx)

    isr = ISRControl(ctrl)



    # ====== 调参入口（先从这里改） ======

    ctrl.set_speed_pid_all(kp=100, ki=15, kd=30, gama=0.4)

    # ctrl.reset_speed_pid_output()  # 需要时手动清历史



    # 10ms 控制节拍（与你 demo 一致）

    pit_flag = False



    def pit_cb(_):

        nonlocal pit_flag

        pit_flag = True



    pit = ticker(1)

    pit.capture_list(hw.enc_fl, hw.enc_fr, hw.enc_b, hw.imu)

    pit.callback(pit_cb)

    pit.start(5)



    last_led = 0

    tick = 0

    while True:

        if pit_flag:

            pit_flag = False

            tick += 1

            isr.pit_ch1_irq()

            ctrl.step()



            # 每 10 个周期发一次调试快照

            if tick % GYRO_DEBUG_DIV == 0:

                d = ctrl.get_debug_snapshot()

                hw.wireless.send_str(

                    "st=%d, e=(%d,%d,%d), tar=(%.1f,%.1f,%.1f), out=(%.1f,%.1f,%.1f), gyro_offset_z=%.3f, gyro_z_deg=%.3f, yaw=%.2f\r\n"

                    % (

                        d["state"],

                        int(d["enc_fl"]),

                        int(d["enc_fr"]),

                        int(d["enc_b"]),

                        d["tar_x"],

                        d["tar_y"],

                        d["tar_z"],

                        d["pid_fl_out"],

                        d["pid_fr_out"],

                        d["pid_b_out"],

                        hw.gyro_offset_z,

                        hw.gyro_z_deg,

                        hw.yaw_deg,

                    )

                )



        # 心跳灯

        import utime



        now = utime.ticks_ms()

        if utime.ticks_diff(now, last_led) >= 1000:

            led.toggle()

            last_led = now





if __name__ == "__main__":

    main()









