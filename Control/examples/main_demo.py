"""
最小主程序示例：
1. 展示 Python 版 main.c / isr.c / controller 的关系
2. 不接入完整视觉状态机，只演示 5ms 控制节拍
3. 适合你先理解“项目是怎么跑起来的”
"""

from machine import Pin
from smartcar import ticker, encoder
from seekfree import IMU660RX


from models import VehicleContext
from controller import VehicleController
from isr_control import ISRControl


class DemoHardware:
    def __init__(self):
        self.imu = IMU660RX()
        self.enc_fl = encoder("C0", "C1")
        self.enc_fr = encoder("C2", "C3")
        self.enc_b = encoder("D13", "D14")

        # 这里只做框架演示，不真正驱动电机。
        self.last_pwm = (0.0, 0.0, 0.0)

    def read_encoder_spd(self):
        return self.enc_fl.get(), self.enc_fr.get(), self.enc_b.get()

    def read_yaw(self):
        return 0.0

    def read_gyro_z(self):
        imu_data = self.imu.get()
        return float(imu_data[5])

    def set_motor_pwm(self, pwm_fl, pwm_fr, pwm_b):
        self.last_pwm = (pwm_fl, pwm_fr, pwm_b)

    def uart_write_byte(self, uart_name, value):
        return None

    def gpio_get_level(self, pin_name):
        return 0

    def image_solve(self):
        return 0.0

    def curve_angle_get(self):
        return 0.0, 0.0

    def get_follow_line_state(self):
        return "NORMAL"

    def mt9v03x_frame_ready(self):
        return False

    def clear_frame_ready(self):
        return None

    def push_left_or_right(self, cls):
        return 0

    def delay_ms(self, ms):
        return None


def main():
    led = Pin("C4", Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

    hw = DemoHardware()
    ctx = VehicleContext()
    ctrl = VehicleController(hw, ctx)
    isr = ISRControl(ctrl)

    # 给一个固定前进目标，帮助理解控制链。
    ctx.move.tar_spd_x = 0.0
    ctx.move.tar_spd_y = 80.0
    ctx.move.tar_spd_z = 0.0

    tick_flag = False
    tick_count = 0

    def pit_handler(_):
        nonlocal tick_flag, tick_count
        tick_flag = True
        tick_count += 1

    pit = ticker(1)
    pit.capture_list(hw.enc_fl, hw.enc_fr, hw.enc_b, hw.imu)
    pit.callback(pit_handler)
    pit.start(5)

    while True:
        if tick_flag:
            tick_flag = False

            # 这一行相当于旧 C 的 PIT_CH1 中断主体
            isr.pit_ch1_irq()

            # 这一行相当于旧 C 的 main while 里跑状态机
            ctrl.step()

            if tick_count % 20 == 0:
                led.toggle()
                snap = ctrl.get_debug_snapshot()
                print(
                    "tick=%d enc=(%.1f,%.1f,%.1f) tar=(%.1f,%.1f,%.1f) wheel=(%.1f,%.1f,%.1f) pwm=(%.1f,%.1f,%.1f)"
                    % (
                        tick_count,
                        snap["enc_fl"],
                        snap["enc_fr"],
                        snap["enc_b"],
                        snap["tar_x"],
                        snap["tar_y"],
                        snap["tar_z"],
                        snap["wheel_fl"],
                        snap["wheel_fr"],
                        snap["wheel_b"],
                        hw.last_pwm[0],
                        hw.last_pwm[1],
                        hw.last_pwm[2],
                    )
                )


if __name__ == "__main__":
    main()




