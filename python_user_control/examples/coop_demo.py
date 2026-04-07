"""
两车协同最小示例：
1. 用假视觉输入和假队友状态把 CoopController 跑起来
2. 重点帮助理解状态切换顺序，不直接驱动真实电机
3. 适合先在串口里观察状态机是否按预期推进
"""

from machine import Pin
from smartcar import ticker, encoder
from seekfree import IMU660RX


from coop_protocol import CoopProtocol, TARGET_BEAR
from controller_coop import CoopController, CoopState
from models import VehicleContext


class CoopDemoHardware:
    def __init__(self):
        self.imu = IMU660RX()
        self.enc_fl = encoder("C0", "C1")
        self.enc_fr = encoder("C2", "C3")
        self.enc_b = encoder("D13", "D14")
        self.last_pwm = (0.0, 0.0, 0.0)
        self.fake_yaw = 0.0
        self.last_uart = []

    def read_encoder_spd(self):
        return self.enc_fl.get(), self.enc_fr.get(), self.enc_b.get()

    def read_yaw(self):
        return self.fake_yaw

    def read_gyro_z(self):
        imu_data = self.imu.get()
        return float(imu_data[5])

    def set_motor_pwm(self, pwm_fl, pwm_fr, pwm_b):
        self.last_pwm = (pwm_fl, pwm_fr, pwm_b)

    def uart_write_byte(self, uart_name, value):
        self.last_uart.append((uart_name, value))
        if len(self.last_uart) > 10:
            self.last_uart.pop(0)

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


STATE_NAME = {
    CoopState.SEARCH_TARGET: "SEARCH_TARGET",
    CoopState.ALIGN_TARGET: "ALIGN_TARGET",
    CoopState.WAIT_PARTNER: "WAIT_PARTNER",
    CoopState.COOP_ROTATE: "COOP_ROTATE",
    CoopState.COOP_PUSH: "COOP_PUSH",
    CoopState.VERIFY_FINISH: "VERIFY_FINISH",
    CoopState.RETURN_ROUTE: "RETURN_ROUTE",
    CoopState.FAIL_SAFE: "FAIL_SAFE",
}


def main():
    led = Pin("C4", Pin.OUT, pull=Pin.PULL_UP_47K, value=True)

    hw = CoopDemoHardware()
    ctx = VehicleContext()
    coop = CoopController(hw, ctx)

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
            coop.pit_5ms_step()

            # ---------------- 模拟输入区 ----------------
            # 前 20 tick: 搜索目标
            if tick_count == 20:
                coop.feed_art_pose(True, 25, 20, TARGET_BEAR)

            # 21~60 tick: 逐步把目标对准
            if 20 < tick_count <= 60:
                x = max(0, 25 - (tick_count - 20))
                y = max(0, 20 - (tick_count - 20))
                coop.feed_art_pose(True, x, y, TARGET_BEAR)

            # 等待队友 ready
            if tick_count == 90:
                coop.feed_partner_state(CoopProtocol.READY, TARGET_BEAR)

            # 模拟旋转过程中 yaw 逐步变化
            if coop.ctx.rotate_active and hw.fake_yaw > -90:
                hw.fake_yaw -= 6.0
                if hw.fake_yaw < -90:
                    hw.fake_yaw = -90.0

            # 推动一段时间后，队友上报完成
            if tick_count == 180:
                coop.feed_partner_state(CoopProtocol.DONE, TARGET_BEAR)
            # ---------------- 模拟输入区结束 ----------------

            coop.step()

            if tick_count % 10 == 0:
                led.toggle()
                snap = coop.get_debug_snapshot()
                print(
                    "tick=%d state=%s target=(%d,%.1f,%.1f) partner=%d tar=(%.1f,%.1f,%.1f) pwm=(%.1f,%.1f,%.1f)"
                    % (
                        tick_count,
                        STATE_NAME.get(snap["coop_state"], "UNKNOWN"),
                        snap["target_found"],
                        snap["target_x"],
                        snap["target_y"],
                        snap["partner_state"],
                        snap["tar_x"],
                        snap["tar_y"],
                        snap["tar_z"],
                        hw.last_pwm[0],
                        hw.last_pwm[1],
                        hw.last_pwm[2],
                    )
                )

            if coop.ctx.task_done or coop.ctx.task_failed or tick_count > 260:
                print(
                    "demo_end state=%s done=%d failed=%d uart=%s"
                    % (
                        STATE_NAME.get(coop.ctx.coop_state, "UNKNOWN"),
                        coop.ctx.task_done,
                        coop.ctx.task_failed,
                        hw.last_uart,
                    )
                )
                break


if __name__ == "__main__":
    main()


