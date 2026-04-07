## ????
- ???????? `/flash/main.py` ????????????? `boot.py`?
- ?? `main.py` ????? `[JINGZHI.py](/D:/Ant_Home/python_user_control/JINGZHI.py)` ????????
- 7 ?????????????? `[JINGZHI.py](/D:/Ant_Home/python_user_control/JINGZHI.py)` ??

﻿# python_user_control (RT1021 MicroPython firmware style)

本目录是把 `project/code/user_control` 迁移成 MicroPython 可接入版本。
代码风格与示例 demo 一致，并使用说明书中的接口族：
- `machine.Pin`
- `machine.UART`
- `smartcar.ticker`
- `smartcar.encoder` (V3.x 支持 `invert` 和 `capture_div`)
- `seekfree.MOTOR_CONTROLLER`
- `seekfree.IMU660RX` (旧固件可能仍为 `IMU660RA`，项目已做兼容)

## 文件说明
- `models.py`: 运行时状态结构（纯 Python 类，兼容 MicroPython）
- `pid.py`: `gyroCtrl/turnCtrl/speedCtrl/posCtrl`
- `move_base.py`: 底盘运动学映射
- `uart_protocol.py`: ART 串口协议处理
- `controller.py`: 主状态机（Line_follow -> ... -> Backing_track）
- `isr_control.py`: ISR 风格事件入口（PIT/UART/GPIO）
- `examples/micropython_runtime_demo.py`: 固件接口示例整合模板

## 与说明书/示例接口的对应
- 外部中断: `Pin.irq(handler, Pin.IRQ_RISING, False)`
- 串口: `UART(id)`, `init(baudrate)`, `any()`, `read(n)`, `write(buf)`
- 周期调度: `ticker(index)`, `capture_list(...)`, `callback(fn)`, `start(ms)`, `stop()`
- 编码器: `encoder(PhaseA, PhaseB, invert=False, capture_div=1)`, `get()`
- 电机: `MOTOR_CONTROLLER(...).duty(value)`
- IMU: `IMU660RX(capture_div=1)` / `imu.get()`

## 接入步骤
1. 将本目录中的 `.py` 文件平铺拷贝到 MicroPython 设备的 `/flash` 根目录。
2. 以 `micropython_runtime_demo.py` 或 `main_demo.py` 为主程序模板。
3. 把模板中的 TODO（`read_yaw/image_solve/curve_angle_get/...`）替换成你的算法函数。
4. 根据你接线修改 encoder、电机、UART、GPIO 引脚。
5. 开启 `ticker.start(5)` 后，每个 5ms 调 `isr.pit_ch1_irq()`，主循环持续 `ctrl.step()`。



