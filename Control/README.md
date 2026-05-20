# Control 控制模块

`Control/` 是主车控制模块，面向 RT1021/SeekFree 风格 MicroPython 固件。当前代码同时保留了迁移后的控制器骨架、真实车运行入口、示例适配器和测试脚本，其中真实运行链路主要集中在 `main.py`。

## 当前运行链路

`main.py` 负责主车真实运行，核心流程如下：

1. 读取 `config.py` 中的电机、编码器、按键、LED、UART 和双车协同参数。
2. 初始化三路电机、三路编码器、相机 UART、IMU yaw runtime、无线协同串口和 5ms `ticker`。
3. 通过 C9 按键、视觉首帧或自动启动逻辑进入发车流程。
4. 发车前可自动标定陀螺仪 Z 轴偏移，并重置 yaw 参考。
5. 主循环持续解析 ART 视觉 UART、无线协同帧、导航 LED 和退出条件。
6. 5ms 节拍中执行三轮运动学分解、速度 PID、陀螺仪内环和 PWM 平滑输出。
7. 退出时停止 ticker、清零所有电机 PWM，并关闭导航 LED。

主要启停输入：

1. `C9`：发车按键，触发陀螺仪标定、场地方向参考刷新和视觉搜索转向。
2. `C8`：退出按键，触发 `KeyboardInterrupt` 并进入安全停止。
3. 无线遥控 `CH7`：作为上位退出触发。
4. 视觉首个有效目标：在未发车状态下也可触发启动。

## 导航状态机

`main.py` 中的视觉导航状态使用字符串常量描述，主要包括：

```txt
SEARCH
SEARCH_TURN_45
COARSE_APPROACH
FINE_ALIGN
PUSH_CLASSIFY
PUSH_ORBIT
PUSH_PREPARE
PUSH_EXECUTE
PUSH_FINISH_BACK
PUSH_FINISH_TURN
POST_TURN_FORWARD
COOP_WAIT_SLAVE
COOP_DONE
```

整体行为是先搜索目标，再粗对准、细对准、分类推送方向、必要时环绕调整姿态，然后执行推送、越过黄线、后退、转回和后续前进。启用双车协同时，主车会在锁定推送姿态后进入 `COOP_WAIT_SLAVE`，等待从车就绪后再发送推送开始信号。

## 视觉通信

主车通过相机 UART 接收视觉端数据，默认配置在 `config.py`：

```python
CAM_UART_ID = 0
CAM_UART_BAUD = 9600
```

`main.py` 向视觉端发送模式命令：

```txt
SEARCH
COARSE
FINE
CLASSIFY
LINE
IDLE
```

视觉端回传的数据由 `poll_art_uart()` 解析，主要帧类型包括：

1. 普通目标误差帧：以 `0xFF` 开头，后两个字节编码目标误差。
2. 分类方向帧：标签 `0xFD`，用于更新推送方向。
3. 黄线状态帧：标签 `0xFC`，用于判断推送是否越过黄线。
4. 无目标帧：`0xFE 0xFE`，用于清除当前目标。

## 双车协同

双车协同开关和参数位于 `config.py`：

```python
COOP_ROLE = "MASTER"
COOP_ENABLE = True
COOP_WIRELESS_BAUD = 460800
```

`coop_protocol.py` 定义无线帧格式和消息类型：

```txt
MSG_TARGET_LOCK
MSG_ACK
MSG_MASTER_READY
MSG_SLAVE_APPROACHING
MSG_SLAVE_READY
MSG_PUSH_START
MSG_PUSH_STOP
MSG_DONE
MSG_ERROR
```

主车在锁定目标和推送姿态后，会周期发送目标锁定、主车就绪、推送开始和推送停止等消息。从车回复 ACK、接近中、就绪、完成或错误状态。通信 LED 脉冲会叠加到导航 LED 显示上，便于现场判断收发是否发生。

## 文件说明

```txt
boot.py                       MicroPython 上电启动选择逻辑，部署前需确认入口文件名
config.py                     电机、编码器、按键、LED、UART、协同和导航基础参数
main.py                       当前真实车主入口，包含视觉导航、闭环控制和双车协同
hardware.py                   硬件抽象层、空硬件实现和真实硬件适配类
imu_runtime.py                IMU660RX yaw 积分、gyro Z 标定和角速度读取
coop_protocol.py              双车无线协同帧编码、解析和消息常量
models.py                     车辆状态、PID 状态、运动状态和 UART FIFO 数据结构
move_base.py                  三轮底盘运动学映射和车体速度反解
pid.py                        陀螺仪环、转向环、速度环和位置控制函数
uart_protocol.py              旧 ART 串口协议解析函数
controller.py                 非阻塞控制器骨架，保留原状态机迁移思路
controller_coop.py            早期双车协同控制器骨架
isr_control.py                ISR 风格事件入口封装
examples/                     PC/固件接口示例和真实车适配模板
Test/                         运动、IMU、单轮方向和调试脚本
```

## 部署提醒

部署到设备前建议逐项确认：

1. `config.py` 的引脚和反向标志是否匹配当前车。
2. `boot.py` 是否执行正确的入口文件。当前文件中存在执行 `/flash/JINGZHI.py` 的逻辑，如果实际入口是 `main.py`，需要部署时同步调整。
3. 视觉端 UART 波特率和控制端 `CAM_UART_BAUD` 是否一致。
4. `COOP_ROLE`、`COOP_ENABLE` 和无线波特率是否与主从车配置一致。
5. 发车前车体是否静止，避免陀螺仪自动标定时引入偏移。
6. 如果只想调试闭环而不让电机输出，可使用 `main.py` 中的 `FORCE_MOTOR_OFF` 调试开关。

## 调试建议

1. 先用 `wireless_selftest.py` 或对应从车脚本确认无线链路。
2. 再用 `motion_sequence_test.py`、`motion_heading_lock_test.py` 或 `Test/` 下脚本确认三轮方向、编码器方向和陀螺仪方向。
3. 然后确认视觉端能收到模式命令，并能回传目标误差、分类方向和黄线状态。
4. 最后再启用完整 `main.py` 流程，逐步调节导航状态机参数。

## 开发注意

本模块运行在内存受限设备上，热循环、UART 解析、ticker 回调和电机闭环中要避免频繁分配对象。修改控制逻辑时优先复用缓冲区和状态变量，必要时在非关键路径主动 `gc.collect()`。
