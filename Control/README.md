# Control

`Control/` 是主车控制代码目录，当前真实运行入口是 `main.py`。它负责三轮底盘速度闭环、IMU yaw/gyro rate 控制、ART 视觉 UART 解析、目标搜索/对准/分类/推送状态机，以及 C8/C9 按键启停。

## 当前模式

当前按单车任务运行：

1. `config.py` 中 `COOP_ENABLE = False`。
2. `main.py` 不轮询协同帧，也不发送目标锁定、ready、push start/stop 等协同消息。
3. 主车完成目标对准和推送准备后，直接从 `PUSH_PREPARE` 进入 `PUSH_EXECUTE`，不再进入等待从车的流程。
4. `main.py` 暂时关闭 `MSG_MASTER_MOTION` 主车运动反馈广播，无线串口用于输出调试日志。
5. `coop_protocol.py`、`controller_coop.py` 和 `examples/coop_demo.py` 保留，作为后续恢复双车协同时的参考代码，当前主流程不调用完整协同状态机。

视觉通信仍然启用：主车通过 `CAM_UART_ID`/`CAM_UART_BAUD` 与视觉端交换搜索、粗对准、细对准、分类和黄线状态。

## 主要文件

```txt
main.py                    主车真实运行入口
config.py                  电机、编码器、按键、LED、相机 UART、协同开关等参数
hardware.py                电机和硬件适配
imu_runtime.py             IMU yaw/gyro rate 运行时
pid.py                     速度环、陀螺仪环、转向环 PID 对象
move_base.py               三轮底盘运动学映射
uart_protocol.py           旧版 ART 串口协议辅助函数
coop_protocol.py           双车协同帧协议，当前主流程暂不发送主车运动反馈帧
controller_coop.py         早期双车协同控制器骨架，当前保留但主流程停用
Test/                      运动、IMU 和调试脚本
examples/                  示例和适配代码
```

## 部署检查

1. 确认 `config.py` 中电机、编码器、按键、LED 和相机 UART 参数匹配当前接线。
2. 确认视觉端 UART 波特率与 `CAM_UART_BAUD` 一致，当前默认 `9600`。
3. 确认主车启动入口指向 `main.py`，或者按固件实际文件名调整 `boot.py`。
4. 若启用从车红外视觉跟随，需要先恢复主车 `MSG_MASTER_MOTION` 运动反馈广播，并确认主从车 `COOP_WIRELESS_BAUD`、协议和运动方向符号一致。
5. 若之后需要恢复完整双车协同，先把 `COOP_ENABLE` 改回 `True`，再检查无线串口、协议和从车流程。
