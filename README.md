# Ant_Home

`Ant_Home` 是一个面向智能车、视觉识别和主从车辅助任务的 MicroPython 项目。仓库按运行设备拆分为主车控制、从车控制和视觉端程序，核心目标是在内存受限的嵌入式环境中完成目标搜索、视觉对准、方向判断、主车推送动作，以及从车基于红外灯板的跟随。

## 目录结构

```txt
Ant_Home/
  Control/    主车控制程序，包含运动闭环、视觉串口、IMU、运动前馈无线发送和测试脚本
  NO.2/       第二辆车/从车控制程序，包含红外跟随入口、无线运动前馈接收和无线自检
  Vision/     视觉端程序，包含 OpenMV/ART 目标检测、分类、黄线检测和 IPM 标定脚本
  AGENTS.md   协作、Git、README 和 MicroPython 内存优化约束
  README.md   项目总览
```

## 模块概览

`Control/` 是主车控制模块。当前真实运行入口主要是 `Control/main.py`，它负责三轮底盘速度闭环、陀螺仪 yaw/gyro rate 控制、ART 视觉串口解析、目标推送状态机、按键启停，并通过当前无线帧结构周期发送主车运动前馈给从车。详细说明见 `Control/README.md`。

`NO.2/` 保存第二辆车相关程序。`NO.2/main.py` 是从车持续跟随入口，读取 OpenMV 红外灯板误差和主车无线运动前馈；`NO.2/mini_main.py` 是从车 OpenMV 红外视觉脚本，按摄像头 180 度旋转校正后识别横排两点红外灯板，并使用 `TRACK`/`IDLE` 命令、QQVGA 与 ROI 锁定来优先保证帧率。

`Vision/` 保存视觉端脚本。`Vision/art_main.py` 使用检测模型和分类模型处理画面，通过 UART 输出目标误差、分类方向和黄线状态；`ipm_calibration_pc.py`、`openmv_ipm_calibration.py` 等脚本用于逆透视标定和参数更新。

## 运行环境

控制端代码面向 RT1021/SeekFree 风格 MicroPython 固件，依赖 `machine`、`smartcar`、`seekfree` 等硬件接口。视觉端代码面向 OpenMV/ART 类运行环境，依赖 `sensor`、`image`、`tf`、`pyb` 和 `machine.UART`。

不同设备部署时，需要确认以下内容：

1. `config.py` 中的电机、编码器、按键、LED、UART 和无线参数是否匹配当前接线。
2. 控制端启动文件是否指向预期入口。当前 `Control/boot.py` 中仍有执行 `/flash/JINGZHI.py` 的逻辑，部署前要按实际文件名确认。
3. 主车视觉端模型路径是否存在，例如 `/sd/detect.tflite` 和 `/sd/classify.tflite`；从车红外视觉不依赖 TFLite 模型。
4. 主车、从车与对应视觉端 UART 波特率是否一致，当前控制端相机 UART 默认是 `9600`。
5. 主从车无线串口参数要在对应 `config.py` 中保持一致，当前无线用于主车运动前馈，不再执行双车协同推物体流程。

## 开发与文档约束

本仓库的协作规则记录在根目录 `AGENTS.md`。修改代码、提交或推送前，请先阅读该文件。

关键约束包括：

1. `D:\Ant_Home` 内的代码或文档修改完成后需要提交到 Git。
2. 提交信息必须使用中文，并且只提交本次任务相关文件。
3. 推送前必须检查 `README.md` 是否需要同步更新。
4. MicroPython 代码应优先按内存受限设备优化，避免在热循环中反复分配对象。

## 建议文档组织

根目录 `README.md` 只放项目总览和跨模块说明。具体模块说明放在对应子目录，例如：

```txt
Control/README.md      主车控制模块说明
docs/                  后续可放硬件接线、标定流程、调试记录等扩展文档
```

如果后续新增硬件连接图、标定流程或比赛调参记录，建议放入 `docs/`，并从本文件链接过去。
