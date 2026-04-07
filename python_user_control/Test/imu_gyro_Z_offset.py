from machine import *
from smartcar import ticker
from seekfree import IMU660RX

import gc
import time

# ====================== 核心配置 ======================
SAMPLE_COUNT    = 200       # 零漂采样次数
SAMPLE_DELAY_MS = 5         # 每次采样间隔（ms）
PRINT_DIV       = 20        # 主循环打印间隔（每N次ticker打印一次）
TICK_PERIOD_MS  = 10        # ticker周期

# ====================== 外设初始化 ======================
time.sleep_ms(100)

led     = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

IMU660RX.help()
imu = IMU660RX()
imu.info()

imu_data = imu.get()

# ====================== 零漂校准 ======================
print("=== 零漂校准中，请保持IMU静止 ===")
print("采样次数：%d，预计耗时：%.1f秒" % (SAMPLE_COUNT, SAMPLE_COUNT * SAMPLE_DELAY_MS / 1000.0))

bias_sum = [0, 0, 0, 0, 0, 0]   # acc xyz + gyro xyz

for i in range(SAMPLE_COUNT):
    data = imu.read()
    for axis in range(6):
        bias_sum[axis] += data[axis]
    if i % 50 == 0:
        print("  采样进度：%d / %d" % (i, SAMPLE_COUNT))
    time.sleep_ms(SAMPLE_DELAY_MS)

bias = [bias_sum[i] // SAMPLE_COUNT for i in range(6)]

print("=== 零漂校准完成 ===")
print("acc  零漂：X=%6d  Y=%6d  Z=%6d" % (bias[0], bias[1], bias[2]))
print("gyro 零漂：X=%6d  Y=%6d  Z=%6d" % (bias[3], bias[4], bias[5]))
print("（偏航轴 gyro Z = %d，补偿后读数 = imu_data[5] - %d）" % (bias[5], bias[5]))

# ====================== ticker 配置 ======================
ticker_flag  = False
ticker_count = 0

def time_pit_handler(ticker_obj):
    global ticker_flag, ticker_count
    ticker_flag  = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else 1

pit1 = ticker(1)
pit1.capture_list(imu)
pit1.callback(time_pit_handler)
pit1.start(TICK_PERIOD_MS)

# ====================== 主循环：实时显示补偿后的值 ======================
print("=== 开始实时监测（补偿后） ===")
print("拨动D9拨码开关退出")

while True:
    if ticker_flag and ticker_count % PRINT_DIV == 0:
        ticker_flag = False
        led.toggle()

        # 原始值
        raw_gx = imu_data[3]
        raw_gy = imu_data[4]
        raw_gz = imu_data[5]

        # 补偿后（减去零漂）
        cor_gx = raw_gx - bias[3]
        cor_gy = raw_gy - bias[4]
        cor_gz = raw_gz - bias[5]

        print(
            "gyro raw=(%6d,%6d,%6d)  corrected=(%6d,%6d,%6d)"
            % (raw_gx, raw_gy, raw_gz, cor_gx, cor_gy, cor_gz)
        )

    if switch2.value() != state2:
        pit1.stop()
        print("=== 程序停止 ===")
        print("最终零漂结论：gyro Z 零漂 = %d" % bias[5])
        break

    gc.collect()
