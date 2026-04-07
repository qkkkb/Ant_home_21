from machine import * 
import time 
 
time.sleep_ms(50) # 上电启动时间延时 
boot_select = Pin('D8', Pin.IN, pull=Pin.PULL_UP_47K) # 选择学习板上的一号拨码开关作为启动选择开关 # 如果拨码开关打开 对应引脚拉低 就启动用户文件 
# 如果拨码开关关闭 对应引脚拉高 就跳过用户文件 直接进入 REPL 模式 
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 上电启动时间延时
time.sleep_ms(50)
# 选择学习板上的一号拨码开关作为启动选择开关
boot_select = Pin('C18', Pin.IN, pull=Pin.PULL_UP_47K, value = True)

# 如果拨码开关打开 对应引脚拉低 就启动用户文件
if boot_select.value() == 1:
    try:
        os.chdir("/flash")
        execfile("JINGZHI.py")
    except:
        print("File not found.")
else:
    print("???")