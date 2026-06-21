from machine import Pin
from array import array
from seekfree import WIRELESS_UART
import config as cfg
import gc
import utime


RUN_MS = 15000
POLL_MS = 20
BUF_LEN = 32


def _mem_free():
    try:
        return gc.mem_free()
    except Exception:
        return -1


def _rx_values(buf, n):
    vals = []
    for i in range(n):
        vals.append(int(buf[i]) & 0xFF)
    return vals


def run():
    gc.collect()
    print("MEM_FREE_BEFORE", _mem_free())

    wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
    led_rx = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
    buf = array("b", [0] * BUF_LEN)
    total = 0
    t0 = utime.ticks_ms()

    while utime.ticks_diff(utime.ticks_ms(), t0) < RUN_MS:
        try:
            n = wireless.receive_bytearray(buf, BUF_LEN)
        except Exception as e:
            print("EXC", e)
            break
        if n:
            total += n
            led_rx.value(0 if led_rx.value() else 1)
            print("RX", n, _rx_values(buf, n))
        utime.sleep_ms(POLL_MS)

    gc.collect()
    print("MEM_FREE_AFTER", _mem_free())
    print("TOTAL_BYTES", total)


run()
