import importlib.util
import os
import sys
import types
import unittest


MASTER_DIR = os.path.join(os.path.dirname(__file__), "..", "New structure")
MASTER_PATH = os.path.join(MASTER_DIR, "coop_master.py")


def load_coop_master():
    utime = types.ModuleType("utime")
    utime.ticks_diff = lambda a, b: a - b
    sys.modules["utime"] = utime

    config = types.ModuleType("config")
    config.MASTER_MOTION_TX_PERIOD_MS = 0
    config.COOP_WIRELESS_BAUD = 115200
    sys.modules["config"] = config

    models = types.ModuleType("models")
    models.MoveBase = type("MoveBase", (), {})
    sys.modules["models"] = models

    move_base = types.ModuleType("move_base")
    move_base.get_car_spd = lambda *args: None
    sys.modules["move_base"] = move_base

    seekfree = types.ModuleType("seekfree")
    seekfree.WIRELESS_UART = lambda baud: None
    sys.modules["seekfree"] = seekfree

    spec = importlib.util.spec_from_file_location("coop_master_tested", MASTER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Capture:
    def __init__(self):
        self.frames = []

    def send_bytearray(self, buf, size):
        self.frames.append(bytes(buf[:size]))


class OrbitProtocolTest(unittest.TestCase):
    def test_search_orbit_matches_normal_orbit_motion_fields(self):
        module = load_coop_master()
        capture = Capture()
        module._wireless = capture
        module._tx_buf = bytearray(16)
        module._vx = 12.3
        module._vy = -4.5
        module._wz = 67.8

        module.send_if_due(100, True, 5, False, 123.4, 1.0, 2.0, 70.0)
        module.send_if_due(200, True, 16, False, 245.6, 1.0, 2.0, 70.0)

        normal, search = capture.frames
        self.assertEqual(normal[5:14], search[5:14])
        self.assertEqual(normal[14], 5)
        self.assertEqual(search[14], 16)
        self.assertTrue(normal[13] & 0x08)
        self.assertTrue(search[13] & 0x08)


if __name__ == "__main__":
    unittest.main()
