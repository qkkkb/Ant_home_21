import importlib.util
import os
import sys
import types
import unittest


MASTER_DIR = os.path.join(os.path.dirname(__file__), "..", "New structure")
MASTER_PATH = os.path.join(MASTER_DIR, "coop_master.py")
MASTER_MAIN_PATH = os.path.join(MASTER_DIR, "main_lsm6dsv16x_95.py")
FOLLOWER_MAIN_PATH = os.path.join(os.path.dirname(__file__), "main.py")


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
    def test_normal_orbit_and_search_spin_keep_body_motion(self):
        module = load_coop_master()
        capture = Capture()
        module._wireless = capture
        module._tx_buf = bytearray(16)
        module._vx = 12.3
        module._vy = -4.5
        module._wz = 67.8
        module._wheel_fl = 11.1
        module._wheel_fr = 22.2
        module._wheel_b = 33.3

        module.send_if_due(100, True, 5, False, 123.4, 1.0, 2.0, 70.0)
        module.send_if_due(200, True, 16, False, 245.6, 1.0, 2.0, 70.0)

        normal, search = capture.frames
        self.assertEqual(normal[14], 5)
        self.assertEqual(search[14], 16)
        self.assertTrue(normal[13] & 0x08)
        self.assertFalse(search[13] & 0x08)
        self.assertTrue(search[13] & 0x10)
        self.assertEqual(normal[5:13], search[5:13])
        self.assertAlmostEqual(
            int.from_bytes(search[5:7], "little", signed=True) / 10.0,
            12.3,
            places=1,
        )
        self.assertAlmostEqual(
            int.from_bytes(search[7:9], "little", signed=True) / 10.0,
            -4.5,
            places=1,
        )
        self.assertAlmostEqual(
            int.from_bytes(search[9:11], "little", signed=True) / 10.0,
            70.0,
            places=1,
        )
        self.assertAlmostEqual(
            int.from_bytes(search[11:13], "little", signed=True) / 10.0,
            67.8,
            places=1,
        )

    def test_search_spin_uses_independent_verified_direction(self):
        with open(MASTER_MAIN_PATH, "r", encoding="utf-8") as source_file:
            source = source_file.read()

        self.assertIn("Nav_Search_Spin_Dir = 1", source)
        self.assertIn(
            "spin_step = Nav_Search_Spin_Dir * gyro_z * spin_dt_ms * 0.001",
            source,
        )
        self.assertIn(
            "turn_rate_cmd = Nav_Search_Spin_Dir * spin_rate_mag",
            source,
        )

    def test_search_spin_uses_body_twist_spin_entry(self):
        with open(FOLLOWER_MAIN_PATH, "r", encoding="utf-8") as source_file:
            source = source_file.read()

        self.assertNotIn("if raw_state_code == 16:", source)
        self.assertIn(
            "explicit_spin and master_state_code != 16",
            source,
        )
        self.assertIn(
            "elif last_follow_mode_key == 0 and mode_key == 3:",
            source,
        )
        self.assertIn("if master_state_code == 16:", source)
        self.assertIn("ff_wz = clamp(ff_wz, -160.0, 160.0)", source)


if __name__ == "__main__":
    unittest.main()
