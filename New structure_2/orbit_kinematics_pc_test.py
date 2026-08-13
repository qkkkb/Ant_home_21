import os
import sys
import unittest


sys.path.insert(0, os.path.dirname(__file__))
import pid


def wheel_targets(vx, vy, vz):
    return (
        -vx * 0.866025 + vy * 0.5 + vz,
        vx * 0.866025 + vy * 0.5 + vz,
        -vy + vz,
    )


class OrbitKinematicsTest(unittest.TestCase):
    def test_search_spin_preserves_rigid_motion_ratio(self):
        out = [0.0] * 8
        pid.orbit_translation(out, 0, 0, 0, 0, 145, False)
        raw_vx = out[0]
        raw_vy = out[1]

        self.assertGreater(abs(raw_vx), 30.0)
        self.assertGreater(abs(raw_vy), 30.0)

        pid.limit_pose_twist_for_wheels(
            out,
            raw_vx,
            raw_vy,
            5.0,
            raw_vx,
            raw_vy,
            0,
            0,
            0,
            0,
            0,
            False,
            2,
            False,
        )

        scale_x = out[0] / raw_vx
        scale_y = out[1] / raw_vy
        scale_yaw = out[2] / 5.0
        self.assertAlmostEqual(scale_x, scale_y, places=6)
        self.assertAlmostEqual(scale_x, scale_yaw, places=6)
        self.assertLessEqual(max(map(abs, wheel_targets(*out[:3]))), 33.000001)

    def test_normal_orbit_keeps_existing_base_limit(self):
        out = [0.0] * 8
        pid.orbit_translation(out, 0, 0, 0, 0, 145, True)
        self.assertAlmostEqual(out[0], -14.0, places=6)
        self.assertAlmostEqual(out[1], -13.480543, places=5)


if __name__ == "__main__":
    unittest.main()
