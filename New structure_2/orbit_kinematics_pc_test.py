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
    def test_wrapped_search_phase_error(self):
        self.assertAlmostEqual(pid.wrapped_angle_error(2, 358), 4.0)
        self.assertAlmostEqual(pid.wrapped_angle_error(358, 2), -4.0)

    def test_search_phase_target_is_continuous_across_zero(self):
        offset = pid.wrapped_angle_error(29.0, 359.0)
        self.assertAlmostEqual(
            pid.wrapped_angle_error(359.0 + offset, 29.0), 0.0
        )
        self.assertAlmostEqual(
            pid.wrapped_angle_error(1.0 + offset, 31.0), 0.0
        )
        self.assertAlmostEqual(pid.wrapped_angle_error(0.2, 359.8), 0.4)

    def test_orbit_base_stays_inside_verified_limit(self):
        out = [0.0] * 8
        pid.orbit_translation(out, 0, 0, 0, 0, 145)
        self.assertAlmostEqual(out[0], -14.0, places=6)
        self.assertAlmostEqual(out[1], -13.480543, places=5)

        pid.limit_pose_twist_for_wheels(
            out,
            out[0],
            out[1],
            5.0,
            out[0],
            out[1],
            0,
            0,
            0,
            0,
            0,
            False,
            2,
            False,
        )
        self.assertEqual(out[3], 100)
        self.assertLessEqual(max(map(abs, wheel_targets(*out[:3]))), 33.000001)

    def test_orbit_feedback_keeps_same_base_limit(self):
        out = [0.0] * 8
        pid.orbit_translation(out, 8, -6, 0, 0, 145)
        self.assertAlmostEqual(out[0], -6.0, places=6)
        self.assertAlmostEqual(out[1], -19.480543, places=5)


if __name__ == "__main__":
    unittest.main()
