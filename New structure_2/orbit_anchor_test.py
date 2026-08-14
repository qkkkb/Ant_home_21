"""PC-only geometry check for rotating around a point beyond follower FR.

Coordinates use x=forward and y=left.  Positive yaw is counter-clockwise.
The wheel map matches New structure_2/move_base.py; this file never imports
hardware or main.py.
"""

import argparse
import math


# Geometry is intentionally explicit so it can be replaced with measured data.
WHEEL_FR = (70.0, -45.0)
WHEEL_FL = (70.0, 45.0)
WHEEL_B = (-70.0, 0.0)
AXIS_FROM_FR = (0.0, 100.0)
ROTATION_RATE = 1.0
SAMPLE_COUNT = 12

WHEEL_POSITIONS = (
    ("FR", WHEEL_FR),
    ("FL", WHEEL_FL),
    ("B", WHEEL_B),
)


def rotate(point, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    x, y = point
    return c * x - s * y, s * x + c * y


def add(a, b):
    return a[0] + b[0], a[1] + b[1]


def sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def axis_from_fr():
    return add(WHEEL_FR, AXIS_FROM_FR)


def wheel_targets(vx, vy, wz):
    """Same normalized wheel map used by move_base.py."""
    return (
        -0.866025 * vx + 0.5 * vy + wz,
        0.866025 * vx + 0.5 * vy + wz,
        -vy + wz,
    )


def twist_about_axis(axis, wz):
    """Body twist that keeps the body-frame point `axis` fixed in world space."""
    axis_x, axis_y = axis
    return wz * axis_y, -wz * axis_x, wz


def world_wheel_positions(body_origin, body_angle):
    return tuple(
        (name, add(body_origin, rotate(position, body_angle)))
        for name, position in WHEEL_POSITIONS
    )


def run_case(direction):
    axis_body = axis_from_fr()
    wz = direction * ROTATION_RATE
    vx, vy, _ = twist_about_axis(axis_body, wz)
    targets = wheel_targets(vx, vy, wz)

    # The fixed world axis is the initial axis point.  For a rigid body
    # rotating around it, the body origin is A - R(theta) * axis_body.
    axis_world = axis_body
    samples = []
    for index in range(SAMPLE_COUNT + 1):
        angle = direction * (2.0 * math.pi * index / SAMPLE_COUNT)
        body_origin = sub(axis_world, rotate(axis_body, angle))
        samples.append((angle, body_origin, world_wheel_positions(body_origin, angle)))

    final_origin = samples[-1][1]
    closure_error = math.hypot(final_origin[0], final_origin[1])
    axis_drift = 0.0
    for angle, body_origin, _ in samples:
        measured_axis = add(body_origin, rotate(axis_body, angle))
        axis_drift = max(
            axis_drift,
            math.hypot(measured_axis[0] - axis_world[0], measured_axis[1] - axis_world[1]),
        )

    return axis_body, (vx, vy, wz), targets, samples, closure_error, axis_drift


def print_case(label, case):
    axis_body, twist, targets, samples, closure_error, axis_drift = case
    vx, vy, wz = twist
    print("\n=== %s ===" % label)
    print("FR=%8.2f,%8.2f  axis-from-FR=%8.2f,%8.2f  axis-body=%8.2f,%8.2f" % (
        WHEEL_FR[0], WHEEL_FR[1], AXIS_FROM_FR[0], AXIS_FROM_FR[1],
        axis_body[0], axis_body[1],
    ))
    print("body twist: vx=%8.3f vy=%8.3f wz=%8.3f" % (vx, vy, wz))
    print("wheel target: FR=%8.3f FL=%8.3f B=%8.3f" % targets)
    print("angle(deg)       body(x,y)                 FR(x,y)                 FL(x,y)                  B(x,y)")
    for angle, body_origin, wheels in samples:
        points = [point for _, point in wheels]
        print(
            "%8.1f   (%8.2f,%8.2f)   (%8.2f,%8.2f)   (%8.2f,%8.2f)   (%8.2f,%8.2f)"
            % (math.degrees(angle), body_origin[0], body_origin[1],
               points[0][0], points[0][1], points[1][0], points[1][1],
               points[2][0], points[2][1])
        )
    print("axis drift=%0.9f  360-degree closure=%0.9f" % (axis_drift, closure_error))


def plot_cases(cases):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is unavailable; trajectory tables are still valid.")
        return

    axis = axis_from_fr()
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharex=True, sharey=True)
    for plot_axis, (label, case) in zip(axes, cases):
        _, _, _, samples, _, _ = case
        for wheel_index, wheel_name in enumerate(("FR", "FL", "B")):
            xs = [item[2][wheel_index][1][0] for item in samples]
            ys = [item[2][wheel_index][1][1] for item in samples]
            plot_axis.plot(xs, ys, label=wheel_name)
        body_x = [item[1][0] for item in samples]
        body_y = [item[1][1] for item in samples]
        plot_axis.plot(body_x, body_y, "k--", label="body origin")
        plot_axis.plot(axis[0], axis[1], "rx", markersize=10, label="fixed axis")
        plot_axis.set_title(label)
        plot_axis.set_aspect("equal", adjustable="box")
        plot_axis.grid(True)
        plot_axis.legend()
    fig.supxlabel("x (forward)")
    fig.supylabel("y (left)")
    fig.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Follower FR extension-axis orbit check")
    parser.add_argument("--plot", action="store_true", help="show trajectories with matplotlib")
    args = parser.parse_args()

    cases = (
        ("CCW (+wz)", run_case(1.0)),
        ("CW (-wz)", run_case(-1.0)),
    )
    print_case(*cases[0])
    print_case(*cases[1])
    if args.plot:
        plot_cases(cases)


if __name__ == "__main__":
    main()
