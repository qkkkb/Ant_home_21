from math import sqrt

BODY_X_SIGN = -1.0


def calc_wheel_spd(move, vx, vy, vz):
    # Keep the old C kinematics, but align caller-side +vx with the real car forward direction.
    vx = BODY_X_SIGN * vx
    move.speed_fr = vx * 0.866025 + vy * 0.5 + vz
    move.speed_fl = -vx * 0.866025 + vy * 0.5 + vz
    move.speed_b = -vy + vz


def calc_wheel_spd_xy(move, vx, vy, vz):
    vx = BODY_X_SIGN * vx
    move.speed_fr = vx * sqrt(3) * 0.5 + vy * 0.5 - vz
    move.speed_fl = -vx * sqrt(3) * 0.5 + vy * 0.5 - vz
    move.speed_b = -vy - vz


def get_car_spd(move, wheel_fr, wheel_fl, wheel_b):
    move.speed_z = (wheel_fr + wheel_fl + wheel_b) * 0.33333
    move.speed_y = move.speed_z - wheel_b
    move.speed_x = BODY_X_SIGN * (wheel_fr - wheel_fl) * 0.57735


def reset_move_base(move):
    move.speed_fl = 0.0
    move.speed_fr = 0.0
    move.speed_b = 0.0
