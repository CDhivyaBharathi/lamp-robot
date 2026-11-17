#
# Real <-> sim conversion: servo raw position <-> MuJoCo joint angle (radians).
#
# The ST3215 reports 4096 raw counts per 360 degrees, so the conversion is
# exact: angle = SIGN * (raw - ZERO_RAW) * 2*pi/4096.
#
# ZERO_RAW is the raw value where the real joint matches the sim model's zero
# pose. These were measured with live_mirror.py: put the real lamp in its
# neutral pose, read the raw values, and set them here. SIGN flips a joint if
# the sim moves the opposite way from the real one (all matched at +1).
#
# The base ('a') is the motor-side gear. In the sim it drives the base output
# joint 'b' through a 1:GEAR_RATIO gear, so whenever 'a' is set in the sim,
# 'b' must be set to a / GEAR_RATIO too (kinematics-only updates don't solve
# the gear constraint on their own).
#

import math

from joint_config import JOINTS, GEAR_RATIO   # noqa: F401  (GEAR_RATIO re-exported)

JOINT_NAMES = {info['id']: name for name, info in JOINTS.items()}   # servo id -> joint name

RAD_PER_COUNT = 2 * math.pi / 4096

SIGN = {name: 1 for name in JOINTS}

ZERO_RAW = {
    'a': 2041,
    'c': 1519,
    'd': 2075,
    'f': 2659,
    'e': 941,
}


def raw_to_rad(joint_name, raw):
    """Real servo position -> sim joint angle."""
    return SIGN[joint_name] * (raw - ZERO_RAW[joint_name]) * RAD_PER_COUNT


def rad_to_raw(joint_name, rad):
    """Sim joint angle -> real servo position."""
    return ZERO_RAW[joint_name] + SIGN[joint_name] * rad / RAD_PER_COUNT
