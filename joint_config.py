#
# The lamp's joints: servo ids, calibrated ranges, gear ratio.
#
#   joint  servo id  part           raw range      direction notes (raw value going up = ...)
#   a      0         base           (no clamp)     geared 1:4 to the base output ('b' in the sim)
#   c      1         lower arm      583 - 2196     forward
#   d      2         upper arm      913 - 3048     backward
#   f      3         head tilt      1688 - 3628    head tilts down (lower = looks up)
#   e      4         head turn      27 - 1842      head turns right
#
# Raw positions are servo encoder counts, 4096 per full turn of the servo.
#

from local_config import DEVICENAME   # noqa: F401  (re-exported for the other modules)

BAUDRATE = 1000000
GEAR_RATIO = 4   # base motor turns per one full turn of the lamp's base

JOINTS = {
    'a': {'id': 0},
    'c': {'id': 1},
    'd': {'id': 2},
    'f': {'id': 3},
    'e': {'id': 4},
}

# Measured by jogging each joint to its mechanical stops.
POS_RANGE = {
    'c': (583, 2196),
    'd': (913, 3048),
    'f': (1688, 3628),
    'e': (27, 1842),
}

# The base has no clamp in POS_RANGE because it's geared and could in
# principle turn further, but everything here keeps it inside ONE motor turn
# (raw ~300-3800) so plain position mode works. Going past that needs wheel
# (continuous) mode, which tipped the lamp over and can leave the motor
# spinning if a script is killed mid-turn.
BASE_SAFE_RANGE = (300, 3800)


def clamp(joint_name, pos):
    if joint_name == 'a':
        low, high = BASE_SAFE_RANGE
    elif joint_name in POS_RANGE:
        low, high = POS_RANGE[joint_name]
    else:
        return pos
    return max(low, min(high, pos))
