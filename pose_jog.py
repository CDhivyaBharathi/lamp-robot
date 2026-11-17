#!/usr/bin/env python
#
# Keyboard control of every joint on the real lamp - for designing poses by hand.
#
#   1-5          pick joint: 1=a (base)  2=c (lower arm)  3=d (upper arm)  4=f (head tilt)  5=e (head turn)
#   Up/Right     move the selected joint +step
#   Down/Left    move the selected joint -step
#   + / -        bigger / smaller step
#   p            print the whole pose (raw values - paste into a pose dict)
#   ESC          quit (servos hold where they are)
#
# Starts from wherever the lamp currently is - no jump on launch. Every joint
# is clamped to its calibrated range (base: one motor turn).
#
#   python pose_jog.py             real lamp
#   python pose_jog.py --mirror    real lamp, shown live in the sim too
# Needs a real terminal (it reads single key presses).
#

import os
import sys

from driver import make_driver
from joint_config import clamp

JOINT_KEYS = {'1': 'a', '2': 'c', '3': 'd', '4': 'f', '5': 'e'}
LABELS = {'a': 'base', 'c': 'lower arm', 'd': 'upper arm', 'f': 'head tilt', 'e': 'head turn'}
STEPS = [5, 10, 25, 50, 100]

driver = make_driver(sys.argv)
pose = {}
for j in JOINT_KEYS.values():
    pos = driver.get_pos(j)
    if pos is None:
        print("couldn't read joint %s - check wiring/power" % j)
        driver.close()
        sys.exit(1)
    pose[j] = pos

selected = 'c'
step_i = 2


def show():
    joints = "  ".join(("[%s=%d]" if j == selected else " %s=%d ") % (j, pose[j]) for j in JOINT_KEYS.values())
    print("\r%s   step=%d   (%s)      " % (joints, STEPS[step_i], LABELS[selected]), end="", flush=True)


if os.name == 'nt':
    import msvcrt
    ARROWS = {b'H': 'UP', b'P': 'DOWN', b'K': 'LEFT', b'M': 'RIGHT'}

    def get_key():
        ch = msvcrt.getch()
        if ch in (b'\x00', b'\xe0'):
            return ARROWS.get(msvcrt.getch())
        if ch == b'\x1b':
            return 'ESC'
        return ch.decode(errors='ignore')
else:
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    ARROWS = {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}

    def get_key():
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                if sys.stdin.read(1) != '[':
                    return 'ESC'
                return ARROWS.get(sys.stdin.read(1))
            return ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


print("1-5 pick joint, arrows move, +/- step size, p print pose, ESC quit")
show()
try:
    while True:
        key = get_key()
        if key == 'ESC':
            break
        if key in JOINT_KEYS:
            selected = JOINT_KEYS[key]
        elif key in ('+', '='):
            step_i = min(step_i + 1, len(STEPS) - 1)
        elif key in ('-', '_'):
            step_i = max(step_i - 1, 0)
        elif key in ('UP', 'RIGHT', 'DOWN', 'LEFT'):
            sign = 1 if key in ('UP', 'RIGHT') else -1
            pose[selected] = clamp(selected, pose[selected] + sign * STEPS[step_i])
            driver.set_pos(selected, pose[selected])
        elif key == 'p':
            print("\npose:", {j: pose[j] for j in ('a', 'c', 'd', 'f', 'e')})
        show()
finally:
    print("\nfinal pose:", {j: pose[j] for j in ('a', 'c', 'd', 'f', 'e')})
    driver.close()
