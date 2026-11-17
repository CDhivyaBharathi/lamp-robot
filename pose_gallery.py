#!/usr/bin/env python
#
# Pose gallery: design poses in the sim (sim -> real), then play them on the lamp.
#
# Each pose is described by body shape (lower arm c, upper arm d), how far the
# head looks up/down (degrees, + = up) and turns (degrees, + = right), plus an
# optional small base turn (degrees). The exact head-tilt (f) value is solved
# against the sim model so "pitch 0" means the face is at the same angle as
# the normal watching pose, whatever the body is doing. That's what turns a
# description like "bow, head down 50 degrees" into raw servo values.
#
# To add a pose: add a row to GALLERY, preview it with --sim, try it with
# --real, and if you like it copy its printed raw values into idle_show.py.
#
#   python pose_gallery.py --list           print every pose's raw servo values
#   python pose_gallery.py                  play all poses in the sim
#   python pose_gallery.py 5 12             play only poses 5 and 12 (in the sim)
#   python pose_gallery.py --real 5 12      play them slowly on the real lamp
#   python pose_gallery.py --mirror 5 12    real lamp + sim together
#

import math
import sys
import time

import mujoco
import numpy as np

from joint_angles import raw_to_rad, GEAR_RATIO
from joint_config import POS_RANGE, clamp
from local_config import SCENE_PATH
from poses import WATCHING as WATCHING_POSE

BASE = WATCHING_POSE["a"]
HEAD_STRAIGHT = WATCHING_POSE["e"]
COUNTS_PER_DEG = 4096 / 360.0
BASE_COUNTS_PER_DEG = 4096 * GEAR_RATIO / 360.0

WATCHING = {j: WATCHING_POSE[j] for j in ("c", "d", "f", "e")}   # reference for "pitch 0"

#         name                    c      d    pitch  turn  base   what it's meant to read as
GALLERY = [
    ("curious",                  927,  1411,    0,     0,    0,  "the normal watching pose"),
    ("alert",                   1286,  1840,    0,     0,    0,  "perked up, paying attention"),
    ("sleepy",                  None,  None,  None,  None,   0,  "existing hand-posed sleepy"),
    ("tall / proud",            None,  None,  None,  None,   0,  "your hand-posed tall stance"),
    ("shy",                     1150,  1250,  -35,   -30,   -5,  "head down, turned away a little"),
    ("sad",                     1450,  1150,  -45,     0,    0,  "drooping, looking at the floor"),
    ("peek left",               1000,  1500,    5,   -60,   -6,  "looking off to its left"),
    ("peek right",              1000,  1500,    5,    60,    6,  "looking off to its right"),
    ("thinking",                1100,  1600,   20,    35,    0,  "head up and to the side, pondering"),
    ("bow",                     1600,  1300,  -50,     0,    0,  "polite bow"),
    ("reading lamp",            1500,  1600,  -70,     0,    0,  "lighting the desk in front of it"),
    ("spotlight",               1100,  1700,  -12,     0,    0,  "shining straight at you"),
    ("stretch",                 1500,  2500,   25,     0,    0,  "reaching up tall"),
    ("tucked in",               1050,   950,  -40,     0,    0,  "folded up small, head down"),
]
RAW_POSES = {
    "sleepy": {"c": 1753, "d": 913, "f": 2903, "e": 894},
    "tall / proud": {"c": 1682, "d": 2771, "f": 2905, "e": 931},
}

model = mujoco.MjModel.from_xml_path(SCENE_PATH)
data = mujoco.MjData(model)
HEAD = model.body("lamp_head").id
BASE_BODY = model.body("part_2_2").id   # sits on the base's rotation axis


def fk(p):
    data.qpos[:] = 0
    for j, raw in p.items():
        if j != "a":
            data.joint(j).qpos[0] = raw_to_rad(j, raw)
    mujoco.mj_forward(model, data)
    return data.xpos[HEAD].copy(), data.xmat[HEAD].reshape(3, 3).copy()


_, R_REF = fk(WATCHING)
_p0, _ = fk(WATCHING)
_pf, _ = fk({**WATCHING, "c": WATCHING["c"] + 250})
FWD = _pf - _p0
FWD[2] = 0
FWD /= np.linalg.norm(FWD)


def level_f(c, d):
    """f that gives the same head angle as the watching pose for this body shape."""
    best = None
    for f in range(POS_RANGE["f"][0], POS_RANGE["f"][1] + 1, 5):
        _, R = fk({"c": c, "d": d, "f": f, "e": HEAD_STRAIGHT})
        err = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R_REF.T @ R) - 1) / 2))))
        if best is None or err < best[0]:
            best = (err, f)
    return best[1]


def build(entry):
    name, c, d, pitch, turn, base, _ = entry
    if name in RAW_POSES:
        pose = dict(RAW_POSES[name])
    else:
        f = level_f(c, d) - pitch * COUNTS_PER_DEG   # head tilt: lower f = up
        e = HEAD_STRAIGHT + turn * COUNTS_PER_DEG    # head turn: higher e = right
        pose = {"c": c, "d": d, "f": int(round(f)), "e": int(round(e))}
    pose["a"] = int(round(BASE + base * BASE_COUNTS_PER_DEG))
    return {j: clamp(j, v) for j, v in pose.items()}


def head_offset_cm(pose):
    """How far in front (+) / behind (-) the base axis the head sits."""
    p, _ = fk(pose)
    return float(np.dot(p - data.xpos[BASE_BODY], FWD)) * 100


def travel_time(a, b, real):
    """Sim moves can be quick; on the real lamp, bigger joint travel gets more
    time so long swings don't build momentum (fast moves tipped it over)."""
    if not real:
        return 1.5
    return min(4.5, 2.5 + max(abs(a[j] - b[j]) for j in ("c", "d", "f")) / 800.0)


def main():
    poses = [(i + 1, entry, build(entry)) for i, entry in enumerate(GALLERY)]
    # Head offset alone doesn't predict tipping: the watching pose sits further
    # back than a crouch that fell, and it's never fallen standing still - the
    # falls came from fast moves. Shown for reference only.
    for n, entry, pose in poses:
        print("%2d. %-14s c=%4d d=%4d f=%4d e=%4d a=%4d | head %+5.1f cm from base | %s"
              % (n, entry[0], pose["c"], pose["d"], pose["f"], pose["e"], pose["a"],
                 head_offset_cm(pose), entry[6]))

    if "--list" in sys.argv:
        return

    wanted = {int(x) for x in sys.argv[1:] if x.isdigit()}
    real = "--real" in sys.argv or "--mirror" in sys.argv
    from poses import move
    from driver import make_driver

    driver = make_driver(["--mirror"] if "--mirror" in sys.argv else [] if real else ["--sim"])
    try:
        move(driver, WATCHING_POSE, 3.0 if real else 1.0)
        for n, entry, pose in poses:
            if wanted and n not in wanted:
                continue
            t = travel_time(WATCHING_POSE, pose, real)
            print("\n>>> %d. %s - %s" % (n, entry[0], entry[6]), flush=True)
            move(driver, pose, t)
            time.sleep(2.5)
            if real:   # back to watching between poses - never swing straight from one extreme to another
                move(driver, WATCHING_POSE, t)
                time.sleep(0.5)
        move(driver, WATCHING_POSE, 3.0 if real else 1.5)
    finally:
        driver.close()


if __name__ == "__main__":
    main()
