#!/usr/bin/env python
#
# Sim -> real: pose the lamp in the MuJoCo viewer and the real lamp follows.
#
# Open the viewer's right-hand "Control" panel and drag the sliders a, c, d,
# f, e. The sim updates instantly; the real lamp follows at a capped speed so
# a fast slider drag can't make it lurch (fast moves tip it over). Sliders
# are limited to the calibrated ranges and start at the lamp's current pose,
# so nothing jumps when it starts. The raw servo values are printed once a
# second - handy for saving a pose you like.
#
#   python sim_to_real.py
# Close the viewer window (or Ctrl+C) to quit; the lamp holds its pose.
#

import time

import mujoco
import mujoco.viewer

from driver import HardwareDriver
from joint_angles import raw_to_rad, rad_to_raw, GEAR_RATIO
from joint_config import BASE_SAFE_RANGE, clamp
from local_config import SCENE_PATH

JOINTS = ("a", "c", "d", "f", "e")
DT = 0.03
MAX_STEP = 25   # raw counts per update (~800 counts/s) - caps how fast the real lamp moves

model = mujoco.MjModel.from_xml_path(SCENE_PATH)
data = mujoco.MjData(model)
act = {j: model.actuator(j).id for j in JOINTS}

# Slider limits = calibrated joint ranges (base: one motor turn).
for j in JOINTS:
    if j == "a":
        lo, hi = (raw_to_rad("a", r) for r in BASE_SAFE_RANGE)
    else:
        lo, hi = model.jnt_range[model.joint(j).id]
    model.actuator_ctrllimited[act[j]] = 1
    model.actuator_ctrlrange[act[j]] = (min(lo, hi), max(lo, hi))

driver = HardwareDriver()
sent = {}
for j in JOINTS:
    raw = driver.get_pos(j)
    if raw is None:
        raise SystemExit("couldn't read joint %s - check wiring/power" % j)
    sent[j] = float(raw)
    data.ctrl[act[j]] = raw_to_rad(j, raw)   # sliders start at the real pose

print("Drag the sliders in the viewer's Control panel - the real lamp follows. Close the window to quit.")
try:
    last_print = 0.0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            viewer.sync()   # picks up slider changes into data.ctrl

            for j in JOINTS:
                rad = float(data.ctrl[act[j]])
                data.joint(j).qpos[0] = rad
                if j == "a":
                    data.joint("b").qpos[0] = rad / GEAR_RATIO   # gear: base output follows the motor

                target = clamp(j, rad_to_raw(j, rad))
                step = max(-MAX_STEP, min(MAX_STEP, target - sent[j]))
                if abs(step) >= 1:
                    sent[j] += step
                    driver.set_pos(j, int(round(sent[j])), speed=max(40, int(abs(step) / DT * 1.5)), acc=0)
            mujoco.mj_forward(model, data)   # kinematics only - the sim shows exactly the slider pose

            if time.time() - last_print > 1.0:
                print("raw:", {j: int(round(sent[j])) for j in JOINTS}, flush=True)
                last_print = time.time()
            time.sleep(DT)
finally:
    driver.close()
