#!/usr/bin/env python
#
# Real -> sim: the MuJoCo model mirrors the real lamp's pose, live.
#
# Reads every servo's raw position and writes the matching joint angle into
# the sim each frame (kinematics only, no physics), so the sim shows exactly
# the pose the real lamp is in.
#
# Only one program can use the servo port at a time, so this can't watch
# while another script moves the lamp. For that, run the moving script with
# --mirror instead (idle_show.py --mirror, pose_jog.py --mirror): it drives
# the real lamp and shows the same motion in the sim at once.
#
# Also the tool for re-checking the calibration in joint_angles.py: put the
# real lamp in its neutral pose - if the sim doesn't look neutral too, read
# the printed raw values and update ZERO_RAW; if a joint moves the opposite
# way in the sim, flip its SIGN.
#
#   python live_mirror.py
# Close the viewer window (or Ctrl+C) to quit.
#

import time

import mujoco
import mujoco.viewer

from driver import HardwareDriver
from joint_angles import raw_to_rad, GEAR_RATIO
from joint_config import JOINTS
from local_config import SCENE_PATH

model = mujoco.MjModel.from_xml_path(SCENE_PATH)
data = mujoco.MjData(model)
driver = HardwareDriver()

print("Mirroring the real lamp into the sim. Close the viewer window to quit.")
try:
    last_print = 0.0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            raws = {}
            for joint in JOINTS:
                raw = driver.get_pos(joint)
                if raw is None:
                    continue
                raws[joint] = raw
                rad = raw_to_rad(joint, raw)
                data.joint(joint).qpos[0] = rad
                if joint == 'a':
                    data.joint('b').qpos[0] = rad / GEAR_RATIO   # gear: base output follows the motor
            mujoco.mj_forward(model, data)
            viewer.sync()

            if time.time() - last_print > 1.0:
                print("raw:", raws, flush=True)
                last_print = time.time()
            time.sleep(0.02)
finally:
    driver.close()
