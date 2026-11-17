#
# A "driver" that renders into the MuJoCo viewer instead of moving servos.
# Same set_pos/get_pos calls as HardwareDriver, so any motion script can run
# in the sim first (usually via a --sim flag).
#
# Note: on some machines MuJoCo segfaults while the viewer window is closing,
# AFTER everything has run - harmless.
#

import mujoco
import mujoco.viewer

from joint_angles import raw_to_rad, ZERO_RAW, GEAR_RATIO
from joint_config import POS_RANGE, clamp
from local_config import SCENE_PATH


class SimViewerDriver:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(SCENE_PATH)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.positions = {}
        self.set_pos('a', ZERO_RAW['a'])
        for joint_name, (low, high) in POS_RANGE.items():
            self.set_pos(joint_name, (low + high) // 2)

    def set_pos(self, joint_name, pos, speed=None, acc=None):
        pos = clamp(joint_name, pos)
        self.positions[joint_name] = pos
        rad = raw_to_rad(joint_name, pos)
        self.data.joint(joint_name).qpos[0] = rad
        if joint_name == 'a':
            self.data.joint('b').qpos[0] = rad / GEAR_RATIO   # gear: base output follows the motor
        mujoco.mj_forward(self.model, self.data)   # kinematics only, no physics
        self.viewer.sync()

    def get_pos(self, joint_name):
        return self.positions[joint_name]

    def close(self):
        self.viewer.close()


class MirroredDriver:
    """Drives the real lamp AND shows every command in the sim at the same
    time. Positions are read back from the real servos."""

    def __init__(self):
        from driver import HardwareDriver
        self.hw = HardwareDriver()
        self.sim = SimViewerDriver()
        for joint in ('a', 'c', 'd', 'f', 'e'):   # start the sim where the lamp actually is
            raw = self.hw.get_pos(joint)
            if raw is not None:
                self.sim.set_pos(joint, raw)

    def set_pos(self, joint_name, pos, speed=None, acc=None):
        self.hw.set_pos(joint_name, pos, speed=speed, acc=acc)
        self.sim.set_pos(joint_name, pos)

    def get_pos(self, joint_name):
        return self.hw.get_pos(joint_name)

    def close(self):
        self.hw.close()
        self.sim.close()
