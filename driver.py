#
# Talks to the real lamp's servos. Every motion script uses the same two calls
# - set_pos(joint, raw) and get_pos(joint) - so it can drive either this or
# SimViewerDriver (sim_viewer_driver.py) unchanged.
#

from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
from joint_config import DEVICENAME, BAUDRATE, JOINTS, clamp

MOVING_SPEED = 2400
MOVING_ACC = 200


class HardwareDriver:
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = sms_sts(self.portHandler)
        if not self.portHandler.openPort():
            raise RuntimeError("couldn't open the servo port %s" % DEVICENAME)
        if not self.portHandler.setBaudRate(BAUDRATE):
            raise RuntimeError("couldn't set the baud rate on %s" % DEVICENAME)

    def set_pos(self, joint_name, pos, speed=None, acc=None):
        """speed/acc default to a fast hop. Smooth motions (poses.move) pass the
        speed that gets there just as the next update is sent, so the joint
        flows instead of sprinting to each step and stopping."""
        pos = int(clamp(joint_name, pos))
        comm_result, error = self.packetHandler.WritePosEx(
            JOINTS[joint_name]['id'], pos,
            MOVING_SPEED if speed is None else speed,
            MOVING_ACC if acc is None else acc,
        )
        if comm_result != COMM_SUCCESS:
            print("[joint %s] %s" % (joint_name, self.packetHandler.getTxRxResult(comm_result)))

    def get_pos(self, joint_name):
        scs_id = JOINTS[joint_name]['id']
        pos, comm_result, error = self.packetHandler.ReadPos(scs_id)
        if comm_result != COMM_SUCCESS:
            print("[joint %s / id %d] %s" % (joint_name, scs_id, self.packetHandler.getTxRxResult(comm_result)))
            return None
        return pos

    def close(self):
        self.portHandler.closePort()


def make_driver(argv):
    """Pick a driver from command-line flags:
         (none)     the real lamp
         --sim      the sim viewer only (needs mujoco)
         --mirror   the real lamp, with the same motion shown in the sim
    The sim is imported only when asked for, so real-lamp runs don't need mujoco."""
    if "--sim" in argv:
        from sim_viewer_driver import SimViewerDriver
        return SimViewerDriver()
    if "--mirror" in argv:
        from sim_viewer_driver import MirroredDriver
        return MirroredDriver()
    return HardwareDriver()
