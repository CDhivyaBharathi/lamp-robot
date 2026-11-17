#
# Machine-specific settings: which serial port the servo board is on, and
# where the sim model lives. Works unchanged on Windows and macOS.
#
# Override with environment variables if auto-detection picks the wrong thing:
#   LAMP_SERVO_PORT   e.g. COM5  or  /dev/cu.usbmodem5B14033939
#   LAMP_SCENE_PATH   path to sim/scene.xml
#

import os
import sys

from serial.tools import list_ports

# USB id of the servo bus board's CH343 USB-serial chip. Matching on this (not
# just "any USB serial port") keeps it from grabbing some other device.
SERVO_BOARD_VID = 0x1A86
SERVO_BOARD_PID = 0x55D3


def find_servo_port():
    override = os.environ.get("LAMP_SERVO_PORT")
    if override:
        return override

    matches = [p.device for p in list_ports.comports()
               if p.vid == SERVO_BOARD_VID and p.pid == SERVO_BOARD_PID]
    if sys.platform == "darwin":
        # macOS lists each device twice (tty.* and cu.*) - cu.* is the one to open
        matches = [m for m in matches if "/cu." in m] or matches
    if matches:
        return matches[0]

    print("WARNING: servo board not found by USB id - check the cable, or set LAMP_SERVO_PORT")
    return "COM5" if sys.platform == "win32" else "/dev/cu.usbmodem"


DEVICENAME = find_servo_port()

_HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_PATH = os.environ.get("LAMP_SCENE_PATH", os.path.join(_HERE, "sim", "scene.xml"))
