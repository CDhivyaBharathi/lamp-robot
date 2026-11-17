#!/usr/bin/env python
#
# Health check: pings servo ids 0-20 and lists which ones answer.
# All five (0-4) should respond. If none do: check the servo power supply and
# the USB cable. If only some do: check the daisy-chain connectors.
#
#   python scan_ids.py
#

from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
from joint_config import DEVICENAME, BAUDRATE

portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    raise SystemExit("couldn't open the servo port %s" % DEVICENAME)

print("Scanning ids 0-20 on %s ..." % DEVICENAME)
found = []
for scs_id in range(21):
    model_number, comm_result, error = packetHandler.ping(scs_id)
    if comm_result == COMM_SUCCESS:
        print("[ID:%03d] responded. Model number: %d" % (scs_id, model_number))
        found.append(scs_id)

if not found:
    print("No servos responded. Check wiring/power.")
elif found != [0, 1, 2, 3, 4]:
    print("Expected ids 0-4 - check the daisy chain.")
portHandler.closePort()
