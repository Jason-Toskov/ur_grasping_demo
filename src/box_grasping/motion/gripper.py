#!/usr/bin/env python3
import socket
import time
from distutils.util import strtobool

import gripperHelper
import rospy
from ur_grasping.srv import Grasp, GraspResponse

HOST = "192.168.0.100"
PORT_DASHBOARD = 29999
PORT_GRIPPER = 30002

WIDTH_COMMAND = "getVariable rg_Width"
GRASP_DETECTED_COMMAND = "getVariable rg_Grip_detected"
CHECK_BUSY_COMMAND = "getVariable rg_Busy"


def ChangeWidth():
    rospy.init_node("change_width_server")
    service = rospy.Service("change_width_and_force", Grasp, callback)
    print("Ready to change width and force")
    rospy.spin()


def changeWidthAndForce(width, force):
    temp = gripperHelper.getWidthString(width, force)
    sendSocketCommand(temp, PORT_GRIPPER)


def callback(req):
    # Check if too close to current width
    start_width = float(sendSocketCommand(WIDTH_COMMAND, PORT_DASHBOARD))
    if abs(start_width - req.width) < 5:
        print("Request too close to current width (within 5), not moving gripper!")
    else:
        changeWidthAndForce(req.width, req.force)

        time_start = time.time()
        busy_detected = False
        while time.time() - time_start < 15:
            try:
                is_busy = strtobool(sendSocketCommand(CHECK_BUSY_COMMAND, PORT_DASHBOARD))
            except ValueError:
                continue

            if is_busy and not busy_detected:
                busy_detected = True

            if busy_detected and not is_busy:
                break

    actual_width = float(sendSocketCommand(WIDTH_COMMAND, PORT_DASHBOARD))
    grasp_detected = strtobool(sendSocketCommand(GRASP_DETECTED_COMMAND, PORT_DASHBOARD))

    return GraspResponse(grasp_detected=grasp_detected, actual_width=actual_width)


def sendSocketCommand(cmd, port) -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, port))
    cmd = cmd + "\n"
    s.sendall(cmd.encode())
    time.sleep(0.01)  # might want to delete this
    rcvd = s.recv(4096)
    if port != PORT_GRIPPER:
        rcvd = rcvd.decode("utf8")
    if port == PORT_DASHBOARD:
        rcvd = rcvd.split("\n")[1]

    s.close()
    return rcvd


if __name__ == "__main__":
    # changeWidthAndForce(26, 24)
    ChangeWidth()
