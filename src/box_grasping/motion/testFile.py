import socket
import time

import gripperHelper

HOST = "192.168.0.100"
PORT_DASHBOARD = 29999
PORT_GRIPPER = 30002

WIDTH_COMMAND = "getVariable rg_Width"
GRASP_DETECTED_COMMAND = "getVariable rg_Grip_detected"


def changeWidthAndForce(width, force):
    temp = gripperHelper.getWidthString(width, force)
    sendDashboardCommand(temp, PORT_GRIPPER)


def sendDashboardCommand(cmd, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, port))
    cmd = cmd + "\n"
    print(cmd)
    s.sendall(cmd.encode())
    # time.sleep(1)  # might want to delete this
    rcvd = s.recv(4096)
    if port != PORT_GRIPPER:
        rcvd = rcvd.decode("utf8")
    s.close()


changeWidthAndForce(40, 5)
