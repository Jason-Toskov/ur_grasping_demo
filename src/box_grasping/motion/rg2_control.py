#!/usr/bin/env python3

import socket
import time
from distutils.util import strtobool
from enum import Enum
from pathlib import Path
from typing import Tuple

import rospy

from ur_grasping.srv import Grasp, GraspRequest, GraspResponse

HOST = "192.168.0.100"
PORT_DASHBOARD = 29999
PORT_GRIPPER = 30002


class RG2DashboardCommand(str, Enum):
    """Common rg2-related dashboard commands"""

    get_width = "getVariable rg_Width"
    is_grasp_detected = "getVariable rg_Grip_detected"
    is_busy = "getVariable rg_Busy"


class RG2GripperControl:
    """Node for all things related to controlling the RG2 gripper"""

    def __init__(self):
        rospy.init_node("rg2_control_node")
        self.start_grasp = GraspRequest(width=80, force=20)
        self.width_error_margin = 5
        self.timeout_period = 15

        # TODO: How can this be removed?????
        # Right now if this node is in the launch file it breaks because of
        # potential headless mode messiness. Waiting here should allow the robot
        # bring-up to complete first, then we do the gripper bring-up after
        rospy.wait_for_service("/ur_hardware_interface/resend_robot_program")
        rospy.sleep(2)

        # Set the initial gripper width
        self.true_gripper_width = self.start_gripper()
        self.true_grasp_detected = False
        if abs(self.true_gripper_width - self.start_grasp.width) > self.width_error_margin:
            self.change_grasp(self.start_grasp, depth_comp=False)

        self.grasp_service = rospy.Service("change_grasp", Grasp, self.change_grasp)
        rospy.loginfo("Ready to change width and force")

        rospy.spin()

    def start_gripper(self) -> float:
        """Activate the gripper for the first time to get the true width.

        This needs a bit of a wait period to ensure the width variable is properly
        updated, so we just run this once upon the node being started."

        Returns:
            The current true gripper width
        """

        # Send header to initialise gripper
        header, ending = self.get_gripper_urscript_wrapping()
        _ = self.send_socket_command(header + ending, PORT_GRIPPER)

        # This sleep seems to be necessary to make things work properly
        rospy.sleep(5)

        return float(self.send_socket_command(RG2DashboardCommand.get_width, PORT_DASHBOARD))

    def change_grasp(self, req: GraspRequest, depth_comp=True) -> GraspResponse:
        rospy.loginfo(f"Command received to move to {req.width} with {req.force} N")

        # Check if too close to current width
        if abs(self.true_gripper_width - req.width) < self.width_error_margin:
            rospy.loginfo("Request too close to current width (within 5), not moving gripper!")
            actual_width = self.true_gripper_width
            grasp_detected = self.true_grasp_detected
        else:
            rospy.loginfo("Moving gripper")
            grip_command = self.get_grasp_change_urscript(req.width, req.force, depth_comp)
            self.send_socket_command(grip_command, PORT_GRIPPER)

            time_start = time.time()
            busy_detected = False
            while time.time() - time_start < self.timeout_period:
                try:
                    is_busy = strtobool(
                        self.send_socket_command(RG2DashboardCommand.is_busy, PORT_DASHBOARD)
                    )
                except ValueError:
                    continue

                if is_busy and not busy_detected:
                    busy_detected = True

                if busy_detected and not is_busy:
                    break

            actual_width = float(
                self.send_socket_command(RG2DashboardCommand.get_width, PORT_DASHBOARD)
            )
            grasp_detected = strtobool(
                self.send_socket_command(RG2DashboardCommand.is_grasp_detected, PORT_DASHBOARD)
            )

            # The gripper may have moved, so update the internal grip width state
            self.true_gripper_width = actual_width
            self.true_grasp_detected = grasp_detected

        return GraspResponse(grasp_detected=grasp_detected, actual_width=actual_width)

    def send_socket_command(self, cmd: str, port: int) -> str:
        """Send a command with tcp/ip via a socket.

        Args:
            cmd: String command to send to socket.
            port: Port to send command over

        Returns:
            str: Response returned by socket.
        """

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, port))

            # Commands always end in a line break
            cmd_string = cmd + "\n"

            s.sendall(cmd_string.encode())
            # This sleep seemed (maybe) necessary to get the full response
            time.sleep(0.1)  # might want to delete this if possible

            # Different commands have different ways to properly decode
            rcvd = s.recv(4096)
            # print(rcvd)
            if port != PORT_GRIPPER:
                rcvd = rcvd.decode("utf8")
            if port == PORT_DASHBOARD:
                rcvd = rcvd.split("\n")[1]

            return rcvd

    def get_grasp_change_urscript(self, width, force, depth_comp) -> str:
        header, ending = self.get_gripper_urscript_wrapping()
        grasp_cmd = f"rg_grip({width},{force}, blocking = True, depth_comp = {depth_comp})"
        return header + grasp_cmd + ending

    def get_gripper_urscript_wrapping(self) -> Tuple[str, str]:
        with open(Path(__file__).parent / "rg2_header.txt") as f:
            header = f.read()

        ending = "end\nend"

        return header, ending


if __name__ == "__main__":
    RG2GripperControl()
