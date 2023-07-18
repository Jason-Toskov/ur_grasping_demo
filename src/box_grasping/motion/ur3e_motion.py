#!/usr/bin/env python3

# Class that provides services to control robot motion using moveit!
# Can be called by other programs that wish to move the robot to a specific position/joint goal.

import sys
from enum import Enum
from typing import Tuple, Union

import moveit_commander
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotTrajectory

# Type alias for a plan
# Note that in python 3.10, this is better started as `PlanTuple: TypeAlias = ...`
PlanTuple = Tuple[bool, RobotTrajectory, float, MoveItErrorCodes]


# Enum of different motion types
class MotionType(str, Enum):
    joint = "joint"
    pose = "pose"


class Ur3eMover:
    """Object that contains all robot motion tools"""

    def __init__(self, visualize_all_plans=True):
        self.vis_all_plans = visualize_all_plans

        # Display
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )

        # Motion planning tools
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        rospy.loginfo("Motion control instantiated")

    def move(
        self,
        target: Union[list, PoseStamped],
        motion_type: MotionType,
        plan: Union[RobotTrajectory, None] = None,
        confirm_plan: bool = False,
    ) -> bool:
        """Execute a robot motion to a specified target.

        Args:
            target: Target to move to.
            motion_type: Type of motion.
            plan: Plan to execute if one exists. Defaults to None.
            confirm_plan: Whether to require motion plan confirmation.
                A False value here can be overridden by the global option
                `self.vis_all_plans`. Defaults to False.

        Raises:
            ValueError: Raised if `motion_type` is invalid.

        Returns:
            Flag indicating whether the motion was attempted.
        """

        # Set motion target
        if motion_type == MotionType.joint:
            self.move_group.set_joint_value_target(target)
        elif motion_type == MotionType.pose:
            self.move_group.set_pose_target(target)
        else:
            raise ValueError(f"Motion type {motion_type} not supported")

        if plan is None:
            plan_tuple: PlanTuple = self.move_group.plan()
            plan = self.unpack_plan(plan_tuple)

            # None if planning failed
            if plan is None:
                rospy.loginfo("No plan was found!")
                return False

        # Execute plan
        if not (confirm_plan or self.vis_all_plans) or self.check_plan_visually(plan):
            attempted = self.move_group.execute(plan, wait=True)
        else:
            rospy.loginfo("Plan is invalid!")
            attempted = False

        # Reset after motion
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return attempted

    def show_motion(self, plan: RobotTrajectory) -> None:
        """Function to preview a planned motion.

        Args:
            plan: Plan to display.
        """

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def check_plan_visually(self, plan: RobotTrajectory) -> bool:
        """Function to check whether a plan is a valid real-world move.
        Requires user to view the motion (probably in rviz) and then confirm if the plan is valid.

        Args:
            plan: Plan to check.

        Returns:
            Flag indicating whether the plan is a valid move.
        """
        run_flag = "d"

        while run_flag == "d":
            self.show_motion(plan)
            run_flag = input("Valid Trajectory [y to run]? or display path again [d to display]:")

        return True if run_flag == "y" else False

    def unpack_plan(self, plan_tuple: PlanTuple) -> Union[RobotTrajectory, None]:
        """Function used to unpack the tuple returned when planning with move_group.
        This seems to be different than is was in ros melodic, so this function
        is needed to adapt the old code to the changes.

        Args:
            plan_tuple: A plan tuple containing the plan and other success data.

        Returns:
            If the planning was successful, a trajectory that can be directly used for
            visualization and motion. If unsuccessful, None is returned.
        """

        # plan_tuple[0] is the success boolean flag
        if plan_tuple[0]:
            return plan_tuple[1]  # The RobotTrajectory
        else:
            # If needed, the exact error code can be parsed from plan_tuple[3]
            return None
