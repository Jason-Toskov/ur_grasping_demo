#!/usr/bin/env python3

from pathlib import Path

import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory

from box_grasping.config import Config
from box_grasping.motion.ur3e_motion import MotionType, Ur3eMover
from box_grasping.utils.moveit_helper import joint_dict_to_robot_state, load_joint_trajectory
from box_grasping.utils.tf_helper import load_yaml
from box_grasping.visual.realsense import RealSenseProcessor


class GraspDemoMaster:
    def __init__(self, params_path):
        rospy.init_node("grasp_demo_controller", anonymous=True)

        self.params = Config.parse_obj(load_yaml(params_path))

        self.mover = Ur3eMover(visualize_all_plans=False)

        self.camera = RealSenseProcessor(forward_clouds=True)

        self.key_joint_states = load_yaml(Path(self.params.paths.root) / self.params.paths.key_locs)

        self.scan_joints = load_joint_trajectory(
            Path(self.params.paths.root) / self.params.paths.trajectories.scene_scan
        )

    def construct_scan_plan(self, initial_state):
        scan_joints = load_joint_trajectory(
            Path(self.params.paths.root) / self.params.paths.trajectories.scene_scan
        )

        start_joints = [initial_state] + scan_joints
        target_joints = scan_joints + [initial_state]

        joint_traj_points = []
        for start, finish in zip(start_joints, target_joints):
            start_robot_state = joint_dict_to_robot_state(start)
            self.mover.move_group.set_start_state(start_robot_state)
            self.mover.move_group.set_joint_value_target(finish)
            plan = self.mover.unpack_plan(self.mover.move_group.plan())
            if plan is None:
                raise ValueError("Scan plan failed!")
            joint_traj_points += plan.joint_trajectory.points
            self.mover.move_group.clear_pose_targets()

        plan_overall = RobotTrajectory(
            joint_trajectory=JointTrajectory(
                joint_names=plan.joint_trajectory.joint_names,
                points=joint_traj_points,
            )
        )

        self.mover.move_group.set_start_state_to_current_state()

        return plan_overall

    def main(self):
        while not rospy.is_shutdown():
            self.mover.move(self.key_joint_states["home"], motion_type=MotionType.joint)
            rospy.sleep(1)

            for joint in self.scan_joints:
                self.mover.move(joint, motion_type=MotionType.joint)


if __name__ == "__main__":
    params_path = "/home/jason/catkin_ws/src/ur_grasping/src/box_grasping/configs/base_config.yaml"
    try:
        grasper = GraspDemoMaster(params_path=params_path)
        grasper.main()
    except KeyboardInterrupt:
        pass
