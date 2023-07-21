#!/usr/bin/env python3

from pathlib import Path

import rospy

from box_grasping.config import Config
from box_grasping.motion.ur3e_motion import MotionType, Ur3eMover
from box_grasping.utils.moveit_helper import load_joint_trajectory
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
