#!/usr/bin/env python3

from pathlib import Path

import rospy

from box_grasping.config import Config
from box_grasping.motion.ur3e_motion import MotionType, Ur3eMover
from box_grasping.utils.helper import load_yaml
from box_grasping.visual.realsense import RealSenseProcessor


class GraspDemoMaster:
    def __init__(self, params_path):
        rospy.init_node("grasp_demo_controller", anonymous=True)

        self.params = Config.parse_obj(load_yaml(params_path))

        self.mover = Ur3eMover(visualize_all_plans=False)

        self.camera = RealSenseProcessor(forward_clouds=True)

        self.key_joint_states = load_yaml(Path(self.params.paths.root) / self.params.paths.key_locs)

        self.state_1 = [
            -1.090975586568014,
            -1.213568465118744,
            1.61973745027651,
            -2.103788991967672,
            -1.33171254793276,
            -0.6551120916949671,
        ]

        self.state_2 = [
            0.24536804854869843,
            -1.2220621568015595,
            1.6057284514056605,
            -2.203883310357565,
            -1.6248214880572718,
            -0.6551120916949671,
        ]

    def main(self):
        while not rospy.is_shutdown():
            self.mover.move(self.state_1, motion_type=MotionType.joint)

            rospy.sleep(1)

            self.mover.move(self.state_2, motion_type=MotionType.joint)

            rospy.sleep(1)


if __name__ == "__main__":
    params_path = "/home/jason/catkin_ws/src/ur_grasping/src/box_grasping/configs/base_config.yaml"
    try:
        grasper = GraspDemoMaster(params_path=params_path)
        grasper.main()
    except KeyboardInterrupt:
        pass