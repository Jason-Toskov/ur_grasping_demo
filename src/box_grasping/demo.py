#!/usr/bin/env python3

from pathlib import Path

import pandas as pd
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

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

        self.scan_waypoints = self.load_trajectory(
            Path(self.params.paths.root) / self.params.paths.trajectories.scene_scan
        )

    def load_trajectory(self, path):
        trajectory_df = pd.read_csv(path)
        waypoints = []
        for _, row in trajectory_df.iterrows():
            position = Point(x=row["pos.x"], y=row["pos.y"], z=row["pos.y"])
            orientation = Quaternion(
                x=row["orient.x"], y=row["orient.y"], z=row["orient.z"], w=row["orient.w"]
            )
            pose = PoseStamped(pose=Pose(position=position, orientation=orientation))
            pose.header.frame_id = row["frame_id"]
            waypoints.append(pose)
        return waypoints

    def main(self):
        while not rospy.is_shutdown():
            self.mover.move(self.key_joint_states["test_locs"][1], motion_type=MotionType.joint)

            rospy.sleep(1)

            self.mover.move(self.key_joint_states["home"], motion_type=MotionType.joint)

            rospy.sleep(1)


if __name__ == "__main__":
    params_path = "/home/jason/catkin_ws/src/ur_grasping/src/box_grasping/configs/base_config.yaml"
    try:
        grasper = GraspDemoMaster(params_path=params_path)
        grasper.main()
    except KeyboardInterrupt:
        pass
