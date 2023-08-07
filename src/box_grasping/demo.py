#!/usr/bin/env python3

import os
from pathlib import Path

import rospy
from laser_assembler.srv import AssembleScans2, AssembleScans2Request
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from box_grasping.config import ROOT_PATH, Config
from box_grasping.motion.ur3e_motion import MotionType, Ur3eMover
from box_grasping.utils.moveit_helper import load_joint_trajectory
from box_grasping.utils.tf_helper import load_yaml
from ur_grasping.srv import Grasp, PCLFwdStatus, PCLFwdStatusRequest


class GraspDemoMaster:
    def __init__(self, params_path):
        rospy.init_node("grasp_demo_controller", anonymous=True)

        rospy.loginfo("Waiting for: set_pcd_fwd_status_server")
        rospy.wait_for_service("/realsense_processing/set_pcd_fwd_status_server")
        self.set_pcd_fwding = rospy.ServiceProxy(
            "/realsense_processing/set_pcd_fwd_status_server", PCLFwdStatus
        )

        rospy.loginfo("Waiting for: stitch_pcd_service")
        rospy.wait_for_service("/realsense_processing/stitch_pcd_service")
        self.stitch_pcds = rospy.ServiceProxy(
            "/realsense_processing/stitch_pcd_service", AssembleScans2
        )

        rospy.loginfo("Waiting for: change_grasp")
        rospy.wait_for_service("change_grasp")
        self.set_gripper_width = rospy.ServiceProxy("change_grasp", Grasp)

        self.reactivate_ext_control = rospy.ServiceProxy(
            "/ur_hardware_interface/resend_robot_program", Trigger
        )

        self.add_pcd_to_assembler = rospy.ServiceProxy(
            "/realsense_processing/add_pcd_to_assembler", Trigger
        )

        self.params = Config.parse_obj(load_yaml(params_path))

        self.mover = Ur3eMover(visualize_all_plans=False)

        self.key_joint_states = load_yaml(Path(self.params.paths.root) / self.params.paths.key_locs)

        self.scan_joints = load_joint_trajectory(
            Path(self.params.paths.root) / self.params.paths.trajectories.single_view
        )

        self.pcl_viz_pub = rospy.Publisher("/grasping_demo/pcl_vis", PointCloud2, queue_size=1)

        self.reactivate_ext_control()
        rospy.sleep(1)

    def scan_object(self) -> PointCloud2:
        start_time = rospy.Time.now()
        self.set_pcd_fwding(Bool(data=True))

        for joint in self.scan_joints:
            self.mover.move(joint, motion_type=MotionType.joint)
            if not rospy.get_param("/realsense_processing/fwd_pcd_with_timer"):
                self.add_pcd_to_assembler()
            rospy.sleep(0.1)

        self.set_pcd_fwding(Bool(data=False))
        end_time = rospy.Time.now()

        stitch_response = self.stitch_pcds(AssembleScans2Request(begin=start_time, end=end_time))
        stitched_cloud: PointCloud2 = stitch_response.cloud

        return stitched_cloud

    def move_gripper(self, width, force):
        ret = self.set_gripper_width(width=width, force=force)
        self.reactivate_ext_control()
        rospy.sleep(0.1)
        return ret

    def main(self):
        while not rospy.is_shutdown():
            self.mover.move(self.key_joint_states["home"], motion_type=MotionType.joint)
            rospy.sleep(1)

            ret = self.move_gripper(30, 20)

            ret = self.move_gripper(30, 20)

            ret = self.move_gripper(100, 20)

            pcd = self.scan_object()

            self.pcl_viz_pub.publish(pcd)


if __name__ == "__main__":
    params_path = os.path.join(ROOT_PATH, "configs/base_config.yaml")
    try:
        grasper = GraspDemoMaster(params_path=params_path)
        grasper.main()
    except KeyboardInterrupt:
        pass
