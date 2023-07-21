#!/usr/bin/env python3

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2

from box_grasping.utils.tf_helper import init_tf_tree
from ur_grasping.msg import TFMatchedPCD


class RealSenseProcessor:
    # Class that provides services to easily interface with an attached RealSense camera
    def __init__(self, forward_clouds: bool = False) -> None:
        # Whether to republish clouds for further processing
        self.forward_clouds = forward_clouds

        self.tf_listener = init_tf_tree()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

        self.pcd_reader = rospy.Subscriber(
            "/camera/depth/color/points", PointCloud2, self.pcd_callback
        )

        self.pcd_forwarder = rospy.Publisher("/pcl_stitcher_input", PointCloud2, queue_size=1)

    def pcd_callback(self, data: PointCloud2) -> None:
        """Stores the most recently received pcl ros message"""
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", data.header.frame_id, data.header.stamp, rospy.Duration(0.1)
            )
        except:
            # TODO: this seems to always occur once, why?
            #   Shouldn't be a major issue but is annoying nonetheless
            rospy.logwarn(f'Transform from "base_link to {data.header.frame_id} not found!"')
            rospy.loginfo("Pcd dropped due to missing transform")
            return

        self.pcl_latest = TFMatchedPCD(pcd=data, tf=tf)

        if self.forward_clouds:
            self.pcd_forwarder.publish(data)

    def stitch_pcds(start_time, end_time) -> PointCloud2:
        # This should call laser assembler with these times to stitch the relevant point clouds
        # The idea here is that we collect clouds through some known time elsewhere which
        # are auto-published to the assembler topic, and then the call here stitches all of
        # the clouds together

        # laser_assembler should (i think) track the transforms on its own, but maybe only
        # for some set time. IE. i don't know if it explicitly tracks the best matching tf
        # for each stored frame or if it just keeps all tfs in a buffer

        # Need to add the laser_assembler package to catkin as well
        pass
