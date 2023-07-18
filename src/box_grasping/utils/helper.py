#!/usr/bin/env python3

import rospy
import tf2_ros
from tf import TransformListener


def init_tf_tree() -> TransformListener:
    """Launch the tf2 transform tree to ensure transforms are available for use"""
    tf_listener = TransformListener()

    transforms_available = False
    while not transforms_available:
        try:
            tf_listener.waitForTransform(
                "/base_link", "/base_link", rospy.Time(), rospy.Duration(0.1)
            )
            transforms_available = True
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.loginfo("Waiting for tf tree")

    return tf_listener
