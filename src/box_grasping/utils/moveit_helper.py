from typing import Dict, List

import pandas as pd
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def load_pose_trajectory(path: str) -> List[Pose]:
    trajectory_df = pd.read_csv(path)
    waypoints = []
    for _, row in trajectory_df.iterrows():
        position = Point(x=row["pos.x"], y=row["pos.y"], z=row["pos.y"])
        orientation = Quaternion(
            x=row["orient.x"], y=row["orient.y"], z=row["orient.z"], w=row["orient.w"]
        )
        # pose = PoseStamped(pose=Pose(position=position, orientation=orientation))
        # pose.header.frame_id = row["frame_id"]
        pose = Pose(position=position, orientation=orientation)
        waypoints.append(pose)
    return waypoints


def load_joint_trajectory(path: str) -> List[Dict[str, List[float]]]:
    df = pd.read_csv(path)
    joint_dicts = [{col: row[col] for col in df.columns} for _, row in df.iterrows()]
    return joint_dicts


def joint_dict_to_robot_state(joint: Dict[str, List[float]]):
    names, values = zip(*joint.items())
    robot_state = RobotState(
        joint_state=(
            JointState(
                name=names,
                position=values,
                header=Header(stamp=rospy.Time.now()),
            )
        )
    )
    return robot_state
