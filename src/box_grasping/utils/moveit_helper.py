import pandas as pd
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion


def load_trajectory(path):
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
