import open3d as o3d

pcd = o3d.io.read_point_cloud("/home/riot/catkin_ws/clouds/1691388221783505.pcd")

o3d.visualization.draw_geometries([pcd])
