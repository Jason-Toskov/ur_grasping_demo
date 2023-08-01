# RIOT Lab UR3E Grasping Demo Package

## Installation

TODO: Add instructions for installing the ROS package and it's dependencies.

## Running the demo

### Environment setup

To run the demo, first make sure the robot is powered on and connected to the network. Also ensure the camera is plugged into the computer. Then, the following commands should be run to set up the ROS environment. Note that each of these commands should be run in a separate terminal window to help avoid ROS sourcing shenanigans.

- Terminal 1:

  ```bash
  source ${ROS1_INSTALL_PATH}/setup.bash
  roscore
  ```

- Terminal 2:

  ```bash
  source ${ROS1_INSTALL_PATH}/setup.bash
  srcros2
  export ROS_MASTER_URI=http://localhost:11311
  ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
  ```

- Terminal 3:

  ```bash
  srcros2 
  ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
  ```

- Terminal 4:

  ```bash
  srcros1
  roslaunch ur_grasping demo.launch
  ```

### Starting the demo

To start the demo, once the environment has been set up run:

```bash
rosrun ur_grasping demo.py
```

## Misc

Some useful links/notes for ros and python:

- [setup.py for ros packages](http://docs.ros.org/en/melodic/api/catkin/html/user_guide/setup_dot_py.html)
- [Might need this?](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html)
- For the catkin-run setup.py to play nice, python files need to end in .py, not have no extension and rely on the shebang line. If the extension isn't included in the filename, it seems to be missed by setup.py.
