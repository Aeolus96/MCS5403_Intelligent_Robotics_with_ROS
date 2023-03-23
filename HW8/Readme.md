# `ROS implementation of Lidar based object detection`
This code uses a premade ROS sim stack. Goal is to find objects that are closest to the robot and drive towards them and stop when close enough.
1. Python code is fed Lidar scan message and it find the closest object
2. Sends cmd_vel to the sim/robot to drive towards the object using proportional controller

> sense_obstacles_py: turns towards the closest object but does not drive towards it.

> find_objects_py: turns towards the closest object and drive towards it.