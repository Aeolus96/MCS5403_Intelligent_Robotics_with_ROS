# `ROS implementation of Camera based edge follower`
This code uses a premade ROS sim stack. Goal is to use Camera images with OpenCV and integrate into a proportional controller to drive the robot along the edge of a mat.
1. Python code is fed Camera images then converted into edge information using OpenCV
2. Sends the turning and speed corrections to the sim/robot using cmd_vel