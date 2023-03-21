# `ROS implementation of navigation concepts`
This code uses a premade ROS navigation stack that displays on rviz. Goal is to use multiple navigation packages and integrate them to be used for navigating a route.
1. Python code is fed a global route message
2. Decodes the route message into local poses to send
3. Premade package drives the robot to the specified pose while avoiding obstacles