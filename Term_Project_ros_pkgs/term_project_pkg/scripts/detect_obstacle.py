#!/usr/bin/env python3

# # I have neither given nor received any unauthorizedaid in completing this work,
# # nor have I presented someone else's work as my own.
# # Your Name: Devson Butani
# # LTU ID: 000732711
# # Date: 12/13/2022

import rospy
from sensor_msgs.msg import LaserScan  # Sub
from geometry_msgs.msg import Twist  # Pub
from term_project_pkg.cfg import DetectObstacleConfig  # packageName.cfg
from dynamic_reconfigure.server import Server  # Dyn_rcfg-server
import math
from math import inf
import time

# Globals
global conf, wait_to_start  # Entire dynamic config
wait_to_start = True

# Get dyn_rcfg values as they update
def dyn_rcfg_cb(config, level):
    global conf, wait_to_start
    conf = config  # Store entire config
    # because variables could be changed without updating the callback lines
    wait_to_start = False
    return config


# Simplified closest object location publisher
def obj_location(distance, angle):
    if conf.pause_detect_obstacle:  # Debug pause from dyn_rcfg
        # "No object found"
        obstacle_msg.linear.x = 0  # X-axis distance from sensor
        obstacle_msg.linear.y = 0  # Y-axis distance from sensor
        obstacle_msg.linear.z = 0  # Distance between sensor and obstacle
        obstacle_msg.angular.x = 0  # Path is clear 0, obstacle 1
        obstacle_msg.angular.y = 0  # Not used yet
        obstacle_msg.angular.z = 0  # Obstacles angle from front center
        # { -z(left) -> 0(straight) -> z(right)}
    else:  # Regular publishing
        # Cartesian Coordinates of object
        cartesian_y = distance * math.cos(math.radians(angle))
        cartesian_x = distance * math.sin(math.radians(angle))
        obstacle_msg.linear.x = cartesian_x
        obstacle_msg.linear.y = cartesian_y
        obstacle_msg.linear.z = distance  # Hypot distance
        ### Check if object in path of the robot's width
        if (
            abs(cartesian_x * 1000) < (conf.squeeze_through_width / 2)  # Front path
            and cartesian_x != 0  # No object
            and (distance * 1000) < conf.turning_clearance  # Inside turning circle
        ):
            obstacle_msg.angular.x = 1  # Path is blocked
        else:
            obstacle_msg.angular.x = 0  # Path is clear
        # No need for y in this application
        obstacle_msg.angular.z = angle  # Angle
    twist_pub.publish(obstacle_msg)


# Detect the closest obstacle's distance and angle
def scan_callback(scan_msg):
    while wait_to_start:
        pass

    points = scan_msg.ranges  # Read range data into an array
    # length = 360
    # index 0 = back of the bot, increases CCW on BOT <<< !!
    # index 0 = front of the bot, increases CW in SIM <<< !!

    ### Find the nearest object's index (angle) using range data (distance)
    closest_obj_dist = inf  # infinite number in Python
    closest_obj_angle = inf
    idx = 0  # index initialization for later
    for i in range(len(points)):  # Check entire array
        if i < conf.lidar_limit_left and i > conf.lidar_limit_right:  # Field of view limits. Note: Specific to hardware
            # Filter out unreliable out-of-range (distance) values
            if points[i] > scan_msg.range_min and points[i] < scan_msg.range_max:
                # Find the lowest value (closest distance) in the entire range
                if points[i] < closest_obj_dist:
                    # Write the lowest value so far so that next check has to be lower
                    closest_obj_dist = points[i]
                    idx = i  # Temp index stored for later (to find angle)

    ### After loop we either have inf or a value within range
    if closest_obj_dist == inf:  # No object in range
        obj_location(0, 0)  # Send distance = 0 for no object found
    else:  # Object found within range
        # Note: 1 degree per increment for this specific sensor
        ## Align sensor readout with bot's heading.
        closest_obj_angle = -(idx - 180)  # Translates entire range from 180 to 0 as front center.
        obj_location(closest_obj_dist, closest_obj_angle)  # meters, degrees


if __name__ == "__main__":
    rospy.init_node("detect_obstacle", anonymous=True)
    scantopic = rospy.get_param("~scan_topic")  # private name
    lidar_sub = rospy.Subscriber(scantopic, LaserScan, scan_callback, queue_size=1)
    # Lidar is slow and it adds latency with bigger queue
    # Better to drop old frames and get the latest data every time
    obstacletopic = rospy.get_param("~obstacle_topic_name")  # private name
    twist_pub = rospy.Publisher(obstacletopic, Twist, queue_size=1)
    obstacle_msg = Twist()
    # l.x = X-axis distance from sensor
    # l.y = Y-axis distance from sensor
    # l.z = Distance between sensor and obstacle
    # a.x = Path is clear 0, obstacle 1
    # a.y = Not used yet
    # a.z = Obstacles angle from front center { -z(left) -> 0(straight) -> z(right)}
    srv = Server(DetectObstacleConfig, dyn_rcfg_cb)

    rospy.spin()
