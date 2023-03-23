#!/usr/bin/env python3

# I have neither given nor received any unauthorizedaid in completing this work,
# nor have I presented someone else's work as my own.
# Your Name: Devson Butani
# LTU ID: 000732711
# Date: 09/21/2022

from cmath import pi
from re import X
from turtle import distance
import rospy
import math
from geometry_msgs.msg import Twist  # for /turtle1/cmd_vel
from geometry_msgs.msg import Point  # for user input
from turtlesim.msg import Pose  # for /turtle1/pose

tt_pose = Pose()  # Current Position of the robot

# Callback for topic: /turtle1/pose
def pose_cb(pose_msg):
    tt_pose.x = pose_msg.x  # absolute x
    tt_pose.y = pose_msg.y  # absolute y
    tt_pose.theta = pose_msg.theta  # heading


# Calculate distance delta between input points
def dist(tt_pose, p):
    x = p.x - tt_pose.x  # Delta x
    y = p.y - tt_pose.y  # Delta y
    distance = math.sqrt((x * x) + (y * y))  # Distance = sqrt of ( dX^2 + dY^2 )
    return distance


# Calculate YAW angle delta between input points
def ang(tt_pose, p):
    x = p.x - tt_pose.x  # Delta x
    y = p.y - tt_pose.y  # Delta y
    goal = math.atan2(y, x)  # Absolute angle between turtle and destination
    heading = tt_pose.theta  # Absolute angle of the turtle. East = 0
    delta = goal - heading  # Angle delta between current heading and goal
    # Check if delta is reflex to find the shorter opposite angle
    if delta > math.pi:  # More than 180 degree CCW, turn CW instead
        delta = delta - (2 * math.pi)
    elif delta < -math.pi:  # More than 180 degree CW, turn CCW instead
        delta = delta + (2 * math.pi)
    return delta


# Move to the destination using P controller
def move2xy(p, tolerance):  # p is goal point
    # Define publisher for Twist
    velocity_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    vel_msg = Twist()  # for writing output
    kv = 0.2  # Distance Kp default = 0.2
    kw = 8.0  # Angle Kp default = 8.0

    err_distance = dist(tt_pose, p)  # Initialize error
    while err_distance > tolerance:  # Control loop to reach destination
        vel_msg.linear.x = kv * err_distance  # P control delta distance
        vel_msg.angular.z = kw * ang(tt_pose, p)  # P control delta heading
        velocity_pub.publish(vel_msg)
        err_distance = dist(tt_pose, p)  # get new value to check after moving

    # Stop movement and print location
    vel_msg.linear.x = 0  # 0 stops the robot
    vel_msg.angular.z = 0
    velocity_pub.publish(vel_msg)
    print(f"--- Stopped at ({tt_pose.x:.2f}, {tt_pose.y:.2f})")


# Main
if __name__ == "__main__":
    rospy.init_node("move2xy_py", anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, pose_cb)
    while tt_pose.x == 0.0:  # Need some time to get pose data from the turtle
        pass  # do nothing

    p = Point()  # Destination point
    while True:  # get user input for destination point        
        # Loop until both inputs valid
        try:
            in_x = float(input("\nGoal x: "))
            if in_x < 0:
                raise Exception("Sorry, no numbers below zero. Try again...")
            in_y = float(input("Goal y: "))
            if in_y < 0:
                raise Exception("Sorry, no numbers below zero. Try again...")
            else: # Input valid
                # Set Destination value and move to destination
                p.x = in_x
                p.y = in_y
                move2xy(p, 0.3)  #  0.3 - tolerance
        except rospy.ROSInterruptException:
            print("ROS Exception")
            break
        except Exception as e:  # Print all exceptions
            print(f"{e}")
            print("Try again...")
