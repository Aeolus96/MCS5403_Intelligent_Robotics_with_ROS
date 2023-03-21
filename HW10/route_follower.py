#!/usr/bin/env python3

import rospy
import math
from move_base_msgs.msg import MoveBaseActionGoal

# http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages#Including_or_Importing_Messages
from route_publisher.msg import Route
from nav_msgs.msg import Odometry

is_executing = False

# see route_publisher json file!!
def route_callback(r):
    global is_executing, current_route, route_index
    print(f"Got new route! {r}")
    if len(r.commands) == 0:
        print("Invalid empty rout!")
        return

    is_executing = True
    current_route = r
    route_index = 0

    # publish initial goal, it is better to create a function, sine it is used in 2 functions
    pub_a_goal(current_route.commands[route_index].x, current_route.commands[route_index].y)  # <====== Complete


def pub_a_goal(goalx, goaly):
    msg = MoveBaseActionGoal()
    # the following is required
    msg.header.frame_id = "map"
    msg.goal.target_pose.header.frame_id = "map"
    msg.goal.target_pose.pose.orientation.w = 1.0

    # Set the position of the goal and publish it
    msg.goal.target_pose.pose.position.x = goalx
    msg.goal.target_pose.pose.position.y = goaly

    print(f"******** Published goal {msg}")
    goal_pub.publish(msg)


def distance(odom, goal):
    # Calculate the distance between the robot's current position (odom) and the goal
    # Use rosmsg info to determine which fields to use for distance calculation

    return math.sqrt(
        pow(odom.pose.pose.position.x - goal.x, 2) + pow(odom.pose.pose.position.y - goal.y, 2)
    )  # <====== complete


def odom_callback(odom):
    global is_executing, route_index
    if is_executing == False:
        return

    dist = distance(odom, current_route.commands[route_index])

    # YOUR CODE HERE <===============
    # Use the distance to determine if we have arrived at the current goal
    if dist < 1.0:
        # If we have, move to the next one and publish it (increment route_index_)
        print(f">>> Reached Index #{route_index}")
        route_index += 1
        # If we are done, set is_executing_ to false and end the callback
        if route_index is len(current_route.commands):
            print(f">>> End of Route at Index #{route_index}")
            is_executing = False
        # Only if we are not at done yet, publish the next index
        else:
            pub_a_goal(current_route.commands[route_index].x, current_route.commands[route_index].y)


if __name__ == "__main__":
    # rospy.init_node('route_follower', anonymous=True)
    rospy.init_node("route_follower")
    route_sub = rospy.Subscriber("/route", Route, route_callback, queue_size=1)
    odom_sub = rospy.Subscriber("/robot/odom", Odometry, odom_callback, queue_size=1)
    goal_pub = rospy.Publisher("/robot/move_base/goal", MoveBaseActionGoal, queue_size=1)
    rospy.spin()
