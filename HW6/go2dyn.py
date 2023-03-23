#!/usr/bin/env python3

# // I have neither given nor received any unauthorizedaid in completing this work,
# // nor have I presented someone else's work as my own.
# // Your Name: Devson Butani
# // LTU ID: 000732711
# // Date: 10/13/2022


import rospy
from dynamic_reconfigure.server import Server

# packageName.cfg , gen.generate <3rdArgAsPrefix>Config
from turtlesim_tst_pkg.cfg import ShuttleConfig
from geometry_msgs.msg import Twist


# Get topic to publish at (bot/sim)
topic_name = rospy.get_param("/out_topic_name")
# Set publisher destination and data type
velocity_publisher = rospy.Publisher(topic_name, Twist, queue_size=10)
vel_msg = Twist()

# Update Dynamic Variables from rqt GUI
def callback(config, level):
    # level is 3rd Arg in gen.add used in Ex: Enum
    # create globals to store dynamic param
    global speed, stop
    # get params
    speed = config.speed
    stop = config.stop

    # Stop the motion if stop is checked in GUI
    if stop == True:
        speed = 0.0

    return config  # this line is required for the server


# Loop to move the turtle/bot in an specified distance
# Timeline:
# Set speed > t1 > calc distance > t0=t1 > sleep
# t0 is set before sleep to account for looping and publish times in t1
# Does not account for calc distance processing time or boundary case at limits
# hence, less accurate over longer distances
def go2(input_distance):
    r = rospy.Rate(20)  # ROS Rate at 20Hz
    t0 = rospy.get_time()  # same as rospy.Time.now().to_sec()
    acc_dist = 0  # Initial pos = zero
    if input_distance < 0:  # If Going Backwards stop just after crossing limit
        while acc_dist > input_distance:
            vel_msg.linear.x = -speed  # Set new speed, negative for backwards motion
            velocity_publisher.publish(vel_msg)  # Publish to get moving at new speed
            t1 = rospy.get_time()  # Motion "pause"(for the purpose of calc) time
            acc_dist += -speed * (t1 - t0)  # Sum(speed * dt)
            t0 = t1  # Motion "resume"(for the purpose of calc) time
            r.sleep()  # Even out the processing time blocks under uniform Rate
    else:  # Else Going Forwards stop just after crossing limit
        while acc_dist < input_distance:
            vel_msg.linear.x = speed  # Set new speed
            velocity_publisher.publish(vel_msg)  # Publish to get moving at new speed
            t1 = rospy.get_time()  # Motion "pause"(for the purpose of calc) time
            acc_dist += speed * (t1 - t0)  # Sum(speed * dt)
            t0 = t1  # Motion "resume"(for the purpose of calc) time
            r.sleep()  # Even out the processing time blocks under uniform Rate
    vel_msg.linear.x = 0  # After the loop, stops the robot
    velocity_publisher.publish(vel_msg)


# Keep shuttling the input distance
def shuttle(distance):
    while not rospy.is_shutdown():
        go2(distance)
        go2(-distance)


if __name__ == "__main__":
    rospy.init_node("go2dyn", anonymous=False)
    srv = Server(ShuttleConfig, callback)

    try:
        shuttle(2.0)
    except rospy.ROSInterruptException:
        pass

    # Never ending loop, then spin, works in python for some reason
    rospy.spin()
