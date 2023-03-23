#!/usr/bin/env python3

# // I have neither given nor received any unauthorizedaid in completing this work,
# // nor have I presented someone else's work as my own.
# // Your Name: Devson Butani
# // LTU ID: 000732711
# // Date: 11/8/2022

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sense_obstacles.cfg import SenseObstaclePyConfig  # packageName.cfg
from dynamic_reconfigure.server import Server
from math import pi
from math import inf


# Get dyn_rcfg values as they update
def dyn_rcfg_cb(config, level):
    global d_speed, d_min, Kp
    d_speed = config.drive_speed
    d_min = config.min_dist
    Kp = config.Kp
    return config


# Simplified motion publisher
def move(speed, steer):
    # x(speed) & z(steer) are axis velocities
    # { -x(backward) -> 0(stop) -> x(forward) } { -z(right) -> 0(straight) -> z(left)}
    # For two wheeled robot they compound together in the driver code
    # ie: x & z get converted into Left/Right motor power
    vel_msg.linear.x = speed
    vel_msg.angular.z = steer
    velocity_pub.publish(vel_msg)
    return


# Callback for Lidar message on "/scan". Used for turning towards the closest
# object and approaching it using proportional control.
def scan_callback(smsg):  # scan msg = smsg
    if d_speed < 0.0001:  # do not move if speed is (near) zero.
        move(0, 0)
        return  # no need to do anything else if not required to move

    points = smsg.ranges  # Read range data from /scan into an array
    # length = 360
    # index 0 = back of the bot, increases CCW on bot
    # index 0 = front of the bot, increases CW in SIM

    ### Find the nearest object's index (angle) using range data (distance)
    closest_obj_dist = inf  # infinite number in Python
    closest_obj_angle = inf
    idx = 0  # index initialization for later
    for i in range(len(points)):  # Check entire array
        if i < 265 and i > 95:  # Field of view limits. Note: Specific to hardware
            # Filter out unreliable out-of-range (distance) values
            if points[i] > smsg.range_min and points[i] < smsg.range_max:
                # Find the lowest value (closest distance) in the entire range
                if points[i] < closest_obj_dist:
                    # Write the lowest value so far so that next check has to be lower
                    closest_obj_dist = points[i]
                    idx = i  # Temp index stored for later (to find angle)

    ### After loop we either have inf or a value within range
    ### Drive toward the closest object and stop once the robot has arrived
    if closest_obj_dist == inf:  # No object in range, don't move (temp safety stop)
        # Could go looking for the object but not needed right now
        move(0, 0)  # Stop
    elif closest_obj_dist > d_min:  # Object found within range
        ## Calculate the angle for closest object found (radians)
        ## angle_min(starting point of range) + nth increment = angle of index
        ## different Lidars could have different angle_min and angle_increment
        closest_obj_angle = smsg.angle_min + (idx * smsg.angle_increment)

        ## Align sensor readout with bot's heading.
        # Translate angle from 0-2pi to 0-(pi,-pi)
        # ie: -pi(left_back) to 0(front) to pi(right_back). Let the sign work for you.
        # #!!!!!!!!!!!!!!  This is for Sim. Front is 360/0 CW !!!!!!!!!!!!!!!!!!!!!!!!!!
        # if closest_obj_angle > pi:
        #     closest_obj_angle -= 2 * pi  # Left half is made negative
        #!!!!!!!!!!!!!!!!!   This is for Bot. Front is 180 CCW !!!!!!!!!!!!!!!!!!!!!!!!!
        closest_obj_angle -= pi  # Translates entire range from 180 to 0.

        ## Proportional Controller for steering the bot towards the object
        bot_center = 0  # Set target angle(radians). Also absolute heading (0) for bot
        # Kp = 0.02  # Proportional constant ***from dyn_cnfg now***
        error = bot_center - closest_obj_angle  # Calculate error
        # Debug stream
        rospy.loginfo(
            "Idx = %2.3f   Angle = %2.3f   Distance = %2.3f   Error = %2.3f",
            idx,
            closest_obj_angle,
            closest_obj_dist,
            error,
        )
        ## Check error tolerance for aiming straight +/-(radians)
        if abs(error) > 0.05:  # Not in tolerance for aiming straight (5 degrees)
            # Calculate steer output
            steer = -Kp * error  # Kp -ve for steer direction flip
            # Set dummy steer for super low values because
            if abs(steer) < 0.15: # Bot doesn't move at all below this value
                if steer < 0:
                    steer = -0.15
                else:
                    steer = 0.15
            move(0, steer)  # Turn bot towards the object
        else:  ## Error within tolerance, no need to steer, simply go straight
            move(0,0) # Temp for testing aim
            #move(d_speed, 0)  # Use value from dyn_rcfg speed slider

    else:  # Object in range but close enough to stop
        move(0, 0)  # Stop


if __name__ == "__main__":
    rospy.init_node("sense_obstacles_py", anonymous=True)
    scantopic = rospy.get_param("~scan_topic")  # private name
    lidar_sub = rospy.Subscriber(scantopic, LaserScan, scan_callback, queue_size=1)
    # Lidar is slow and it adds latency with bigger queue
    # Better to drop old frames and get the latest data every time
    vel_msg = Twist()
    twisttopic = rospy.get_param("~twist_topic_name")  # private name
    velocity_pub = rospy.Publisher(twisttopic, Twist, queue_size=1)
    srv = Server(SenseObstaclePyConfig, dyn_rcfg_cb)
    rospy.spin()
