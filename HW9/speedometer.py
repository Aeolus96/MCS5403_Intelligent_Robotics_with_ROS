#!/usr/bin/env python3

# // I have neither given nor received any unauthorizedaid in completing this work,
# // nor have I presented someone else's work as my own.
# // Your Name: Devson Butani
# // LTU ID: 000732711
# // Date: 11/13/2022

import rospy
import math
from sensor_msgs.msg import NavSatFix


# Convert Degree to Radians
def degToRad(deg):
    rad = deg * (math.pi / 180)
    return rad


# Calculate Distance between input coordinates. Returns in meters
def haversine_dist(lat_1, lon_1, lat_2, lon_2):
    # Convert input to radians
    phi_1 = degToRad(lat_1)
    phi_2 = degToRad(lat_2)
    lam_1 = degToRad(lon_1)
    lam_2 = degToRad(lon_2)
    # Radius of Earth in meters
    radius_of_earth = 6371000
    # Break equation into parts and combine
    i = (math.sin((phi_2 - phi_1) / 2)) ** 2
    j = math.cos(phi_1) * math.cos(phi_2) * (math.sin((lam_2 - lam_1) / 2)) ** 2
    k = math.sqrt(i + j)
    distance = 2 * radius_of_earth * math.asin(k)
    # Display Calculated distance in meters
    # print(f"Distance between ({lat_1:.6f}, {lon_1:.6f}) and ({lat_2:.6f}, {lon_2:.6f}) = {d:.6f} meters")
    # Return value if needed elsewhere
    return distance


# Initialize global variables
firstGPS = NavSatFix()  # Set type to accept GPS message
currGPS = NavSatFix()  # Set type to accept GPS message
prevGPS = NavSatFix()  # Set type to accept GPS message
count = 0  # Message Counter. 0 since no message received right now

# GPS output gets sent here
def gps_callback(msg):
    global currGPS, prevGPS, firstGPS, count
    # check if this callback function is called first time or not
    if count == 0:  # Set permanent firstGPS message that never changes
        firstGPS = msg
        count = 1
        return  # No need to do anything else because only 1 msg received yet
    if count == 1:  # Second message, two values available so can calc speed
        prevGPS = firstGPS  # Store first message as previous
        currGPS = msg  # Store latest message as current
    if count > 1:  # All messages after the 2nd message
        prevGPS = currGPS  # Store old message as previous
        currGPS = msg  # Store latest message as current
        # Check if ROS bag is looping back to first value
        if currGPS.header.stamp <= firstGPS.header.stamp: # If <= then past hence reset
            # Always set newest firstGPS because it's either the 1st or earlier than previous 1st
            # Since node can start anytime, firstGPS could be the nth value (not actual bag's 1st)
            # Reset it once a new lower value is found. 
            # That way every loop, 1st is the most earliest (past) value from node's perspective
            firstGPS = msg
            count = 1  # next callback go to second message case
            return  # No need to do anything else because only 1 msg "exists" yet
    speedometer(count, currGPS, prevGPS)
    count += 1  # Add one to message counter


# Compute time and distance between two ros GPS messages and print
def speedometer(counter, current_GPS, previous_GPS):
    # Duration a.k.a delta(time)
    dt = current_GPS.header.stamp - previous_GPS.header.stamp
    dt_seconds = dt.to_sec()  # Convert time from ?milliseconds? to seconds
    if dt_seconds == 0:
        dt_seconds = 0.000001
    # Distance a.k.a delta(position)
    dx = haversine_dist(current_GPS.latitude, current_GPS.longitude, previous_GPS.latitude, previous_GPS.longitude)
    # Instantaneous Speed = dx / dt
    speed = dx / dt_seconds
    # print to display as a Speedometer
    print(f"Coordinate: {counter:2d},    Duration: {dt_seconds:5.3f} seconds,    Speed: {speed:6.3f} m/s")


if __name__ == "__main__":
    rospy.init_node("speedometer", anonymous=True)
    gps_sub = rospy.Subscriber("/fix", NavSatFix, gps_callback, queue_size=1)
    rospy.spin()
