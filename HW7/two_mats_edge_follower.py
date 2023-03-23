#!/usr/bin/env python3

# // I have neither given nor received any unauthorizedaid in completing this work,
# // nor have I presented someone else's work as my own.
# // Your Name: Devson Butani
# // LTU ID: 000732711
# // Date: 10/31/2022

import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from two_mats_pkg.cfg import TwoMatsEdgeFollowConfig  # packageName.cfg
from dynamic_reconfigure.server import Server
import numpy as np

# Global variables
vel_msg = Twist()
bridge = CvBridge()

# Reconfigurable parameters from dyn_rcfg GUI
def dyn_rcfg_cb(config, level):
    global bw_thresh, mirror_img, edge_thresh, edge_direction, bot_speed
    global bot_kp, enable_drive

    bw_thresh = config.bw_thresh  # Black/White conversion threshold
    mirror_img = config.mirror_img  # Flip image around verticle center

    edge_thresh = config.edge_thresh  # Edge bias in image to follow
    edge_direction = config.edge_direction  # Follow Edge CW(T)/CCW(F)

    bot_speed = config.bot_speed  # Speed for Linear.x
    bot_kp = config.bot_kp  # Controller constant Kp for Angular.z

    enable_drive = config.enable_drive  # E-stop
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


# Get camera image
def image_cb(ros_image):
    # convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    # from now on, you can work exactly like with opencv

    hw7_controller(bgr_to_bw(cv_image))  # Feed the image to the controller


# Bot vs Sim Kp/speed values are different
# Bot Speed: 0.2 - 2.0      # Sim Speed: 0.2 - 2.0
# Bot Kp: 0.02 - 0.06       # Sim Kp: 0.2 - 0.6

# Uses Black/White binary image to control steering and speed
# No left/right needed as overall white % is used proportionally
# Simple and effective method for carpet case. Carpet much larger
# than camera view. Line following would need additional processing
def hw7_controller(bw_image):
    # Get overall image shape for scaled image
    (rows, cols) = bw_image.shape
    pixel_count = rows * cols
    # White pixel percentage overall
    white_pct = 100 * cv2.countNonZero(bw_image) / pixel_count
    # Slow down speed if too white or too black
    if white_pct > 70 or white_pct < 30:
        speed = bot_speed//2
    else:
        speed = bot_speed
    # E-Stop for motion
    if enable_drive == True:
        # Bias Proportional (bias = robot center to edge center)
        # Maintains constant distance from edge based on blk:wht ratio
        error = edge_thresh - white_pct
        steer = bot_kp * error
        if edge_direction == False:  # CCW
            steer = -steer
        move(speed, steer)  # use speed from dyn_rcfg
        # Display data on image
        telemetry(bw_to_bgr(bw_image), white_pct, speed, steer)
    else:
        move(0, 0)
        # rqt controller can override this
        # need a routing node later - might help with tele-op switching


# Display metrics as overlay on image
def telemetry(image, white_pct, speed, steer):
    # Get image shape data
    (rows, cols, channels) = image.shape
    # Insert White % for telemetry
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(
        image,
        f"Overall White Percent = {white_pct:2.3f}%",
        (10, rows - 10),
        font,
        1,
        (255, 0, 0),  # Blue
        2,
        cv2.LINE_AA,
    )
    # Insert swinging line to show speed and direction
    p1 = (int(cols / 2), int(rows))  # Center point for ellipse
    angle = -90 + int(-25 * steer)  # 0 east and -90 is north
    image = cv2.ellipse(image, p1, (rows, rows), angle, -1, 1, (0, 0, 255), -1)
    # (image, center_coordinates, axesLength, angle, startAngle, endAngle, color, thickness)

    # Display image for telemetry
    cv2.imshow("Image Feedback", image)
    # Move window to a specific spot, all new frames get moved
    cv2.moveWindow("Image Feedback", 0, 0)  # (img, row, col)
    cv2.waitKey(3)  # 3ms wait before it goes away


# Process input black and white image to color
def bw_to_bgr(bw_image):
    # Resize the smaller bw_image for display purposes
    image = cv2.resize(bw_image, None, fx=2, fy=2, interpolation=cv2.INTER_AREA)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image


# Process input image to black(0) and white(255) binary matrix
# Using reduced scale image, reduces calculations/frame and
# improves blur quality
def bgr_to_bw(bgr_image):

    # Scale down image to speed up calculations
    bgr_image = cv2.resize(bgr_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    # Mirror image using dynamic param check
    if mirror_img == True:
        bgr_image = cv2.flip(bgr_image, 1)

    # Blur image to decrease noise from small objects or debris
    blur_kernel = 13  # must be odd, 1, 3, 5, 7 ...
    bgr_image = cv2.medianBlur(bgr_image, blur_kernel)

    # Convert 3 channel BGR image to 1 channel Grayscale image
    gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

    # Convert 1 channel Grayscale image to Black and White only
    # Removes all channels from img.shape
    ret, bw_image = cv2.threshold(
        gray_image,  # input image from above
        bw_thresh,  # black/white threshold value from dyn_rcfg
        255,  # set max value in image
        cv2.THRESH_BINARY,  # use this binary type threshold
    )
    # function outputs converted black and white image
    return bw_image


if __name__ == "__main__":
    rospy.init_node("two_mats_edge_follow", anonymous=True)  # Start node

    # Inputs:
    imgtopic = rospy.get_param("~img_topic_name")  # private param name
    twisttopic = rospy.get_param("~twist_topic_name")  # private param name
    rospy.Subscriber(imgtopic, Image, image_cb)  # Subscribe to image from camera
    srv = Server(TwoMatsEdgeFollowConfig, dyn_rcfg_cb)  # Start dyn_rcfg server

    # Outputs:
    velocity_pub = rospy.Publisher(twisttopic, Twist, queue_size=1)
    # Publisher set to bot or sim-bot based on topic param from launch args

    # Run callbacks as they recieve values. Rate varies based on publish freq
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
