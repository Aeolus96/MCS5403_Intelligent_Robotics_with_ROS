#!/usr/bin/env python3

# # I have neither given nor received any unauthorizedaid in completing this work,
# # nor have I presented someone else's work as my own.
# # Your Name: Devson Butani
# # LTU ID: 000732711
# # Date: 12/13/2022

import rospy
from term_project_pkg.cfg import DetectCupConfig  # packageName.cfg
from dynamic_reconfigure.server import Server  # Dyn_rcfg-server
from sensor_msgs.msg import Image  # Sub
from geometry_msgs.msg import Twist  # Pub
import cv2  # OpenCV module
from cv_bridge import CvBridge, CvBridgeError  # Converter: ROSmsg > OpenCV
import numpy as np
import math

# Globals
bridge = CvBridge()  # Converter class instance: ROSmsg > OpenCV
global conf  # Entire dynamic config

# Get dyn_rcfg values as they update
def dyn_rcfg_cb(config, level):
    global conf
    conf = config  # Store entire config
    # because variables could be changed without updating the callback lines
    return config


# Recieves images from Camera topic
def image_callback(ros_image):
    global bridge
    try:  # convert ros_image into an opencv-compatible image
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    # from now on, you can work exactly like with opencv
    # Scale down image for processing
    cv_image = cv2.resize(cv_image, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    blob_detect(cv_image)
    cv2.waitKey(3)


# Detect white blobs, average between centroids and return heading
def blob_detect(cv_image):
    # Blur image to decrease noise from small objects or debris
    blur_kernel = 3  # must be odd, 1, 3, 5, 7 ...
    cv_image = cv2.medianBlur(cv_image, blur_kernel)

    (rows, cols, channels) = cv_image.shape  # Get size of image
    img_area = rows * cols  # Area in square pixels
    minimum = (conf.min_cup_size / 100) * img_area  # Cup size in percentage converted to
    maximum = (conf.max_cup_size / 100) * img_area  # squere pixels relative to image size

    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # Convert BGR to HSV

    # lower, upper, to get white, 200 < V < 255
    thresh = cv2.inRange(hsv_img, np.array((0, 0, 225)), np.array((179, 255, 255)))

    # find contours in the threshold image
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Find centroid for each blob and print
    # https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
    closest_dist = math.inf  # infinite number in Python
    closest_x = 0
    closest_y = 0
    for c in contours:
        M = cv2.moments(c)
        if M["m00"] != 0:
            area = cv2.contourArea(c)
            if minimum < area < maximum:
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                frame = cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                translated_x = cx - (cols / 2)
                translated_y = rows - cy
                dist_from_bot = math.hypot(translated_x, translated_y)
                # print(f"x = {cx}   y = {cy}   a = {area}   d = {dist_from_bot}")
                if dist_from_bot < closest_dist and translated_y > (rows * 0.35):  # &Trim bottom of image
                    # Write the lowest value so far so that next check has to be lower
                    closest_dist = dist_from_bot
                    closest_x = translated_x
                    closest_y = translated_y

    # Debug
    # print(f"--------min = {minimum}---max = {maximum}---dist = {closest_dist}---------")
    # cv2.imshow("With a centroid dot", frame)
    # cv2.imshow("Thresh", thresh)

    ## Boundaries to default values to
    if closest_dist == math.inf:  # When no cup is found distance is 0
        closest_dist = 0

    angle = math.degrees(math.atan2(closest_y, closest_x))

    ### Publish the closest cup's location
    blob_msg.linear.x = int(closest_x)  # X-axis distance from sensor
    blob_msg.linear.y = int(closest_y)  # Y-axis distance from sensor
    blob_msg.linear.z = int(closest_dist)  # Distance between sensor and blob
    # blob_msg.angular.x = 0  # Path is clear 0, blob 1
    # blob_msg.angular.y = 0  # Not used yet
    blob_msg.angular.z = int(angle)  # blobs angle from front center
    blob_pub.publish(blob_msg)
    return


# Main call
if __name__ == "__main__":
    # Start node
    rospy.init_node("detect_cup", anonymous=True)

    # Get topics at launch
    imgtopic = rospy.get_param("~img_topic_name")  # Camera input topic
    blobTopic = rospy.get_param("~blob_topic_name")  # Blob Detect output topic

    # Set Subscribers/Publishers - queue_size=1 because multiple buffers make node slow
    image_sub = rospy.Subscriber(imgtopic, Image, image_callback, queue_size=1)
    blob_pub = rospy.Publisher(blobTopic, Twist, queue_size=1)
    blob_msg = Twist()

    # Dynamic Reconfigure parameter server
    srv = Server(DetectCupConfig, dyn_rcfg_cb)  # Using common cfg file for entire project

    # Spin the node for callbacks
    rospy.spin()
