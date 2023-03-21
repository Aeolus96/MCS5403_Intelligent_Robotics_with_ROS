#!/usr/bin/env python3

# # I have neither given nor received any unauthorizedaid in completing this work,
# # nor have I presented someone else's work as my own.
# # Your Name: Devson Butani
# # LTU ID: 000732711
# # Date: 12/13/2022

import rospy
from term_project_pkg.cfg import InsideMatConfig  # packageName.cfg
from dynamic_reconfigure.server import Server  # Dyn_rcfg-server
from sensor_msgs.msg import Image  # Sub
from geometry_msgs.msg import Twist  # Pub
import cv2  # OpenCV module
from cv_bridge import CvBridge, CvBridgeError  # Converter: ROSmsg > OpenCV
import numpy as np
import math
import time

# Globals
bridge = CvBridge()  # Converter class instance: ROSmsg > OpenCV
global conf, wait_to_start  # Entire dynamic config
wait_to_start = True
global rows, cols, boundary, cv_image  # For image cropping
rows = 0
cols = 0

# Get dyn_rcfg values as they update
def dyn_rcfg_cb(config, level):
    global conf, wait_to_start
    conf = config  # Store entire config
    # because variables could be changed without updating the callback lines
    wait_to_start = False
    return config


# Recieves images from Camera topic
def image_callback(ros_image):
    global bridge, cv_image, wait_to_start, boundary

    while wait_to_start:  # Wait for dyn_rcfg to load
        pass

    try:  # convert ros_image into an opencv-compatible image
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    # from now on, you can work exactly like with opencv

    ### Resize for faster calculations
    cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    ### Crop using constant crop percents and variables from dyn_rcfg
    (rows, cols, channels) = cv_image.shape
    width = int(conf.bot_crop * cols)
    bot_left = int(cols / 2 - width / 2)
    bot_right = int(cols / 2 + width / 2) - 14  # + Offset for camera mounting alignment
    bot_front = int(rows * conf.front_crop)  # adjusted area considered as front of bot
    sides_bottom = int(rows * conf.bottom_crop)
    align_row = int(sides_bottom - (rows * conf.wheel_align_crop))  # how much robot goes out when dumping cups

    ## Image cutout boundaries used as individual sensors inside dictionary for faster calls from callbacks
    boundary = {  # "Crop_Section": [(from_row), (upto_row), (from_col), (upto_col)]
        "left": [(0), (sides_bottom), (0), (bot_left)],  # from left edge of image to bot
        "front": [(0), (bot_front), (bot_left), (bot_right)],  # from bot to right edge of image
        "right": [(0), (sides_bottom), (bot_right), (cols)],  # from top of image to inside bot's plow
        "left_wheel": [
            (align_row),
            (sides_bottom),
            (0),
            (bot_left),
        ],  # for precise aligning at edge
        "bot": [(bot_front), (rows), (bot_left), (bot_right)],  # for cropping out bot
        "right_wheel": [
            (align_row),
            (sides_bottom),
            (bot_right),
            (cols),
        ],  # for precise aligning at edge
    }

    ### Calculate color % in cutouts
    ## Create Masks for Blue and White
    blue_image = blue_mask(cv_image)  # Convert input to Blue mask
    # cv2.imshow("blue", blue_image)  # Display useful image
    white_image = bgr_to_bw(cv_image)  # Convert input to White mask
    # cv2.imshow("white", white_image)  # Display useful image

    ## Crop masks into smaller sections
    left_img = crop_section(blue_image, "left")
    right_img = crop_section(blue_image, "right")
    front_img = crop_section(blue_image, "front")
    left_wheel_img = crop_section(blue_image, "left_wheel")
    right_wheel_img = crop_section(blue_image, "right_wheel")
    bot_img = crop_section(white_image, "bot")  # For white

    ## Blue % after masking. ie: it will increase only when blue is detected
    left_blue_pct = blue_percent(left_img)
    front_blue_pct = blue_percent(front_img)
    right_blue_pct = blue_percent(right_img)
    left_wheel_blue_pct = blue_percent(left_wheel_img)
    right_wheel_blue_pct = blue_percent(right_wheel_img)

    ## White % after masking. ie: it detects how white the surroundings are
    inside_mat_white_pct = white_percent(white_image)

    ### Publish the % inside twist type
    edge_msg.linear.x = left_blue_pct
    edge_msg.linear.y = front_blue_pct
    edge_msg.linear.z = right_blue_pct
    edge_msg.angular.x = left_wheel_blue_pct
    edge_msg.angular.y = inside_mat_white_pct
    edge_msg.angular.z = right_wheel_blue_pct
    edge_pub.publish(edge_msg)

    ### If enabled display debug information (Can be setup with a slower thread later)
    if conf.edge_detect_debug:
        ## Merge images for boosted colors that are in range
        blue_white_img = cv2.bitwise_or(white_image, blue_image)  # Merge masks into one
        # cv2.imshow("blue + white", blue_white_img)  # Display useful image
        debug_img = cv2.bitwise_and(cv_image, cv_image, mask=blue_white_img)  # Mask input images

        ## Insert Color % in image for display
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 1
        pct_width = 70
        line1 = 40
        line2 = rows - pct_width
        line3 = cols - pct_width
        cv2.putText(
            debug_img,
            f"{left_blue_pct:3.0f}%",
            (0, line1),  # Start point
            font,
            1,
            (255, 0, 0),  # Color
            thickness,  # Line thickness
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_img,
            f"{right_blue_pct:3.0f}%",
            (line3, line1),  # Start point
            font,
            1,
            (255, 0, 0),  # Color
            thickness,  # Line thickness
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_img,
            f"{front_blue_pct:3.0f}%",
            (int((cols / 2) - pct_width), line1),  # Start point
            font,
            1,
            (255, 0, 0),  # Color
            thickness,  # Line thickness
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_img,
            f"{left_wheel_blue_pct:3.0f}%",
            (0, line2),  # Start point
            font,
            1,
            (255, 0, 0),  # Color
            thickness,  # Line thickness
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_img,
            f"{right_wheel_blue_pct:3.0f}%",
            (line3, line2),  # Start point
            font,
            1,
            (255, 0, 0),  # Color
            thickness,  # Line thickness
            cv2.LINE_AA,
        )
        cv2.putText(
            debug_img,
            f"{inside_mat_white_pct:3.0f}%",
            (int((cols / 2) - pct_width), line2),  # Start point
            font,
            1,
            (0, 0, 255),  # Color
            thickness,  # Line thickness
            cv2.LINE_AA,
        )

        ## Insert dividing lines for crop sections
        cv2.line(  # Bot left
            debug_img,
            (bot_left, 0),  # Start point
            (bot_left, rows),  # End point
            (0, 0, 255),  # Color
            thickness,  # Thickness
        )
        cv2.line(  # Bot right
            debug_img,
            (bot_right, 0),  # Start point
            (bot_right, bot_right),  # End point
            (0, 0, 255),  # Color
            thickness,  # Thickness
        )
        cv2.line(  # Bot front
            debug_img,
            (bot_left, bot_front),  # Start point
            (bot_right, bot_front),  # End point
            (0, 0, 255),  # Color
            thickness,  # Thickness
        )
        cv2.line(  # Left bottom
            debug_img,
            (0, sides_bottom),  # Start point
            (bot_left, sides_bottom),  # End point
            (0, 0, 255),  # Color
            thickness,  # Thickness
        )
        cv2.line(  # Right bottom
            debug_img,
            (bot_right, sides_bottom),  # Start point
            (cols, sides_bottom),  # End point
            (0, 0, 255),  # Color
            thickness,  # Thickness
        )
        cv2.line(  # Left Align
            debug_img,
            (0, align_row),  # Start point
            (bot_left, align_row),  # End point
            (0, 0, 255),  # Color
            thickness,  # Thickness
        )
        cv2.line(  # Right Align
            debug_img,
            (bot_right, align_row),  # Start point
            (cols, align_row),  # End point
            (0, 0, 255),  # Color
            thickness,  # Thickness
        )

        ### Resize for bigger display
        debug_img = cv2.resize(debug_img, None, fx=1.75, fy=1.75, interpolation=cv2.INTER_AREA)

        ## Display the final image
        cv2.imshow("edge_detect_view", debug_img)  # Display useful image
    cv2.waitKey(1)


# Crop helper, uses globals to simplify code reading
def crop_section(image, section_name):
    global boundary
    return image[
        boundary.get(section_name)[0] : boundary.get(section_name)[1],
        boundary.get(section_name)[2] : boundary.get(section_name)[3],
    ]


# Finds black percent from input image and returns percentage value
def black_percent(bw_image):
    # bw_image = bgr_to_bw(cv_image)  # Convert input to Black and White
    (rows, cols) = bw_image.shape
    pixel_count = rows * cols  # Find total # pixels
    return int(100 * (1 - cv2.countNonZero(bw_image) / pixel_count))  # Black % --> (1 - white = black)


# Finds white percent from input image and returns percentage value
def white_percent(bw_image):
    # bw_image = bgr_to_bw(cv_image)  # Convert input to Black and White
    (rows, cols) = bw_image.shape
    pixel_count = rows * cols  # Find total # pixels
    return int(100 * (cv2.countNonZero(bw_image) / pixel_count))  # white %


# Process input image to black(0) and white(255) binary matrix
def bgr_to_bw(bgr_image):
    # Blur image to decrease noise from small objects or debris
    blur_kernel = 15  # must be odd, 1, 3, 5, 7 ...
    bgr_image = cv2.medianBlur(bgr_image, blur_kernel)

    # Convert 3 channel BGR image to 1 channel Grayscale image
    gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

    # Convert 1 channel Grayscale image to Black and White only
    # Removes all channels from img.shape
    ret, bw_image = cv2.threshold(
        gray_image,  # input image from above
        conf.bw_threshold,  # black/white threshold value from dyn_rcfg
        255,  # set max value in image
        cv2.THRESH_BINARY,  # use this binary type threshold
    )
    # function outputs converted black and white image
    return bw_image


# Finds blue percent from input image and returns percentage value
def blue_percent(bw_image):
    # bw_image = blue_mask(cv_image)  # Convert input to Blue mask
    (rows, cols) = bw_image.shape
    pixel_count = rows * cols  # Find total # pixels
    return int(100 * (cv2.countNonZero(bw_image) / pixel_count))  # Blue % in input image


# Returns a mask with only Blue color
def blue_mask(cv_image):
    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  # Convert BGR to HSV
    # Make and merge color specific masks with origninal image
    blue_lower = (90, 210, 170)  # Lower bounds of H, S, V for Blue (90, 150, 100)
    blue_upper = (150, 255, 255)  # Upper bounds of H, S, V for Blue (150, 255, 255)
    edge_mask = cv2.inRange(hsv_img, blue_lower, blue_upper)  # Threshold range - white/black

    # Glow mask for bigger % coverage around blue objects
    glow_strength = 1  # 0: no glow, no maximum
    glow_radius = 35  # blur radius, must be odd, 1, 3, 5, 7 ...
    img_blurred = cv2.GaussianBlur(edge_mask, (glow_radius, glow_radius), 1)
    img_blended = cv2.addWeighted(edge_mask, 1, img_blurred, glow_strength, 0)

    return img_blended  # Return the masked black and white image (white is color in range)


# Main call
if __name__ == "__main__":
    # Start node
    rospy.init_node("inside_mat", anonymous=True)

    # Get topics at launch
    imgTopic = rospy.get_param("~img_topic_name")  # Camera input topic
    edgeTopic = rospy.get_param("~edge_topic_name")  # Edge Detect output topic

    # Set Subscribers/Publishers - queue_size=1 because multiple buffers make node slow
    image_sub = rospy.Subscriber(imgTopic, Image, image_callback, queue_size=1)
    edge_pub = rospy.Publisher(edgeTopic, Twist, queue_size=1)
    edge_msg = Twist()  # Not twist values but same data type:
    # edge_msg.linear.x = left_blue_pct
    # edge_msg.linear.y = front_blue_pct
    # edge_msg.linear.z = right_blue_pct
    # edge_msg.angular.x = left_wheel_blue_pct
    # edge_msg.angular.y = inside_mat_white_pct
    # edge_msg.angular.z = right_wheel_blue_pct

    # Dynamic Reconfigure parameter server
    srv = Server(InsideMatConfig, dyn_rcfg_cb)  # Using common cfg file for entire project

    # Spin the node for callbacks
    rospy.spin()
