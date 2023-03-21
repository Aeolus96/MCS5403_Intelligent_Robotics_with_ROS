#!/usr/bin/env python3

# # I have neither given nor received any unauthorizedaid in completing this work,
# # nor have I presented someone else's work as my own.
# # Your Name: Devson Butani
# # LTU ID: 000732711
# # Date: 12/13/2022

import rospy
from term_project_pkg.cfg import SnowBlowerConfig  # packageName.cfg
from dynamic_reconfigure.server import Server  # Dyn_rcfg-server
from sensor_msgs.msg import Joy  # Sub
from sensor_msgs.msg import Image  # Sub
import cv2  # OpenCV module
from cv_bridge import CvBridge, CvBridgeError  # Converter: ROSmsg > OpenCV
from sensor_msgs.msg import JoyFeedbackArray  # Pub
from sensor_msgs.msg import JoyFeedback  # Pub
from geometry_msgs.msg import Twist  # Pub
from math import pi
from math import inf
import numpy as np
import math
import threading
import time

### Globals and initializations - shared variables between threads and functions
bridge = CvBridge()  # Converter class instance: ROSmsg > OpenCV
# Dynamic config Variables
global conf
global wait_to_start
wait_to_start = True

# Obstacle Avoidance Variables
global obj_distance
obj_distance = 0
global obj_angle
obj_angle = 0
global path_clear
path_clear = True

# Cup Detection Variables
global blob_steer
blob_steer = 0
global blob_speed
blob_speed = 0
global dump_sequence
dump_sequence = False
global blob_dist
blob_dist = 0
global blob_angle
blob_angle = 0

# Edge Detection Variables
global left_align
left_align = False
global right_align
right_align = False
global left_edge
left_edge = False
global right_edge
right_edge = False
global front_edge
front_edge = False
global plow_full
plow_full = False
global inside_mat
inside_mat = False

# Joystick input Variables
global joy_buffer
joy_buffer = Joy()
global leftStick_x, leftStick_y, rightStick_x, rightStick_y, leftTrigger, rightTrigger
leftTrigger = 0.0
rightTrigger = 0.0
global dPad_left, dPad_right, dPad_up, dPad_down
global leftStick_btn, rightStick_btn, leftBumper, rightBumper
leftBumper = 0
rightBumper = 0
global btnA, btnB, btnX, btnY, btnBack, btnStart
global joy_override
joy_override = False
global tele_op_toggled
tele_op_toggled = False

# Debug options
global debug_display
debug_display = False

# Image inputs
global cv_image_front, cv_image_down

# Bot speeds
global min_speed
min_speed = 0.4


### Get dyn_rcfg values as they update
def dyn_rcfg_cb(config, level):
    global conf, wait_to_start
    conf = config  # Store entire config because variables can added later
    wait_to_start = False  # Wait for first config to load before starting state machine thread

    ## Call when value changes regardless of state
    if conf.e_stop:  # Emergency STOP
        move(0, 0)

    return config


### Clamp values between min and max
def clamp(minimum, x, maximum):
    return max(minimum, min(x, maximum))


### Simplified motion publisher with built-in software E-STOP and Joystick Override
def move(speed, steer):
    # x(speed) & z(steer) are axis velocities
    # { -x(backward) -> 0(stop) -> x(forward) } { -z(right) -> 0(straight) -> z(left)}
    # For two wheeled robot they compound together in the driver code
    # ie: x & z get converted into Left/Right motor power

    ## Joystick Override Autonomous ( 7(LT) && 8(RT) ):
    if joy_override or tele_op_toggled:
        if tele_op_toggled and not path_clear:  # Prevent front collision in tele-op mode
            speed = 0
        else:  # Allow collision while in override
            speed = leftStick_y * 3  # (-1 to 1) * 3(speed limit)
        steer = -rightStick_x * 0.6  # (1 to -1)

    ## Speed limits for EduBot:
    if speed < 0:
        speed = -1 * clamp(min_speed, abs(speed), 3)
    elif speed > 0:
        speed = clamp(min_speed, speed, 3)
    if steer < 0:
        steer = -1 * clamp(min_speed, abs(steer), 1)
    elif steer > 0:
        steer = clamp(min_speed, steer, 1)

    ## 4 ways to STOP MOTION of the robot (ordered by effectiveness):
    #   1) Power Button on robot (back right corner)            - Hardware stop
    #   2) Red button on Prizm Controller on robot              - Hardware stop
    #   3) Dynamic Reconfig GUI > snow_blower > E-Stop toggle   - Software stop
    #   4) Joystick buttons 5(LMB) OR 6(RMB)                    - Software stop
    if conf.e_stop or leftBumper == 1 or rightBumper == 1:  # << 3) & 4)
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
    else:
        vel_msg.linear.x = speed
        vel_msg.angular.z = steer
    velocity_pub.publish(vel_msg)
    return


### Send Force Feedback to Joystick. motor > [0, 1] intensity > [0.0 - 1.0]
def force_feedback(motor, intensity):
    global ff_msg
    rumble = JoyFeedback()
    rumble.type = JoyFeedback.TYPE_RUMBLE
    rumble.id = motor
    rumble.intensity = intensity
    ff_msg.array = [rumble]
    ff_pub.publish(ff_msg)
    return


### Joystick Map for ROS sensor_msgs/Joy
def joystick_profile_RumblePad_2():  # Logitech Cordless RumblePad 2
    global leftStick_x, leftStick_y, rightStick_x, rightStick_y, leftTrigger, rightTrigger
    global dPad_left, dPad_right, dPad_up, dPad_down
    global leftStick_btn, rightStick_btn, leftBumper, rightBumper
    global btnA, btnB, btnX, btnY, btnBack, btnStart

    ## Axes Inputs:
    leftStick_x = -1.0 * joy_buffer.axes[0]  # Inverts the X-axis from (1, -1) to (-1, 1)
    leftStick_y = joy_buffer.axes[1]
    rightStick_x = -1.0 * joy_buffer.axes[2]  # Inverts the X-axis from (1, -1) to (-1, 1)
    rightStick_y = joy_buffer.axes[3]
    # D-Pad is a directional button pad considered as axes
    if joy_buffer.axes[4] == 1.0:  # D-Pad left|right axis to button converter
        dPad_left = True
        dPad_right = False
    elif joy_buffer.axes[4] == -1.0:
        dPad_left = False
        dPad_right = True
    else:
        dPad_left = False
        dPad_right = False
    if joy_buffer.axes[5] == 1.0:  # D-Pad up|down axis to button converter
        dPad_up = True
        dPad_down = False
    elif joy_buffer.axes[5] == -1.0:
        dPad_up = False
        dPad_down = True
    else:
        dPad_up = False
        dPad_down = False
    ## Button Inputs:
    btnA = joy_buffer.buttons[1]  # A(2)
    btnB = joy_buffer.buttons[2]  # B(3)
    btnX = joy_buffer.buttons[0]  # X(1)
    btnY = joy_buffer.buttons[3]  # Y(4)
    leftBumper = joy_buffer.buttons[4]
    rightBumper = joy_buffer.buttons[5]
    leftTrigger = joy_buffer.buttons[6]  # 0/1 Buttons on this Joystick
    rightTrigger = joy_buffer.buttons[7]  # 0/1 Buttons on this Joystick
    btnBack = joy_buffer.buttons[8]  # Left middle
    btnStart = joy_buffer.buttons[9]  # Right middle
    leftStick_btn = joy_buffer.buttons[10]
    rightStick_btn = joy_buffer.buttons[11]
    # If needed features like deadzone and debounce can be added here:


### Joystick Map for ROS sensor_msgs/Joy
def joystick_profile_f710_x():  # Logitech Wireless Gamepad F710: mode x
    global leftStick_x, leftStick_y, rightStick_x, rightStick_y, leftTrigger, rightTrigger
    global dPad_left, dPad_right, dPad_up, dPad_down
    global leftStick_btn, rightStick_btn, leftBumper, rightBumper
    global btnA, btnB, btnX, btnY, btnBack, btnStart

    ## Axes Inputs:
    leftStick_x = -1.0 * joy_buffer.axes[0]  # Inverts the X-axis from (1, -1) to (-1, 1)
    leftStick_y = joy_buffer.axes[1]
    rightStick_x = -1.0 * joy_buffer.axes[3]  # Inverts the X-axis from (1, -1) to (-1, 1)
    rightStick_y = joy_buffer.axes[4]
    # Invert -> Translate -> Scale
    leftTrigger = ((-1.0 * joy_buffer.axes[2]) + 1.0) / 2.0  # Changed from (1, -1) where 1 is normal to (0, 1)
    rightTrigger = ((-1.0 * joy_buffer.axes[5]) + 1.0) / 2.0  # Changed from (1, -1) where 1 is normal to (0, 1)

    # D-Pad is a directional button pad considered as axes
    if joy_buffer.axes[6] == 1.0:  # D-Pad left|right axis to button converter
        dPad_left = True
        dPad_right = False
    elif joy_buffer.axes[6] == -1.0:
        dPad_left = False
        dPad_right = True
    else:
        dPad_left = False
        dPad_right = False
    if joy_buffer.axes[7] == 1.0:  # D-Pad up|down axis to button converter
        dPad_up = True
        dPad_down = False
    elif joy_buffer.axes[7] == -1.0:
        dPad_up = False
        dPad_down = True
    else:
        dPad_up = False
        dPad_down = False
    ## Button Inputs:
    btnA = joy_buffer.buttons[0]  # A(2)
    btnB = joy_buffer.buttons[1]  # B(3)
    btnX = joy_buffer.buttons[2]  # X(1)
    btnY = joy_buffer.buttons[3]  # Y(4)
    leftBumper = joy_buffer.buttons[4]
    rightBumper = joy_buffer.buttons[5]
    btnBack = joy_buffer.buttons[6]  # Left middle
    btnStart = joy_buffer.buttons[7]  # Right middle
    leftStick_btn = joy_buffer.buttons[9]
    rightStick_btn = joy_buffer.buttons[10]
    # If needed features like deadzone and debounce can be added here:


### Retrieve Joystick input and apply profile
def joy_callback(joy_msg):
    global joy_buffer, debug_display, joy_override, tele_op_toggled
    joy_buffer = joy_msg
    joystick_profile_f710_x()  # Load Profile with input message
    if leftBumper == 1 or rightBumper == 1:  # E-stop <<< specific to robot config only
        move(0, 0)
    debug_display = btnA == 1 or conf.debug_overlay  # Button A(2) is used for debug display while pressed
    if btnStart == 1:  # Toggle Tele-Op mode
        tele_op_toggled = not tele_op_toggled
        time.sleep(1)
    joy_override = leftTrigger >= 0.25 and rightTrigger >= 0.25  # Joystick Override Autonomous (LT + RT)


### Recieves images from Camera topic
def front_image_callback(ros_image):
    global bridge, cv_image_front, wait_to_start, debug_display
    try:  # convert ros_image into an opencv-compatible image
        cv_image_front = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    # from now on, you can work exactly like with opencv

    while wait_to_start:  # wait for dyn_rcfg to start up
        pass

    # ### Resize for faster calculations
    # cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    ## Get image specifications
    (rows_f, cols_f, channels_f) = cv_image_front.shape
    # front = cv_image_front.copy()
    # Make a blank image of same size for overlay
    front = np.zeros((rows_f, cols_f, channels_f), np.uint8)

    font = cv2.FONT_HERSHEY_SIMPLEX

    ## Check Tele-Op status
    if joy_override:
        color = (0, 0, 255)  # RED
        text = "Joystick Override"
    elif tele_op_toggled:  # Joystick control active
        color = (255, 0, 0)  # BLUE
        text = "Tele-Op Active"
    else:  # Autonomous control acitve
        color = (0, 255, 0)  # GREEN
        text = "Autonomous Active"
    # Control mode status indicator. Red = Joystick, Green = Auto
    cv2.circle(
        front,  # img
        (40, 40),  # center(x,y)
        15,  # radius(pixels)
        color,  # color(BGR)
        8,  # thickness
    )
    if debug_display:  # Check if Debug enabled - Additional display elements
        # Display text guidelines for circles above
        # Left side Tele-Op / Auto text
        cv2.putText(
            front,
            text,
            (65, 45),  # Start point
            font,
            0.6,
            color,  # Color
            1,  # Line thickness
            cv2.LINE_AA,
        )

    ## Check Obstacle status
    if 0 < obj_distance < 0.5:  # Within visible range, issue warning
        # YELLOW circle on the right to indicate warning
        cv2.circle(
            front,  # img
            (int(cols_f - 40), 40),  # center(x,y)
            15,  # radius(pixels)
            (0, 150, 255),  # color(BGR)
            8,  # thickness
        )
        if debug_display:  # Check if Debug enabled - Additional display elements
            # Display text guidelines for circles above
            # Left side Tele-Op / Auto text
            cv2.putText(
                front,
                "Obstacle",
                ((cols_f - 145), 45),  # Start point
                font,
                0.6,
                (0, 150, 255),  # Color
                1,  # Line thickness
                cv2.LINE_AA,
            )
        # Insert swinging line to show object's direction when in range
        p1 = (int(cols_f / 2), int(rows_f))  # Center point for ellipse
        angle = -90 + int(obj_angle)  # 0 east and -90 is north
        cv2.ellipse(front, p1, (int(rows_f / 2), int(rows_f / 2)), angle, -2, 2, (0, 150, 255), -1)
        # (image, center_coordinates, axesLength, angle, startAngle, endAngle, color, thickness)

    ## Check inside mat status
    if not inside_mat:
        # RED circle on the right to indicate warning
        cv2.circle(
            front,  # img
            (int(cols_f / 2), int(rows_f / 2)),  # center(x,y)
            100,  # radius(pixels)
            (0, 0, 255),  # color(BGR)
            15,  # thickness
        )
        if debug_display:  # Check if Debug enabled - Additional display elements
            # Display text guidelines for circles above
            cv2.putText(
                front,
                "OUT OF LIMITS",
                (int((cols_f / 2) - 70), int((rows_f / 2) + 10)),  # Start point
                font,
                0.65,
                (0, 0, 255),  # Color
                2,  # Line thickness
                cv2.LINE_AA,
            )

    ## Check Cup Detection status
    if 30 < blob_dist < 200:  # unit in pixels
        # Insert swinging line to show object's direction when in range
        p1 = (int(cols_f / 2), int(rows_f))  # Center point for ellipse
        angle = int(-blob_angle)  # 0 east and -90 is north (input angle 90 is center)
        cv2.ellipse(front, p1, (int(rows_f / 2), int(rows_f / 2)), angle, -1, 1, (0, 200, 0), -1)
        # (image, center_coordinates, axesLength, angle, startAngle, endAngle, color, thickness)

    ## Blend two images - original + overlay
    alpha = 1  # Strength for A
    beta = 5  # Strength for B
    blended_img = cv2.addWeighted(cv_image_front, alpha, front, beta, 0.0)

    # ### Resize for faster calculations
    # blended_img = cv2.resize(blended_img, None, fx=1.75, fy=1.75, interpolation=cv2.INTER_AREA)

    cv2.imshow("front_view", blended_img)  # Display useful image
    # print("GUI on")
    cv2.waitKey(1)


### Recieves images from Camera topic - NOT USED
def down_image_callback(ros_image):
    global bridge, cv_image_down
    try:  # convert ros_image into an opencv-compatible image
        cv_image_down = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    # from now on, you can work exactly like with opencv


### Retrieve near Edge and plow capacity values
def edge_callback(edge_msg):
    global left_align, right_align, left_edge, right_edge, front_edge, plow_full, inside_mat
    ## Read msg
    left_blue_pct = edge_msg.linear.x
    front_blue_pct = edge_msg.linear.y
    right_blue_pct = edge_msg.linear.z
    left_wheel_blue_pct = edge_msg.angular.x
    inside_mat_white_pct = edge_msg.angular.y
    right_wheel_blue_pct = edge_msg.angular.z

    ## Set global checks
    if left_blue_pct >= conf.edge_detect_blue_percent:
        left_edge = True
    else:
        left_edge = False
    if right_blue_pct >= conf.edge_detect_blue_percent:
        right_edge = True
    else:
        right_edge = False
    if front_blue_pct >= conf.edge_detect_blue_percent:
        front_edge = True
    else:
        front_edge = False
    if left_wheel_blue_pct >= (conf.edge_detect_blue_percent * 1.5):
        left_align = True
    else:
        left_align = False
    if right_wheel_blue_pct >= (conf.edge_detect_blue_percent * 1.5):
        right_align = True
    else:
        right_align = False
    if inside_mat_white_pct > conf.mat_detect_white_percent:  # Robot out of mat
        inside_mat = False
        force_feedback(1, 0.5)
    else:
        inside_mat = True
        force_feedback(1, 0)


### Retrieve Obstacle's distance and angle from bot
def obstacle_callback(obj_location_msg):
    # Type: Twist
    # See detect_obstacle.py for more info
    global obj_distance, obj_angle, path_clear
    # Raw values
    obj_distance = obj_location_msg.linear.z
    obj_angle = obj_location_msg.angular.z

    # Within collision range AND in robot's path to move forward
    # No need to worry if in path but not in range
    if obj_location_msg.angular.x == 1:
        path_clear = False
        force_feedback(0, 1)
    else:
        path_clear = True
        force_feedback(0, 0)


### Retrieve the nearest cup measurements and set values for move towards blob
def blob_callback(blob_msg):
    global blob_speed, blob_steer, blob_angle, blob_dist
    x = blob_msg.linear.x
    y = blob_msg.linear.y
    blob_dist = blob_msg.linear.z
    blob_angle = blob_msg.angular.z

    # Check storage capacity
    # if blob_msg.angular.z == 1:
    #     dump_sequence = True  # Dump the snow if plough is full
    # else:
    #     dump_sequence = False

    # Proportional Steer towards the blob (SP=0, trim<0.2)
    k = 100  # Multiplier
    blob_speed = clamp(0.5, (y / k), 1.5)
    temp_var = clamp(-0.8, (-x / k), 0.8)
    if abs(temp_var) < 0.2:
        temp_var = 0
    blob_steer = temp_var


### Dump cups using this sequence of dummy commands
def dump_cups():
    ## Align bot when edge detected
    # while not (left_align or right_align):
    #     # Move Forward slowly for precision alignment
    #     move(1, 0)
    # move(0,0)
    # time.sleep(3)
    # if left_align:  # If Left wheel reached edge first
    #     # Swing right wheel forward, ie: left swing turn
    #     while not right_align:
    #         move(0.75, 0.75)
    # else:  # If Right wheel reached edge first
    #     # Swing left wheel forward, ie: right swing turn
    #     while not left_align:
    #         move(0.75, -0.75)
    # move(0, 0)
    # time.sleep(5)
    
    ## Now the bot is sort of perpendicular to the edge
    move(2, 0)  # Push cups out
    time.sleep(1)
    move(-2, 0)  # reverse
    time.sleep(2)
    move(0, 0.5)  # Turn around
    time.sleep(1.25)  # Can adjust time for more/less angle <<<<<
    # move(0,0)
    # time.sleep(3)


### Parallel thread to run decision making state machine <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
def state_machine_thread():
    global dump_sequence, blob_speed, blob_steer, inside_mat, path_clear, wait_to_start

    while wait_to_start:  # wait for dyn_rcfg to start up
        time.sleep(3)  # Check every 3 second - buffer for values to update

    rate = rospy.Rate(20)  # Loop rate - fast for quick reaction time
    while not rospy.is_shutdown():
        print("running loop ---------------------------")

        if tele_op_toggled or joy_override:  # State machine gives control to user
            print("USER CONTROL ACTIVE")
            move(0, 0) # Stop or use joystick values

        else:  # State machine takes over control to drive autonomously
            ## Stay inside working area 1st Priority - [works okay]
            if inside_mat:
                print("MAT")
                ## Edge Avoidance 2nd Priority - [works okay]
                if front_edge:
                    print("--- FRONT edge")
                    # Check if bot is facing the edge head on. Dump no matter the quantity
                    if left_edge and right_edge:  # Maybe not a good idea, could work but needs testing
                        print("--- --- LEFT & RIGHT edge")
                        print("--- --- --- DUMPING CUPS")
                        dump_cups()
                        # move(0, 1)  # ??? LEFT
                    # Check if front and left only
                    elif left_edge:
                        print("--- --- LEFT edge")
                        move(1, -1)  # Go right
                    # Check if front and right only
                    elif right_edge:
                        print("--- --- RIGHT edge")
                        move(1, 1)  # Go left
                    else:  # Bot and line facing the same direction
                        # Turn towards the inside of mat
                        print("--- --- ALONG edge")
                        move(-1, 1)  # Turn left until this changes ??

                else:  # No Edge Detected
                    print("--- ")
                    ## Object Avoidance 3rd Priority - [works good]
                    if not path_clear:  # Path NOT clear, obj in way
                        print("--- --- BLOCKED")
                        # Turn away from the object
                        if obj_angle > 0:  # Object on the right
                            move(0, 1)  # Turn left until out of state
                        else:  # Object on the left
                            move(0, -1)  # Turn right until out of state

                    else:  # Find Blobs and go towards them - 4th Priority - [works really good*]
                        # *background tiles are roughly same size* will work consistently with 
                        # a solid background
                        print("--- --- CLEAR")
                        if 0 < blob_dist < 80:  # Unit pixels_from_bot. Check for cups in CLOSE range
                            # Sometimes detects
                            print("--- --- --- CUP detected")
                            move(blob_speed, -blob_steer)  # Go towards cups
                        else:  # No cups in range
                            print("--- --- --- ---")
                            move(2, 0)  # Go Forward

            else:  # Outside working area
                print("!!! OFF MAT !!!")
                # move(0,0)
                # Can use another alogrithm to detect mat but not needed for this application
                move(0.5, -0.5)  # Drive with slight steer, hopefully end up inside mat
                time.sleep(1)  # Delay for other conditions to reset
        rate.sleep()


### Main call
if __name__ == "__main__":
    # Start node
    rospy.init_node("snow_blower", anonymous=True)

    # Get topics at launch
    twistTopic = rospy.get_param("~twist_topic_name")  # Motion twist output topic
    obstacleTopic = rospy.get_param("~obstacle_topic_name")  # Obstacle Detect input topic
    edgeTopic = rospy.get_param("~edge_topic_name")  # Edge Detect input topic
    blobTopic = rospy.get_param("~blob_topic_name")  # Blob Detect input topic
    joyTopic = rospy.get_param("~joy_topic_name")  # Joystick input topic
    joyFFTopic = rospy.get_param("~joy_ff_topic_name")  # Joystick output topic
    frontImgTopic = rospy.get_param("~front_img_topic_name")  # Image input topic
    # downImgTopic = rospy.get_param("~down_img_topic_name")  # Image input topic

    # Set Subscribers/Publishers - queue_size=1 because multiple buffers make node slow
    edge_sub = rospy.Subscriber(edgeTopic, Twist, edge_callback, queue_size=1)
    obstacle_sub = rospy.Subscriber(obstacleTopic, Twist, obstacle_callback, queue_size=1)
    blob_sub = rospy.Subscriber(blobTopic, Twist, blob_callback, queue_size=1)
    joy_sub = rospy.Subscriber(joyTopic, Joy, joy_callback, queue_size=1)
    front_image_sub = rospy.Subscriber(frontImgTopic, Image, front_image_callback, queue_size=1)
    # down_image_sub = rospy.Subscriber(downImgTopic, Image, down_image_callback, queue_size=1)
    velocity_pub = rospy.Publisher(twistTopic, Twist, queue_size=1)
    vel_msg = Twist()  # Motion message
    ff_pub = rospy.Publisher(joyFFTopic, JoyFeedbackArray, queue_size=1)
    ff_msg = JoyFeedbackArray()  # Joystick Force Feedback message

    # Dynamic Reconfigure parameter server
    srv = Server(SnowBlowerConfig, dyn_rcfg_cb)

    worker1 = threading.Thread(target=state_machine_thread)
    worker1.start()  # Start the thread to handle decisions and states

    # Spin the node for callbacks
    rospy.spin()  # ROS python auto runs main, spin and each callback as individual threads
    # This means this program becomes a thread which does not shutdown until
    # spin thread calls it off. ie: globals are shared between them
