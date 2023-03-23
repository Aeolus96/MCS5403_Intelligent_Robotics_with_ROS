// I have neither given nor received any unauthorizedaid in completing this work,
// nor have I presented someone else's work as my own.
// Your Name: Devson Butani
// LTU ID: 000732711
// Date: 10/6/2022

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <algorithm>

ros::Publisher twistPub;
sensor_msgs::Joy joy_buffer;

using namespace std;

// Defining joystick functions
double leftStick_x, leftStick_y, rightStick_x, rightStick_y, leftTrigger, rightTrigger;
bool dPad_left, dPad_right, dPad_up, dPad_down;
bool leftStick_btn, rightStick_btn, leftBumper, rightBumper;
bool btnA, btnB, btnX, btnY, btnBack, btnStart;

// Joystick Map for ROS sensor_msgs/Joy
void joystick_profile_x() // Logitech Gamepad F310: mode X
{
  // Axes Inputs:
  leftStick_x = -1.0 * joy_buffer.axes[0]; // Inverts the X-axis from (1, -1) to (-1, 1)
  leftStick_y = joy_buffer.axes[1];
  rightStick_x = -1.0 * joy_buffer.axes[3]; // Inverts the X-axis from (1, -1) to (-1, 1)
  rightStick_y = joy_buffer.axes[4];
  // Invert -> Translate -> Scale
  leftTrigger = ((-1.0 * joy_buffer.axes[2]) + 1.0) / 2.0;  // Changed from (1, -1) where 1 is normal to (0, 1)
  rightTrigger = ((-1.0 * joy_buffer.axes[2]) + 1.0) / 2.0; // Changed from (1, -1) where 1 is normal to (0, 1)
  // D-Pad is a directional button pad considered as axes
  if (joy_buffer.axes[6] == 1.0) // D-Pad left|right axis to button converter
  {
    dPad_left = true;
    dPad_right = false;
  }
  else if (joy_buffer.axes[6] == -1.0)
  {
    dPad_left = false;
    dPad_right = true;
  }
  else
  {
    dPad_left = false;
    dPad_right = false;
  }
  if (joy_buffer.axes[7] == 1.0) // D-Pad up|down axis to button converter
  {
    dPad_up = true;
    dPad_down = false;
  }
  else if (joy_buffer.axes[7] == -1.0)
  {
    dPad_up = false;
    dPad_down = true;
  }
  else
  {
    dPad_up = false;
    dPad_down = false;
  }

  // Button Inputs:
  btnA = joy_buffer.buttons[0]; // A(2)
  btnB = joy_buffer.buttons[1]; // B(3)
  btnX = joy_buffer.buttons[2]; // X(1)
  btnY = joy_buffer.buttons[3]; // Y(4)
  leftBumper = joy_buffer.buttons[4];
  rightBumper = joy_buffer.buttons[5];
  btnBack = joy_buffer.buttons[6];
  btnStart = joy_buffer.buttons[7];
  // joy_buffer.buttons[8] is not a physical button in this profile
  leftStick_btn = joy_buffer.buttons[9];
  rightStick_btn = joy_buffer.buttons[10];

  // If needed features like deadzone and debounce can be added here:
}

// Clamp from C++17 because current ROS setup uses C++11
double clip(double n, double lower, double upper)
{
  // https://stackoverflow.com/questions/9323903/most-efficient-elegant-way-to-clip-a-number
  return std::max(lower, std::min(n, upper));
}

// Controller specific to HW5 requirements
void hw5_controller(double linear_scale = 0.8, double angular_scale = 0.4) // Defaults for EduBot
{
  // Limit scaling for EduBot capabilities
  clip(linear_scale, 0, 3);
  clip(angular_scale, 0, 1.5);
  // Write Twist Message
  geometry_msgs::Twist twistMsg;
  twistMsg.linear.x = leftStick_y * linear_scale;           // Forward Backward
  twistMsg.angular.z = rightStick_x * -1.0 * angular_scale; // Left Right inverted because Absolute East
  // Publish to topic
  ROS_INFO_STREAM("Linear: " << twistMsg.linear.x);
  ROS_INFO_STREAM("Angular: " << twistMsg.angular.z);
  twistPub.publish(twistMsg);
}

void joystick_callback(const sensor_msgs::Joy &joy)
{
  joy_buffer = joy;     // Store input into buffer
  joystick_profile_x(); // Update status of joystick features
  hw5_controller();     // Run controller specific to HW5-1/2
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_control_node");
  ros::NodeHandle nh;
  // Input from:
  ros::Subscriber sub = nh.subscribe("/joy", 10, joystick_callback); // Set Subscriber
  string topic_name = "/turtle1/cmd_vel";                            // default topic name
  nh.getParam("/out_topic_name", topic_name);                        // update topic from param if available
  // Output to:
  twistPub = nh.advertise<geometry_msgs::Twist>(topic_name, 10); // Set Publisher
  ros::spin();                                                   // Give total control to callback until shutdown
}