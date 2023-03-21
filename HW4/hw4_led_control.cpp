// I have neither given nor received any unauthorizedaid in completing this work,
// nor have I presented someone else's work as my own.
// Your Name: Devson Butani
// LTU ID: 000732711
// Date: 09/28/2022

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <string.h>

using namespace std;

// Global variables and buffers
string led_color = "green"; // defualt color is Green
int int_buffer;             // buffer to hold keys from topic
double pub_freq = 5.0;      // default rate is 5Hz
bool state = false;         // flag for state of led

// Callback function for /keyboard
void receive_keyboard_cb(const std_msgs::Int32 &msg)
{
  // ROS_INFO_STREAM("Key ID: " << msg.data);
  int_buffer = msg.data; // Store value for global use
}

// Simple state selector function
void set_state(int key)
{
  // Check if value received is for 1 or 0. If not do nothing.
  if (key == 49) // 1. Turn LED On
  {
    state = true;
  }
  else if (key == 48) // 0. Turn LED Off
  {
    state = false;
  }
}

int main(int argc, char **argv)
{
  // Initialize
  ros::init(argc, argv, "hw4_led_control");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/keyboard", 10, receive_keyboard_cb); // Set Subscriber to /keyboard

  nh.getParam("hw4_pub_freq", pub_freq); // Get publishing frequency from launch params
  ros::Rate rate(pub_freq);

  nh.getParam("led_color", led_color); // Get color from launch params
  // Set Publisher using color based topic for switching in loop
  // Advertise to both because changing topics for same pub in loop scope doesn't work
  ros::Publisher pub_green = nh.advertise<std_msgs::Bool>("/green_led", 10);
  ros::Publisher pub_red = nh.advertise<std_msgs::Bool>("/red_led", 10);
  std_msgs::Bool led_state_msg;

  while (ros::ok())
  {
    rate.sleep();    // sleep first not to loose the 1st message
    ros::spinOnce(); // Get new messages. Callbacks from queue and then return here.

    set_state(int_buffer); // Check keyboard and set on/off state irrespective of color
    led_state_msg.data = state;

    // Real time param check for changing colors on the fly
    nh.getParam("led_color", led_color); // Get color from launch params
    if (led_color == "green")            // This could be a switch for more colors but C++...
    {
      pub_green.publish(led_state_msg);
    }
    if (led_color == "red")
    {
      pub_red.publish(led_state_msg);
    }
    if (led_color != "red" && led_color != "green")
    {
      pub_red.publish(led_state_msg); // Default Fallback to red for testing
      // Could turn both off or send out error here if needed
    }
  }
}