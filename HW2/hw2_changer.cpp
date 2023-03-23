// I have neither given nor received any unauthorizedaid in completing this work,
// nor have I presented someone else's work as my own.
// Your Name: Devson Butani
// LTU ID: 000732711
// Date: 09/14/2022

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <time.h>

using namespace std;

class NewsChanger
{
public:
  NewsChanger()
  {
    // Sub topics
    sub1 = nh.subscribe("/hw2_topic_str", 10, &NewsChanger::receive_string_cb, this);
    sub2 = nh.subscribe("/hw2_topic_int", 10, &NewsChanger::receive_int_cb, this);
    // Pub topics
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hw2_topic_changed", 10);
    std_msgs::String msg;
    // Get pub rate
    nh.getParam("hw2_pub_freq", pub_freq); // Get frequency from core params
    ros::Rate rate(pub_freq);

    // Data processing loop
    while (ros::ok())
    {
      rate.sleep();    // sleep first not to loose the 1st message
      ros::spinOnce(); // Get new messages. Callbacks from queue and then return here.
      if (update)      // Check update flag for change. Bypass pub if no change
      {
        update = false;                                     // Reset flag to recieve more messages
        if (str_buffer.length() != 0 && int_buffer >= 0)    // Bypass if both str & int are not received yet
        {                                                   // Modify input string
          string str_mod = modify(str_buffer, int_buffer);  // Store modified string
          msg.data = str_mod;                               // Enter string data into ROS string type
          pub.publish(msg);                                 // Replace modified string with original string
          ROS_INFO_STREAM("Changer OUT str: " << msg.data); // Display for debugging
        }
      }
    }
  }

  // Modification algorithm replaces characters with $ from a random index
  string modify(string input_str, int input_int)
  {
    int len = (input_str.length() == 0) ? 1 : input_str.length(); // Find string length. 1 if empty string.
    double deg_of_ch = input_int / 10.0;                          // Calculate degree of change
    int char_to_ch = floor(len * deg_of_ch);                      // Calculate characters that need changing
    if (char_to_ch != 0)                                          // Bypass condition if no change needed
    {
      int idx = (rand() % len);            // Generate a random index value within string length
      for (int i = 0; i < char_to_ch; i++) // Replace characters in string with '$' starting from randomly
      {                                    // generated index in a circullar array fashion
        input_str[(idx % len)] = '$';
        idx = idx + 1;
      }
    }
    return input_str;
  }

  // Callback function for string storage
  void receive_string_cb(const std_msgs::String &msg)
  {
    ROS_INFO_STREAM("Changer IN str: " << msg.data); // Display for debugging
    str_buffer = msg.data;                           // Store recieved messages in the string buffer
    update = true;                                   // Flags new messages as recieved for Data processing loop
  }

  // Callback function for integer storage
  void receive_int_cb(const std_msgs::Int32 &msg)
  {
    ROS_INFO_STREAM("Changer IN int: " << msg.data); // Display for debugging
    int_buffer = msg.data;                           // Store recieved messages in the int buffer
    update = true;                                   // Flags new messages as recieved for Data processing loop
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber sub1;
  ros::Subscriber sub2;
  double pub_freq = 1; // defualt freq is 1 Hz
  int int_buffer = -1; // buffer for messages
  string str_buffer;   // buffer for messages
  bool update = false; // flag for new message

}; // End of class

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw2_changer");
  srand(time(0)); // Initialize random generator using system time
  NewsChanger ns{};
}