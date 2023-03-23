// I have neither given nor received any unauthorizedaid in completing this work,
// nor have I presented someone else's work as my own.
// Your Name: Devson Butani
// LTU ID: 000732711
// Date: 09/14/2022

#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

class NewsChanger
{
public:
  NewsChanger()
  {
    nh.getParam("hw2_sub_freq", sub_freq); // Get frequency from core params
    ros::Rate rate(sub_freq);
    // Define subscriber
    sub = nh.subscribe("/hw2_topic_changed", 10, &NewsChanger::receive_topic1data_cb, this);
    while (ros::ok())
    {
      rate.sleep(); // sleep first not to loose the 1st message
      ros::spinOnce();
      ROS_INFO_STREAM("WWW Radio: " << str_buffer); // Broadcast-like continuous stream
    }
  }
  // Callback function from base
  void receive_topic1data_cb(const std_msgs::String &msg)
  {
    // ROS_INFO_STREAM("WWW Radio: " << msg.data); // Instead of writing only when it updates
    str_buffer = msg.data;                         // Store value for continuous stream
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  double sub_freq = 1; // defualt freq is 1 Hz
  string str_buffer;   // Buffer for incoming string
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw2_radioStation");
  NewsChanger ns{};
}