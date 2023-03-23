// I have neither given nor received any unauthorizedaid in completing this work,
// nor have I presented someone else's work as my own.
// Your Name: Devson Butani
// LTU ID: 000732711
// Date: 09/14/2022

#include <ros/ros.h>
#include <std_msgs/Int32.h>

using namespace std;

class hw2PubStr
{
public:
  hw2PubStr()
  {
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("/hw2_topic_int", 10);
    std_msgs::Int32 msg;
    while (ros::ok())
    {
      int a = get_input();
      if (a != -99) // Publish only if no exceptions
      {
        msg.data = a;
        pub.publish(msg);
      }
    }
  }

  // Get user input from terminal
  int get_input()
  {
    string in;
    try
    {
      cout << "Enter the degree of change (0~10): "; // Prompt for degree of change
      getline(cin, in);                              // Get all string input
      int out = stoi(in);
      if (out < 0 || out > 10) // Check input against specified limits
      {
        throw "> Integer not between 0 and 10! Try again.";
      }
      else // Value is acceptable and return
      {
        return out;
      }
    }
    // Handle errors
    catch (const char *msg)
    {
      cout << msg << endl;
    }
    catch (exception &e)
    { // all other exceptions
      cout << e.what() << " error" << endl;
    }
    return -99; // Returns -99 to show exception encountered
  }

private:
  ros::NodeHandle nh;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw2_pub_int");
  hw2PubStr np{};
  // ros::spin();  Not needed since there is no subscriber
}