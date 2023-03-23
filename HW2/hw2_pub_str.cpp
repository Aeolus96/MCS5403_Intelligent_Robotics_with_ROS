// I have neither given nor received any unauthorizedaid in completing this work,
// nor have I presented someone else's work as my own.
// Your Name: Devson Butani
// LTU ID: 000732711
// Date: 09/14/2022

#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

class hw2PubStr
{
public:
  hw2PubStr()
  {
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hw2_topic_str", 10);
    std_msgs::String msg;
    while (ros::ok())
    {
      string a = get_input();
      if (a != "") // Publish only if no exceptions or not empty string
      {
        msg.data = a;
        pub.publish(msg);
      }
    }
  }

  // Get user input from terminal
  string get_input()
  {
    string in;
    try // No conditions inside input function. Return string as is.
    {
      cout << "Enter news: "; // Prompt for news
      getline(cin, in);       // Get all string input
      return in;
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
    return ""; // Returns "" (empty string) to show exception encountered
  }

private:
  ros::NodeHandle nh;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw2_pub_str");
  hw2PubStr np{};
  // ros::spin();  Not needed since there is no subscriber
}