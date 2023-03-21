// I have neither given nor received any unauthorizedaid in completing this work,
// nor have I presented someone else's work as my own.
// Your Name: Devson Butani
// LTU ID: 000732711
// Date: 09/02/2022

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <regex>
using namespace std; // you can use "string" directly instead of std::string

// Convert Degree to Radian
double degtorad(double deg)
{
	double pi = 3.14159265359; // Precise enough for use case
	double rad = deg * (pi / 180);
	return rad;
}

// Checks for numeric input using regular expressions
bool isNumber(const string &token)
{
	return regex_match(token, regex("(\\+|-)?[0-9]*(\\.?([0-9]+))$"));
}

// Convert to degrees of Latitude and Longitude within specified limits
double degree_input(double min_degree, double max_degree)
{
	double degree;
	string in;
	// Convert input string to degree value
	try
	{
		cout << " "; // Space for cursor to separate in console
		getline(cin, in);
		if (!isNumber(in)) // Check input for valid number format
		{
			throw "> Input non-numerical!";
		}
		else
		{
			degree = stod(in);
			if (degree < min_degree || degree > max_degree) // Check input against specified limits
			{
				throw "> Not within specified numeric range!";
			}
			else // Value is acceptable and return
			{
				return degree;
			}
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
	return 0; // returns 0 if input is incorrect. Used for asking for input again
}

// Calculate distance between two coordinates using Haversine formula
static double haversine(double lat_1, double lon_1, double lat_2, double lon_2)
{
	// Convert input to radians for calculation
	double phi_1 = degtorad(lat_1);
	double phi_2 = degtorad(lat_2);
	double lam_1 = degtorad(lon_1);
	double lam_2 = degtorad(lon_2);
	// Radius of Earth 6371 kilometers
	double rofearth = 6371000; // in meters
	// Break equation into parts and combine
	double i = pow(sin((phi_2 - phi_1) / 2), 2);
	double j = (cos(phi_1) * cos(phi_2) * pow(sin((lam_2 - lam_1) / 2), 2));
	double k = sqrt(i + j);
	// distance in meters
	double d = 2 * rofearth * asin(k);
	// Display calculated distance in meters
	cout << fixed << setprecision(6) << "Distance between ("
		 << lat_1 << ", " << lon_1 << ") and ("
		 << lat_2 << ", " << lon_2 << ") = "
		 << d << " meters" << endl;
	// Return value if needed elsewhere later
	return d;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hw1_node"); //>>>>> recommended to match with CMakeList.txt's node name
	ros::NodeHandle nh;
	ROS_INFO_STREAM("hw1_node starts.\n--------------\n");

	// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	double lat1 = 42.475116, lon1 = 83.249930; // waypoint1 - manhole1 outside automatic door of MGT bld
	double lat2 = 42.475190, lon2 = 83.249804; // waypoint2
	haversine(lat1, lon1, lat2, lon2);
	// Reset values
	lat1 = 0;
	lon1 = 0;
	lat2 = 0;
	lon2 = 0;

	// Get user input
	cout << "\nNew coordinates:\n";
	// First waypoint
	while (lat1 == 0)
	{
		cout << "Enter lat1, between -90 and 90:";
		lat1 = degree_input(-90, 90);
	}
	while (lon1 == 0)
	{
		cout << "Enter lon1, between -180 and 180:";
		lon1 = degree_input(-180, 180);
	}
	// Second waypoint
	while (lat2 == 0)
	{
		cout << "Enter lat2, between -90 and 90:";
		lat2 = degree_input(-90, 90);
	}
	while (lon2 == 0)
	{
		cout << "Enter lon2, between -180 and 180:";
		lon2 = degree_input(-180, 180);
	}
	// Calculate distance between new coordinates
	haversine(lat1, lon1, lat2, lon2);
}
