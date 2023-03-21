#!/usr/bin/env python3

import rospy
import math
import re

# Class for a waypoint data
class WayPoint:
    # Initialize lat and lon values
    def __init__(self, a=999.999, b=999.999):
        self.lat = a
        self.lon = b

    # debugging display for class values
    def __str__(self) -> str:
        return f'Lattitude: {self.lat:.6f}\nLongitude: {self.lon:.6f}'

    # Reset to default values
    def reset(self):
        self.lat = 999.999
        self.lon = 999.999

    # Set new values. First reset and then acquire
    # typ = lat | lon , num = "1" | "2"
    def set_degree(self, typ, num):
        if typ == "lat":
            min = -90.0
            max = 90.0
            # Reset values before getting new ones
            self.lat = 999.999
        elif typ == "lon":
            min = -180.0
            max = 180.0
            # Reset values before getting new ones
            self.lon = 999.999
        else:
            print(f'{typ} is not <lat> or <lon>')
            return
        val = 999.999
        # Get input
        while val == 999.999:
            print(f'"Enter {typ}{num}, between {min} and {max}:')
            val = degreeInput(min, max)

        # Set the input value accordingly
        if typ == "lat":
            self.lat = val
        elif typ == "lon":
            self.lon = val
        else:
            print(f'Value not set because {typ} is not <lat> or <lon>')
            return
# End of Class


# Convert Degree to Radians
def degToRad(deg):
    rad = deg * (math.pi/180)
    return rad

# Check if input matches Regular expression for real float numbers
def isNumber(token):
    pattern = "^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)$"
    r = re.compile(pattern)
    return r.match(token)

# Take degree input based on input range
def degreeInput(min, max):
    try:
        n = input(">")  # Take string input
        if not isNumber(n):  # Check if input is a number
            raise Exception(f'{n} is not a number')
        else:
            n = float(n)
            if min <= n <= max:  # Check if within min and max
                #print(f"Entered number is {n:.6f}")
                return n
            else:
                raise Exception(f'{n} is not within {min} and {max}')
    except Exception as e:  # Print all exceptions
        print(f"{e}")
        return 999.999

# Calculate Distance between input coordinates
def haversine(lat_1, lon_1, lat_2, lon_2):
    # Convert input to radians
    phi_1 = degToRad(lat_1)
    phi_2 = degToRad(lat_2)
    lam_1 = degToRad(lon_1)
    lam_2 = degToRad(lon_2)
    # Radius of Earth in meters
    rofearth = 6371000
    # Break equation into parts and combine
    i = (math.sin((phi_2 - phi_1) / 2))**2
    j = math.cos(phi_1) * math.cos(phi_2) * (math.sin((lam_2 - lam_1) / 2))**2
    k = math.sqrt(i + j)
    d = 2 * rofearth * math.asin(k)
    # Display Calculated distance in meters
    print(f"Distance between ({lat_1:.6f}, {lon_1:.6f}) and ({lat_2:.6f}, {lon_2:.6f}) = {d:.6f} meters")
    # Return value if needed elsewhere
    return d


# Main called here for execution
if __name__ == '__main__':
    rospy.init_node('hw1_node_Python')
    w1 = WayPoint(42.475116, 83.249930)
    w2 = WayPoint(42.475190, 83.249804)
    haversine(w1.lat, w1.lon, w2.lat, w2.lon)

    print("\nInput New Coordinates:")
    # Get new values
    w1.set_degree("lat", "1")
    w1.set_degree("lon", "1")
    w2.set_degree("lat", "2")
    w2.set_degree("lon", "2")
    # have to use "1" and "2" just for display purpose
    haversine(w1.lat, w1.lon, w2.lat, w2.lon)
