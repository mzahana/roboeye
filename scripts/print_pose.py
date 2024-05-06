#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import sys

def print_position(x, y, z):
    # Move the cursor up three lines before printing
    sys.stdout.write('\033[F\033[F\033[F')
    # Clear the line and print new data
    print(f"x: {x}\033[K")
    print(f"y: {y}\033[K")
    print(f"z: {z}\033[K")

def callback(data):
    position = data.pose.pose.position
    print_position(position.x, position.y, position.z)

def listener():
    rospy.init_node('odometry_listener', anonymous=True)
    rospy.Subscriber("/mavros/odometry/out", Odometry, callback)
    # Initialize display
    print("x:\ny:\nz:")
    rospy.spin()  # Keep the script alive

if __name__ == '__main__':
    listener()