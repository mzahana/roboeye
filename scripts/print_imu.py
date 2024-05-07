#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import sys

def print_imu_data(angular_velocity, linear_acceleration):
    # Move the cursor up six lines before printing
    sys.stdout.write('\033[F\033[F\033[F\033[F\033[F\033[F')
    # Clear the line and print new data
    print(f"Angular Velocity x: {angular_velocity.x}\033[K")
    print(f"Angular Velocity y: {angular_velocity.y}\033[K")
    print(f"Angular Velocity z: {angular_velocity.z}\033[K")
    print(f"Acceleration x: {linear_acceleration.x}\033[K")
    print(f"Acceleration y: {linear_acceleration.y}\033[K")
    print(f"Acceleration z: {linear_acceleration.z}\033[K")

def callback(data):
    angular_velocity = data.angular_velocity
    linear_acceleration = data.linear_acceleration
    print_imu_data(angular_velocity, linear_acceleration)

def listener():
    rospy.init_node('imu_data_listener', anonymous=True)
    rospy.Subscriber("/imu/data_raw", Imu, callback)
    # Initialize display
    print("Angular Velocity x:\nAngular Velocity y:\nAngular Velocity z:\nAcceleration x:\nAcceleration y:\nAcceleration z:")
    rospy.spin()  # Keep the script alive

if __name__ == '__main__':
    listener()