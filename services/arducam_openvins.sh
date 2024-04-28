#!/bin/bash

source /home/vio/ros_noetic/catkin_ws/install_isolated/setup.bash
source /home/vio/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311

# Function to check and set IP address
set_ros_ip() {
    # Get the IPv4 address from a specific interface, e.g., wlan0
    IP_ADDR=$(ip -4 addr show wlan1 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
    if [[ -z "$IP_ADDR" ]]; then
        echo "Waiting for a valid IP on wlan0..."
        sleep 5  # Wait for 5 seconds before trying again
        set_ros_ip  # Recursive call until an IPv4 is found
    else
        export ROS_IP=$IP_ADDR
        echo "Setting ROS_IP to $ROS_IP"
    fi
}

# This is not needed if you are running the system completely locally
# and transferring data, e.g., serially.
set_ros_ip

# Start the ROS nodes
roslaunch /home/vio/arducam_mpu_openvins.launch

#wait