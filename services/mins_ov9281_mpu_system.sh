#!/bin/bash

if [ -f "/home/vio/ros_noetic/catkin_ws/install_isolated/setup.bash" ]; then
    source /home/vio/ros_noetic/catkin_ws/install_isolated/setup.bash
else
    echo "ERROR: Could not find /home/vio/ros_noetic/catkin_ws/install_isolated/setup.bash . EXIT"
    exit 1
fi

if [ -f "/home/vio/catkin_ws/devel/setup.bash" ]; then
    source /home/vio/catkin_ws/devel/setup.bash
else
    echo "ERROR: Could not find /home/vio/catkin_ws/devel/setup.bash . EXIT"
    exit 1
fi

if [ -f "/home/vio/mavros_ws/devel/setup.bash" ]; then
    source /home/vio/mavros_ws/devel/setup.bash
else
    echo "ERROR: Could not find /home/vio/mavros_ws/devel/setup.bash . EXIT"
    exit 1
fi

if [ -f "/home/vio/mins_ws/devel/setup.bash" ]; then
    source /home/vio/mins_ws/devel/setup.bash
else
    echo "ERROR: Could not find /home/vio/mins_ws/devel/setup.bash . EXIT"
    exit 1
fi


if [ -f "/home/vio/config/mins_ov9281_mpu_system_config.sh" ]; then
    source /home/vio/config/mins_ov9281_mpu_system_config.sh
else
    echo "ERROR: Could not find /home/vio/config/mins_ov9281_mpu_system_config.sh . EXIT"
    exit 1
fi

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOME=/home/vio/logs

# Function to check and set IP address
set_ros_ip() {
    # Get the IPv4 address from a specific interface, e.g., wlan0
    IP_ADDR=$(ip -4 addr show wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
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
#set_ros_ip

# Start the ROS nodes
roslaunch /home/vio/launch/mins_ov9281_mpu_system.launch

#wait
