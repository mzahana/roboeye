#!/bin/bash
ROS_WS=$HOME/ros2_humble
ROS2_INSTALL=$HOME/ros2_humble/install
if [ -f "${ROS2_INSTALL}/setup.bash" ]; then
    echo "sourcing ${ROS2_INSTALL}/setup.bash"
    source ${ROS2_INSTALL}/setup.bash
fi
VINS_WS=$HOME/vins_ws
if [ -f "${VINS_WS}/install/setup.bash" ]; then
    echo "sourcing ${VINS_WS}/install/setup.bash"
    source ${VINS_WS}/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10
if [ -f "$HOME/rtps_udp_profile.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/rtps_udp_profile.xml
    print_info "FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/rtps_udp_profile.xml"
else
    print_warning " Could not find $HOME/rtps_udp_profile.xml"
fi

#
# Modules
#
export RUN_OPENVINS=False
export RUN_ARDUCAM_STEREO=False


#
# OpenVins
#
# Example: export OPENVINS_YAML='/home/d2d/shared_volume/ros2_ws/src/open_vins/config/custom_d455/estimator_config.yaml'
export OPENVINS_YAML=''


# The following code creates convenience aliases to be used inside the d2dtracker container
# bashrc_file="$HOME/.bashrc"
# line_to_check="alias viz='rviz2 -d /home/d2d/shared_volume/ros2_ws/src/open_vins/ov_msckf/launch/display_ros2.rviz'"
# if ! grep -qF "$line_to_check" "$bashrc_file"; then
#     echo "$line_to_check" >> "$bashrc_file"
# fi


bashrc_file="$HOME/.bashrc"
line_to_check="alias vins='ros2 launch ov_msckf subscribe.launch.py config_path:=$OPENVINS_YAML'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias cb='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias cbs='colcon  build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias cbupto='colcon  build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

# Give read/write access to i2c ports
sudo chmod +777 /dev/i2c-*

source $HOME/.bashrc