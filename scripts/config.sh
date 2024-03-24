#!/bin/bash

if [ -f "/opt/ros/humble/install/setup.bash" ]; then
    echo "sourcing /opt/ros/humble/install/setup.bash"
    source /opt/ros/humble/install/setup.bash
fi
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "sourcing /opt/ros/humble/setup.bash"
    source /opt/ros/humble/setup.bash
fi
if [ -f "$HOME/shared_volume/ros2_ws/install/setup.bash" ]; then
    echo "sourcing $HOME/ros2_ws/install/setup.bash"
    source $HOME/shared_volume/ros2_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=10

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
line_to_check="alias rs='ros2 launch realsense2_camera rs.launch.py'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias vins='ros2 launch ov_msckf subscribe.launch.py config_path:=$OPENVINS_YAML'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias cb='colcon build'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias cbs='colcon  build --packages-select'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi

bashrc_file="$HOME/.bashrc"
line_to_check="alias cbupto='colcon  build --packages-up-to'"
if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
fi


source $HOME/.bashrc