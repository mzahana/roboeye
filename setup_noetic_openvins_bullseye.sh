#!/bin/bash -e

# You can set these variables before executiong this script
#BUILD_ROS=false
#BUILD_OPENVINS=false
#INSTALL_ARDUCAM=false

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh

# Get the Codename value from the lsb_release output
codename=$(lsb_release -a 2>/dev/null | grep 'Codename:' | awk '{print $2}')

# Check if the codename is "bullseye"
if [ "$codename" = "bullseye" ]; then
    print_info "Running on $codename"
else
    print_error "This script requires Debian Bullseye. Exiting."
    exit 1
fi

#
# Some ENV variables
#
ROS_DISTRO=noetic
ROS_WS=${HOME}/ros_noetic/catkin_ws
CATKIN_WS=${HOME}/catkin_ws
CATKIN_WS_SRC=${CATKIN_WS}/src

#
# Create workspaces
#
if [ ! -d "$HOME/ros_noetic" ]; then
    print_info "Creating $HOME/ros_noetic/catkin_ws workspace"
    mkdir -p ${HOME}/ros_noetic/catkin_ws/src
fi

if [ ! -d "$CATKIN_WS" ]; then
    print_info "Creating $CATKIN_WS workspace"
    mkdir -p ${CATKIN_WS}/src
fi

if [ "$BUILD_ROS" = true ]; then
    print_info "Building ROS $ROS_DISTRO"
    print_warning "This may take a few hours on Raspberry Pi with limited memory ..." && sleep 1

    print_info "Installing build tools... " && sleep 1
    sudo apt-get update && sudo apt-get install -y \
                            python3-rosdep2 \
                            python3-rosinstall-generator \
                            python3-vcstools \
                            build-essential \
                            && sudo rm -rf /var/lib/apt/lists/* \
                            && sudo apt-get clean
    
    sudo pip install -U vcstool
    
    sudo apt upgrade -y

    sudo rosdep init
    rosdep update

    cd $ROS_WS
    rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-perception.rosinstall
    vcs import --input noetic-perception.rosinstall ./src

    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

    ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
else
    print_warning "Skipping building ROS"
fi

source $ROS_WS/install_isolated/setup.bash

#
# OpenVins depndencies
#
print_info "Installing openvins dependencies..."
sudo apt-get update && sudo apt-get install -y \
    libeigen3-dev \
    libboost-all-dev \
    libceres-dev \
    && sudo rm -rf /var/lib/apt/lists/* \
    && sudo apt-get clean

#
# Cloning open_vins
#
print_info "Cloning open_vins ..." && sleep 1
if [ ! -d "${CATKIN_WS_SRC}/open_vins" ];then
    cd ${CATKIN_WS_SRC}
    git clone https://github.com/rpng/open_vins.git
else
    cd ${CATKIN_WS_SRC}/open_vins
    git pull origin master
fi

print_info "Copying arducam stereo config files to ${CATKIN_WS_SRC}/open_vins/config/rpi_vi_kit"
if [ ! -d "${CATKIN_WS_SRC}/open_vins/config/rpi_vi_kit" ]; then
    mkdir -p ${CATKIN_WS_SRC}/open_vins/config/rpi_vi_kit
fi
cp ${ROOT}/config/openvins/* ${CATKIN_WS_SRC}/open_vins/config/rpi_vi_kit/


#
# i2c_device_ros: Required by mpu6050_driver
#
print_info "Cloning i2c_device_ros ..." && sleep 1
if [ ! -d "${CATKIN_WS_SRC}/i2c_device_ros" ];then
    cd ${CATKIN_WS_SRC}
    git clone https://github.com/Brazilian-Institute-of-Robotics/i2c_device_ros.git

else
    cd ${CATKIN_WS_SRC}/i2c_device_ros
    git pull origin master
fi

#
# mpu6050_driver
#
print_info "Cloning mpu6050_driver ..." && sleep 1
if [ ! -d "${CATKIN_WS_SRC}/mpu6050_driver" ];then
    cd ${CATKIN_WS_SRC}
    git clone https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver.git
else
    cd ${CATKIN_WS_SRC}/mpu6050_driver
    git pull origin master
fi

#
# camera_ros (for RPi and Arducam cameras)
#
print_info "Cloning camera_ros ..." && sleep 1
if [ ! -d "${CATKIN_WS_SRC}/camera_ros" ];then
    cd ${CATKIN_WS_SRC}
    git clone -b noetic https://github.com/mzahana/camera_ros.git
else
    cd ${CATKIN_WS_SRC}/camera_ros
    git pull origin noetic
fi


#
# Arducam drivers
# This is to use Arducam camera array adapter
# Example product: 
#   https://www.uctronics.com/arducam-1mp-4-quadrascopic-camera-bundle-kit-for-raspberry-pi-nvidia-jetson-nano-xavier-nx.html
# Ref: https://docs.arducam.com/Raspberry-Pi-Camera/Multi-Camera-CamArray/quick-start/
#
if [ "$INSTALL_ARDUCAM" = true ]; then
    print_info "Installing Arducam drivers..." && sleep 1

    cd ${HOME}
    wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
    chmod +x install_pivariety_pkgs.sh
    ./install_pivariety_pkgs.sh -p libcamera_dev
    ./install_pivariety_pkgs.sh -p libcamera_apps

    # Path to the config.txt file
    CONFIG_FILE="/boot/config.txt"

    # The line you want to add
    LINE_TO_ADD="dtoverlay=arducam-pivariety"

    # Check if the line already exists in the file
    if ! sudo grep -q "$LINE_TO_ADD" "$CONFIG_FILE"; then
    # If the line does not exist, append it after [all]
    if sudo grep -q "\[all\]" "$CONFIG_FILE"; then
        # Using sudo with sed to append after [all]. A temporary file is used for sed's in-place editing to ensure sudo permissions are respected
        sudo sed -i "/\[all\]/a $LINE_TO_ADD" "$CONFIG_FILE"
    else
        # Using echo with sudo to append at the end of the file
        echo "$LINE_TO_ADD" | sudo tee -a "$CONFIG_FILE" > /dev/null
    fi
    echo "Line added to config.txt"
    else
    echo "Line already exists in config.txt"
    fi
else
    print_warning "Skippig installation of Arducam drivers"
fi

if [ "$BUILD_OPENVINS" = true ]; then
    
    if [ -f "${ROS_WS}/install_isolated/setup.bash" ]; then
        echo "sourcing ${ROS_WS}/install_isolated/setup.bash"
        source ${ROS_WS}/install_isolated/setup.bash
        print_info "Building $CATKIN_WS ... " && sleep 1
        cd $CATKIN_WS
        catkin build -j1
    else
        print_error "Could not find ${ROS_WS}/install_isolated/setup.bash"
        print_error "Not building $CATKIN_WS"
    fi
else
    print_warning "SKipping building $CATKIN_WS"
fi

#
# Copy launch file
#
print_info "Copying $ROOT/launch/arducam_mpu_openvins.launch to $HOME/" && sleep 1
cp $ROOT/launch/arducam_mpu_openvins.launch $HOME/
print_info "Copying $ROOT/launch/arducam_mpu.launch to $HOME/" && sleep 1
cp $ROOT/launch/arducam_mpu.launch $HOME/

#
# DONE!
#
cd $ROOT
