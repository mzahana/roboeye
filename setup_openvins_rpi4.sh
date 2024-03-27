#!/bin/bash -e

# You can set these variables before executiong this script
#BUILD_ROS=false
#BUILD_OPENVINS=false
#INSTALL_ARDUCAM=false

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh

ROS_WS=$HOME/ros2_humble
ROS2_INSTALL=$HOME/ros2_humble/install
ROS_DISTRO=humble
VINS_WS=$HOME/vins_ws
VINS_WS_SRC=$HOME/vins_ws/src

if [ ! -d "$HOME/ros2_humble" ]; then
    print_info "Creating $HOME/ros2_humble workspace"
    mkdir -p $HOME/ros2_humble/src
fi

print_info "Installing some tools... " && sleep 1
sudo apt update && sudo apt install -y \
                        i2c-tools \
                        libi2c-dev \
                        git \
                        colcon \
                        python3-rosdep2 \
                        vcstool \
                        wget \
                        python3-flake8-docstrings \
                        python3-pip \
                        python3-pytest-cov \
                        python3-flake8-blind-except \
                        python3-flake8-builtins \
                        python3-flake8-class-newline \
                        python3-flake8-comprehensions \
                        python3-flake8-deprecated \
                        python3-flake8-import-order \
                        python3-flake8-quotes \
                        python3-pytest-repeat \
                        python3-pytest-rerunfailures \
                        python3-vcstools \
                        libx11-dev \
                        libxrandr-dev \
                        libasio-dev \
                        libtinyxml2-dev \
                        python3-rosinstall-generator

if [ "$BUILD_ROS" = true ]; then
    print_info "Building ROS2 $ROS_DISTRO"
    print_warning "This may take a few hours on Raspberry Pi with limited memory ..." && sleep 1
    cd $HOME/ros2_humble
    cp $ROOT/ros2.humble.rosinstall $HOME/ros2_humble/
    vcs import --input $HOME/ros2_humble/ros2.humble.rosinstall src
    sudo apt upgrade -y
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "rviz fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"
    colcon build --symlink-install  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w" --packages-skip-build-finished --packages-ignore-regex .*rviz.* --packages-ignore fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool
else
    print_warning "Skipping building ROS"
fi
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

pip3 install future

if [ ! -d "$HOME/vins_ws" ]; then
    print_info "Creating $HOME/vins_ws/src "
    mkdir -p $HOME/vins_ws/src
fi

#
# Cloning open_vins
#
print_info "Cloning open_vins ..." && sleep 1
if [ ! -d "${VINS_WS_SRC}/open_vins" ];then
    cd ${VINS_WS_SRC}
    git clone https://github.com/rpng/open_vins/
else
    cd ${VINS_WS_SRC}/open_vins
    git pull origin master
fi
print_info "patching ROS2Visualizer.h ..." && sleep 1
cp $ROOT/docker/patches/ROS2Visualizer.h ${VINS_WS_SRC}/open_vins/ov_msckf/src/ros/

#
# ros2_mpu6050_driver
#
print_info "Cloning ros2_mpu6050_driver ..." && sleep 1
if [ ! -d "${VINS_WS_SRC}/ros2_mpu6050_driver" ];then
    cd ${VINS_WS_SRC}
    git clone https://github.com/mzahana/ros2_mpu6050_driver.git
else
    cd ${VINS_WS_SRC}/ros2_mpu6050_driver
    git pull origin main
fi

#
# arducam_ros2
#
print_info "Cloning arducam_ros2 ..." && sleep 1
if [ ! -d "${VINS_WS_SRC}/arducam_ros2" ];then
    cd ${VINS_WS_SRC}
    git clone https://github.com/mzahana/arducam_ros2.git
else
    cd ${VINS_WS_SRC}/arducam_ros2
    git pull origin main
fi

if [ "$BUILD_OPENVINS" = true ]; then
    if [ -f "${ROS2_INSTALL}/setup.bash" ]; then
        echo "sourcing ${ROS2_INSTALL}/setup.bash"
        source ${ROS2_INSTALL}/setup.bash
        print_info "Building $VINS_WS ... " && sleep 1
        cd $VINS_WS
        MAKEFLAGS="-j1 -l1" colcon build --executor sequential  --packages-skip-build-finished --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w"
    else
        print_error "Could not find ${ROS2_INSTALL}/setup.bash"
        print_error "Not building $VINS_WS"
    fi
else
    print_warning "SKipping building $VINS_WS"
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
    CONFIG_FILE="/boot/firmware/config.txt"

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

#
# Copy rtps_udp_profile.xml
#
print_info "Copying $ROOT/docker/middleware_profiles/rtps_udp_profile.xml to $HOME" && sleep 1
cp $ROOT/docker/middleware_profiles/rtps_udp_profile.xml $HOME
###########################

cd $ROOT