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

# Prompt the user for confirmation
read -p "Do you want to upgrade system packages? (yes/no): " answer

# Check the user's response
if [[ $answer == "yes" ]]; then
    # Perform system upgrade
    sudo apt update && sudo apt upgrade -y
    print_info "System packages upgraded successfully."
elif [[ $answer == "no" ]]; then
    print_warning "Skipping system package upgrade."
else
    print_error "Invalid input. Please enter 'yes' or 'no'."
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

#
# NOTE: For some reason ARDUCAM needs to be installed first as it installs other pkgs needed by ROS
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
# image_splitter_ros (for splitting stereo Arducam cameras)
#
print_info "Cloning image_splitter_ros ..." && sleep 1
if [ ! -d "${CATKIN_WS_SRC}/image_splitter_ros" ];then
    cd ${CATKIN_WS_SRC}
    git clone -b noetic https://github.com/mzahana/image_splitter_ros.git
else
    cd ${CATKIN_WS_SRC}/image_splitter_ros
    git pull origin noetic
fi

if [ "$BUILD_ROS" = true ]; then
    print_info "Building ROS $ROS_DISTRO"
    print_warning "This may take a few hours on Raspberry Pi with limited memory ..." && sleep 1

# libconsole-bridge-dev \
# libpoco-dev \
# libtinyxml2-dev \
# libtinyxml-dev \
# qtbase5-dev \
# python3-sip \
# python3-pyqt5 \
# python3-empy \
    print_info "Installing build tools... " && sleep 1
    sudo apt-get update && sudo apt-get install -y \
                            python3-rosdep2 \
                            python3-rosinstall-generator \
                            python3-vcstools \
                            build-essential \
                            git \
                            python3-pip \
                            libboost-all-dev \
                            cmake \
                            libconsole-bridge-dev \
                            python3-nose \
                            libpoco-dev \
                            libtinyxml2-dev \
                            libtinyxml-dev \
                            python3-empy \
                            libcurl4-openssl-dev \
                            libgtest-dev \
                            liblz4-dev \
                            libeigen3-dev \
                            liburdfdom-headers-dev \
                            liburdfdom-dev \
                            uuid-dev \
                            libbz2-dev \
                            libgpgme-dev \
                            libopencv-dev \
                            liblog4cxx-dev \
                            libyaml-cpp-dev \
                            python3-pycryptodome \
                            tmux \
                            && sudo rm -rf /var/lib/apt/lists/* \
                            && sudo apt-get clean

    sudo pip install -U vcstool
    sudo pip3 install git+https://github.com/catkin/catkin_tools.git
    sudo pip install psutil
    sudo pip install defusedxml
    sudo pip install netifaces

    sudo rosdep init
    rosdep update

    cd $ROS_WS
    cp $ROOT/noetic-perception.rosinstall $ROS_WS/
    #rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-perception.rosinstall
    vcs import --input noetic-perception.rosinstall ./src

    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

    ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
else
    print_warning "Skipping building ROS"
fi


if [ "$BUILD_OPENVINS" = true ]; then
    if [ -f "${ROS_WS}/install_isolated/setup.bash" ]; then
        print_info "Found ${ROS_WS}/install_isolated/setup.bash" && sleep 1
    else
        print_error "Could not find ${ROS_WS}/install_isolated/setup.bash"
        print_error "Not building $CATKIN_WS. Compile ROS fisrt by setting BUILD_ROS=true before executing this script"
        exit 1
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

    # Building openvins
    print_info "sourcing ${ROS_WS}/install_isolated/setup.bash"
    source ${ROS_WS}/install_isolated/setup.bash
    print_info "Building $CATKIN_WS ... " && sleep 1
    cd $CATKIN_WS
    catkin build ov_msckf image_splitter_ros mpu6050_driver libcamera_ros

    print_info "Copying $ROOT/launch/arducam_mpu_openvins.launch to $HOME/" && sleep 1
    cp $ROOT/launch/arducam_mpu_openvins.launch $HOME/
else
    print_warning "SKipping building open_vins"
fi

if [ "$BUILD_ROVIO" = true ]; then
    if [ -f "${ROS_WS}/install_isolated/setup.bash" ]; then
        print_info "Found ${ROS_WS}/install_isolated/setup.bash" && sleep 1
    else
        print_error "Could not find ${ROS_WS}/install_isolated/setup.bash"
        print_error "Not building $CATKIN_WS. Compile ROS fisrt by setting BUILD_ROS=true before executing this script"
        exit 1
    fi
    print_info "Installing some dependencies..."
    sudo apt-get update && sudo apt-get install -y \
        libeigen3-dev \
        libboost-all-dev \
        libceres-dev \
        && sudo rm -rf /var/lib/apt/lists/* \
        && sudo apt-get clean
        
    #
    # Cloning rovio
    #
    sudo apt-get update && sudo apt-get install -y freeglut3-dev libglew-dev
    print_info "Cloning ROVIO ..." && sleep 1
    if [ ! -d "${CATKIN_WS_SRC}/rovio" ];then
        cd ${CATKIN_WS_SRC}
        git clone -b ros_noetic https://github.com/mzahana/rovio.git
        cd ${CATKIN_WS_SRC}/rovio
        git submodule update --init --recursive
    else
        cd ${CATKIN_WS_SRC}/rovio
        git pull origin ros_noetic
    fi


    #
    # Cloning kindr: required by rovio
    #
    print_info "Cloning kindr ..." && sleep 1
    if [ ! -d "${CATKIN_WS_SRC}/kindr" ];then
        cd ${CATKIN_WS_SRC}
        git clone https://github.com/ethz-asl/kindr.git
    else
        cd ${CATKIN_WS_SRC}/kindr
        git pull origin master
    fi

    # Copying rovio config files for arducam stereo cam and MPU6050
    print_info "Copying rovio_config directory to $HOME ..." && sleep 1
    if [ ! -d "${HOME}/rovio_config" ];then
        cd ${HOME}
        cp -R $ROOT/config/rovio_config $HOME/
    fi

    # Build pkgs
    echo "sourcing ${ROS_WS}/install_isolated/setup.bash"
    source ${ROS_WS}/install_isolated/setup.bash
    print_info "Building $CATKIN_WS ... " && sleep 1
    cd $CATKIN_WS
    catkin build rovio mpu6050_driver libcamera_ros image_splitter_ros -j 1 --mem-limit 50% --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=OFF -DROVIO_NCAM=1 -DROVIO_PATCHSIZE=4 -DROVIO_NMAXFEATURE=15

    print_info "Copying $ROOT/launch/arducam_mpu_rovio.launch to $HOME/" && sleep 1
    cp $ROOT/launch/arducam_mpu_rovio.launch $HOME/
else
    print_warning "SKipping building ROVIO"
fi

#
# Copy launch file
#
print_info "Copying $ROOT/launch/arducam_mpu.launch to $HOME/" && sleep 1
cp $ROOT/launch/arducam_mpu.launch $HOME/

#
# DONE!
#
cd $ROOT
