#!/bin/bash -e

USERNAME=d2d

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh


# Define the image name and tag
IMAGE_NAME="mzahana/openvins_rpi"
TAG="rpi4"
FULL_IMAGE_NAME="$IMAGE_NAME:$TAG"

# Check if the image already exists locally
#docker images | grep "$IMAGE_NAME" | grep "$TAG" > /dev/null 2>&1

if [[ "$FORCE_BUILD" == "true" ]]; then
    print_info "FORCE_BUILD: Building $FULL_IMAGE_NAME ..."
	cd $ROOT/docker && make openvins-rpi4 UNAME="${USERNAME}" USER_ID=`id -u` U_GID=`id -g`
else
	# $? is a special variable that holds the exit status of the last command executed
	if docker images | grep "$IMAGE_NAME" | grep "$TAG" > /dev/null 2>&1; then
	  print_info "Image $FULL_IMAGE_NAME already exists locally."
	else
	    # Try to pull the image
	    print_info "Trying to pull $FULL_IMAGE_NAME"
	    #docker pull $FULL_IMAGE_NAME

	    # Check if the pull was successful
	    if docker pull $FULL_IMAGE_NAME; then
		    print_info "Successfully pulled $FULL_IMAGE_NAME"
	    else
            print_error "Failed to pull $FULL_IMAGE_NAME, building locally..."
            print_info "Building mzahana/d2dtracker-jetson:r${L4T_VERSION} ..."
            cd $ROOT/docker && make openvins-rpi4 UNAME="${USERNAME}" USER_ID=`id -u` U_GID=`id -g`
	    fi
	fi
fi

source $HOME/.bashrc

CONTAINER_NAME="openvins-container"
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume" ]; then
    print_info "Creating container's shared volume: $HOME/${CONTAINER_NAME}_shared_volume" && sleep 1
    mkdir $HOME/${CONTAINER_NAME}_shared_volume
fi
# print_info "copying config.sh to container shared volume at ${HOME}/${CONTAINER_NAME}_shared_volume" && sleep 1
# cp $ROOT/scripts/config.sh $HOME/${CONTAINER_NAME}_shared_volume/


#
# Create ros2_ws
#
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws" ]; then
    print_info "Creating ros2_ws in $HOME/${CONTAINER_NAME}_shared_volume" && sleep 1
    mkdir $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws
fi
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src" ]; then
    mkdir $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
fi

#
# Cloning open_vins
#
print_info "Cloning open_vins ..." && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/open_vins" ];then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone https://github.com/rpng/open_vins/
else
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/open_vins
    git pull origin master
fi
print_info "patching ROS2Visualizer.h ..." && sleep 1
cp $ROOT/docker/patches/ROS2Visualizer.h $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/open_vins/ov_msckf/src/ros/
####################### Done cloneing open_vins #####################


#
# topic_tools
#
print_info "Cloning topic_tools package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/topic_tools" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone -b humble https://github.com/ros-tooling/topic_tools.git
fi

####################### Done with topic_tools #####################



#
# rqt_tf_tree 
#
print_info "Cloning rqt_tf_tree package ... " && sleep 1
if [ ! -d "$HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src/rqt_tf_tree" ]; then
    cd $HOME/${CONTAINER_NAME}_shared_volume/ros2_ws/src
    git clone -b humble https://github.com/ros-visualization/rqt_tf_tree.git
fi

#
# install d2dtracker.service
#
# print_info "Copying d2dtracker.service to /etc/systemd/system/ " && sleep 1
# sudo cp $ROOT/services/d2dtracker.service /etc/systemd/system/
# print_info "Copying d2d_service_entrypoint.sh to $HOME/${CONTAINER_NAME}_shared_volume/ " && sleep 1
# cp $ROOT/services/d2d_service_entrypoint.sh $HOME/${CONTAINER_NAME}_shared_volume/
# sudo systemctl daemon-reload
# print_info "Enable d2dtracker.service using: sudo systemctl enable d2dtracker.service"
# print_info "Start d2dtracker.service using: sudo systemctl start d2dtracker.service" && sleep 1
####################### Done with copying d2dtracker.service ##############

#
# Copy rtps_udp_profile.xml
#
print_info "Copying $ROOT/docker/middleware_profiles/rtps_udp_profile.xml to $HOME/${CONTAINER_NAME}_shared_volume/ " && sleep 1
cp $ROOT/docker/middleware_profiles/rtps_udp_profile.xml $HOME/${CONTAINER_NAME}_shared_volume/
###########################

#
# Arducam drivers
# This is to use Arducam camera array adapter
# Example product: 
#   https://www.uctronics.com/arducam-1mp-4-quadrascopic-camera-bundle-kit-for-raspberry-pi-nvidia-jetson-nano-xavier-nx.html
# Ref: https://docs.arducam.com/Raspberry-Pi-Camera/Multi-Camera-CamArray/quick-start/
#
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

bashrc_file="$HOME/.bashrc"
line_to_check="alias openvins_container='. $ROOT/scripts/run_openvins.sh'"

if ! grep -qF "$line_to_check" "$bashrc_file"; then
    echo "$line_to_check" >> "$bashrc_file"
    print_info "openvins_container alias added to .bashrc file."
else
    print_warning "openvins_container alias already exists in .bashrc file. No changes made."
fi

echo "You can execute " && print_info "openvins_container " && echo "to start the openvins-container"
print_warning "If this is the first time you setup openvins-container on Raspberry pi, enter the container and build the ros2_ws wokspace"
print_warning "You need to reboot your RPi"
print_info "DONE!"
cd $HOME
