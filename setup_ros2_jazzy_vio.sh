#!/bin/bash -e

# References: https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html
# Ref2: https://markuspetermann.net/Linux/Build_ROS_2_on_debian.md


ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/utils/print_color.sh

# Get the Codename value from the lsb_release output
codename=$(lsb_release -a 2>/dev/null | grep 'Codename:' | awk '{print $2}')

DEBIAN_VERSION=bookworm
# Check if the codename is "bullseye"
if [ "$codename" = "${DEBIAN_VERSION}" ]; then
    print_info "Running on $codename"
else
    print_error "This script requires Debian ${DEBIAN_VERSION}. Exiting."
    exit 1
fi


#References: https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html
# Ref2: https://markuspetermann.net/Linux/Build_ROS_2_on_debian.md
locale  # check for UTF-8

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools

sudo apt install -y cmake colcon git python3-colcon-argcomplete python3-colcon-bash python3-colcon-cd python3-colcon-cmake python3-colcon-core python3-colcon-defaults python3-colcon-devtools python3-colcon-library-path python3-colcon-metadata python3-colcon-notification python3-colcon-output python3-colcon-package-information python3-colcon-package-selection python3-colcon-parallel-executor python3-colcon-python-setup-py python3-colcon-recursive-crawl python3-colcon-ros python3-colcon-test-result python3-colcon-zsh python3-rosdep2 python3-rosinstall-generator python3-setuptools vcstool wget

mkdir -p ~/ros2_jazzy/src
cd ~/ros2_jazzy
#vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src
rosinstall_generator desktop_full --deps --rosdistro jazzy | vcs import src/

# You can move all the gz packages, as usually gazebo won't run efficiently on Raspberry Pi

mkdir -p ~/src/gz_ros_pkgs
mv ~/ros2_jazzy/src/*gz* ~/src/gz_ros_pkgs/


# Rust is required to build rmw_zenoh from source
# Press ENTER when prompted to proceed with default options.
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
# Clone rmw_zenoh separately
cd ~/ros2_jazzy/src
git clone https://github.com/ros2/rmw_zenoh.git -b jazzy

sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --rosdistro jazzy --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers ros-jazzy-gz-math-vendor ros-jazzy-ros-gz-bridge ros-gz-sim-demos ros-jazzy-gz-cmake-vendor"
# Sometimes the previous command insists on installing gz package although they are included in the  skip-keys
# Therefore, you can pass the --simulate flag to the previous command to see the remaning packages
# Make sure to install the remaining packages and ignore the gz packages

# Needed by MAVROS
cd ~/ros2_jazzy/src
git clone https://github.com/ros/eigen_stl_containers.git -b ros2

# Build the ROS 2 workspace
# This may take a while!
cd ~/ros2_jazzy
colcon build --symlink-install --packages-skip sdformat_vendor  ompl sdformat_test_files sdformat_urdf rviz_default_plugins


#
# OpenVINS
#

# Dependencies
sudo apt-get install -y libeigen3-dev libboost-all-dev libceres-dev

# Clone open_vins
mkdir -p ~/openvins_ws/src
cd ~/openvins_ws/src
git clone git clone https://github.com/mzahana/open_vins.git -b ros_jazzy

cd ~/openvins_ws/
MAKEFLAGS="-j1 -l1" colcon build --executor sequential  --packages-skip-build-finished  --cmake-args DCMAKE_BUILD_TYPE=Release


#
# Workspace for camera and IMU drivers
#
mkdir -p ~/drivers_ws/src
cd ~/drivers_ws/src
git clone https://github.com/mzahana/i2c_device_ros.git -b ros2_humble
git clone https://github.com/mzahana/mpu6050_driver.git -b ros2_humble


#
# TODO: Copy the Zenoh router and sessino config files
# TODO: Copy the source_ros.sh files
# TODO: Create the config folder
# TODO: Copy the system_config.sh to config folder
#

#
# camera_ros
#

#Deps
sudo apt -y install python3-colcon-meson
sudo apt install -y pkg-config libcamera-dev
cd ~/drivers_ws/src
git clone https://github.com/mzahana/camera_ros.git

#
# For AI camera
#

# deps
sudo apt install -y imx500-all imx500-tools
sudo apt install -y python3-opencv python3-munkres
sudo apt install -y python3-picamera2


#
# MAVROS
#
sudo apt update
sudo apt install -y geographiclib-tools
# Geoid (EGM96, 5’ grid)
sudo geographiclib-get-geoids egm96-5
# Gravity model (EGM96)
sudo geographiclib-get-gravity egm96
# World Magnetic Model 
sudo geographiclib-get-magnetic wmm2020

cd ~/drivers_ws/
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos
vcs import src < /tmp/mavlink.repos
vcs import src < /tmp/mavros.repos

colcon build

#
# Build MINS
#

mkdir -p ~/mins_ws/src
cd ~/mins_ws/src
git clone https://github.com/mzahana/mins.git -b ros2_jazzy
cd ~/mins_ws/
colcon build --paths src/mins/thirdparty/*
source install/setup.bash
colcon build --paths src/mins/thirdparty/open_vins/*
source install/setup.bash
colcon build --paths src/mins/mins
colcon build --paths src/mins/mins_data
source install/setup.bash
colcon build --paths src/mins/mins_eval


#
# Clone VINS-Fusion-ROS2-jazzy
#
mkdir -p vinsfusion_ws/src
cd $HOME/vinsfusion_ws/src
git clone https://github.com/mzahana/VINS-Fusion-ROS2-jazzy.git -b ros2_jazzy
cd $HOME/vinsfusion_ws/
colcon build

#
# Create config folder which contins MPU and YAML config files
#
mkdir -p $HOME/config
