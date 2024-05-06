# vins_raspberrypi
Low cost Visual Inertial Odometry (VIO) software for Raspberry Pi.

# Hardware
* Raspberry Pi 4B, 4GB RAM
* IMU: MPU6050
* Global shutter monochrome camera (for example, [OV9281](https://www.arducam.com/product/arducam-ov9281-1mp-global-shutter-noir-mono-mipi-camera-with-130deg-m12-mount-for-raspberry-pi/), OR [this](https://www.uctronics.com/arducam-ov9281-monochrome-global-shutter-camera-module-wide-angle.html))


# Setup
**NOTE** This is tested onm Raspberry Pi 4 with 4GB RAM with Bullseye OS. [Raspberry Pi OS (Legacy) with desktop](https://www.raspberrypi.com/software/operating-systems/). The ROS Noetic version works way better than the ROS2 humble version. In fact, openvins estimator tends to diverge quickly using the ROS2 version. However, a setup script is provided to install the ros2 system.

## Recommended Setup
* Use RPi OS 64-bit, [Raspberry Pi OS (Legacy) with desktop](https://www.raspberrypi.com/software/operating-systems/)
* Clone this pkg in your RPi.
    ```sh
    mkdir -p ~/src
    cd $HOME/src && git clone https://github.com/mzahana/vins_raspberrypi.git
    ```
* Enable environment variables
    ```sh
    BUILD_ROS=true
    BUILD_OPENVINS=true
    # If you are using Arducam camera hat enable the following
    INSTALL_ARDUCAM=true
    ```
* Execute the `setup_noetic_openvins_bullseye.sh` script
    ```sh
    source setup_noetic_openvins_bullseye.sh
    ```
The setup amy take long time!

* Add the following to your `.bashrc`.
    ```sh
    source /home/$USER/ros_noetic/catkin_ws/install_isolated/setup.bash
    source /home/$USER/catkin_ws/devel/setup.bash
    ```
# Run
* After the setup is done, reboot your RPi
* You will need to calibrate your IMU. Follow instructions [here](https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver)
* You will need to  calibrate your camera-imu setup using [Kalibr tool](https://github.com/ethz-asl/kalibr). It is recommended that you record a rosbag of your camera/IMU topics. You can check [launch/arducam_mpu.launch](launch/arducam_mpu.launch) on how to run camera and IMU nodes, then record a bag.  Then use Kalibr to perfrom the calibration. Do the calibration on your PC. A good calibration is important!
* Transfer the calibration data from Kalibr output to a yaml files that openvins can read. See the examples in [config/openvins](config/openvins)
* You can use [arducam_mpu_openvins.launch](launch/arducam_mpu_openvins.launch) to run the entire system.
* Run Rviz on your PC to visualize the odomery (make sure to set `ROS_MASTER_URI` and `ROS_HOSTNAME` properly).

# System Service
The camera, imu, and openvins can be run upon system startup using the the system service [arducam_stereo_mpu_openvins.service](services/arducam_stereo_mpu_openvins.service) which runs the [arducam_openvins.sh](services/arducam_openvins.sh) script that runs the required nodes. The service assumes that this shell script is located in the `$HOME` directory of the user `vio`. 
* To install the service, copy it to `/etc/systemd/system/`
* Reload `sudo systemctl daemon-reload`
* Enable the service `sudo systemctl enable arducam_stereo_mpu_openvins.service`
* Start the service `sudo systemctl start arducam_stereo_mpu_openvins.service`
* To stop the service `sudo systemctl stop arducam_stereo_mpu_openvins.service`
* To disable the service `sudo systemctl disable arducam_stereo_mpu_openvins.service`

# Useful commands
To add to `.bashrc`

```sh
if [ -f "/home/vio/ros_noetic/catkin_ws/install_isolated/setup.bash" ]; then
    source /home/vio/ros_noetic/catkin_ws/install_isolated/setup.bash
fi

if [ -f "/home/vio/catkin_ws/devel/setup.bash" ]; then
    source /home/vio/catkin_ws/devel/setup.bash
fi

if [ -f "/home/vio/mavros_ws/devel/setup.bash" ]; then
    source /home/vio/mavros_ws/devel/setup.bash
fi

export ROS_MASTER_URI=http://localhost:11311
#export ROS_HOSTNAME=192.168.8.124

# Assuming arducam_mpu_openvins is available in /etc/systemd/system/
alias run_openvins='roslaunch $HOME/arducam_mpu_openvins.launch'
alias enable_openvins_service='sudo systemctl enable arducam_stereo_mpu_openvins.service'
alias disable_openvins_service='sudo systemctl disable arducam_stereo_mpu_openvins.service'
alias start_openvins_service='sudo systemctl start arducam_stereo_mpu_openvins.service'
alias stop_openvins_service='sudo systemctl stop arducam_stereo_mpu_openvins.service'
alias status_openvins_service='sudo systemctl status arducam_stereo_mpu_openvins.service'
alias restart_openvins_service='sudo systemctl restart arducam_stereo_mpu_openvins.service'

# Assuming arducam_stereo_mpu_rovio.service is available in /etc/systemd/system/
alias run_rovio='roslaunch $HOME/launch/arducam_mpu_rovio.launch'
alias enable_rovio_service='sudo systemctl enable arducam_stereo_mpu_rovio.service'
alias disable_rovio_service='sudo systemctl disable arducam_stereo_mpu_rovio.service'
alias start_rovio_service='sudo systemctl start arducam_stereo_mpu_rovio.service'
alias stop_rovio_service='sudo systemctl stop arducam_stereo_mpu_rovio.service'
alias status_rovio_service='sudo systemctl status arducam_stereo_mpu_rovio.service'
alias restart_rovio_service='sudo systemctl restart arducam_stereo_mpu_rovio.service'

# Assuming mavros.service is available in /etc/systemd/system/
alias run_mavros='roslaunch mavros px4.launch fcu_url:=/dev/ttyAMA0:921600'
alias enable_mavros_service='sudo systemctl enable mavros.service'
alias disable_mavros_service='sudo systemctl disable mavros.service'
alias start_mavros_service='sudo systemctl start mavros.service'
alias stop_mavros_service='sudo systemctl stop mavros.service'
alias status_mavros_service='sudo systemctl status mavros.service'
alias restart_mavros_service='sudo systemctl restart mavros.service'

alias enable_all_services='sudo systemctl enable roscore.service && sudo systemctl enable mavros.service && sudo systemctl enable arducam_stereo_mpu_rovio.service && sudo systemctl enable vio_watchdog.service'
alias disble_all_services='sudo systemctl disable roscore.service && sudo systemctl disble mavros.service && sudo systemctl disable arducam_stereo_mpu_rovio.service && sudo systemctl disable vio_watchdog.service'

alias start_all_services='sudo systemctl start roscore.service && sudo systemctl start mavros.service && sudo systemctl start arducam_stereo_mpu_rovio.service && sudo systemctl start vio_watchdog.service'
alias stop_all_services='sudo systemctl stop roscore.service && sudo systemctl stop mavros.service && sudo systemctl stop arducam_stereo_mpu_rovio.service && sudo systemctl stop vio_watchdog.service'
```