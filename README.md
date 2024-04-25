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