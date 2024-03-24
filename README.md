# vins_raspberrypi
Visual inertial odometry software for Raspberry Pi

# Setup
This is tested onm Raspberry Pi 4 with bookworm OS

* Use RPi OS 64-bit 
* Increase swap in `/etc/dphys-swapfile` file. Modify the `CONF_SWAPSIZE` according to your needs. For example, `CONF_SWAPSIZE=2048`
* Clone this pkg in your RPi
* Execute the `setup_openvins_rpi4.sh

# Run
* After the setup is done, reboot your RPi
* Execute the alias `openvins_container` to enter the container
* Inside the container, `source ~/shared_volume/config.sh`
* Build the `~/shared_volume/ros2_ws` inside the container. You may need to use
    ```bash
    colcon build  --parallel-workers 1 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
* You will need to  calibrate your camera-imu setup using Kalibr tool
* Place the calibratio Yaml files inside the shared volume
* Update the env variable inside the `config.sh` that is inside the shared volume
* source `config.sh`, and run openvins