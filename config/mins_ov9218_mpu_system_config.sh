#!/bin/bash

#
# Flags:
#
export ENABLE_CAMERA=true
export ENABLE_IMU=true
#  You need it if your stereo camera publishes stitched iamges into a single topic
export ENABLE_IMAGE_SPLITTER=true
export ENABLE_MINS=true
export ENABLE_MAVROS=true

#
# libcamera_ros
#
export CAMERA_NAME=''
export CAMERA_ID=0
export CAMERA_STREAM_ROLE='still'
export CAMERA_PIXEL_FORMAT='RGB888'
export CAMERA_FRAME='libcamera_frame'
# export CAMERA_CALIB_URL=''
export CAMERA_WIDTH=640
export CAMERA_HEIGHT=400

#
# image_splitter_ros
#
export IMG_SPLIT_CAM0_TOPIC='/image_mono'
export IMG_SPLIT_CAM1_TOPIC='/right_image'
export IMG_SPLIT_STEREO_TOPIC='/libcamera_ros/image_raw'
export IMG_SPLIT_IS_GREY=true

#
# mpu6050_driver
#
export MPU_YAML='/home/vio/config/mpu_settings.yaml'

#
# mavros
#
export MAVROS_FCU_URL='/dev/ttyAMA0:921600'
export MAVROS_TGT_SYSTEM=1
export MAVROS_GCS_URL=''

#
# MINS
#
export MINS_CONFIG_PATH='/home/vio/config/mins_ov9218_mpu/config.yaml'
export MINS_ODOM_TOPIC='/mavros/odometry/out'
