#!/bin/bash

#
# Flags: 
#
export ENABLE_CAMERA=true
export ENABLE_IMU=true
#  You need it if your stereo camera publishes stitched iamges into a single topic
export ENABLE_IMAGE_SPLITTER=true
export ENABLE_ROVIO=true
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
export CAMERA_WIDTH=1280
export CAMERA_HEIGHT=400

#
# image_splitter_ros
#
export IMG_SPLIT_CAM0_TOPIC='/left_image'
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
# rovio
#
export VIO_CONFIG_FILE='/home/vio/config/rovio_config/rovio_rpi.info'
export VIO_CAM0_CONFIG_FILE='/home/vio/config/rovio_config/arducam_cam0.yaml'
export VIO_CAM1_CONFIG_FILE='/home/vio/config/rovio_config/arducam_cam1.yaml'
export VIO_IMU_TOPIC='/imu/data_raw'
export VIO_CAM0_TOPIC='/left_image'
export VIO_CAM1_TOPIC='/right_image'
export VIO_ODOM_OUT_TOPIC='/mavros/odometry/out'
export VIO_HEALTH_MONITOR=true
export VIO_VEL_TO_CONSIDER_STATIC=0.1
export VIO_MAX_SUBSEQUENT_UNHEALTHY_UPDATES=2
export VIO_HEALTHY_FEATURE_DIST_COV=0.5
export VIO_HEALTHY_FEATURE_DIST_COV_INCREMENT=0.3
export VIO_UNHEALTHY_FEATURE_DIST_COV=10
export VIO_UNHEALTHY_VEL=6.0
export VIO_WORLD_FRAME='odom'
export VIO_MAP_FRAME='map'
export VIO_IMU_FRAME='imu'
export VIO_CAMERA_FRAME='camera'