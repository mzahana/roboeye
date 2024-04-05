import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'width', default_value='1280',
            description='Image width'),
        DeclareLaunchArgument(
            'height', default_value='400',
            description='image height'),
        DeclareLaunchArgument(
            'pixelformat', default_value='XBGR8888',
            description='Pixel format e.g. XBGR8888, YUVU'),
        DeclareLaunchArgument(
            'contrast', default_value='',
            description='Image contrast'),
        DeclareLaunchArgument(
            'ov_config', default_value='',
            description='Path to openvins config file'),
        # Add more arguments as needed
    ]

    # Paths to the packages where the launch files are located
    image_splitter_ros_path = get_package_share_directory('image_splitter_ros')
    mpu6050_driver_path = get_package_share_directory('mpu6050_driver')
    ov_msckf_path = get_package_share_directory('ov_msckf')

    # Run arducam and image splitter nodes in one container
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([image_splitter_ros_path, '/launch/stereo_image_split.launch.py']),
        launch_arguments={'width': LaunchConfiguration('width'),
                          'height': LaunchConfiguration('height'),
                          'contrast': LaunchConfiguration('contrast')}.items(),
    )

    # Include launch file from package_b and pass arguments to it
    mpu6050_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([mpu6050_driver_path, '/launch/mpu6050_driver.launch.py']),
    )

    # Include launch file from package_c without passing arguments
    # If you need to pass arguments, follow the pattern above
    ov_msckf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ov_msckf_path, '/launch/subscribe.launch.py']),
        launch_arguments={'config_path': LaunchConfiguration('ov_config')}.items(),
    )

    # Construct and return the LaunchDescription
    return LaunchDescription(launch_args + [camera_launch, mpu6050_driver_launch, ov_msckf_launch])
