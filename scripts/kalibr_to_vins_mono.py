"""
Converts Kalibr calibration outputs to VINS-Mono configuration files.

This script takes calibration data from Kalibr (camera and IMU) and generates a YAML configuration file suitable for VINS-Mono. It supports handling different distortion models, specifically 'radtan' (radial-tangential) and 'equidistant'. Based on the distortion model specified in the Kalibr output, appropriate parameters are set for the VINS-Mono configuration.

Usage:
    python convert_kalibr_to_vins.py <camera_yaml> <imu_yaml> [--output <output_path>]

Arguments:
    camera_yaml    Path to the Kalibr camera calibration YAML file.
    imu_yaml       Path to the Kalibr IMU calibration YAML file.
    --output       Optional. Specifies the path to save the generated VINS-Mono configuration file.
                   If not provided, the configuration file is saved in the same directory as the camera YAML file.

Example:
    python convert_kalibr_to_vins.py cam_calib.yaml imu_calib.yaml --output vins_config.yaml

The script outputs a YAML file with the following main sections:
    - Common parameters: IMU topic, image topic, output path
    - Camera calibration: model type, camera name, image dimensions, distortion and projection parameters
    - Extrinsic parameters: rotation and translation from the camera frame to the IMU frame
    - Feature tracker parameters: settings related to feature tracking for VINS-Mono
    - Optimization parameters: solver settings for real-time performance
    - IMU parameters: noise densities and random walk parameters
    - Loop closure parameters: settings related to loop closure in VINS-Mono
    - Visualization parameters: settings for visual outputs and debugging
"""

import argparse
import yaml
import numpy as np
import os

def read_kalibr_yaml(file_path):
    with open(file_path) as file:
        return yaml.load(file, Loader=yaml.FullLoader)

def invert_homogeneous_matrix(T):
    R = T[:3, :3]
    t = T[:3, 3]
    R_transposed = R.T
    t_inverted = -np.dot(R_transposed, t)
    return R_transposed, t_inverted

def opencv_matrix(matrix):
    return {
        'rows': matrix.shape[0],
        'cols': matrix.shape[1],
        'dt': 'd',
        'data': matrix.flatten().tolist()
    }

def represent_opencv_matrix(dumper, matrix):
    return dumper.represent_mapping('tag:yaml.org,2002:map', matrix)

def represent_dict_order(dumper, data):
    if all(key in data for key in ['rows', 'cols', 'dt', 'data']):
        return represent_opencv_matrix(dumper, data)
    else:
        return dumper.represent_dict(data.items())

def generate_vins_mono_yaml(kalibr_data, imu_data, file_name):
    cam0 = kalibr_data.get('cam0', {})
    imu0 = imu_data.get('imu0', {})
    
    T_cam0_imu = np.array(cam0.get('T_cam_imu', np.identity(4)))
    R_transposed, t_inverted = invert_homogeneous_matrix(T_cam0_imu)
    
    intrinsics = cam0.get('intrinsics', [])
    distortion_coeffs = cam0.get('distortion_coeffs', [])
    resolution = cam0.get('resolution', [640, 480])
    td = cam0.get('timeshift_cam_imu', 0.0)  # Default to 0 if not specified

    distortion_model = cam0.get('distortion_model', 'radtan').lower()

    if distortion_model == 'radtan':
        model_type = 'PINHOLE'
        distortion_parameters = {
            'k1': distortion_coeffs[0],
            'k2': distortion_coeffs[1],
            'p1': distortion_coeffs[2],
            'p2': distortion_coeffs[3]
        }
        projection_parameters = {
            'fx': intrinsics[0],
            'fy': intrinsics[1],
            'cx': intrinsics[2],
            'cy': intrinsics[3]
        }
    elif distortion_model == 'equidistant':
        model_type = 'KANNALA_BRANDT'
        projection_parameters = {
            'k2': distortion_coeffs[0],
            'k3': distortion_coeffs[1],
            'k4': distortion_coeffs[2],
            'k5': distortion_coeffs[3],
            'mu': intrinsics[0],
            'mv': intrinsics[1],
            'u0': intrinsics[2],
            'v0': intrinsics[3]
        }
        distortion_parameters = {}

    vins_mono_yaml = {
        'imu_topic': imu0.get('rostopic', '/imu0'),
        'image_topic': cam0.get('rostopic', '/cam0/image_raw'),
        'output_path': "/home/shaozu/output/",
        'model_type': model_type,
        'camera_name': 'camera',
        'image_width': resolution[0],
        'image_height': resolution[1],
        'distortion_parameters': distortion_parameters,
        'projection_parameters': projection_parameters,
        'estimate_extrinsic': 0,
        'extrinsicRotation': opencv_matrix(R_transposed),
        'extrinsicTranslation': opencv_matrix(t_inverted.reshape(3,1)),
        'max_cnt': 150,
        'min_dist': 30,
        'freq': 10,
        'F_threshold': 1.0,
        'show_track': 1,
        'equalize': 1,
        'fisheye': 0,
        'max_solver_time': 0.04,
        'max_num_iterations': 8,
        'keyframe_parallax': 10.0,
        'acc_n': imu0.get('accelerometer_noise_density', 0.08),
        'gyr_n': imu0.get('gyroscope_noise_density', 0.004),
        'acc_w': imu0.get('accelerometer_random_walk', 0.00004),
        'gyr_w': imu0.get('gyroscope_bias_random_walk', 0.000002),
        'g_norm': 9.81007,
        'loop_closure': 1,
        'load_previous_pose_graph': 0,
        'fast_relocalization': 1,
        'pose_graph_save_path': "/home/vio/output/pose_graph/",
        'estimate_td': 1,
        'td': td,
        'rolling_shutter': 0,
        'rolling_shutter_tr': 0,
        'save_image': 0,
        'visualize_imu_forward': 0,
        'visualize_camera_size': 0.4
    }

    with open(file_name, 'w') as outfile:
        outfile.write("%YAML:1.0\n---\n")
        yaml.dump(vins_mono_yaml, outfile, indent=3, default_flow_style=None, sort_keys=False)

    print(f"{file_name} generated successfully.")

def main():
    parser = argparse.ArgumentParser(description="Convert Kalibr calibration data to VINS-Mono format.")
    parser.add_argument("camera_yaml", help="Path to the Kalibr camera calibration YAML file.")
    parser.add_argument("imu_yaml", help="Path to the Kalibr IMU calibration YAML file.")
    parser.add_argument("--output", help="Optional: Path to save the generated configuration file. If not provided, saves in the same directory as the camera YAML file.")

    args = parser.parse_args()

    if args.output:
        output_path = args.output
    else:
        directory = os.path.dirname(args.camera_yaml)
        output_path = os.path.join(directory, 'vins_mono_config.yaml')

    kalibr_data = read_kalibr_yaml(args.camera_yaml)
    imu_data = read_kalibr_yaml(args.imu_yaml)

    generate_vins_mono_yaml(kalibr_data, imu_data, output_path)

if __name__ == "__main__":
    main()

