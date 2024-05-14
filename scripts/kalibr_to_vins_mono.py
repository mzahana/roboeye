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
    """ Ensure matrices are represented as a single-line array. """
    return dumper.represent_mapping('tag:yaml.org,2002:map', matrix)

def represent_dict_order(dumper, data):
    """ Enforce multiline dictionary representation for simpler dictionaries. """
    if all(key in data for key in ['rows', 'cols', 'dt', 'data']):  # Check for matrix pattern
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

    vins_mono_yaml = {
        'imu_topic': imu0.get('rostopic', '/imu0'),
        'image_topic': cam0.get('rostopic', '/cam0/image_raw'),
        'output_path': "/home/shaozu/output/",
        'model_type': 'PINHOLE',
        'camera_name': 'camera',
        'image_width': resolution[0],
        'image_height': resolution[1],
        'distortion_parameters': {
            'k1': distortion_coeffs[0],
            'k2': distortion_coeffs[1],
            'p1': distortion_coeffs[2],
            'p2': distortion_coeffs[3],
        },
        'projection_parameters': {
            'fx': intrinsics[0],
            'fy': intrinsics[1],
            'cx': intrinsics[2],
            'cy': intrinsics[3],
        },
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

    #yaml.add_representer(dict, represent_dict_order)

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

    # Set output file path
    if args.output:
        output_path = args.output
    else:
        # Save in the same directory as the camera YAML file
        directory = os.path.dirname(args.camera_yaml)
        output_path = os.path.join(directory, 'vins_mono_config.yaml')

    kalibr_data = read_kalibr_yaml(args.camera_yaml)
    imu_data = read_kalibr_yaml(args.imu_yaml)

    generate_vins_mono_yaml(kalibr_data, imu_data, output_path)

if __name__ == "__main__":
    main()