import argparse
import yaml
import numpy as np
from collections import OrderedDict

def read_kalibr_yaml(file_path):
    with open(file_path) as file:
        return yaml.load(file, Loader=yaml.FullLoader)

def invert_homogeneous_matrix(T):
    R = T[:3, :3]
    t = T[:3, 3]
    print("t: ", t)
    R_transposed = R.T
    t_inverted = -np.dot(R_transposed, t)
    print("t_inverted: ", t_inverted)
    T_inverted = np.identity(4)
    T_inverted[:3, :3] = R_transposed
    T_inverted[:3, 3] = t_inverted
    return T_inverted.flatten().tolist()

def generate_camera_yaml(cam_data, file_name):
    intrinsics = cam_data.get('intrinsics', [])
    distortion_coeffs = cam_data.get('distortion_coeffs', [0.0, 0.0, 0.0, 0.0])
    resolution = cam_data.get('resolution', [640, 480])  # Default resolution

    camera_yaml = {
        'model_type': 'PINHOLE',
        'camera_name': 'camera',
        'image_width': resolution[0],
        'image_height': resolution[1],
        'distortion_parameters': {
            'k1': distortion_coeffs[0],
            'k2': distortion_coeffs[1],
            'p1': distortion_coeffs[2],
            'p2': distortion_coeffs[3]
        },
        'projection_parameters': {
            'fx': intrinsics[0],
            'fy': intrinsics[1],
            'cx': intrinsics[2],
            'cy': intrinsics[3]
        }
    }

    with open(file_name, 'w') as outfile:
        outfile.write("%YAML:1.0\n---\n")
        yaml.dump(camera_yaml, outfile, default_flow_style=False)

    print(f"{file_name} generated successfully.")

def generate_vins_config_yaml(kalibr_data, imu_data, file_name):
    cam0 = kalibr_data.get('cam0', {})
    cam1 = kalibr_data.get('cam1', {})
    imu0 = imu_data.get('imu0', {})
    resolution0 = cam0.get('resolution', [640, 480])
    resolution1 = cam1.get('resolution', [640, 480])
    
    T_cam0_imu = np.array(cam0.get('T_cam_imu', np.identity(4)))
    T_cam1_imu = np.array(cam1.get('T_cam_imu', np.identity(4)))

    body_T_cam0 = invert_homogeneous_matrix(T_cam0_imu)
    body_T_cam1 = invert_homogeneous_matrix(T_cam1_imu)

    vins_config_yaml = {
        'imu': 1,
        'num_of_cam': 2,
        'imu_topic': imu0.get('rostopic', '/imu/data_raw'),
        'image0_topic': cam0.get('rostopic', '/camera/infra1/image_rect_raw'),
        'image1_topic': cam1.get('rostopic', '/camera/infra2/image_rect_raw'),
        'cam0_calib': "first_camera.yaml",
        'cam1_calib': "second_camera.yaml",
        'image_width': resolution0[0],
        'image_height': resolution0[1],
        'use_gpu': 0,
        'use_gpu_acc_flow': 0,
        'use_gpu_ceres': 0,
        'estimate_extrinsic': 1,
        'body_T_cam0': {
            'rows': 4,
            'cols': 4,
            'dt': 'd',
            'data': body_T_cam0
        },
        'body_T_cam1': {
            'rows': 4,
            'cols': 4,
            'dt': 'd',
            'data': body_T_cam1
        },
        'multiple_thread': 1,
        'max_cnt': 150,
        'min_dist': 30,
        'freq': 10,
        'F_threshold': 1.0,
        'show_track': 1,
        'flow_back': 1,
        'max_solver_time': 0.04,
        'max_num_iterations': 8,
        'keyframe_parallax': 10.0,
        'acc_n': imu0.get('accelerometer_noise_density', 0.1),
        'gyr_n': imu0.get('gyroscope_noise_density', 0.01),
        'acc_w': imu0.get('accelerometer_random_walk', 0.001),
        'gyr_w': imu0.get('gyroscope_random_walk', 0.0001),
        'g_norm': 9.805,
        'estimate_td': 1,
        'td': cam0.get('timeshift_cam_imu', 0.0),
        'load_previous_pose_graph': 0,
        'pose_graph_save_path': "/home/dji/output/pose_graph/",
        'save_image': 0,
    }

    # Adjustments for yaml.dump to format matrices correctly
    def represent_list(dumper, data):
        if len(data) == 16:  # Matrix data
            return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)
        return dumper.represent_list(data)

    yaml.add_representer(list, represent_list)

    with open(file_name, 'w') as outfile:
        outfile.write("%YAML:1.0\n---\n")
        yaml.dump(vins_config_yaml, outfile, default_flow_style=False, sort_keys=False)

    print(f"{file_name} generated successfully.")

def main():
    parser = argparse.ArgumentParser(description="Convert Kalibr calibration data to VINS-Fusion format.")
    parser.add_argument("camera_yaml", help="Path to the Kalibr camera calibration YAML file.")
    parser.add_argument("imu_yaml", help="Path to the Kalibr IMU calibration YAML file.")

    args = parser.parse_args()

    kalibr_data = read_kalibr_yaml(args.camera_yaml)
    imu_data = read_kalibr_yaml(args.imu_yaml)

    generate_camera_yaml(kalibr_data.get('cam0', {}), 'first_camera.yaml')
    generate_camera_yaml(kalibr_data.get('cam1', {}), 'second_camera.yaml')
    generate_vins_config_yaml(kalibr_data, imu_data, 'vins_config.yaml')

if __name__ == "__main__":
    main()
