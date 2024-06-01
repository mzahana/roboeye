from flask import Flask, render_template, jsonify, request
import subprocess
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud
from mavros_msgs.msg import State
import threading
import base64
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import signal
import sys
import psutil
import math

app = Flask(__name__)

odom_data = {}
image_data = None
bridge = CvBridge()
total_path_length = 0.0
last_position = None
features_count = 0
health_status = "Healthy"
health_reason = ""
system_rebooting = False
send_image = False
VIO_SERVICE="openvins_ov9281_mpu_system.service"
#VIO_SERVICE="vio_ov9281_mpu_system.service"

# MAVLink data
mav_odom_data = {}
mav_state_data = {}

# Function to calculate distance between two points
def calculate_distance(p1, p2):
    if p1 is not None and p2 is not None:
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)
    else:
        return 0.0

# Callback for odometry data
def odom_callback(msg):
    global odom_data, total_path_length, last_position, health_status, health_reason
    orientation = msg.pose.pose.orientation
    position = msg.pose.pose.position

    # Calculate traveled path length
    if last_position is not None:
        total_path_length += calculate_distance(last_position, position)
    last_position = position

    odom_data = {
        'x': position.x,
        'y': position.y,
        'z': position.z,
        'roll': 0,
        'pitch': 0,
        'yaw': 0
    }

    # Compute roll, pitch, yaw from quaternion (orientation)
    import tf.transformations as tf
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    roll, pitch, yaw = tf.euler_from_quaternion(quaternion)
    odom_data['roll'] = roll*180.0/math.pi
    odom_data['pitch'] = pitch*180.0/math.pi
    odom_data['yaw'] = yaw*180.0/math.pi

    # Health check based on odometry values
    if abs(position.x) > 1000 or abs(position.y) > 1000 or abs(position.z) > 1000:
        health_status = "Unhealthy"
        health_reason = "Odometry values diverging"
    else:
        health_status = "Healthy"
        health_reason = ""

# Callback for image data
def image_callback(msg):
    global image_data, send_image
    if not send_image:
        return
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        success, buffer = cv2.imencode('.jpg', cv_image)
        if not success:
            rospy.logwarn("Failed to encode image")
            return
        image_data = base64.b64encode(buffer).decode('utf-8')
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge error: {e}")
    except Exception as e:
        rospy.logerr(f"Error in image_callback: {e}")

# Callback for loop_features data
def loop_features_callback(msg):
    global features_count, health_status, health_reason
    features_count = len(msg.points)
    if features_count < 10:
        health_status = "Unhealthy"
        health_reason = "Low number of features"
    elif health_status != "Unhealthy":
        health_status = "Healthy"
        health_reason = ""

# Callback for MAVLink odometry data
def mav_odom_callback(msg):
    global mav_odom_data
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    mav_odom_data = {
        'x': position.x,
        'y': position.y,
        'z': position.z,
        'roll': 0,
        'pitch': 0,
        'yaw': 0
    }

    # Compute roll, pitch, yaw from quaternion (orientation)
    import tf.transformations as tf
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    roll, pitch, yaw = tf.euler_from_quaternion(quaternion)
    mav_odom_data['roll'] = roll * 180.0 / math.pi
    mav_odom_data['pitch'] = pitch * 180.0 / math.pi
    mav_odom_data['yaw'] = yaw * 180.0 / math.pi

# Callback for MAVLink state data
def mav_state_callback(msg):
    global mav_state_data
    mav_state_data = {
        'connected': msg.connected,
        'armed': msg.armed,
        'guided': msg.guided,
        'manual_input': msg.manual_input,
        'mode': msg.mode,
        'system_status': msg.system_status
    }

def ros_listener():
    rospy.Subscriber('/mavros/odometry/out', Odometry, odom_callback)
    rospy.Subscriber('/image_mono', Image, image_callback)
    rospy.Subscriber('/ov_msckf/loop_feats', PointCloud, loop_features_callback)
    rospy.Subscriber('/mavros/local_position/odom', Odometry, mav_odom_callback)
    rospy.Subscriber('/mavros/state', State, mav_state_callback)
    rospy.spin()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/odom')
def get_odom():
    return jsonify(odom_data)

@app.route('/image')
def get_image():
    return jsonify({'image': image_data})

@app.route('/update_image_status', methods=['POST'])
def update_image_status():
    global send_image
    data = request.json
    send_image = data.get('send_image', False)
    return jsonify({'status': 'success'})

@app.route('/restart_vio', methods=['POST'])
def restart_vio():
    global total_path_length, last_position, odom_data, features_count, health_status, health_reason
    # Reset variables
    total_path_length = 0.0
    last_position = None
    odom_data = {}
    features_count = 0
    health_status = "Healthy"
    health_reason = ""
    # Restart VIO service
    subprocess.call(['sudo', 'systemctl', 'restart', VIO_SERVICE])
    return '', 204

@app.route('/reboot_system', methods=['POST'])
def reboot_system():
    global system_rebooting, total_path_length, last_position, odom_data, features_count, health_status, health_reason
    # Reset variables
    system_rebooting = True
    total_path_length = 0.0
    last_position = None
    odom_data = {}
    features_count = 0
    health_status = "Healthy"
    health_reason = ""
    # Reboot system
    subprocess.call(['sudo', 'reboot'])
    return '', 204

@app.route('/shutdown_system', methods=['POST'])
def shutdown_system():
    global system_rebooting, total_path_length, last_position, odom_data, features_count, health_status, health_reason
    # Reset variables
    system_rebooting = True
    total_path_length = 0.0
    last_position = None
    odom_data = {}
    features_count = 0
    health_status = "Healthy"
    health_reason = ""
    # Shutdown system
    subprocess.call(['sudo', 'shutdown', 'now'])
    return '', 204

@app.route('/system_stats')
def system_stats():
    global system_rebooting
    cpu_usage = psutil.cpu_percent()
    memory_usage = psutil.virtual_memory().percent

    # Fetch temperature using vcgencmd
    temp_output = subprocess.check_output(['vcgencmd', 'measure_temp']).decode()
    temp = float(temp_output.split('=')[1].split('\'')[0])

    return jsonify({'cpu_usage': cpu_usage, 'memory_usage': memory_usage, 'temperature': temp, 'rebooting': system_rebooting})

@app.route('/features_count')
def get_features_count():
    return jsonify({'features_count': features_count})

@app.route('/health_status')
def get_health_status():
    # Check if the VIO service is running
    service_status = subprocess.run(['systemctl', 'is-active', '--quiet', VIO_SERVICE])
    if service_status.returncode != 0:
        return jsonify({'health_status': 'Unhealthy', 'health_reason': 'VIO service not running'})
    return jsonify({'health_status': health_status, 'health_reason': health_reason})

@app.route('/path_length')
def get_path_length():
    global total_path_length
    if total_path_length is None or math.isnan(total_path_length):
        total_path_length = 0.0
    return jsonify({'path_length': total_path_length})

@app.route('/mav_odom')
def get_mav_odom():
    return jsonify(mav_odom_data)

@app.route('/mav_state')
def get_mav_state():
    return jsonify(mav_state_data)

def run_flask_app():
    app.run(host='0.0.0.0', port=5000)

def signal_handler(sig, frame):
    print('Shutting down...')
    rospy.signal_shutdown('Signal received')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    rospy.init_node('web_interface_node', anonymous=True)
    ros_thread = threading.Thread(target=ros_listener)
    ros_thread.start()

    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.start()

    ros_thread.join()
    flask_thread.join()
