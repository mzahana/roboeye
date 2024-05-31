from flask import Flask, render_template, jsonify
import subprocess
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import threading
import base64
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import signal
import sys

app = Flask(__name__)

odom_data = {}
image_data = None
bridge = CvBridge()

def odom_callback(msg):
    global odom_data
    orientation = msg.pose.pose.orientation
    position = msg.pose.pose.position
    odom_data = {
        'x': position.x,
        'y': position.y,
        'z': position.z,
        'roll': 0,  # Placeholder, will calculate below
        'pitch': 0,  # Placeholder, will calculate below
        'yaw': 0,  # Placeholder, will calculate below
    }
    # Compute roll, pitch, yaw from quaternion (orientation)
    import tf.transformations as tf
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    roll, pitch, yaw = tf.euler_from_quaternion(quaternion)
    odom_data['roll'] = roll
    odom_data['pitch'] = pitch
    odom_data['yaw'] = yaw

def image_callback(msg):
    global image_data
    try:
        # Convert the ROS Image message to a numpy array using cv_bridge
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        # Encode the image to JPEG format
        success, buffer = cv2.imencode('.jpg', cv_image)
        if not success:
            rospy.logwarn("Failed to encode image")
            return
        # Convert the buffer to base64
        image_data = base64.b64encode(buffer).decode('utf-8')
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge error: {e}")
    except Exception as e:
        rospy.logerr(f"Error in image_callback: {e}")

def ros_listener():
    rospy.Subscriber('/mavros/odometry/out', Odometry, odom_callback)
    rospy.Subscriber('/image_mono', Image, image_callback)
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

@app.route('/restart_vio', methods=['POST'])
def restart_vio():
    subprocess.call(['sudo', 'systemctl', 'restart', 'openvins_ov9281_mpu_system.service'])
    return '', 204

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
