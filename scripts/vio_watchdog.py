import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import rovio.srv  # Import the service definitions from the rovio package

class ShockDetector:
    def __init__(self):
        rospy.init_node('vio_watchdog', anonymous=True)

        # Subscribers
        self.imu_sub = rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('/mavros/odometry/out', Odometry, self.odom_callback)

        # Publishers
        self.odom_pub = rospy.Publisher('/reinit_odom', Odometry, queue_size=10)
        self.jerk_pub = rospy.Publisher('/jerk_magnitude', Float64, queue_size=10)

        self.last_good_odom = None
        self.last_acceleration = {'x': 0, 'y': 0, 'z': 0}
        self.last_timestamp = None

        # Filter and detection parameters
        self.alpha = 0.1  # Smoothing factor for LPF on acceleration
        self.jerk_threshold = 250.0  # Threshold for detecting shock in m/s^3

        # Service Client for resetting odometry
        rospy.wait_for_service('/rovio/reset_to_pose')
        self.reset_to_pose = rospy.ServiceProxy('/rovio/reset_to_pose', rovio.srv.SrvResetToPose)

    def call_reset_service(self, pose):
        try:
            response = self.reset_to_pose(pose)
            rospy.loginfo("Service call successful, response: %s", response)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def imu_callback(self, data):
        current_timestamp = data.header.stamp
        if self.last_timestamp is None:
            self.last_timestamp = current_timestamp
            return

        dt = (current_timestamp - self.last_timestamp).to_sec()
        self.last_timestamp = current_timestamp

        if dt == 0:
            return

        # Current filtered acceleration
        filtered_acc_x = self.last_acceleration['x'] * (1 - self.alpha) + data.linear_acceleration.x * self.alpha
        filtered_acc_y = self.last_acceleration['y'] * (1 - self.alpha) + data.linear_acceleration.y * self.alpha
        filtered_acc_z = self.last_acceleration['z'] * (1 - self.alpha) + data.linear_acceleration.z * self.alpha

        # Calculate jerk
        jerk_x = (filtered_acc_x - self.last_acceleration['x']) / dt
        jerk_y = (filtered_acc_y - self.last_acceleration['y']) / dt
        jerk_z = (filtered_acc_z - self.last_acceleration['z']) / dt

        # Update last acceleration values
        self.last_acceleration = {'x': filtered_acc_x, 'y': filtered_acc_y, 'z': filtered_acc_z}

        # Calculate jerk magnitude
        jerk_magnitude = (jerk_x**2 + jerk_y**2 + jerk_z**2)**0.5

        # Publish jerk magnitude
        self.jerk_pub.publish(jerk_magnitude)

        # Check if jerk exceeds threshold
        if jerk_magnitude > self.jerk_threshold:
            rospy.loginfo("Shock detected! Publishing last good odom for reinitialization.")
            if self.last_good_odom:
                reset_pose = Pose()
                reset_pose.position = self.last_good_odom.pose.pose.position
                reset_pose.orientation = self.last_good_odom.pose.pose.orientation
                self.call_reset_service(reset_pose)
                self.odom_pub.publish(self.last_good_odom)

    def odom_callback(self, data):
        self.last_good_odom = data

if __name__ == '__main__':
    try:
        shock_detector = ShockDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass