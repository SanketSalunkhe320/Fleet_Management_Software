import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, LaserScan, Image
from nav_msgs.msg import Odometry
from flask import Flask, jsonify
from flask_cors import CORS
import threading

# ----------------------------
# Flask App
# ----------------------------
app = Flask(__name__)
CORS(app)

robot_status_data = {
    "robot_status": "IDLE",
    "imu_status": "FAULT",
    "lidar_status": "FAULT",
    "camera_status": "FAULT",
    "motor_status": "FAULT",
    "robot_position": "UNKNOWN"
}

@app.route("/status")
def get_status():
    return jsonify(robot_status_data)

# ----------------------------
# ROS2 Node
# ----------------------------
class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__('robot_status_monitor')

        self.prev_pose = None

        # ---- Store last message time ----
        self.last_imu = self.get_clock().now()
        self.last_lidar = self.get_clock().now()
        self.last_camera = self.get_clock().now()
        self.last_motor = self.get_clock().now()
        self.last_pose = self.get_clock().now()

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.camera_callback, 10)
        self.create_subscription(Odometry, '/odom_raw', self.motor_callback, 10)

        # ---- Timer to check for timeouts ----
        self.create_timer(1.0, self.health_check)   # check every 1 sec

        self.get_logger().info("✅ RobotStatusMonitor started")

    # -------- ROBOT POSITION + RUNNING / IDLE ----------
    def pose_callback(self, msg):
        pose = msg.pose.pose.position

        self.last_pose = self.get_clock().now()

        robot_status_data["robot_position"] = {
            "x": float(pose.x),
            "y": float(pose.y)
        }

        if self.prev_pose:
            dx = abs(pose.x - self.prev_pose.x)
            dy = abs(pose.y - self.prev_pose.y)
            robot_status_data["robot_status"] = "RUNNING" if dx > 0.005 or dy > 0.005 else "IDLE"
        else:
            robot_status_data["robot_status"] = "IDLE"

        self.prev_pose = pose

    # -------- IMU ----------
    def imu_callback(self, msg):
        self.last_imu = self.get_clock().now()
        robot_status_data["imu_status"] = "OK"

    # -------- LIDAR ----------
    def lidar_callback(self, msg):
        self.last_lidar = self.get_clock().now()
        robot_status_data["lidar_status"] = "OK"

    # -------- CAMERA ----------
    def camera_callback(self, msg):
        self.last_camera = self.get_clock().now()
        robot_status_data["camera_status"] = "OK"

    # -------- MOTOR ----------
    def motor_callback(self, msg):
        self.last_motor = self.get_clock().now()
        robot_status_data["motor_status"] = "OK"


# ----------------------------
# Run ROS2 in background thread
# ----------------------------
def ros_spin():
    rclpy.init()
    node = RobotStatusMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# ----------------------------
# Entry Point
# ----------------------------
if __name__ == "__main__":
    threading.Thread(target=ros_spin, daemon=True).start()
    print("🚀 Flask server running on http://127.0.0.1:5002/status")
    app.run(host="0.0.0.0", port=5002)
