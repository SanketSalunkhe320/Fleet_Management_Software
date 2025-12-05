

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from sensor_msgs.msg import Imu, LaserScan, Image
# from nav_msgs.msg import Odometry
# from std_msgs.msg import UInt16
# from flask import Flask, jsonify
# from flask_cors import CORS
# import threading
# import time

# # ----------------------------
# # Flask App
# # ----------------------------
# app = Flask(__name__)
# CORS(app)

# # ----------------------------
# # Robot Status Data
# # ----------------------------
# robot_status_data = {
#     "robot_status": "IDLE",
#     "imu_status": "FAULT",
#     "lidar_status": "FAULT",
#     "camera_status": "FAULT",
#     "motor_status": "FAULT",
#     "battery": "N/A",
#     "battery_voltage": 0.0,
#     "robot_position": {"x": 0.0, "y": 0.0}
# }

# # ----------------------------
# # Last message timestamps
# # ----------------------------
# last_msg_time = {
#     "pose": 0.0, "imu": 0.0, "lidar": 0.0,
#     "camera": 0.0, "motor": 0.0, "battery": 0.0,
# }

# # ----------------------------
# # Fault timeouts
# # ----------------------------
# FAULT_TIMEOUTS = {
#     "imu": 1.0,
#     "lidar": 2.0,
#     "camera": 2.0,
#     "motor": 1.0,
#     "battery": 3.0,
# }
# ROBOT_OFFLINE_TIMEOUT = 10.0  # seconds without any pose update → OFFLINE
# # ============================================================
# # Battery Helpers
# # ============================================================
# def convert_battery_voltage(raw):
#     return raw / 1000.0


# def convert_battery_percentage(voltage):
#     maxV = 12.6
#     minV = 9.0
#     voltage = max(minV, min(maxV, voltage))
#     return round(((voltage - minV) / (maxV - minV)) * 100)


# # ============================================================
# # ROS2 Node
# # ============================================================
# class RobotStatusMonitor(Node):
#     def __init__(self):
#         super().__init__('robot_status_monitor')
#         self.prev_pose = None

#         self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
#         self.create_subscription(Imu, '/imu', self.imu_callback, 10)
#         self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
#         self.create_subscription(Image, '/depth_cam/depth/image_raw', self.camera_callback, 10)
#         self.create_subscription(Odometry, '/odom', self.motor_callback, 10)
#         self.create_subscription(UInt16, '/ros_robot_controller/battery', self.battery_callback, 10)

#         self.get_logger().info("RobotStatusMonitor started ✔")

#     # ------------------ Callbacks ------------------
#     def pose_callback(self, msg):
#         last_msg_time["pose"] = time.time()
#         pose = msg.pose.pose.position

#         robot_status_data["robot_position"] = {"x": float(pose.x), "y": float(pose.y)}

#         if self.prev_pose:
#             dx = abs(pose.x - self.prev_pose.x)
#             dy = abs(pose.y - self.prev_pose.y)
#             robot_status_data["robot_status"] = "RUNNING" if dx > 0.005 or dy > 0.005 else "IDLE"
#         else:
#             robot_status_data["robot_status"] = "IDLE"

#         self.prev_pose = pose

#     def imu_callback(self, msg):
#         last_msg_time["imu"] = time.time()
#         robot_status_data["imu_status"] = "OK" if msg.orientation.w != 0 else "FAULT"

#     def lidar_callback(self, msg):
#         last_msg_time["lidar"] = time.time()
#         robot_status_data["lidar_status"] = "OK" if msg.ranges and len(msg.ranges) > 5 else "FAULT"

#     def camera_callback(self, msg):
#         last_msg_time["camera"] = time.time()
#         robot_status_data["camera_status"] = "OK"

#     def motor_callback(self, msg):
#         last_msg_time["motor"] = time.time()
#         linear = msg.twist.twist.linear.x
#         angular = msg.twist.twist.angular.z
#         robot_status_data["motor_status"] = "OK" if abs(linear) > 0 or abs(angular) > 0 else "FAULT"

#     def battery_callback(self, msg):
#         last_msg_time["battery"] = time.time()
#         voltage = convert_battery_voltage(msg.data)
#         robot_status_data["battery_voltage"] = round(voltage, 3)
#         robot_status_data["battery"] = convert_battery_percentage(voltage)


# # ============================================================
# # Fault Checker
# # ============================================================
# def fault_checker():
#     while True:
#         now = time.time()

#         for sensor, timeout in FAULT_TIMEOUTS.items():
#             robot_status_data[f"{sensor}_status"] = (
#                 "FAULT" if (now - last_msg_time[sensor] > timeout) else "OK"
#             )

#         # Robot status stays IDLE if no movement — never OFFLINE
#         if robot_status_data["motor_status"] != "OK":
#             robot_status_data["robot_status"] = "IDLE"

#         time.sleep(0.1)


# # ============================================================
# # Status Printer (NEW)
# # ============================================================
# def status_printer():
#     while True:
#         print("STATUS:", robot_status_data)
#         time.sleep(1)


# # ============================================================
# # Flask endpoint
# # ============================================================
# @app.route("/status")
# def get_status():
#     return jsonify(robot_status_data)


# # ============================================================
# # ROS2 Thread
# # ============================================================
# def ros_spin():
#     rclpy.init()
#     node = RobotStatusMonitor()
#     try:
#         rclpy.spin(node)
#     except Exception as e:
#         print("ROS Spin Error:", e)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# # ============================================================
# # Main
# # ============================================================
# if __name__ == "__main__":
#     threading.Thread(target=ros_spin, daemon=True).start()
#     threading.Thread(target=fault_checker, daemon=True).start()
#     threading.Thread(target=status_printer, daemon=True).start()

#     print("Flask API running: http://127.0.0.1:5002/status")
#     app.run(host="0.0.0.0", port=5002)


#!/usr/bin/env python3
"""
topicstatus.py - ROS2 → Flask status bridge (clean, thread-safe)

Features:
- ROS2 subscriptions update shared robot_status_data
- Flask serves /status (optionally restrict to a CIDR)
- Thread-safe access using a Lock
- Fault/timeouts logic preserved
"""

import os
import time
import threading
import ipaddress
from flask import Flask, jsonify, request
from flask_cors import CORS

# ROS imports (keep these at top so script fails fast if ROS not installed)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16

# ----------------------------
# Configuration
# ----------------------------
# Set this to a CIDR string to enable network filtering, or None to disable.
# Example: "192.168.149.0/24"
ALLOWED_NETWORK_STR = os.getenv("ALLOWED_NETWORK", None)

# If an environment CIDR provided, parse to ip_network; else keep None
ALLOWED_NETWORK = ipaddress.ip_network(ALLOWED_NETWORK_STR) if ALLOWED_NETWORK_STR else None

# Fault timeouts (seconds)
FAULT_TIMEOUTS = {
    "imu": 1.0,
    "lidar": 2.0,
    "camera": 2.0,
    "motor": 1.0,
    "battery": 3.0,
}
ROBOT_OFFLINE_TIMEOUT = 10.0  # seconds without pose → OFFLINE

# Flask
app = Flask(__name__)
CORS(app)

# ----------------------------
# Shared robot status (thread-safe)
# ----------------------------
robot_status_lock = threading.Lock()
robot_status_data = {
    "robot_status": "IDLE",
    "imu_status": "FAULT",
    "lidar_status": "FAULT",
    "camera_status": "FAULT",
    "motor_status": "FAULT",
    "battery": "N/A",
    "battery_voltage": 0.0,
    "robot_position": {"x": 0.0, "y": 0.0},
    "error_status": "NONE"
}

# Last message timestamps (for fault detection)
last_msg_time = {
    "pose": 0.0, "imu": 0.0, "lidar": 0.0,
    "camera": 0.0, "motor": 0.0, "battery": 0.0,
}


# ----------------------------
# Helpers
# ----------------------------
def convert_battery_voltage(raw):
    return raw / 1000.0


def convert_battery_percentage(voltage):
    maxV = 12.6
    minV = 9.0
    voltage = max(minV, min(maxV, voltage))
    return round(((voltage - minV) / (maxV - minV)) * 100)


def is_allowed_client(ip_str: str) -> bool:
    """
    Returns True when the client is permitted to read /status.
    Behavior:
      - If ALLOWED_NETWORK is None -> allow all.
      - Always allow loopback (127.0.0.1 and ::1).
      - If ALLOWED_NETWORK set -> allow only IPs within that CIDR.
    """
    if ALLOWED_NETWORK is None:
        return True

    try:
        ip_addr = ipaddress.ip_address(ip_str)
    except Exception:
        return False

    # allow loopback for local dev (e.g., browser on same machine)
    if ip_addr.is_loopback:
        return True

    return ip_addr in ALLOWED_NETWORK


# ----------------------------
# ROS2 Node
# ----------------------------
class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__("robot_status_monitor")
        self.prev_pose = None

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.create_subscription(LaserScan, "/scan_raw", self.lidar_callback, 10)
        self.create_subscription(Image, "/depth_cam/depth/image_raw", self.camera_callback, 10)
        self.create_subscription(Odometry, "/odom", self.motor_callback, 10)
        self.create_subscription(UInt16, "/ros_robot_controller/battery", self.battery_callback, 10)

        self.get_logger().info("RobotStatusMonitor started ✔")

    # ---------- Callbacks ----------
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        now = time.time()
        last_msg_time["pose"] = now
        p = msg.pose.pose.position

        with robot_status_lock:
            robot_status_data["robot_position"] = {"x": float(p.x), "y": float(p.y)}

            if self.prev_pose:
                dx = abs(p.x - self.prev_pose.x)
                dy = abs(p.y - self.prev_pose.y)
                robot_status_data["robot_status"] = "RUNNING" if (dx > 0.005 or dy > 0.005) else "IDLE"
            else:
                robot_status_data["robot_status"] = "IDLE"

        self.prev_pose = p

    def imu_callback(self, msg: Imu):
        last_msg_time["imu"] = time.time()
        with robot_status_lock:
            robot_status_data["imu_status"] = "OK" if msg.orientation.w != 0 else "FAULT"

    def lidar_callback(self, msg: LaserScan):
        last_msg_time["lidar"] = time.time()
        with robot_status_lock:
            robot_status_data["lidar_status"] = "OK" if (msg.ranges and len(msg.ranges) > 5) else "FAULT"

    def camera_callback(self, msg: Image):
        last_msg_time["camera"] = time.time()
        with robot_status_lock:
            robot_status_data["camera_status"] = "OK"

    def motor_callback(self, msg: Odometry):
        last_msg_time["motor"] = time.time()
        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z
        with robot_status_lock:
            robot_status_data["motor_status"] = "OK" if (abs(linear) > 0 or abs(angular) > 0) else "FAULT"

    def battery_callback(self, msg: UInt16):
        last_msg_time["battery"] = time.time()
        voltage = convert_battery_voltage(msg.data)
        with robot_status_lock:
            robot_status_data["battery_voltage"] = round(voltage, 3)
            robot_status_data["battery"] = convert_battery_percentage(voltage)


# ----------------------------
# Fault checker thread
# ----------------------------
def fault_checker():
    while True:
        now = time.time()
        with robot_status_lock:
            for sensor, timeout in FAULT_TIMEOUTS.items():
                status_key = f"{sensor}_status"
                last_t = last_msg_time.get(sensor, 0.0)
                robot_status_data[status_key] = "FAULT" if (now - last_t > timeout) else "OK"

            # If motors not OK, set robot to IDLE (keep offline logic at frontend/backend level)
            if robot_status_data.get("motor_status") != "OK":
                robot_status_data["robot_status"] = "IDLE"
        time.sleep(0.1)


# ----------------------------
# Simple status printer (for debugging)
# ----------------------------
def status_printer():
    while True:
        with robot_status_lock:
            print("STATUS:", robot_status_data)
        time.sleep(1)


# ----------------------------
# Flask endpoints
# ----------------------------
@app.route("/status")
def get_status():
    client_ip = request.remote_addr
    if not is_allowed_client(client_ip):
        app.logger.warning("Unauthorized /status request from %s", client_ip)
        # Return sanitized OFFLINE view
        return jsonify({
            "robot_status": "OFFLINE",
            "imu_status": "FAULT",
            "lidar_status": "FAULT",
            "camera_status": "FAULT",
            "motor_status": "FAULT",
            "battery": "N/A",
            "battery_voltage": 0.0,
            "robot_position": None,
            "error_status": "UNAUTHORIZED_NETWORK"
        })

    # Authorized — return current snapshot
    with robot_status_lock:
        # Return a shallow copy to avoid exposing lock-held structure
        snapshot = dict(robot_status_data)
    return jsonify(snapshot)


# ----------------------------
# ROS spinning thread
# ----------------------------
def ros_spin():
    try:
        rclpy.init()
        node = RobotStatusMonitor()
        try:
            rclpy.spin(node)
        except Exception as e:
            node.get_logger().error("ROS spin error: %s", str(e))
        finally:
            node.destroy_node()
    except Exception as exc:
        # if ROS cannot initialize, log and exit thread
        print("Failed to initialize ROS:", exc)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    # Start threads
    threading.Thread(target=ros_spin, daemon=True).start()
    threading.Thread(target=fault_checker, daemon=True).start()
    threading.Thread(target=status_printer, daemon=True).start()

    print("Flask API running: http://127.0.0.1:5002/status (listening on 0.0.0.0:5002)")
    # Run Flask
    app.run(host="0.0.0.0", port=5002)
