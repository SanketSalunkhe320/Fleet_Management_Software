

# #!/usr/bin/env python3
# """
# UNIFIED AMR SYSTEM API - PORT 5000

# Combines all three applications into one:
# 1. Map Database API (app.py) - Port 5000
# 2. ROS2 AMCL Position Reader (app1.py) - Port 5001 
# 3. ROS2 Robot Status Monitor (topicstatus.py) - Port 5002

# WITH DATABASE STORAGE FOR AMCL POSITIONS

# All running on single port: 5000
# """

# from flask import Flask, request, jsonify
# from flask_cors import CORS
# from flask_sqlalchemy import SQLAlchemy
# from datetime import datetime
# import sqlite3
# import json
# import os
# import logging
# import re
# import math
# import threading
# import time
# import ipaddress
# import subprocess

# # ROS2 imports (optional - will gracefully degrade if not available)
# ROS_AVAILABLE = False
# try:
#     import rclpy
#     from rclpy.node import Node
#     from geometry_msgs.msg import PoseWithCovarianceStamped
#     from sensor_msgs.msg import Imu, LaserScan, Image
#     from nav_msgs.msg import Odometry
#     from std_msgs.msg import UInt16
#     ROS_AVAILABLE = True
# except ImportError as e:
#     ROS_AVAILABLE = False

# # ============================================================================
# # Configuration
# # ============================================================================

# # Configure logging
# logging.basicConfig(
#     level=logging.INFO,
#     format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
# )
# logger = logging.getLogger(__name__)

# # Create Flask app
# app = Flask(__name__)
# CORS(app)

# # Database configuration for robot positions
# app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///robot_positions.db'
# app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
# db = SQLAlchemy(app)

# # Database Model for robot positions
# class RobotPosition(db.Model):
#     id = db.Column(db.Integer, primary_key=True)
#     x = db.Column(db.Float, nullable=False)
#     y = db.Column(db.Float, nullable=False)
#     yaw = db.Column(db.Float, nullable=False)
#     timestamp = db.Column(db.DateTime, nullable=False, default=datetime.utcnow)
#     header_stamp = db.Column(db.String(50))
#     message_count = db.Column(db.Integer)
#     created_at = db.Column(db.DateTime, nullable=False, default=datetime.utcnow)
    
#     def to_dict(self):
#         return {
#             'id': self.id,
#             'x': self.x,
#             'y': self.y,
#             'yaw': self.yaw,
#             'timestamp': self.timestamp.isoformat() if self.timestamp else None,
#             'header_stamp': self.header_stamp,
#             'message_count': self.message_count,
#             'created_at': self.created_at.isoformat() if self.created_at else None
#         }

# # Map annotation database configuration
# MAP_DATABASE_DIR = 'map_databases'

# # Network filtering (optional)
# ALLOWED_NETWORK_STR = os.getenv("ALLOWED_NETWORK", None)
# ALLOWED_NETWORK = ipaddress.ip_network(ALLOWED_NETWORK_STR) if ALLOWED_NETWORK_STR else None

# # Fault timeouts (seconds)
# FAULT_TIMEOUTS = {
#     "imu": 1.0,
#     "lidar": 2.0,
#     "camera": 2.0,
#     "motor": 1.0,
#     "battery": 3.0,
# }

# # ============================================================================
# # Global Data Stores (Thread-safe)
# # ============================================================================

# # AMCL Position data (from app1.py)
# amcl_position_lock = threading.Lock()
# latest_robot_position = {
#     'x': 0.0,
#     'y': 0.0,
#     'yaw': 0.0,
#     'timestamp': 0,
#     'message_count': 0,
#     'status': 'waiting'
# }

# # Robot Status data (from topicstatus.py)
# robot_status_lock = threading.Lock()
# robot_status_data = {
#     "robot_status": "IDLE",
#     "imu_status": "FAULT",
#     "lidar_status": "FAULT",
#     "camera_status": "FAULT",
#     "motor_status": "FAULT",
#     "battery": "N/A",
#     "battery_voltage": 0.0,
#     "robot_position": {"x": 0.0, "y": 0.0},
#     "error_status": "NONE"
# }

# # Last message timestamps (from topicstatus.py)
# last_msg_time = {
#     "pose": 0.0, "imu": 0.0, "lidar": 0.0,
#     "camera": 0.0, "motor": 0.0, "battery": 0.0,
# }

# # ============================================================================
# # Helper Functions for Map Databases
# # ============================================================================

# def sanitize_map_name(map_name):
#     """Sanitize map name for use as filename"""
#     sanitized = re.sub(r'[^\w\-_\. ]', '', map_name)
#     sanitized = sanitized.replace(' ', '_')
#     if not sanitized:
#         sanitized = 'default_map'
#     return sanitized

# def get_map_database_path(map_name):
#     """Get database path for a specific map"""
#     if not os.path.exists(MAP_DATABASE_DIR):
#         os.makedirs(MAP_DATABASE_DIR)
    
#     safe_map_name = sanitize_map_name(map_name)
#     database_name = f"{safe_map_name}.db"
#     return os.path.join(MAP_DATABASE_DIR, database_name)

# def get_map_db_connection(map_name):
#     """Get database connection for a specific map with error handling"""
#     try:
#         db_path = get_map_database_path(map_name)
#         conn = sqlite3.connect(db_path)
#         conn.row_factory = sqlite3.Row
#         return conn
#     except sqlite3.Error as e:
#         logger.error(f"Database connection error for map '{map_name}': {e}")
#         raise

# def init_map_database_for_map(map_name):
#     """Initialize database for a specific map with simplified schema"""
#     try:
#         conn = get_map_db_connection(map_name)
        
#         # Create simplified tables with only essential columns
#         tables = {
#             'stations': '''
#                 CREATE TABLE IF NOT EXISTS stations (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     node_type TEXT NOT NULL DEFAULT 'station',
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     z REAL DEFAULT 0.0
#                 )
#             ''',
#             'docking_points': '''
#                 CREATE TABLE IF NOT EXISTS docking_points (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     node_type TEXT NOT NULL DEFAULT 'docking',
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     z REAL DEFAULT 0.0
#                 )
#             ''',
#             'waypoints': '''
#                 CREATE TABLE IF NOT EXISTS waypoints (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     node_type TEXT NOT NULL DEFAULT 'waypoint',
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     z REAL DEFAULT 0.0
#                 )
#             ''',
#             'home_positions': '''
#                 CREATE TABLE IF NOT EXISTS home_positions (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     node_type TEXT NOT NULL DEFAULT 'home',
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     z REAL DEFAULT 0.0
#                 )
#             ''',
#             'charging_stations': '''
#                 CREATE TABLE IF NOT EXISTS charging_stations (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     node_type TEXT NOT NULL DEFAULT 'charging',
#                     x REAL NOT NULL,
#                     y REAL NOT NULL,
#                     z REAL DEFAULT 0.0
#                 )
#             ''',
#             'arrows': '''
#                 CREATE TABLE IF NOT EXISTS arrows (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     from_node_id TEXT NOT NULL,
#                     to_node_id TEXT NOT NULL
#                 )
#             ''',
#             'zones': '''
#                 CREATE TABLE IF NOT EXISTS zones (
#                     id INTEGER PRIMARY KEY AUTOINCREMENT,
#                     map_name TEXT NOT NULL,
#                     name TEXT NOT NULL,
#                     zone_type TEXT DEFAULT 'normal',
#                     points_json TEXT NOT NULL
#                 )
#             '''
#         }
        
#         for table_name, create_sql in tables.items():
#             conn.execute(create_sql)
#             logger.info(f"✅ Table '{table_name}' initialized for map '{map_name}'")
        
#         conn.commit()
#         conn.close()
#         logger.info(f"✅ Database initialization completed for map '{map_name}'")
        
#     except Exception as e:
#         logger.error(f"❌ Database initialization failed for map '{map_name}': {e}")
#         raise

# def convert_battery_voltage(raw):
#     return raw / 1000.0

# def convert_battery_percentage(voltage):
#     maxV = 12.6
#     minV = 9.0
#     voltage = max(minV, min(maxV, voltage))
#     return round(((voltage - minV) / (maxV - minV)) * 100)

# def is_allowed_client(ip_str: str) -> bool:
#     """Check if client IP is allowed (from topicstatus.py)"""
#     if ALLOWED_NETWORK is None:
#         return True

#     try:
#         ip_addr = ipaddress.ip_address(ip_str)
#     except Exception:
#         return False

#     # allow loopback for local dev
#     if ip_addr.is_loopback:
#         return True

#     return ip_addr in ALLOWED_NETWORK

# # ============================================================================
# # ROS2 Nodes (if available)
# # ============================================================================

# if ROS_AVAILABLE:
#     # From app1.py - AMCL Pose Listener with Database Storage
#     class AmclPoseListener(Node):
#         def __init__(self):
#             super().__init__('amcl_pose_listener')
#             self.subscription = self.create_subscription(
#                 PoseWithCovarianceStamped,
#                 '/amcl_pose',
#                 self.listener_callback,
#                 10)
#             self.message_count = 0
#             self.get_logger().info("🎯 AMCL Pose Listener Started")
#             self.get_logger().info("📡 Waiting for ROS bag data on /amcl_pose...")

#         def listener_callback(self, msg):
#             try:
#                 # Extract position
#                 x = msg.pose.pose.position.x
#                 y = msg.pose.pose.position.y

#                 # Extract orientation (quaternion → yaw)
#                 q = msg.pose.pose.orientation
#                 yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
#                                  1.0 - 2.0 * (q.y**2 + q.z**2))
#                 yaw_degrees = math.degrees(yaw)

#                 # Update global position
#                 self.message_count += 1
#                 with amcl_position_lock:
#                     latest_robot_position.update({
#                         'x': x,
#                         'y': y,
#                         'yaw': yaw_degrees,
#                         'timestamp': time.time(),
#                         'message_count': self.message_count,
#                         'header_stamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
#                         'status': 'success'
#                     })

#                 # Save to database
#                 with app.app_context():
#                     position = RobotPosition(
#                         x=x,
#                         y=y,
#                         yaw=yaw_degrees,
#                         timestamp=datetime.utcnow(),
#                         header_stamp=f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
#                         message_count=self.message_count
#                     )
#                     db.session.add(position)
#                     db.session.commit()
#                     print(f"💾 Saved position #{self.message_count} to database (ID: {position.id})")

#                 print(f"🎯 AMCL #{self.message_count:02d}: x={x:.3f}, y={y:.3f}, yaw={yaw_degrees:.1f}°")
                
#             except Exception as e:
#                 print(f"❌ Error processing AMCL message: {e}")
#                 with amcl_position_lock:
#                     latest_robot_position['status'] = 'error'

#     # From topicstatus.py - Robot Status Monitor
#     class RobotStatusMonitor(Node):
#         def __init__(self):
#             super().__init__("robot_status_monitor")
#             self.prev_pose = None

#             # Subscribers
#             self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
#             self.create_subscription(Imu, "/imu", self.imu_callback, 10)
#             self.create_subscription(LaserScan, "/scan_raw", self.lidar_callback, 10)
#             self.create_subscription(Image, "/depth_cam/depth/image_raw", self.camera_callback, 10)
#             self.create_subscription(Odometry, "/odom", self.motor_callback, 10)
#             self.create_subscription(UInt16, "/ros_robot_controller/battery", self.battery_callback, 10)

#             self.get_logger().info("RobotStatusMonitor started ✔")

#         def pose_callback(self, msg: PoseWithCovarianceStamped):
#             now = time.time()
#             last_msg_time["pose"] = now
#             p = msg.pose.pose.position

#             with robot_status_lock:
#                 robot_status_data["robot_position"] = {"x": float(p.x), "y": float(p.y)}

#                 if self.prev_pose:
#                     dx = abs(p.x - self.prev_pose.x)
#                     dy = abs(p.y - self.prev_pose.y)
#                     robot_status_data["robot_status"] = "RUNNING" if (dx > 0.005 or dy > 0.005) else "IDLE"
#                 else:
#                     robot_status_data["robot_status"] = "IDLE"

#             self.prev_pose = p

#         def imu_callback(self, msg: Imu):
#             last_msg_time["imu"] = time.time()
#             with robot_status_lock:
#                 robot_status_data["imu_status"] = "OK" if msg.orientation.w != 0 else "FAULT"

#         def lidar_callback(self, msg: LaserScan):
#             last_msg_time["lidar"] = time.time()
#             with robot_status_lock:
#                 robot_status_data["lidar_status"] = "OK" if (msg.ranges and len(msg.ranges) > 5) else "FAULT"

#         def camera_callback(self, msg: Image):
#             last_msg_time["camera"] = time.time()
#             with robot_status_lock:
#                 robot_status_data["camera_status"] = "OK"

#         def motor_callback(self, msg: Odometry):
#             last_msg_time["motor"] = time.time()
#             linear = msg.twist.twist.linear.x
#             angular = msg.twist.twist.angular.z
#             with robot_status_lock:
#                 robot_status_data["motor_status"] = "OK" if (abs(linear) > 0 or abs(angular) > 0) else "FAULT"

#         def battery_callback(self, msg: UInt16):
#             last_msg_time["battery"] = time.time()
#             voltage = convert_battery_voltage(msg.data)
#             with robot_status_lock:
#                 robot_status_data["battery_voltage"] = round(voltage, 3)
#                 robot_status_data["battery"] = convert_battery_percentage(voltage)

#     # ============================================================================
#     # ROS2 Background Threads
#     # ============================================================================

#     def ros_spin_combined():
#         """Run both ROS2 nodes in the same thread (from app1.py + topicstatus.py)"""
#         try:
#             rclpy.init()
            
#             # Create both nodes
#             amcl_node = AmclPoseListener()
#             status_node = RobotStatusMonitor()
            
#             print("🚀 Starting combined ROS2 nodes...")
#             print("📡 Listening for:")
#             print("   - AMCL pose on /amcl_pose")
#             print("   - IMU on /imu")
#             print("   - Lidar on /scan_raw")
#             print("   - Camera on /depth_cam/depth/image_raw")
#             print("   - Odometry on /odom")
#             print("   - Battery on /ros_robot_controller/battery")
#             print("💾 AMCL positions will be saved to database")
            
#             # Create an executor to handle multiple nodes
#             executor = rclpy.executors.MultiThreadedExecutor()
#             executor.add_node(amcl_node)
#             executor.add_node(status_node)
            
#             try:
#                 executor.spin()
#             except KeyboardInterrupt:
#                 print("\n🛑 Shutting down ROS2 nodes...")
#             except Exception as e:
#                 print(f"❌ ROS2 executor error: {e}")
#             finally:
#                 executor.shutdown()
#                 amcl_node.destroy_node()
#                 status_node.destroy_node()
#                 rclpy.shutdown()
                
#         except Exception as e:
#             print(f"❌ Failed to initialize ROS2: {e}")

#     def fault_checker():
#         """Check for sensor faults based on timeouts (from topicstatus.py)"""
#         while True:
#             now = time.time()
#             with robot_status_lock:
#                 for sensor, timeout in FAULT_TIMEOUTS.items():
#                     status_key = f"{sensor}_status"
#                     last_t = last_msg_time.get(sensor, 0.0)
#                     robot_status_data[status_key] = "FAULT" if (now - last_t > timeout) else "OK"

#                 # If motors not OK, set robot to IDLE
#                 if robot_status_data.get("motor_status") != "OK":
#                     robot_status_data["robot_status"] = "IDLE"
#             time.sleep(0.1)

#     def status_printer():
#         """Print robot status periodically (from topicstatus.py)"""
#         while True:
#             with robot_status_lock:
#                 print("🤖 ROBOT STATUS:", robot_status_data)
#             time.sleep(1)

# # ============================================================================
# # Flask Routes - Unified API (ALL ON PORT 5000)
# # ============================================================================

# @app.route('/')
# def index():
#     """Root endpoint with API information"""
#     return jsonify({
#         'status': 'success',
#         'message': 'Unified AMR Map Editor & Robot Monitoring API - PORT 5000',
#         'services': {
#             'map_database': 'Available',
#             'robot_position': 'Available' if ROS_AVAILABLE else 'Disabled (ROS2 not found)',
#             'robot_status': 'Available' if ROS_AVAILABLE else 'Disabled (ROS2 not found)',
#             'position_database': 'Available (robot_positions.db)'
#         },
#         'endpoints': {
#             # Map Database Endpoints (from app.py)
#             'save_nodes': 'POST /save-nodes',
#             'load_nodes': 'GET /load-nodes',
#             'clear_nodes': 'DELETE /clear-nodes',
#             'list_maps': 'GET /list-maps',
#             'delete_map': 'DELETE /delete-map',
            
#             # Robot Position Endpoints (from app1.py)
#             'robot_position': 'GET /robot_position',
#             'position_status': 'GET /position_status',
#             'position_health': 'GET /position_health',
            
#             # Robot Status Endpoints (from topicstatus.py)
#             'robot_status': 'GET /status',
#             'system_health': 'GET /health',
#             'debug': 'GET /debug',
#             'topics': 'GET /topics',
            
#             # Database Endpoints (from app1.py database features)
#             'positions': 'GET /positions',
#             'positions_count': 'GET /positions/count',
#             'positions_latest': 'GET /positions/latest',
#             'positions_export': 'GET /positions/export',
#             'positions_delete': 'DELETE /positions/delete/<id>',
#             'positions_clear': 'DELETE /positions/clear'
#         },
#         'version': '4.0.0',
#         'port': 5000,
#         'ros_available': ROS_AVAILABLE,
#         'databases': {
#             'robot_positions': 'robot_positions.db',
#             'map_annotations': f'{MAP_DATABASE_DIR}/'
#         }
#     })

# # ============================================================================
# # Map Database Endpoints (from app.py)
# # ============================================================================

# @app.route('/health')
# def health_check():
#     """Health check endpoint (from app.py)"""
#     try:
#         # Test with default map
#         db_path = get_map_database_path('default')
#         if not os.path.exists(db_path):
#             init_map_database_for_map('default')
        
#         conn = sqlite3.connect(db_path)
#         conn.execute('SELECT 1')
#         conn.close()
        
#         return jsonify({
#             'status': 'success',
#             'message': 'API is healthy',
#             'database_system': 'per-map databases',
#             'database_directory': MAP_DATABASE_DIR,
#             'position_database': 'robot_positions.db',
#             'timestamp': datetime.now().isoformat()
#         })
#     except Exception as e:
#         return jsonify({
#             'status': 'error',
#             'message': f'Health check failed: {str(e)}'
#         }), 500

# @app.route('/save-nodes', methods=['POST'])
# def save_nodes():
#     """Save nodes, arrows, and zones to database (from app.py)"""
#     try:
#         data = request.get_json()
#         if not data:
#             return jsonify({
#                 'status': 'error',
#                 'message': 'No JSON data provided'
#             }), 400
        
#         map_name = data.get('mapName', 'default')
        
#         # Initialize database for this map if it doesn't exist
#         db_path = get_map_database_path(map_name)
#         if not os.path.exists(db_path):
#             init_map_database_for_map(map_name)
        
#         nodes = data.get('nodes', [])
#         arrows = data.get('arrows', [])
#         zones = data.get('zones', [])
        
#         logger.info(f"💾 Saving {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones for map '{map_name}'")
        
#         conn = get_map_db_connection(map_name)
#         saved_counts = {
#             'stations': 0,
#             'docking_points': 0,
#             'waypoints': 0,
#             'home_positions': 0,
#             'charging_stations': 0,
#             'arrows': 0,
#             'zones': 0
#         }
        
#         # Clear existing data for this map
#         tables_to_clear = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
#         for table in tables_to_clear:
#             result = conn.execute(f'DELETE FROM {table} WHERE map_name = ?', (map_name,))
#             logger.info(f"🧹 Cleared {result.rowcount} records from {table}")
        
#         # Save nodes to respective tables
#         for node in nodes:
#             table_name = None
#             if node.get('type') == 'station':
#                 table_name = 'stations'
#                 saved_counts['stations'] += 1
#             elif node.get('type') == 'docking':
#                 table_name = 'docking_points'
#                 saved_counts['docking_points'] += 1
#             elif node.get('type') == 'waypoint':
#                 table_name = 'waypoints'
#                 saved_counts['waypoints'] += 1
#             elif node.get('type') == 'home':
#                 table_name = 'home_positions'
#                 saved_counts['home_positions'] += 1
#             elif node.get('type') == 'charging':
#                 table_name = 'charging_stations'
#                 saved_counts['charging_stations'] += 1
            
#             if table_name:
#                 conn.execute(
#                     f'INSERT INTO {table_name} (map_name, node_type, x, y, z) VALUES (?, ?, ?, ?, ?)',
#                     (map_name, node.get('type', ''), node.get('rosX', 0), node.get('rosY', 0), 0.0)
#                 )
        
#         # Save arrows
#         for arrow in arrows:
#             conn.execute(
#                 'INSERT INTO arrows (map_name, from_node_id, to_node_id) VALUES (?, ?, ?)',
#                 (map_name, arrow.get('fromId', ''), arrow.get('toId', ''))
#             )
#             saved_counts['arrows'] += 1
        
#         # Save zones with zone_type
#         for zone in zones:
#             zone_type = zone.get('type', 'normal')
#             conn.execute(
#                 'INSERT INTO zones (map_name, name, zone_type, points_json) VALUES (?, ?, ?, ?)',
#                 (map_name, zone.get('name', ''), zone_type, json.dumps(zone.get('points', [])))
#             )
#             saved_counts['zones'] += 1
        
#         conn.commit()
#         conn.close()
        
#         total_records = sum(saved_counts.values())
#         logger.info(f"✅ Successfully saved {total_records} records for map '{map_name}'")
        
#         return jsonify({
#             'status': 'success',
#             'message': f'Saved {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones',
#             'map_name': map_name,
#             'database_file': os.path.basename(db_path),
#             'database_path': db_path,
#             'saved_counts': saved_counts,
#             'total_records': total_records,
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error saving nodes: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to save nodes: {str(e)}'
#         }), 500

# @app.route('/load-nodes', methods=['GET'])
# def load_nodes():
#     """Load nodes, arrows, and zones from database (from app.py)"""
#     try:
#         map_name = request.args.get('map_name', 'default')
#         logger.info(f"📂 Loading data for map '{map_name}'")
        
#         db_path = get_map_database_path(map_name)
        
#         # Check if database exists
#         if not os.path.exists(db_path):
#             return jsonify({
#                 'status': 'error',
#                 'message': f'No database found for map: {map_name}',
#                 'suggestion': 'Save nodes first to create database'
#             }), 404
        
#         conn = get_map_db_connection(map_name)
        
#         # Load nodes from all tables
#         nodes = []
        
#         # Load from each node table
#         node_tables = {
#             'stations': 'station',
#             'docking_points': 'docking',
#             'waypoints': 'waypoint',
#             'home_positions': 'home',
#             'charging_stations': 'charging'
#         }
        
#         for table_name, node_type in node_tables.items():
#             cursor = conn.execute(
#                 f'SELECT * FROM {table_name} WHERE map_name = ?', (map_name,)
#             )
#             rows = cursor.fetchall()
            
#             for row in rows:
#                 row_dict = dict(row)
#                 label = f"{node_type}_{row_dict['id']}"
#                 nodes.append({
#                     'id': f"node_{row_dict['id']}",
#                     'type': node_type,
#                     'label': label,
#                     'x': row_dict['x'],
#                     'y': row_dict['y'],
#                     'rosX': row_dict['x'],
#                     'rosY': row_dict['y'],
#                     'dbId': row_dict['id']
#                 })
        
#         # Load arrows
#         arrows = []
#         cursor = conn.execute(
#             'SELECT * FROM arrows WHERE map_name = ?', (map_name,)
#         )
#         arrow_rows = cursor.fetchall()
        
#         for row in arrow_rows:
#             row_dict = dict(row)
#             arrows.append({
#                 'id': row_dict['id'],
#                 'fromId': row_dict['from_node_id'],
#                 'toId': row_dict['to_node_id']
#             })
        
#         # Load zones with zone_type
#         zones = []
#         cursor = conn.execute(
#             'SELECT * FROM zones WHERE map_name = ?', (map_name,)
#         )
#         zone_rows = cursor.fetchall()
        
#         for row in zone_rows:
#             row_dict = dict(row)
#             try:
#                 points_data = json.loads(row_dict['points_json'])
#                 zones.append({
#                     'id': row_dict['id'],
#                     'name': row_dict['name'],
#                     'type': row_dict.get('zone_type', 'normal'),
#                     'points': points_data
#                 })
#             except json.JSONDecodeError:
#                 logger.warning(f"Invalid JSON in zone {row_dict['id']}, skipping")
#                 continue
        
#         conn.close()
        
#         logger.info(f"✅ Loaded {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones")
        
#         return jsonify({
#             'status': 'success',
#             'map_name': map_name,
#             'database_file': os.path.basename(db_path),
#             'nodes': nodes,
#             'arrows': arrows,
#             'zones': zones,
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error loading nodes: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to load nodes: {str(e)}'
#         }), 500

# @app.route('/clear-nodes', methods=['DELETE'])
# def clear_nodes():
#     """Clear all nodes, arrows, and zones for a specific map (from app.py)"""
#     try:
#         map_name = request.args.get('map_name', 'default')
#         logger.info(f"🗑️ Clearing data for map '{map_name}'")
        
#         db_path = get_map_database_path(map_name)
        
#         # Check if database exists
#         if not os.path.exists(db_path):
#             return jsonify({
#                 'status': 'error',
#                 'message': f'No database found for map: {map_name}',
#                 'suggestion': 'Save nodes first to create database'
#             }), 404
        
#         conn = get_map_db_connection(map_name)
        
#         tables_to_clear = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
#         cleared_counts = {}
        
#         for table in tables_to_clear:
#             result = conn.execute(f'DELETE FROM {table} WHERE map_name = ?', (map_name,))
#             cleared_counts[table] = result.rowcount
#             logger.info(f"🧹 Cleared {result.rowcount} records from {table}")
        
#         conn.commit()
#         conn.close()
        
#         total_cleared = sum(cleared_counts.values())
#         logger.info(f"✅ Successfully cleared {total_cleared} records")
        
#         return jsonify({
#             'status': 'success',
#             'message': f'Cleared {total_cleared} records from {map_name}',
#             'map_name': map_name,
#             'cleared_counts': cleared_counts,
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error clearing nodes: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to clear nodes: {str(e)}'
#         }), 500

# @app.route('/list-maps', methods=['GET'])
# def list_maps():
#     """List all available maps/databases (from app.py)"""
#     try:
#         if not os.path.exists(MAP_DATABASE_DIR):
#             os.makedirs(MAP_DATABASE_DIR)
#             return jsonify({
#                 'status': 'success',
#                 'maps': [],
#                 'total': 0,
#                 'message': 'No databases created yet'
#             })
        
#         # Get all .db files in the directory
#         db_files = [f for f in os.listdir(MAP_DATABASE_DIR) if f.endswith('.db')]
        
#         maps_info = []
#         for db_file in db_files:
#             map_name = os.path.splitext(db_file)[0].replace('_', ' ')
#             db_path = os.path.join(MAP_DATABASE_DIR, db_file)
#             db_size = os.path.getsize(db_path) / (1024 * 1024)  # MB
            
#             # Get record counts
#             try:
#                 conn = sqlite3.connect(db_path)
#                 cursor = conn.cursor()
                
#                 # Count total records
#                 tables = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
#                 total_records = 0
#                 for table in tables:
#                     cursor.execute(f"SELECT COUNT(*) FROM {table}")
#                     total_records += cursor.fetchone()[0]
                
#                 conn.close()
                
#                 maps_info.append({
#                     'map_name': map_name,
#                     'database_file': db_file,
#                     'file_size_mb': round(db_size, 2),
#                     'total_records': total_records,
#                     'created_at': datetime.fromtimestamp(os.path.getctime(db_path)).isoformat()
#                 })
#             except:
#                 maps_info.append({
#                     'map_name': map_name,
#                     'database_file': db_file,
#                     'file_size_mb': round(db_size, 2),
#                     'total_records': 'Unknown',
#                     'created_at': datetime.fromtimestamp(os.path.getctime(db_path)).isoformat()
#                 })
        
#         return jsonify({
#             'status': 'success',
#             'maps': maps_info,
#             'total': len(maps_info),
#             'database_directory': MAP_DATABASE_DIR,
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error listing maps: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to list maps: {str(e)}'
#         }), 500

# @app.route('/delete-map', methods=['DELETE'])
# def delete_map():
#     """Delete a map database file (from app.py)"""
#     try:
#         map_name = request.args.get('map_name', 'default')
        
#         if not map_name:
#             return jsonify({
#                 'status': 'error',
#                 'message': 'Map name is required'
#             }), 400
        
#         db_path = get_map_database_path(map_name)
        
#         if not os.path.exists(db_path):
#             return jsonify({
#                 'status': 'error',
#                 'message': f'No database found for map: {map_name}'
#             }), 404
        
#         # Delete the database file
#         os.remove(db_path)
#         logger.info(f"🗑️ Deleted database for map '{map_name}': {db_path}")
        
#         return jsonify({
#             'status': 'success',
#             'message': f'Successfully deleted database for map: {map_name}',
#             'deleted_file': os.path.basename(db_path),
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error deleting map: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to delete map: {str(e)}'
#         }), 500

# # ============================================================================
# # Robot Position Endpoints (from app1.py) WITH DATABASE FEATURES
# # ============================================================================

# @app.route('/robot_position', methods=['GET'])
# def get_robot_position():
#     """Get the current robot position from ROS2 (from app1.py)"""
#     if not ROS_AVAILABLE:
#         return jsonify({
#             'status': 'waiting',
#             'data': {
#                 'x': 0.0,
#                 'y': 0.0,
#                 'yaw': 0.0,
#                 'timestamp': time.time(),
#                 'message_count': 0
#             },
#             'message': 'Waiting for first AMCL message from ROS2... Make sure bag is playing.'
#         })
    
#     with amcl_position_lock:
#         # If no messages received yet
#         if latest_robot_position['message_count'] == 0:
#             return jsonify({
#                 'status': 'waiting',
#                 'data': {
#                     'x': 0.0,
#                     'y': 0.0,
#                     'yaw': 0.0,
#                     'timestamp': time.time(),
#                     'message_count': 0
#                 },
#                 'message': 'Waiting for first AMCL message from ROS2... Make sure bag is playing.'
#             })
        
#         return jsonify({
#             'status': 'success',
#             'data': latest_robot_position,
#             'timestamp': time.time(),
#             'message': f'Received {latest_robot_position["message_count"]} AMCL messages',
#             'database_count': RobotPosition.query.count()
#         })

# @app.route('/position_status', methods=['GET'])
# def get_position_status():
#     """Get detailed connection status (from app1.py)"""
#     if not ROS_AVAILABLE:
#         return jsonify({
#             'ros_connected': False,
#             'api_ready': True,
#             'messages_received': 0,
#             'last_position': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
#             'last_update_age': 'Never',
#             'topic': '/amcl_pose',
#             'message': 'ROS2 not available',
#             'database_count': RobotPosition.query.count()
#         })
    
#     with amcl_position_lock:
#         # Get database info
#         try:
#             db_count = RobotPosition.query.count()
#             latest_db = RobotPosition.query.order_by(RobotPosition.timestamp.desc()).first()
#             db_status = 'connected'
#         except Exception as e:
#             db_count = 0
#             latest_db = None
#             db_status = f'error: {str(e)}'
        
#         status_info = {
#             'ros_connected': True,
#             'api_ready': True,
#             'database_status': db_status,
#             'database_count': db_count,
#             'messages_received': latest_robot_position['message_count'],
#             'last_position': latest_robot_position,
#             'last_db_position': latest_db.to_dict() if latest_db else None,
#             'last_update_age': time.time() - latest_robot_position['timestamp'] if latest_robot_position['timestamp'] > 0 else 'Never',
#             'topic': '/amcl_pose',
#             'database_file': 'robot_positions.db'
#         }
    
#     return jsonify(status_info)

# @app.route('/position_health', methods=['GET'])
# def position_health_check():
#     """Health check endpoint (from app1.py)"""
#     try:
#         db_count = RobotPosition.query.count()
#         db_healthy = True
#     except:
#         db_count = 0
#         db_healthy = False
    
#     return jsonify({
#         'status': 'healthy' if ROS_AVAILABLE else 'disabled',
#         'service': 'ROS2 AMCL Position API',
#         'timestamp': time.time(),
#         'ros_available': ROS_AVAILABLE,
#         'database_available': db_healthy,
#         'database_count': db_count
#     })

# # ============================================================================
# # Robot Status Endpoints (from topicstatus.py)
# # ============================================================================

# @app.route("/status", methods=['GET'])
# def get_robot_status():
#     """Get robot system status (from topicstatus.py - original /status endpoint)"""
#     client_ip = request.remote_addr
#     if not is_allowed_client(client_ip):
#         app.logger.warning("Unauthorized /status request from %s", client_ip)
#         return jsonify({
#             "robot_status": "OFFLINE",
#             "imu_status": "FAULT",
#             "lidar_status": "FAULT",
#             "camera_status": "FAULT",
#             "motor_status": "FAULT",
#             "battery": "N/A",
#             "battery_voltage": 0.0,
#             "robot_position": None,
#             "error_status": "UNAUTHORIZED_NETWORK"
#         })

#     # Authorized — return current snapshot
#     with robot_status_lock:
#         snapshot = dict(robot_status_data)
    
#     snapshot['timestamp'] = datetime.now().isoformat()
#     snapshot['ros_available'] = ROS_AVAILABLE
    
#     return jsonify(snapshot)

# # ============================================================================
# # Database Endpoints for Stored Positions (from app1.py database features)
# # ============================================================================

# @app.route('/positions', methods=['GET'])
# def get_all_positions():
#     """Get all stored positions from database"""
#     try:
#         positions = RobotPosition.query.order_by(RobotPosition.timestamp.desc()).all()
#         return jsonify({
#             'status': 'success',
#             'count': len(positions),
#             'positions': [pos.to_dict() for pos in positions]
#         })
#     except Exception as e:
#         return jsonify({'status': 'error', 'message': str(e)}), 500

# @app.route('/positions/<int:id>', methods=['GET'])
# def get_position_by_id(id):
#     """Get specific position by ID"""
#     try:
#         position = RobotPosition.query.get_or_404(id)
#         return jsonify({
#             'status': 'success',
#             'position': position.to_dict()
#         })
#     except Exception as e:
#         return jsonify({'status': 'error', 'message': str(e)}), 404

# @app.route('/positions/latest', methods=['GET'])
# def get_latest_positions():
#     """Get latest N positions (default: 100)"""
#     try:
#         limit = request.args.get('limit', default=100, type=int)
#         positions = RobotPosition.query.order_by(
#             RobotPosition.timestamp.desc()
#         ).limit(limit).all()
        
#         return jsonify({
#             'status': 'success',
#             'limit': limit,
#             'count': len(positions),
#             'positions': [pos.to_dict() for pos in positions]
#         })
#     except Exception as e:
#         return jsonify({'status': 'error', 'message': str(e)}), 500

# @app.route('/positions/count', methods=['GET'])
# def get_positions_count():
#     """Get count of stored positions"""
#     try:
#         count = RobotPosition.query.count()
#         return jsonify({
#             'status': 'success',
#             'count': count,
#             'database': 'robot_positions.db'
#         })
#     except Exception as e:
#         return jsonify({'status': 'error', 'message': str(e)}), 500

# @app.route('/positions/delete/<int:id>', methods=['DELETE'])
# def delete_position(id):
#     """Delete position by ID"""
#     try:
#         position = RobotPosition.query.get_or_404(id)
#         db.session.delete(position)
#         db.session.commit()
#         return jsonify({
#             'status': 'success',
#             'message': f'Position {id} deleted successfully'
#         })
#     except Exception as e:
#         return jsonify({'status': 'error', 'message': str(e)}), 500

# @app.route('/positions/clear', methods=['DELETE'])
# def clear_all_positions():
#     """Clear all positions from database"""
#     try:
#         if request.args.get('confirm') != 'true':
#             return jsonify({
#                 'status': 'warning',
#                 'message': 'Add ?confirm=true to confirm deletion'
#             }), 400
        
#         count = RobotPosition.query.delete()
#         db.session.commit()
#         return jsonify({
#             'status': 'success',
#             'message': f'Cleared {count} positions from database'
#         })
#     except Exception as e:
#         return jsonify({'status': 'error', 'message': str(e)}), 500

# @app.route('/positions/export', methods=['GET'])
# def export_positions():
#     """Export positions as CSV or JSON"""
#     try:
#         format_type = request.args.get('format', 'json')
#         positions = RobotPosition.query.order_by(RobotPosition.timestamp.asc()).all()
        
#         if format_type == 'csv':
#             import csv
#             from io import StringIO
            
#             output = StringIO()
#             writer = csv.writer(output)
#             writer.writerow(['id', 'x', 'y', 'yaw', 'timestamp', 'header_stamp', 'message_count', 'created_at'])
            
#             for pos in positions:
#                 writer.writerow([
#                     pos.id,
#                     pos.x,
#                     pos.y,
#                     pos.yaw,
#                     pos.timestamp.isoformat() if pos.timestamp else '',
#                     pos.header_stamp,
#                     pos.message_count,
#                     pos.created_at.isoformat() if pos.created_at else ''
#                 ])
            
#             response = app.response_class(
#                 response=output.getvalue(),
#                 status=200,
#                 mimetype='text/csv',
#                 headers={'Content-Disposition': 'attachment; filename=robot_positions.csv'}
#             )
#             return response
        
#         else:  # JSON
#             return jsonify({
#                 'status': 'success',
#                 'count': len(positions),
#                 'positions': [pos.to_dict() for pos in positions]
#             })
            
#     except Exception as e:
#         return jsonify({'status': 'error', 'message': str(e)}), 500

# @app.route('/debug', methods=['GET'])
# def debug_info():
#     """Debug endpoint to see what's happening (from app1.py)"""
#     try:
#         db_count = RobotPosition.query.count()
#         latest_10 = RobotPosition.query.order_by(RobotPosition.timestamp.desc()).limit(10).all()
#     except Exception as e:
#         db_count = 0
#         latest_10 = []
    
#     debug_data = {
#         'latest_position': latest_robot_position,
#         'database_count': db_count,
#         'recent_positions': [pos.to_dict() for pos in latest_10],
#         'system_time': time.time(),
#         'ros_available': ROS_AVAILABLE,
#         'instructions': [
#             '1. Make sure ROS2 environment is sourced',
#             '2. Play ROS2 bag: ros2 bag play my_amcl_bag_0.db3',
#             '3. Check if /amcl_pose topic has data: ros2 topic echo /amcl_pose',
#             '4. Positions are automatically saved to database'
#         ]
#     }
    
#     return jsonify(debug_data)

# @app.route('/topics', methods=['GET'])
# def list_topics():
#     """List available ROS2 topics (from app1.py)"""
#     if not ROS_AVAILABLE:
#         return jsonify({'error': 'ROS2 not available', 'topics': []})
    
#     try:
#         result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
#         topics = result.stdout.strip().split('\n') if result.stdout else []
#         return jsonify({
#             'topics': topics,
#             'amcl_pose_available': '/amcl_pose' in topics
#         })
#     except Exception as e:
#         return jsonify({'error': str(e), 'topics': []})

# # ============================================================================
# # Error Handlers
# # ============================================================================

# @app.errorhandler(404)
# def not_found(error):
#     return jsonify({
#         'status': 'error',
#         'message': 'Endpoint not found',
#         'available_endpoints': [
#             '/', '/health', '/save-nodes', '/load-nodes', '/clear-nodes',
#             '/list-maps', '/delete-map', '/robot_position', '/position_status',
#             '/position_health', '/status', '/debug', '/topics',
#             '/positions', '/positions/count', '/positions/latest', '/positions/export'
#         ]
#     }), 404

# @app.errorhandler(500)
# def internal_error(error):
#     return jsonify({
#         'status': 'error',
#         'message': 'Internal server error'
#     }), 500

# # ============================================================================
# # Main Application Startup
# # ============================================================================

# if __name__ == '__main__':
#     # Print startup banners from all three apps
#     print("\n" + "="*70)
#     print("🚀 UNIFIED AMR SYSTEM API - ALL SERVICES ON PORT 5000")
#     print("="*70)
    
#     # From app.py
#     print("📁 Map Database system: Per-map databases")
#     print(f"📂 Map Database directory: {os.path.abspath(MAP_DATABASE_DIR)}")
#     print("📊 Map Database schema: map_name, node_type, x, y, z")
    
#     # From app1.py
#     print(f"🤖 ROS2 Available: {'✅ YES' if ROS_AVAILABLE else '❌ NO'}")
    
#     # Database initialization for robot positions
#     with app.app_context():
#         db.create_all()
#         count = RobotPosition.query.count()
#         print(f"💾 Robot Position Database: robot_positions.db ({count} positions already stored)")
    
#     # From topicstatus.py
#     print(f"🌐 API Port: 5000")
#     print("="*70)
    
#     # Create map database directory if it doesn't exist
#     if not os.path.exists(MAP_DATABASE_DIR):
#         os.makedirs(MAP_DATABASE_DIR)
#         print(f"📁 Created map database directory: {MAP_DATABASE_DIR}")
    
#     # Start ROS2 threads if available
#     if ROS_AVAILABLE:
#         # Start ROS2 threads (from topicstatus.py)
#         print("\n🔄 Starting ROS2 integration...")
#         ros_thread = threading.Thread(target=ros_spin_combined, daemon=True)
#         ros_thread.start()
        
#         fault_thread = threading.Thread(target=fault_checker, daemon=True)
#         fault_thread.start()
        
#         status_thread = threading.Thread(target=status_printer, daemon=True)
#         status_thread.start()
        
#         print("✅ ROS2 integration started")
#         print("💾 AMCL positions will be automatically saved to database")
        
#         print("\n🎯 Next Steps:")
#         print("   1. Keep this Flask app running")
#         print("   2. Open a NEW terminal")
#         print("   3. Source your ROS2 environment")
#         print("   4. Play your ROS2 bag: ros2 bag play my_amcl_bag_0.db3")
#         print("   5. Watch for position updates above!")
#         print("   6. Positions are automatically saved to database")
#     else:
#         print("⚠️ ROS2 not available - robot position/status features disabled")
#         print("💡 Install ROS2 Python bindings to enable robot tracking")
    
#     # Print combined endpoint information
#     print("\n" + "="*70)
#     print("📍 UNIFIED FLASK API ENDPOINTS (ALL ON PORT 5000):")
#     print("="*70)
#     print("\n🎒 MAP DATABASE API (from app.py):")
#     print("   http://localhost:5000/              - API information")
#     print("   http://localhost:5000/health        - Health check")
#     print("   http://localhost:5000/save-nodes    - POST: Save map annotations")
#     print("   http://localhost:5000/load-nodes    - GET: Load map annotations")
#     print("   http://localhost:5000/clear-nodes   - DELETE: Clear map annotations")
#     print("   http://localhost:5000/list-maps     - GET: List available maps")
#     print("   http://localhost:5000/delete-map    - DELETE: Delete a map")
    
#     print("\n🎯 ROBOT POSITION API with DATABASE (from app1.py):")
#     print("   http://localhost:5000/robot_position    - GET: Current robot position")
#     print("   http://localhost:5000/position_status   - GET: Position connection status")
#     print("   http://localhost:5000/position_health   - GET: Position health check")
#     print("   http://localhost:5000/positions         - GET: All stored positions")
#     print("   http://localhost:5000/positions/count   - GET: Count of positions")
#     print("   http://localhost:5000/positions/latest  - GET: Latest positions")
#     print("   http://localhost:5000/positions/export  - GET: Export as CSV/JSON")
    
#     print("\n🤖 ROBOT STATUS API (from topicstatus.py):")
#     print("   http://localhost:5000/status        - GET: Robot system status")
#     print("   http://localhost:5000/debug         - GET: Debug information")
#     print("   http://localhost:5000/topics        - GET: List ROS2 topics")
    
#     print("\n💾 DATABASES:")
#     print("   - Map annotations: map_databases/*.db")
#     print("   - Robot positions: robot_positions.db")
    
#     print("\n✅ API ready!")
#     print("\n🌐 Starting Flask server...")
#     print("   📍 Local: http://localhost:5000")
#     print("   🌍 Network: http://0.0.0.0:5000")
#     print("="*70 + "\n")
    
#     # Start Flask app
#     app.run(debug=True, host='0.0.0.0', port=5000, use_reloader=False)


#!/usr/bin/env python3
"""
UNIFIED AMR SYSTEM API - PORT 5000

Combines all three applications into one:
1. Map Database API (app.py) - Port 5000
2. ROS2 AMCL Position Reader (app1.py) - Port 5001 
3. ROS2 Robot Status Monitor (topicstatus.py) - Port 5002

WITH DATABASE STORAGE FOR AMCL POSITIONS AND SPEED MONITORING

All running on single port: 5000
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime
import sqlite3
import json
import os
import logging
import re
import math
import threading
import time
import ipaddress
import subprocess

# ROS2 imports (optional - will gracefully degrade if not available)
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import Imu, LaserScan, Image
    from nav_msgs.msg import Odometry
    from std_msgs.msg import UInt16
    ROS_AVAILABLE = True
except ImportError as e:
    ROS_AVAILABLE = False

# ============================================================================
# Configuration
# ============================================================================

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)




# Create Flask app
app = Flask(__name__)
CORS(app)

# Database configuration for robot positions
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///robot_positions.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

# Database Model for robot positions
class RobotPosition(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    x = db.Column(db.Float, nullable=False)
    y = db.Column(db.Float, nullable=False)
    yaw = db.Column(db.Float, nullable=False)
    timestamp = db.Column(db.DateTime, nullable=False, default=datetime.utcnow)
    header_stamp = db.Column(db.String(50))
    message_count = db.Column(db.Integer)
    created_at = db.Column(db.DateTime, nullable=False, default=datetime.utcnow)
    
    def to_dict(self):
        return {
            'id': self.id,
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'timestamp': self.timestamp.isoformat() if self.timestamp else None,
            'header_stamp': self.header_stamp,
            'message_count': self.message_count,
            'created_at': self.created_at.isoformat() if self.created_at else None
        }

# Map annotation database configuration
MAP_DATABASE_DIR = 'map_databases'

# Network filtering (optional)
ALLOWED_NETWORK_STR = os.getenv("ALLOWED_NETWORK", None)
ALLOWED_NETWORK = ipaddress.ip_network(ALLOWED_NETWORK_STR) if ALLOWED_NETWORK_STR else None

# Fault timeouts (seconds)
FAULT_TIMEOUTS = {
    "imu": 1.0,
    "lidar": 2.0,
    "camera": 2.0,
    "motor": 1.0,
    "battery": 3.0,
}

# ============================================================================
# Global Data Stores (Thread-safe)
# ============================================================================

# AMCL Position data (from app1.py)
amcl_position_lock = threading.Lock()
latest_robot_position = {
    'x': 0.0,
    'y': 0.0,
    'yaw': 0.0,
    'timestamp': 0,
    'message_count': 0,
    'status': 'waiting'
}

# Robot Status data (from topicstatus.py)
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
    "error_status": "NONE",
    "speed": {
        "linear": 0.0,
        "angular": 0.0
    },
    "odometry": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
    }
}

# Last message timestamps (from topicstatus.py)
last_msg_time = {
    "pose": 0.0, "imu": 0.0, "lidar": 0.0,
    "camera": 0.0, "motor": 0.0, "battery": 0.0,
    "odom_raw": 0.0
}

# ============================================================================
# Helper Functions for Map Databases
# ============================================================================

def sanitize_map_name(map_name):
    """Sanitize map name for use as filename"""
    sanitized = re.sub(r'[^\w\-_\. ]', '', map_name)
    sanitized = sanitized.replace(' ', '_')
    if not sanitized:
        sanitized = 'default_map'
    return sanitized

def get_map_database_path(map_name):
    """Get database path for a specific map"""
    if not os.path.exists(MAP_DATABASE_DIR):
        os.makedirs(MAP_DATABASE_DIR)
    
    safe_map_name = sanitize_map_name(map_name)
    database_name = f"{safe_map_name}.db"
    return os.path.join(MAP_DATABASE_DIR, database_name)

def get_map_db_connection(map_name):
    """Get database connection for a specific map with error handling"""
    try:
        db_path = get_map_database_path(map_name)
        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        return conn
    except sqlite3.Error as e:
        logger.error(f"Database connection error for map '{map_name}': {e}")
        raise

def init_map_database_for_map(map_name):
    """Initialize database for a specific map with simplified schema"""
    try:
        conn = get_map_db_connection(map_name)
        
        # Create simplified tables with only essential columns
        tables = {
            'stations': '''
                CREATE TABLE IF NOT EXISTS stations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    node_type TEXT NOT NULL DEFAULT 'station',
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL DEFAULT 0.0
                )
            ''',
            'docking_points': '''
                CREATE TABLE IF NOT EXISTS docking_points (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    node_type TEXT NOT NULL DEFAULT 'docking',
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL DEFAULT 0.0
                )
            ''',
            'waypoints': '''
                CREATE TABLE IF NOT EXISTS waypoints (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    node_type TEXT NOT NULL DEFAULT 'waypoint',
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL DEFAULT 0.0
                )
            ''',
            'home_positions': '''
                CREATE TABLE IF NOT EXISTS home_positions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    node_type TEXT NOT NULL DEFAULT 'home',
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL DEFAULT 0.0
                )
            ''',
            'charging_stations': '''
                CREATE TABLE IF NOT EXISTS charging_stations (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    node_type TEXT NOT NULL DEFAULT 'charging',
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    z REAL DEFAULT 0.0
                )
            ''',
            'arrows': '''
                CREATE TABLE IF NOT EXISTS arrows (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    from_node_id TEXT NOT NULL,
                    to_node_id TEXT NOT NULL
                )
            ''',
            'zones': '''
                CREATE TABLE IF NOT EXISTS zones (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    map_name TEXT NOT NULL,
                    name TEXT NOT NULL,
                    zone_type TEXT DEFAULT 'normal',
                    points_json TEXT NOT NULL
                )
            '''
        }
        
        for table_name, create_sql in tables.items():
            conn.execute(create_sql)
            logger.info(f"✅ Table '{table_name}' initialized for map '{map_name}'")
        
        conn.commit()
        conn.close()
        logger.info(f"✅ Database initialization completed for map '{map_name}'")
        
    except Exception as e:
        logger.error(f"❌ Database initialization failed for map '{map_name}': {e}")
        raise

def convert_battery_voltage(raw):
    return raw / 1000.0

def convert_battery_percentage(voltage):
    maxV = 12.6
    minV = 9.0
    voltage = max(minV, min(maxV, voltage))
    return round(((voltage - minV) / (maxV - minV)) * 100)

def is_allowed_client(ip_str: str) -> bool:
    """Check if client IP is allowed (from topicstatus.py)"""
    if ALLOWED_NETWORK is None:
        return True

    try:
        ip_addr = ipaddress.ip_address(ip_str)
    except Exception:
        return False

    # allow loopback for local dev
    if ip_addr.is_loopback:
        return True

    return ip_addr in ALLOWED_NETWORK

# ============================================================================
# ROS2 Nodes (if available)
# ============================================================================

if ROS_AVAILABLE:
    # From app1.py - AMCL Pose Listener with Database Storage
    class AmclPoseListener(Node):
        def __init__(self):
            super().__init__('amcl_pose_listener')
            self.subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.listener_callback,
                10)
            self.message_count = 0
            self.get_logger().info("🎯 AMCL Pose Listener Started")
            self.get_logger().info("📡 Waiting for ROS bag data on /amcl_pose...")

        def listener_callback(self, msg):
            try:
                # Extract position
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y

                # Extract orientation (quaternion → yaw)
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y**2 + q.z**2))
                yaw_degrees = math.degrees(yaw)

                # Update global position
                self.message_count += 1
                with amcl_position_lock:
                    latest_robot_position.update({
                        'x': x,
                        'y': y,
                        'yaw': yaw_degrees,
                        'timestamp': time.time(),
                        'message_count': self.message_count,
                        'header_stamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
                        'status': 'success'
                    })

                # Save to database
                with app.app_context():
                    position = RobotPosition(
                        x=x,
                        y=y,
                        yaw=yaw_degrees,
                        timestamp=datetime.utcnow(),
                        header_stamp=f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
                        message_count=self.message_count
                    )
                    db.session.add(position)
                    db.session.commit()
                    print(f"💾 Saved position #{self.message_count} to database (ID: {position.id})")

                print(f"🎯 AMCL #{self.message_count:02d}: x={x:.3f}, y={y:.3f}, yaw={yaw_degrees:.1f}°")
                
            except Exception as e:
                print(f"❌ Error processing AMCL message: {e}")
                with amcl_position_lock:
                    latest_robot_position['status'] = 'error'

    # From topicstatus.py - Robot Status Monitor WITH ODOM_RAW
    class RobotStatusMonitor(Node):
        def __init__(self):
            super().__init__("robot_status_monitor")
            self.prev_pose = None

            # Subscribers
            self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
            self.create_subscription(Imu, "/imu", self.imu_callback, 10)
            self.create_subscription(LaserScan, "/scan_raw", self.lidar_callback, 10)
            self.create_subscription(Image, "/depth_cam/depth/image_raw", self.camera_callback, 10)
            self.create_subscription(Odometry, "/odom_raw", self.odom_raw_callback, 10)  # For speed monitoring
            self.create_subscription(Odometry, "/odom", self.motor_callback, 10)
            self.create_subscription(UInt16, "/ros_robot_controller/battery", self.battery_callback, 10)

            self.get_logger().info("RobotStatusMonitor started ✔")
            self.get_logger().info("📡 Listening for /odom_raw topic for speed monitoring")

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

        def odom_raw_callback(self, msg: Odometry):
            """Callback for /odom_raw topic - captures speed data"""
            last_msg_time["odom_raw"] = time.time()
            linear = msg.twist.twist.linear.x
            angular = msg.twist.twist.angular.z
            
            with robot_status_lock:
                # Update speed data
                robot_status_data["speed"]["linear"] = round(float(linear), 3)
                robot_status_data["speed"]["angular"] = round(float(angular), 3)
                
                # Update odometry data
                robot_status_data["odometry"]["position"] = {
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "z": float(msg.pose.pose.position.z)
                }
                robot_status_data["odometry"]["orientation"] = {
                    "x": float(msg.pose.pose.orientation.x),
                    "y": float(msg.pose.pose.orientation.y),
                    "z": float(msg.pose.pose.orientation.z),
                    "w": float(msg.pose.pose.orientation.w)
                }
                
                # Determine robot status based on speed
                if abs(linear) > 0.01 or abs(angular) > 0.01:
                    robot_status_data["robot_status"] = "RUNNING"
                else:
                    robot_status_data["robot_status"] = "IDLE"

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

    # ============================================================================
    # ROS2 Background Threads
    # ============================================================================

    def ros_spin_combined():
        """Run both ROS2 nodes in the same thread (from app1.py + topicstatus.py)"""
        try:
            rclpy.init()
            
            # Create both nodes
            amcl_node = AmclPoseListener()
            status_node = RobotStatusMonitor()
            
            print("🚀 Starting combined ROS2 nodes...")
            print("📡 Listening for:")
            print("   - AMCL pose on /amcl_pose")
            print("   - IMU on /imu")
            print("   - Lidar on /scan_raw")
            print("   - Camera on /depth_cam/depth/image_raw")
            print("   - Odometry RAW on /odom_raw (for speed)")
            print("   - Odometry on /odom")
            print("   - Battery on /ros_robot_controller/battery")
            print("💾 AMCL positions will be saved to database")
            
            # Create an executor to handle multiple nodes
            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(amcl_node)
            executor.add_node(status_node)
            
            try:
                executor.spin()
            except KeyboardInterrupt:
                print("\n🛑 Shutting down ROS2 nodes...")
            except Exception as e:
                print(f"❌ ROS2 executor error: {e}")
            finally:
                executor.shutdown()
                amcl_node.destroy_node()
                status_node.destroy_node()
                rclpy.shutdown()
                
        except Exception as e:
            print(f"❌ Failed to initialize ROS2: {e}")

    def fault_checker():
        """Check for sensor faults based on timeouts (from topicstatus.py)"""
        while True:
            now = time.time()
            with robot_status_lock:
                for sensor, timeout in FAULT_TIMEOUTS.items():
                    status_key = f"{sensor}_status"
                    last_t = last_msg_time.get(sensor, 0.0)
                    robot_status_data[status_key] = "FAULT" if (now - last_t > timeout) else "OK"

                # If motors not OK, set robot to IDLE
                if robot_status_data.get("motor_status") != "OK":
                    robot_status_data["robot_status"] = "IDLE"
                    
                # Check odom_raw timeout for speed monitoring
                odom_raw_timeout = 1.0
                last_odom_raw = last_msg_time.get("odom_raw", 0.0)
                if now - last_odom_raw > odom_raw_timeout:
                    robot_status_data["speed"]["linear"] = 0.0
                    robot_status_data["speed"]["angular"] = 0.0
                    
            time.sleep(0.1)

    def status_printer():
        """Print robot status periodically (from topicstatus.py)"""
        while True:
            with robot_status_lock:
                status_copy = dict(robot_status_data)
                speed_info = f"Speed: L={status_copy['speed']['linear']:.3f}, A={status_copy['speed']['angular']:.3f}"
                print(f"🤖 STATUS: {status_copy['robot_status']} | {speed_info} | "
                      f"IMU: {status_copy['imu_status']} | "
                      f"LIDAR: {status_copy['lidar_status']} | "
                      f"CAM: {status_copy['camera_status']} | "
                      f"MOTOR: {status_copy['motor_status']} | "
                      f"BATT: {status_copy['battery']}%")
            time.sleep(1)

# ============================================================================
# Flask Routes - Unified API (ALL ON PORT 5000)
# ============================================================================

@app.route('/')
def index():
    """Root endpoint with API information"""
    return jsonify({
        'status': 'success',
        'message': 'Unified AMR Map Editor & Robot Monitoring API - PORT 5000',
        'services': {
            'map_database': 'Available',
            'robot_position': 'Available' if ROS_AVAILABLE else 'Disabled (ROS2 not found)',
            'robot_status': 'Available' if ROS_AVAILABLE else 'Disabled (ROS2 not found)',
            'position_database': 'Available (robot_positions.db)',
            'speed_monitoring': 'Available' if ROS_AVAILABLE else 'Disabled (ROS2 not found)'
        },
        'endpoints': {
            # Map Database Endpoints (from app.py)
            'save_nodes': 'POST /save-nodes',
            'load_nodes': 'GET /load-nodes',
            'clear_nodes': 'DELETE /clear-nodes',
            'list_maps': 'GET /list-maps',
            'delete_map': 'DELETE /delete-map',
            
            # Robot Position Endpoints (from app1.py)
            'robot_position': 'GET /robot_position',
            'position_status': 'GET /position_status',
            'position_health': 'GET /position_health',
            
            # Robot Status Endpoints (from topicstatus.py) - WITH SPEED
            'robot_status': 'GET /status',
            'speed_data': 'GET /speed',
            'odometry_data': 'GET /odometry',
            'system_health': 'GET /health',
            'debug': 'GET /debug',
            'topics': 'GET /topics',
            
            # Database Endpoints (from app1.py database features)
            'positions': 'GET /positions',
            'positions_count': 'GET /positions/count',
            'positions_latest': 'GET /positions/latest',
            'positions_export': 'GET /positions/export',
            'positions_delete': 'DELETE /positions/delete/<id>',
            'positions_clear': 'DELETE /positions/clear'
        },
        'version': '4.1.0',
        'port': 5000,
        'ros_available': ROS_AVAILABLE,
        'databases': {
            'robot_positions': 'robot_positions.db',
            'map_annotations': f'{MAP_DATABASE_DIR}/'
        },
        'speed_topic': '/odom_raw' if ROS_AVAILABLE else 'Not available'
    })

# ============================================================================
# Map Database Endpoints (from app.py)
# ============================================================================

@app.route('/health')
def health_check():
    """Health check endpoint (from app.py)"""
    try:
        # Test with default map
        db_path = get_map_database_path('default')
        if not os.path.exists(db_path):
            init_map_database_for_map('default')
        
        conn = sqlite3.connect(db_path)
        conn.execute('SELECT 1')
        conn.close()
        
        return jsonify({
            'status': 'success',
            'message': 'API is healthy',
            'database_system': 'per-map databases',
            'database_directory': MAP_DATABASE_DIR,
            'position_database': 'robot_positions.db',
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': f'Health check failed: {str(e)}'
        }), 500

@app.route('/save-nodes', methods=['POST'])
def save_nodes():
    """Save nodes, arrows, and zones to database (from app.py)"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({
                'status': 'error',
                'message': 'No JSON data provided'
            }), 400
        
        map_name = data.get('mapName', 'default')
        
        # Initialize database for this map if it doesn't exist
        db_path = get_map_database_path(map_name)
        if not os.path.exists(db_path):
            init_map_database_for_map(map_name)
        
        nodes = data.get('nodes', [])
        arrows = data.get('arrows', [])
        zones = data.get('zones', [])
        
        logger.info(f"💾 Saving {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones for map '{map_name}'")
        
        conn = get_map_db_connection(map_name)
        saved_counts = {
            'stations': 0,
            'docking_points': 0,
            'waypoints': 0,
            'home_positions': 0,
            'charging_stations': 0,
            'arrows': 0,
            'zones': 0
        }
        
        # Clear existing data for this map
        tables_to_clear = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
        for table in tables_to_clear:
            result = conn.execute(f'DELETE FROM {table} WHERE map_name = ?', (map_name,))
            logger.info(f"🧹 Cleared {result.rowcount} records from {table}")
        
        # Save nodes to respective tables
        for node in nodes:
            table_name = None
            if node.get('type') == 'station':
                table_name = 'stations'
                saved_counts['stations'] += 1
            elif node.get('type') == 'docking':
                table_name = 'docking_points'
                saved_counts['docking_points'] += 1
            elif node.get('type') == 'waypoint':
                table_name = 'waypoints'
                saved_counts['waypoints'] += 1
            elif node.get('type') == 'home':
                table_name = 'home_positions'
                saved_counts['home_positions'] += 1
            elif node.get('type') == 'charging':
                table_name = 'charging_stations'
                saved_counts['charging_stations'] += 1
            
            if table_name:
                conn.execute(
                    f'INSERT INTO {table_name} (map_name, node_type, x, y, z) VALUES (?, ?, ?, ?, ?)',
                    (map_name, node.get('type', ''), node.get('rosX', 0), node.get('rosY', 0), 0.0)
                )
        
        # Save arrows
        for arrow in arrows:
            conn.execute(
                'INSERT INTO arrows (map_name, from_node_id, to_node_id) VALUES (?, ?, ?)',
                (map_name, arrow.get('fromId', ''), arrow.get('toId', ''))
            )
            saved_counts['arrows'] += 1
        
        # Save zones with zone_type
        for zone in zones:
            zone_type = zone.get('type', 'normal')
            conn.execute(
                'INSERT INTO zones (map_name, name, zone_type, points_json) VALUES (?, ?, ?, ?)',
                (map_name, zone.get('name', ''), zone_type, json.dumps(zone.get('points', [])))
            )
            saved_counts['zones'] += 1
        
        conn.commit()
        conn.close()
        
        total_records = sum(saved_counts.values())
        logger.info(f"✅ Successfully saved {total_records} records for map '{map_name}'")
        
        return jsonify({
            'status': 'success',
            'message': f'Saved {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones',
            'map_name': map_name,
            'database_file': os.path.basename(db_path),
            'database_path': db_path,
            'saved_counts': saved_counts,
            'total_records': total_records,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error saving nodes: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to save nodes: {str(e)}'
        }), 500

@app.route('/load-nodes', methods=['GET'])
def load_nodes():
    """Load nodes, arrows, and zones from database (from app.py)"""
    try:
        map_name = request.args.get('map_name', 'default')
        logger.info(f"📂 Loading data for map '{map_name}'")
        
        db_path = get_map_database_path(map_name)
        
        # Check if database exists
        if not os.path.exists(db_path):
            return jsonify({
                'status': 'error',
                'message': f'No database found for map: {map_name}',
                'suggestion': 'Save nodes first to create database'
            }), 404
        
        conn = get_map_db_connection(map_name)
        
        # Load nodes from all tables
        nodes = []
        
        # Load from each node table
        node_tables = {
            'stations': 'station',
            'docking_points': 'docking',
            'waypoints': 'waypoint',
            'home_positions': 'home',
            'charging_stations': 'charging'
        }
        
        for table_name, node_type in node_tables.items():
            cursor = conn.execute(
                f'SELECT * FROM {table_name} WHERE map_name = ?', (map_name,)
            )
            rows = cursor.fetchall()
            
            for row in rows:
                row_dict = dict(row)
                label = f"{node_type}_{row_dict['id']}"
                nodes.append({
                    'id': f"node_{row_dict['id']}",
                    'type': node_type,
                    'label': label,
                    'x': row_dict['x'],
                    'y': row_dict['y'],
                    'rosX': row_dict['x'],
                    'rosY': row_dict['y'],
                    'dbId': row_dict['id']
                })
        
        # Load arrows
        arrows = []
        cursor = conn.execute(
            'SELECT * FROM arrows WHERE map_name = ?', (map_name,)
        )
        arrow_rows = cursor.fetchall()
        
        for row in arrow_rows:
            row_dict = dict(row)
            arrows.append({
                'id': row_dict['id'],
                'fromId': row_dict['from_node_id'],
                'toId': row_dict['to_node_id']
            })
        
        # Load zones with zone_type
        zones = []
        cursor = conn.execute(
            'SELECT * FROM zones WHERE map_name = ?', (map_name,)
        )
        zone_rows = cursor.fetchall()
        
        for row in zone_rows:
            row_dict = dict(row)
            try:
                points_data = json.loads(row_dict['points_json'])
                zones.append({
                    'id': row_dict['id'],
                    'name': row_dict['name'],
                    'type': row_dict.get('zone_type', 'normal'),
                    'points': points_data
                })
            except json.JSONDecodeError:
                logger.warning(f"Invalid JSON in zone {row_dict['id']}, skipping")
                continue
        
        conn.close()
        
        logger.info(f"✅ Loaded {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones")
        
        return jsonify({
            'status': 'success',
            'map_name': map_name,
            'database_file': os.path.basename(db_path),
            'nodes': nodes,
            'arrows': arrows,
            'zones': zones,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error loading nodes: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to load nodes: {str(e)}'
        }), 500

@app.route('/clear-nodes', methods=['DELETE'])
def clear_nodes():
    """Clear all nodes, arrows, and zones for a specific map (from app.py)"""
    try:
        map_name = request.args.get('map_name', 'default')
        logger.info(f"🗑️ Clearing data for map '{map_name}'")
        
        db_path = get_map_database_path(map_name)
        
        # Check if database exists
        if not os.path.exists(db_path):
            return jsonify({
                'status': 'error',
                'message': f'No database found for map: {map_name}',
                'suggestion': 'Save nodes first to create database'
            }), 404
        
        conn = get_map_db_connection(map_name)
        
        tables_to_clear = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
        cleared_counts = {}
        
        for table in tables_to_clear:
            result = conn.execute(f'DELETE FROM {table} WHERE map_name = ?', (map_name,))
            cleared_counts[table] = result.rowcount
            logger.info(f"🧹 Cleared {result.rowcount} records from {table}")
        
        conn.commit()
        conn.close()
        
        total_cleared = sum(cleared_counts.values())
        logger.info(f"✅ Successfully cleared {total_cleared} records")
        
        return jsonify({
            'status': 'success',
            'message': f'Cleared {total_cleared} records from {map_name}',
            'map_name': map_name,
            'cleared_counts': cleared_counts,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error clearing nodes: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to clear nodes: {str(e)}'
        }), 500

@app.route('/list-maps', methods=['GET'])
def list_maps():
    """List all available maps/databases (from app.py)"""
    try:
        if not os.path.exists(MAP_DATABASE_DIR):
            os.makedirs(MAP_DATABASE_DIR)
            return jsonify({
                'status': 'success',
                'maps': [],
                'total': 0,
                'message': 'No databases created yet'
            })
        
        # Get all .db files in the directory
        db_files = [f for f in os.listdir(MAP_DATABASE_DIR) if f.endswith('.db')]
        
        maps_info = []
        for db_file in db_files:
            map_name = os.path.splitext(db_file)[0].replace('_', ' ')
            db_path = os.path.join(MAP_DATABASE_DIR, db_file)
            db_size = os.path.getsize(db_path) / (1024 * 1024)  # MB
            
            # Get record counts
            try:
                conn = sqlite3.connect(db_path)
                cursor = conn.cursor()
                
                # Count total records
                tables = ['stations', 'docking_points', 'waypoints', 'home_positions', 'charging_stations', 'arrows', 'zones']
                total_records = 0
                for table in tables:
                    cursor.execute(f"SELECT COUNT(*) FROM {table}")
                    total_records += cursor.fetchone()[0]
                
                conn.close()
                
                maps_info.append({
                    'map_name': map_name,
                    'database_file': db_file,
                    'file_size_mb': round(db_size, 2),
                    'total_records': total_records,
                    'created_at': datetime.fromtimestamp(os.path.getctime(db_path)).isoformat()
                })
            except:
                maps_info.append({
                    'map_name': map_name,
                    'database_file': db_file,
                    'file_size_mb': round(db_size, 2),
                    'total_records': 'Unknown',
                    'created_at': datetime.fromtimestamp(os.path.getctime(db_path)).isoformat()
                })
        
        return jsonify({
            'status': 'success',
            'maps': maps_info,
            'total': len(maps_info),
            'database_directory': MAP_DATABASE_DIR,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error listing maps: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to list maps: {str(e)}'
        }), 500

@app.route('/delete-map', methods=['DELETE'])
def delete_map():
    """Delete a map database file (from app.py)"""
    try:
        map_name = request.args.get('map_name', 'default')
        
        if not map_name:
            return jsonify({
                'status': 'error',
                'message': 'Map name is required'
            }), 400
        
        db_path = get_map_database_path(map_name)
        
        if not os.path.exists(db_path):
            return jsonify({
                'status': 'error',
                'message': f'No database found for map: {map_name}'
            }), 404
        
        # Delete the database file
        os.remove(db_path)
        logger.info(f"🗑️ Deleted database for map '{map_name}': {db_path}")
        
        return jsonify({
            'status': 'success',
            'message': f'Successfully deleted database for map: {map_name}',
            'deleted_file': os.path.basename(db_path),
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"❌ Error deleting map: {e}")
        return jsonify({
            'status': 'error',
            'message': f'Failed to delete map: {str(e)}'
        }), 500

# ============================================================================
# Robot Position Endpoints (from app1.py) WITH DATABASE FEATURES
# ============================================================================

@app.route('/robot_position', methods=['GET'])
def get_robot_position():
    """Get the current robot position from ROS2 (from app1.py)"""
    if not ROS_AVAILABLE:
        return jsonify({
            'status': 'waiting',
            'data': {
                'x': 0.0,
                'y': 0.0,
                'yaw': 0.0,
                'timestamp': time.time(),
                'message_count': 0
            },
            'message': 'Waiting for first AMCL message from ROS2... Make sure bag is playing.'
        })
    
    with amcl_position_lock:
        # If no messages received yet
        if latest_robot_position['message_count'] == 0:
            return jsonify({
                'status': 'waiting',
                'data': {
                    'x': 0.0,
                    'y': 0.0,
                    'yaw': 0.0,
                    'timestamp': time.time(),
                    'message_count': 0
                },
                'message': 'Waiting for first AMCL message from ROS2... Make sure bag is playing.'
            })
        
        return jsonify({
            'status': 'success',
            'data': latest_robot_position,
            'timestamp': time.time(),
            'message': f'Received {latest_robot_position["message_count"]} AMCL messages',
            'database_count': RobotPosition.query.count()
        })

@app.route('/position_status', methods=['GET'])
def get_position_status():
    """Get detailed connection status (from app1.py)"""
    if not ROS_AVAILABLE:
        return jsonify({
            'ros_connected': False,
            'api_ready': True,
            'messages_received': 0,
            'last_position': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'last_update_age': 'Never',
            'topic': '/amcl_pose',
            'message': 'ROS2 not available',
            'database_count': RobotPosition.query.count()
        })
    
    with amcl_position_lock:
        # Get database info
        try:
            db_count = RobotPosition.query.count()
            latest_db = RobotPosition.query.order_by(RobotPosition.timestamp.desc()).first()
            db_status = 'connected'
        except Exception as e:
            db_count = 0
            latest_db = None
            db_status = f'error: {str(e)}'
        
        status_info = {
            'ros_connected': True,
            'api_ready': True,
            'database_status': db_status,
            'database_count': db_count,
            'messages_received': latest_robot_position['message_count'],
            'last_position': latest_robot_position,
            'last_db_position': latest_db.to_dict() if latest_db else None,
            'last_update_age': time.time() - latest_robot_position['timestamp'] if latest_robot_position['timestamp'] > 0 else 'Never',
            'topic': '/amcl_pose',
            'database_file': 'robot_positions.db'
        }
    
    return jsonify(status_info)

@app.route('/position_health', methods=['GET'])
def position_health_check():
    """Health check endpoint (from app1.py)"""
    try:
        db_count = RobotPosition.query.count()
        db_healthy = True
    except:
        db_count = 0
        db_healthy = False
    
    return jsonify({
        'status': 'healthy' if ROS_AVAILABLE else 'disabled',
        'service': 'ROS2 AMCL Position API',
        'timestamp': time.time(),
        'ros_available': ROS_AVAILABLE,
        'database_available': db_healthy,
        'database_count': db_count
    })

# ============================================================================
# Robot Status Endpoints (from topicstatus.py) - WITH SPEED DATA
# ============================================================================

@app.route("/status", methods=['GET'])
def get_robot_status():
    """Get robot system status (from topicstatus.py - original /status endpoint)"""
    client_ip = request.remote_addr
    if not is_allowed_client(client_ip):
        app.logger.warning("Unauthorized /status request from %s", client_ip)
        return jsonify({
            "robot_status": "OFFLINE",
            "imu_status": "FAULT",
            "lidar_status": "FAULT",
            "camera_status": "FAULT",
            "motor_status": "FAULT",
            "battery": "N/A",
            "battery_voltage": 0.0,
            "robot_position": None,
            "error_status": "UNAUTHORIZED_NETWORK",
            "speed": {"linear": 0.0, "angular": 0.0}
        })

    # Authorized — return current snapshot
    with robot_status_lock:
        snapshot = dict(robot_status_data)
    
    snapshot['timestamp'] = datetime.now().isoformat()
    snapshot['ros_available'] = ROS_AVAILABLE
    snapshot['speed_topic'] = '/odom_raw' if ROS_AVAILABLE else 'Not available'
    
    return jsonify(snapshot)

@app.route("/speed", methods=['GET'])
def get_speed_data():
    """Get speed data from /odom_raw topic"""
    if not ROS_AVAILABLE:
        return jsonify({
            "status": "error",
            "message": "ROS2 not available",
            "speed": {"linear": 0.0, "angular": 0.0},
            "timestamp": datetime.now().isoformat()
        })
    
    with robot_status_lock:
        speed_data = dict(robot_status_data["speed"])
        odom_raw_age = time.time() - last_msg_time.get("odom_raw", 0)
    
    return jsonify({
        "status": "success",
        "topic": "/odom_raw",
        "speed": speed_data,
        "data_age_seconds": round(odom_raw_age, 3),
        "data_fresh": odom_raw_age < 1.0,
        "timestamp": datetime.now().isoformat()
    })

@app.route("/odometry", methods=['GET'])
def get_odometry_data():
    """Get full odometry data from /odom_raw topic"""
    if not ROS_AVAILABLE:
        return jsonify({
            "status": "error",
            "message": "ROS2 not available",
            "odometry": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
            },
            "timestamp": datetime.now().isoformat()
        })
    
    with robot_status_lock:
        odom_data = dict(robot_status_data["odometry"])
        speed_data = dict(robot_status_data["speed"])
        odom_raw_age = time.time() - last_msg_time.get("odom_raw", 0)
    
    return jsonify({
        "status": "success",
        "topic": "/odom_raw",
        "position": odom_data["position"],
        "orientation": odom_data["orientation"],
        "speed": speed_data,
        "data_age_seconds": round(odom_raw_age, 3),
        "timestamp": datetime.now().isoformat()
    })

# ============================================================================
# Database Endpoints for Stored Positions (from app1.py database features)
# ============================================================================

@app.route('/positions', methods=['GET'])
def get_all_positions():
    """Get all stored positions from database"""
    try:
        positions = RobotPosition.query.order_by(RobotPosition.timestamp.desc()).all()
        return jsonify({
            'status': 'success',
            'count': len(positions),
            'positions': [pos.to_dict() for pos in positions]
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/positions/<int:id>', methods=['GET'])
def get_position_by_id(id):
    """Get specific position by ID"""
    try:
        position = RobotPosition.query.get_or_404(id)
        return jsonify({
            'status': 'success',
            'position': position.to_dict()
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 404

@app.route('/positions/latest', methods=['GET'])
def get_latest_positions():
    """Get latest N positions (default: 100)"""
    try:
        limit = request.args.get('limit', default=100, type=int)
        positions = RobotPosition.query.order_by(
            RobotPosition.timestamp.desc()
        ).limit(limit).all()
        
        return jsonify({
            'status': 'success',
            'limit': limit,
            'count': len(positions),
            'positions': [pos.to_dict() for pos in positions]
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/positions/count', methods=['GET'])
def get_positions_count():
    """Get count of stored positions"""
    try:
        count = RobotPosition.query.count()
        return jsonify({
            'status': 'success',
            'count': count,
            'database': 'robot_positions.db'
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/positions/delete/<int:id>', methods=['DELETE'])
def delete_position(id):
    """Delete position by ID"""
    try:
        position = RobotPosition.query.get_or_404(id)
        db.session.delete(position)
        db.session.commit()
        return jsonify({
            'status': 'success',
            'message': f'Position {id} deleted successfully'
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/positions/clear', methods=['DELETE'])
def clear_all_positions():
    """Clear all positions from database"""
    try:
        if request.args.get('confirm') != 'true':
            return jsonify({
                'status': 'warning',
                'message': 'Add ?confirm=true to confirm deletion'
            }), 400
        
        count = RobotPosition.query.delete()
        db.session.commit()
        return jsonify({
            'status': 'success',
            'message': f'Cleared {count} positions from database'
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/positions/export', methods=['GET'])
def export_positions():
    """Export positions as CSV or JSON"""
    try:
        format_type = request.args.get('format', 'json')
        positions = RobotPosition.query.order_by(RobotPosition.timestamp.asc()).all()
        
        if format_type == 'csv':
            import csv
            from io import StringIO
            
            output = StringIO()
            writer = csv.writer(output)
            writer.writerow(['id', 'x', 'y', 'yaw', 'timestamp', 'header_stamp', 'message_count', 'created_at'])
            
            for pos in positions:
                writer.writerow([
                    pos.id,
                    pos.x,
                    pos.y,
                    pos.yaw,
                    pos.timestamp.isoformat() if pos.timestamp else '',
                    pos.header_stamp,
                    pos.message_count,
                    pos.created_at.isoformat() if pos.created_at else ''
                ])
            
            response = app.response_class(
                response=output.getvalue(),
                status=200,
                mimetype='text/csv',
                headers={'Content-Disposition': 'attachment; filename=robot_positions.csv'}
            )
            return response
        
        else:  # JSON
            return jsonify({
                'status': 'success',
                'count': len(positions),
                'positions': [pos.to_dict() for pos in positions]
            })
            
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/debug', methods=['GET'])
def debug_info():
    """Debug endpoint to see what's happening (from app1.py)"""
    try:
        db_count = RobotPosition.query.count()
        latest_10 = RobotPosition.query.order_by(RobotPosition.timestamp.desc()).limit(10).all()
    except Exception as e:
        db_count = 0
        latest_10 = []
    
    with robot_status_lock:
        current_speed = dict(robot_status_data["speed"])
        current_odom = dict(robot_status_data["odometry"])
    
    debug_data = {
        'latest_position': latest_robot_position,
        'database_count': db_count,
        'recent_positions': [pos.to_dict() for pos in latest_10],
        'system_time': time.time(),
        'ros_available': ROS_AVAILABLE,
        'speed_data': current_speed,
        'odometry_data': current_odom,
        'last_message_times': {k: round(time.time() - v, 3) for k, v in last_msg_time.items() if v > 0},
        'odom_raw_available': last_msg_time.get("odom_raw", 0) > 0,
        'instructions': [
            '1. Make sure ROS2 environment is sourced',
            '2. Play ROS2 bag: ros2 bag play my_amcl_bag_0.db3',
            '3. Check if /amcl_pose topic has data: ros2 topic echo /amcl_pose',
            '4. Check if /odom_raw topic has data: ros2 topic echo /odom_raw',
            '5. Positions are automatically saved to database'
        ]
    }
    
    return jsonify(debug_data)

@app.route('/topics', methods=['GET'])
def list_topics():
    """List available ROS2 topics (from app1.py)"""
    if not ROS_AVAILABLE:
        return jsonify({'error': 'ROS2 not available', 'topics': []})
    
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
        topics = result.stdout.strip().split('\n') if result.stdout else []
        
        # Check for specific topics
        topic_checks = {
            '/amcl_pose': '/amcl_pose' in topics,
            '/odom_raw': '/odom_raw' in topics,
            '/imu': '/imu' in topics,
            '/scan_raw': '/scan_raw' in topics,
            '/odom': '/odom' in topics,
            '/ros_robot_controller/battery': '/ros_robot_controller/battery' in topics
        }
        
        return jsonify({
            'topics': topics,
            'topic_checks': topic_checks,
            'status': 'Missing required topics' if not all(topic_checks.values()) else 'All topics available'
        })
    except Exception as e:
        return jsonify({'error': str(e), 'topics': []})

# ============================================================================
# Error Handlers
# ============================================================================

@app.errorhandler(404)
def not_found(error):
    return jsonify({
        'status': 'error',
        'message': 'Endpoint not found',
        'available_endpoints': [
            '/', '/health', '/save-nodes', '/load-nodes', '/clear-nodes',
            '/list-maps', '/delete-map', '/robot_position', '/position_status',
            '/position_health', '/status', '/speed', '/odometry',
            '/debug', '/topics', '/positions', '/positions/count', 
            '/positions/latest', '/positions/export'
        ]
    }), 404

@app.errorhandler(500)
def internal_error(error):
    return jsonify({
        'status': 'error',
        'message': 'Internal server error'
    }), 500

# ============================================================================
# Main Application Startup
# ============================================================================

if __name__ == '__main__':
    # Print startup banners from all three apps
    print("\n" + "="*70)
    print("🚀 UNIFIED AMR SYSTEM API - ALL SERVICES ON PORT 5000")
    print("="*70)
    
    # From app.py
    print("📁 Map Database system: Per-map databases")
    print(f"📂 Map Database directory: {os.path.abspath(MAP_DATABASE_DIR)}")
    print("📊 Map Database schema: map_name, node_type, x, y, z")
    
    # From app1.py
    print(f"🤖 ROS2 Available: {'✅ YES' if ROS_AVAILABLE else '❌ NO'}")
    
    # Database initialization for robot positions
    with app.app_context():
        db.create_all()
        count = RobotPosition.query.count()
        print(f"💾 Robot Position Database: robot_positions.db ({count} positions already stored)")
    
    # Speed monitoring info
    print(f"📈 Speed Monitoring via /odom_raw: {'✅ Enabled' if ROS_AVAILABLE else '❌ Disabled'}")
    
    # From topicstatus.py
    print(f"🌐 API Port: 5000")
    print("="*70)
    
    # Create map database directory if it doesn't exist
    if not os.path.exists(MAP_DATABASE_DIR):
        os.makedirs(MAP_DATABASE_DIR)
        print(f"📁 Created map database directory: {MAP_DATABASE_DIR}")
    
    # Start ROS2 threads if available
    if ROS_AVAILABLE:
        # Start ROS2 threads (from topicstatus.py)
        print("\n🔄 Starting ROS2 integration...")
        ros_thread = threading.Thread(target=ros_spin_combined, daemon=True)
        ros_thread.start()
        
        fault_thread = threading.Thread(target=fault_checker, daemon=True)
        fault_thread.start()
        
        status_thread = threading.Thread(target=status_printer, daemon=True)
        status_thread.start()
        
        print("✅ ROS2 integration started")
        print("📈 Speed monitoring via /odom_raw enabled")
        print("💾 AMCL positions will be automatically saved to database")
        
        print("\n🎯 Next Steps:")
        print("   1. Keep this Flask app running")
        print("   2. Open a NEW terminal")
        print("   3. Source your ROS2 environment")
        print("   4. Play your ROS2 bag: ros2 bag play my_amcl_bag_0.db3")
        print("   5. Watch for position and speed updates above!")
        print("   6. Positions are automatically saved to database")
    else:
        print("⚠️ ROS2 not available - robot position/status/speed features disabled")
        print("💡 Install ROS2 Python bindings to enable robot tracking")
    
    # Print combined endpoint information
    print("\n" + "="*70)
    print("📍 UNIFIED FLASK API ENDPOINTS (ALL ON PORT 5000):")
    print("="*70)
    print("\n🎒 MAP DATABASE API (from app.py):")
    print("   http://localhost:5000/              - API information")
    print("   http://localhost:5000/health        - Health check")
    print("   http://localhost:5000/save-nodes    - POST: Save map annotations")
    print("   http://localhost:5000/load-nodes    - GET: Load map annotations")
    print("   http://localhost:5000/clear-nodes   - DELETE: Clear map annotations")
    print("   http://localhost:5000/list-maps     - GET: List available maps")
    print("   http://localhost:5000/delete-map    - DELETE: Delete a map")
    
    print("\n🎯 ROBOT POSITION API with DATABASE (from app1.py):")
    print("   http://localhost:5000/robot_position    - GET: Current robot position")
    print("   http://localhost:5000/position_status   - GET: Position connection status")
    print("   http://localhost:5000/position_health   - GET: Position health check")
    print("   http://localhost:5000/positions         - GET: All stored positions")
    print("   http://localhost:5000/positions/count   - GET: Count of positions")
    print("   http://localhost:5000/positions/latest  - GET: Latest positions")
    print("   http://localhost:5000/positions/export  - GET: Export as CSV/JSON")
    
    print("\n🤖 ROBOT STATUS API with SPEED MONITORING (from topicstatus.py):")
    print("   http://localhost:5000/status        - GET: Full robot system status")
    print("   http://localhost:5000/speed         - GET: Current speed (linear & angular)")
    print("   http://localhost:5000/odometry      - GET: Full odometry data")
    print("   http://localhost:5000/debug         - GET: Debug information")
    print("   http://localhost:5000/topics        - GET: List ROS2 topics")
    
    print("\n💾 DATABASES:")
    print("   - Map annotations: map_databases/*.db")
    print("   - Robot positions: robot_positions.db")
    
    print("\n✅ API ready!")
    print("\n🌐 Starting Flask server...")
    print("   📍 Local: http://localhost:5000")
    print("   🌍 Network: http://0.0.0.0:5000")
    print("="*70 + "\n")
    
    # Start Flask app
    app.run(debug=True, host='0.0.0.0', port=5000, use_reloader=False)