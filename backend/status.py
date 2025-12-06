# #!/usr/bin/env python3
# """
# Combined Flask API for:
# 1. ROS2 AMCL Pose Reader (originally port 5001 - app1.py)
# 2. AMR Map Editor API (originally port 5000 - app.py) 
# 3. ROS2 Status Bridge (originally port 5002 - topicstatus.py)

# Run with: python combined_app.py
# """

# import os
# import time
# import threading
# import math
# import json
# import sqlite3
# import re
# import logging
# import ipaddress
# from datetime import datetime
# from flask import Flask, request, jsonify
# from flask_cors import CORS

# # ROS2 imports (only if available)
# try:
#     import rclpy
#     from rclpy.node import Node
#     from geometry_msgs.msg import PoseWithCovarianceStamped
#     from sensor_msgs.msg import Imu, LaserScan, Image
#     from nav_msgs.msg import Odometry
#     from std_msgs.msg import UInt16
#     ROS_AVAILABLE = True
# except ImportError:
#     print("⚠️  ROS2 not available - running in API-only mode")
#     ROS_AVAILABLE = False

# # Configure logging
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)

# app = Flask(__name__)
# CORS(app)  # Enable CORS for all routes

# # ============================================================================
# # MODULE 1: ROS2 AMCL POSE READER
# # ============================================================================

# # Global variable to store the latest robot position
# latest_robot_position = {
#     'x': 0.0,
#     'y': 0.0,
#     'yaw': 0.0,
#     'timestamp': 0,
#     'message_count': 0,
#     'status': 'waiting'
# }

# if ROS_AVAILABLE:
#     class AmclPoseListener(Node):
#         def __init__(self):
#             super().__init__('amcl_pose_listener')
            
#             # Subscribe to AMCL pose topic
#             self.subscription = self.create_subscription(
#                 PoseWithCovarianceStamped,
#                 '/amcl_pose',
#                 self.listener_callback,
#                 10)
            
#             self.message_count = 0
#             self.get_logger().info("🎯 AMCL Pose Listener Started")
#             self.get_logger().info("📡 Waiting for ROS bag data on /amcl_pose...")

#         def listener_callback(self, msg):
#             global latest_robot_position
            
#             try:
#                 # Extract position
#                 x = msg.pose.pose.position.x
#                 y = msg.pose.pose.position.y

#                 # Extract orientation (quaternion → yaw)
#                 q = msg.pose.pose.orientation
#                 # Convert quaternion to yaw (rotation around Z axis)
#                 yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
#                                 1.0 - 2.0 * (q.y**2 + q.z**2))

#                 # Update global position
#                 self.message_count += 1
#                 latest_robot_position = {
#                     'x': x,
#                     'y': y,
#                     'yaw': math.degrees(yaw),
#                     'timestamp': time.time(),
#                     'message_count': self.message_count,
#                     'header_stamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
#                     'status': 'success'
#                 }

#                 # Print to terminal with colorful output
#                 print(f"🎯 AMCL #{self.message_count:02d}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°")
                
#             except Exception as e:
#                 print(f"❌ Error processing AMCL message: {e}")
#                 latest_robot_position['status'] = 'error'

#     # ============================================================================
#     # MODULE 3: ROS2 STATUS BRIDGE
#     # ============================================================================

#     # Configuration
#     ALLOWED_NETWORK_STR = os.getenv("ALLOWED_NETWORK", None)
#     ALLOWED_NETWORK = ipaddress.ip_network(ALLOWED_NETWORK_STR) if ALLOWED_NETWORK_STR else None

#     # Fault timeouts (seconds)
#     FAULT_TIMEOUTS = {
#         "imu": 1.0,
#         "lidar": 2.0,
#         "camera": 2.0,
#         "motor": 1.0,
#         "battery": 3.0,
#     }

#     # Shared robot status (thread-safe)
#     robot_status_lock = threading.Lock()
#     robot_status_data = {
#         "robot_status": "IDLE",
#         "imu_status": "FAULT",
#         "lidar_status": "FAULT",
#         "camera_status": "FAULT",
#         "motor_status": "FAULT",
#         "battery": "N/A",
#         "battery_voltage": 0.0,
#         "robot_position": {"x": 0.0, "y": 0.0},
#         "error_status": "NONE"
#     }

#     # Last message timestamps (for fault detection)
#     last_msg_time = {
#         "pose": 0.0, "imu": 0.0, "lidar": 0.0,
#         "camera": 0.0, "motor": 0.0, "battery": 0.0,
#     }

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

#         # ---------- Callbacks ----------
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
#             voltage = self.convert_battery_voltage(msg.data)
#             with robot_status_lock:
#                 robot_status_data["battery_voltage"] = round(voltage, 3)
#                 robot_status_data["battery"] = self.convert_battery_percentage(voltage)
        
#         def convert_battery_voltage(self, raw):
#             return raw / 1000.0
        
#         def convert_battery_percentage(self, voltage):
#             maxV = 12.6
#             minV = 9.0
#             voltage = max(minV, min(maxV, voltage))
#             return round(((voltage - minV) / (maxV - minV)) * 100)

# # ============================================================================
# # MODULE 2: AMR MAP EDITOR API
# # ============================================================================

# # Database base directory
# DATABASE_DIR = 'map_databases'

# def sanitize_map_name(map_name):
#     """Sanitize map name for use as filename"""
#     sanitized = re.sub(r'[^\w\-_\. ]', '', map_name)
#     sanitized = sanitized.replace(' ', '_')
#     if not sanitized:
#         sanitized = 'default_map'
#     return sanitized

# def get_database_path(map_name):
#     """Get database path for a specific map"""
#     if not os.path.exists(DATABASE_DIR):
#         os.makedirs(DATABASE_DIR)
    
#     safe_map_name = sanitize_map_name(map_name)
#     database_name = f"{safe_map_name}.db"
#     return os.path.join(DATABASE_DIR, database_name)

# def get_db_connection(map_name):
#     """Get database connection for a specific map with error handling"""
#     try:
#         db_path = get_database_path(map_name)
#         conn = sqlite3.connect(db_path)
#         conn.row_factory = sqlite3.Row
#         return conn
#     except sqlite3.Error as e:
#         logger.error(f"Database connection error for map '{map_name}': {e}")
#         raise

# def init_database_for_map(map_name):
#     """Initialize database for a specific map with simplified schema"""
#     try:
#         conn = get_db_connection(map_name)
        
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

# # ============================================================================
# # HELPER FUNCTIONS
# # ============================================================================

# def is_allowed_client(ip_str: str) -> bool:
#     """Check if client IP is allowed"""
#     if ALLOWED_NETWORK is None:
#         return True

#     try:
#         ip_addr = ipaddress.ip_address(ip_str)
#     except Exception:
#         return False

#     if ip_addr.is_loopback:
#         return True

#     return ip_addr in ALLOWED_NETWORK

# def fault_checker():
#     """Check for sensor faults"""
#     while True:
#         now = time.time()
#         if ROS_AVAILABLE:
#             with robot_status_lock:
#                 for sensor, timeout in FAULT_TIMEOUTS.items():
#                     status_key = f"{sensor}_status"
#                     last_t = last_msg_time.get(sensor, 0.0)
#                     robot_status_data[status_key] = "FAULT" if (now - last_t > timeout) else "OK"

#                 if robot_status_data.get("motor_status") != "OK":
#                     robot_status_data["robot_status"] = "IDLE"
#         time.sleep(0.1)

# def start_ros_node():
#     """Start ROS2 node in a separate thread"""
#     if not ROS_AVAILABLE:
#         print("⚠️  ROS2 not available - skipping ROS node startup")
#         return
    
#     try:
#         rclpy.init()
        
#         # Create both ROS nodes
#         amcl_node = AmclPoseListener()
#         status_node = RobotStatusMonitor()
        
#         print("🚀 Starting combined ROS2 nodes...")
#         print("💡 Make sure your ROS bag is playing: ros2 bag play my_amcl_bag_0.db3")
#         print("📊 Waiting for ROS topics...")
        
#         # Create a multi-threaded executor for both nodes
#         executor = rclpy.executors.MultiThreadedExecutor()
#         executor.add_node(amcl_node)
#         executor.add_node(status_node)
        
#         try:
#             executor.spin()
#         except KeyboardInterrupt:
#             print("\n🛑 Shutting down ROS2 nodes...")
#         except Exception as e:
#             print(f"❌ ROS2 executor error: {e}")
#         finally:
#             amcl_node.destroy_node()
#             status_node.destroy_node()
#             executor.shutdown()
#             rclpy.shutdown()
            
#     except Exception as e:
#         print(f"❌ ROS2 initialization error: {e}")

# # ============================================================================
# # FLASK ROUTES - MODULE 1: ROS2 AMCL POSE READER
# # ============================================================================

# @app.route('/api/pose/')
# def pose_home():
#     return jsonify({
#         'message': 'ROS2 AMCL Position API',
#         'status': 'Ready to receive ROS2 data',
#         'endpoints': {
#             '/api/pose/robot_position': 'GET - Get current robot position from ROS2',
#             '/api/pose/status': 'GET - Get connection status',
#             '/api/pose/health': 'GET - Health check',
#             '/api/pose/debug': 'GET - Debug information',
#             '/api/pose/topics': 'GET - List ROS2 topics'
#         }
#     })

# @app.route('/api/pose/robot_position', methods=['GET'])
# def get_robot_position():
#     """Get the current robot position from ROS2"""
#     global latest_robot_position
    
#     if latest_robot_position['message_count'] == 0:
#         return jsonify({
#             'status': 'waiting',
#             'data': {
#                 'x': 0.0,
#                 'y': 0.0,
#                 'yaw': 0.0,
#                 'timestamp': time.time(),
#                 'message_count': 0
#             },
#             'message': 'Waiting for first AMCL message from ROS2...'
#         })
    
#     return jsonify({
#         'status': 'success',
#         'data': latest_robot_position,
#         'timestamp': time.time(),
#         'message': f'Received {latest_robot_position["message_count"]} AMCL messages'
#     })

# @app.route('/api/pose/status', methods=['GET'])
# def get_pose_status():
#     """Get detailed connection status"""
#     global latest_robot_position
    
#     status_info = {
#         'ros_connected': ROS_AVAILABLE,
#         'api_ready': True,
#         'messages_received': latest_robot_position['message_count'],
#         'last_position': latest_robot_position,
#         'last_update_age': time.time() - latest_robot_position['timestamp'] if latest_robot_position['timestamp'] > 0 else 'Never',
#         'topic': '/amcl_pose'
#     }
    
#     return jsonify(status_info)

# @app.route('/api/pose/health', methods=['GET'])
# def pose_health_check():
#     """Health check endpoint"""
#     return jsonify({
#         'status': 'healthy',
#         'service': 'ROS2 AMCL Position API',
#         'timestamp': time.time(),
#         'ros_available': ROS_AVAILABLE
#     })

# @app.route('/api/pose/debug', methods=['GET'])
# def pose_debug_info():
#     """Debug endpoint to see what's happening"""
#     global latest_robot_position
    
#     return jsonify({
#         'latest_position': latest_robot_position,
#         'system_time': time.time(),
#         'ros_available': ROS_AVAILABLE,
#         'instructions': [
#             '1. Make sure ROS2 environment is sourced',
#             '2. Play ROS2 bag: ros2 bag play my_amcl_bag_0.db3',
#             '3. Check if /amcl_pose topic has data',
#             '4. Verify Flask app is receiving data'
#         ]
#     })

# @app.route('/api/pose/topics', methods=['GET'])
# def list_pose_topics():
#     """List available ROS2 topics (for debugging)"""
#     try:
#         import subprocess
#         result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
#         topics = result.stdout.strip().split('\n') if result.stdout else []
#         return jsonify({
#             'topics': topics,
#             'amcl_pose_available': '/amcl_pose' in topics
#         })
#     except Exception as e:
#         return jsonify({'error': str(e), 'topics': []})

# # ============================================================================
# # FLASK ROUTES - MODULE 2: AMR MAP EDITOR API
# # ============================================================================

# @app.route('/api/map/')
# def map_index():
#     """Root endpoint with API information"""
#     return jsonify({
#         'status': 'success',
#         'message': 'AMR Map Editor API',
#         'endpoints': {
#             'save_nodes': 'POST /api/map/save-nodes',
#             'load_nodes': 'GET /api/map/load-nodes',
#             'clear_nodes': 'DELETE /api/map/clear-nodes',
#             'database_info': 'GET /api/map/database-info',
#             'health': 'GET /api/map/health',
#             'list_maps': 'GET /api/map/list-maps',
#             'delete_map': 'DELETE /api/map/delete-map'
#         },
#         'version': '1.0.0'
#     })

# @app.route('/api/map/health')
# def map_health_check():
#     """Health check endpoint"""
#     try:
#         db_path = get_database_path('default')
#         if not os.path.exists(db_path):
#             init_database_for_map('default')
        
#         conn = sqlite3.connect(db_path)
#         conn.execute('SELECT 1')
#         conn.close()
#         return jsonify({
#             'status': 'success',
#             'message': 'Map API is healthy',
#             'database_system': 'per-map databases',
#             'database_directory': DATABASE_DIR,
#             'timestamp': datetime.now().isoformat()
#         })
#     except Exception as e:
#         return jsonify({
#             'status': 'error',
#             'message': f'Health check failed: {str(e)}'
#         }), 500

# @app.route('/api/map/save-nodes', methods=['POST'])
# def save_nodes():
#     """Save nodes, arrows, and zones to database"""
#     try:
#         data = request.get_json()
#         if not data:
#             return jsonify({
#                 'status': 'error',
#                 'message': 'No JSON data provided'
#             }), 400
        
#         map_name = data.get('mapName', 'default')
        
#         # Initialize database for this map if it doesn't exist
#         db_path = get_database_path(map_name)
#         if not os.path.exists(db_path):
#             init_database_for_map(map_name)
        
#         nodes = data.get('nodes', [])
#         arrows = data.get('arrows', [])
#         zones = data.get('zones', [])
        
#         logger.info(f"💾 Saving {len(nodes)} nodes, {len(arrows)} arrows, {len(zones)} zones for map '{map_name}'")
        
#         conn = get_db_connection(map_name)
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

# @app.route('/api/map/load-nodes', methods=['GET'])
# def load_nodes():
#     """Load nodes, arrows, and zones from database"""
#     try:
#         map_name = request.args.get('map_name', 'default')
#         logger.info(f"📂 Loading data for map '{map_name}'")
        
#         db_path = get_database_path(map_name)
        
#         # Check if database exists
#         if not os.path.exists(db_path):
#             return jsonify({
#                 'status': 'error',
#                 'message': f'No database found for map: {map_name}',
#                 'suggestion': 'Save nodes first to create database'
#             }), 404
        
#         conn = get_db_connection(map_name)
        
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

# @app.route('/api/map/clear-nodes', methods=['DELETE'])
# def clear_nodes():
#     """Clear all nodes, arrows, and zones for a specific map"""
#     try:
#         map_name = request.args.get('map_name', 'default')
#         logger.info(f"🗑️ Clearing data for map '{map_name}'")
        
#         db_path = get_database_path(map_name)
        
#         # Check if database exists
#         if not os.path.exists(db_path):
#             return jsonify({
#                 'status': 'error',
#                 'message': f'No database found for map: {map_name}',
#                 'suggestion': 'Save nodes first to create database'
#             }), 404
        
#         conn = get_db_connection(map_name)
        
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

# @app.route('/api/map/list-maps', methods=['GET'])
# def list_maps():
#     """List all available maps/databases"""
#     try:
#         if not os.path.exists(DATABASE_DIR):
#             os.makedirs(DATABASE_DIR)
#             return jsonify({
#                 'status': 'success',
#                 'maps': [],
#                 'total': 0,
#                 'message': 'No databases created yet'
#             })
        
#         # Get all .db files in the directory
#         db_files = [f for f in os.listdir(DATABASE_DIR) if f.endswith('.db')]
        
#         maps_info = []
#         for db_file in db_files:
#             map_name = os.path.splitext(db_file)[0].replace('_', ' ')
#             db_path = os.path.join(DATABASE_DIR, db_file)
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
#             'database_directory': DATABASE_DIR,
#             'timestamp': datetime.now().isoformat()
#         })
        
#     except Exception as e:
#         logger.error(f"❌ Error listing maps: {e}")
#         return jsonify({
#             'status': 'error',
#             'message': f'Failed to list maps: {str(e)}'
#         }), 500

# @app.route('/api/map/delete-map', methods=['DELETE'])
# def delete_map():
#     """Delete a map database file"""
#     try:
#         map_name = request.args.get('map_name', 'default')
        
#         if not map_name:
#             return jsonify({
#                 'status': 'error',
#                 'message': 'Map name is required'
#             }), 400
        
#         db_path = get_database_path(map_name)
        
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
# # FLASK ROUTES - MODULE 3: ROS2 STATUS BRIDGE
# # ============================================================================

# @app.route('/api/status/')
# def status_home():
#     return jsonify({
#         'message': 'ROS2 Status Bridge API',
#         'endpoints': {
#             '/api/status/': 'GET - Get robot status',
#             '/api/status/health': 'GET - Health check'
#         }
#     })

# @app.route('/api/status/health', methods=['GET'])
# def status_health_check():
#     """Health check endpoint for status bridge"""
#     return jsonify({
#         'status': 'healthy',
#         'service': 'ROS2 Status Bridge',
#         'timestamp': time.time(),
#         'ros_available': ROS_AVAILABLE
#     })

# @app.route('/api/status/', methods=['GET'])
# @app.route('/api/status/status', methods=['GET'])
# def get_status_bridge():
#     """Get robot status with network filtering"""
#     client_ip = request.remote_addr
    
#     # If ROS is not available, return offline status
#     if not ROS_AVAILABLE:
#         return jsonify({
#             "robot_status": "OFFLINE",
#             "imu_status": "FAULT",
#             "lidar_status": "FAULT",
#             "camera_status": "FAULT",
#             "motor_status": "FAULT",
#             "battery": "N/A",
#             "battery_voltage": 0.0,
#             "robot_position": None,
#             "error_status": "ROS_NOT_AVAILABLE",
#             "ros_available": False
#         })
    
#     # Check if client is allowed
#     if not is_allowed_client(client_ip):
#         logger.warning("Unauthorized /status request from %s", client_ip)
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
#         snapshot['ros_available'] = True
#     return jsonify(snapshot)

# # ============================================================================
# # MAIN APPLICATION ROUTES
# # ============================================================================

# @app.route('/')
# def root():
#     """Main application root"""
#     return jsonify({
#         'status': 'success',
#         'message': 'Combined AMR Control System API',
#         'version': '1.0.0',
#         'modules': {
#             'pose': 'ROS2 AMCL Pose Reader - /api/pose/',
#             'map': 'AMR Map Editor API - /api/map/',
#             'status': 'ROS2 Status Bridge - /api/status/'
#         },
#         'health_checks': {
#             'pose': '/api/pose/health',
#             'map': '/api/map/health',
#             'status': '/api/status/health'
#         }
#     })

# @app.route('/health')
# def combined_health():
#     """Combined health check for all modules"""
#     return jsonify({
#         'status': 'healthy',
#         'timestamp': time.time(),
#         'modules': {
#             'pose': 'active',
#             'map': 'active',
#             'status': 'active'
#         },
#         'ros_available': ROS_AVAILABLE,
#         'database_system': 'active'
#     })

# # ============================================================================
# # ERROR HANDLERS
# # ============================================================================

# @app.errorhandler(404)
# def not_found(error):
#     return jsonify({
#         'status': 'error',
#         'message': 'Endpoint not found',
#         'available_modules': ['/api/pose/', '/api/map/', '/api/status/']
#     }), 404

# @app.errorhandler(500)
# def internal_error(error):
#     return jsonify({
#         'status': 'error',
#         'message': 'Internal server error'
#     }), 500

# # ============================================================================
# # MAIN
# # ============================================================================

# if __name__ == '__main__':
#     app_start_time = time.time()
    
#     # Check if ROS2 is available
#     if ROS_AVAILABLE:
#         try:
#             import subprocess
#             result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
#             if result.returncode == 0:
#                 print(f"✅ ROS2 found: {result.stdout.strip()}")
#             else:
#                 print("❌ ROS2 not found. Make sure to source your ROS2 environment.")
#                 print("   Example: source /opt/ros/humble/setup.bash")
#         except Exception as e:
#             print(f"❌ Error checking ROS2: {e}")
#     else:
#         print("⚠️  Running in API-only mode (ROS2 not available)")
    
#     # Start ROS2 node in a separate thread if available
#     if ROS_AVAILABLE:
#         ros_thread = threading.Thread(target=start_ros_node, daemon=True)
#         ros_thread.start()
        
#         # Start fault checker thread
#         fault_thread = threading.Thread(target=fault_checker, daemon=True)
#         fault_thread.start()
    
#     # Create database directory if it doesn't exist
#     if not os.path.exists(DATABASE_DIR):
#         os.makedirs(DATABASE_DIR)
#         print(f"📁 Created database directory: {DATABASE_DIR}")
    
#     # ============================================================================
#     # DISPLAY ORIGINAL APP MESSAGES
#     # ============================================================================
    
#     print("\n" + "="*60)
#     print("🎒 ROS2 AMCL POSITION READER (Originally port 5001)")
#     print("="*60)
#     print("📍 Combined Flask API Endpoints:")
#     print("   http://localhost:5001/robot_position - Get robot position")
#     print("   http://localhost:5001/status - Connection status") 
#     print("   http://localhost:5001/debug - Debug information")
#     print("   http://localhost:5001/health - Health check")
#     print("   http://localhost:5001/topics - List ROS2 topics")
#     print("\n🎯 Next Steps:")
#     print("   1. Keep this Flask app running")
#     print("   2. Open a NEW terminal")
#     print("   3. Source your ROS2 environment")
#     print("   4. Play your ROS2 bag: ros2 bag play my_amcl_bag_0.db3")
#     print("   5. Watch for position updates above!")
#     print("   6. Open your React frontend to see the robot move")
#     print("="*60)
    
#     print("\n" + "="*60)
#     print("🗺️  AMR MAP EDITOR API (Originally port 5000)")
#     print("="*60)
#     print("📁 Database system: Per-map databases")
#     print(f"📂 Database directory: {os.path.abspath(DATABASE_DIR)}")
#     print("📊 Simplified schema: map_name, node_type, x, y, z")
#     print("✅ API ready!")
#     print("\n📍 Combined Endpoints:")
#     print("   POST /api/map/save-nodes - Save map data")
#     print("   GET  /api/map/load-nodes - Load map data")
#     print("   GET  /api/map/list-maps - List available maps")
#     print("   DELETE /api/map/clear-nodes - Clear map data")
#     print("   DELETE /api/map/delete-map - Delete a map")
#     print("="*60)
    
#     print("\n" + "="*60)
#     print("📊 ROS2 STATUS BRIDGE (Originally port 5002)")
#     print("="*60)
#     print("Flask API running: http://127.0.0.1:5002/status (listening on 0.0.0.0:5002)")
#     print("\n📍 Combined Endpoints:")
#     print("   GET /api/status/ - Get robot status")
#     print("   GET /api/status/health - Health check")
#     print("="*60)
    
#     print("\n" + "="*60)
#     print("🤖 COMBINED AMR CONTROL SYSTEM API")
#     print("="*60)
#     print("🚀 Starting combined server on PORT 5000...")
#     print("   📍 Local: http://localhost:5000")
#     print("   🌍 Network: http://0.0.0.0:5000")
#     print("\n🎯 Combined Endpoints:")
#     print("   🌐 Root: http://localhost:5000/")
#     print("   🩺 Health: http://localhost:5000/health")
#     print("\n   📍 Pose API (app1.py): /api/pose/")
#     print("   🗺️  Map API (app.py): /api/map/")
#     print("   📊 Status API (topicstatus.py): /api/status/")
#     print("="*60)
    
#     # Run Flask app on port 5000 (original app.py port)
#     app.run(debug=True, host='0.0.0.0', port=5000, use_reloader=False)