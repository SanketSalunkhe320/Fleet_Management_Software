
# #!/usr/bin/env python3
# """
# Flask API that reads AMCL pose data from ROS2 in real-time
# Make sure ROS2 environment is sourced and bag is playing
# """

# from flask import Flask, jsonify
# from flask_cors import CORS
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseWithCovarianceStamped
# import math
# import threading
# import time
# import os

# app = Flask(__name__)
# CORS(app)

# # Global variable to store the latest robot position
# latest_robot_position = {
#     'x': 0.0,
#     'y': 0.0,
#     'yaw': 0.0,
#     'timestamp': 0,
#     'message_count': 0,
#     'status': 'waiting'
# }

# class AmclPoseListener(Node):
#     def __init__(self):
#         super().__init__('amcl_pose_listener')
        
#         # Subscribe to AMCL pose topic
#         self.subscription = self.create_subscription(
#             PoseWithCovarianceStamped,
#             '/amcl_pose',
#             self.listener_callback,
#             10)
        
#         self.message_count = 0
#         self.get_logger().info("🎯 AMCL Pose Listener Started")
#         self.get_logger().info("📡 Waiting for ROS bag data on /amcl_pose...")

#     def listener_callback(self, msg):
#         global latest_robot_position
        
#         try:
#             # Extract position
#             x = msg.pose.pose.position.x
#             y = msg.pose.pose.position.y

#             # Extract orientation (quaternion → yaw)
#             q = msg.pose.pose.orientation
#             # Convert quaternion to yaw (rotation around Z axis)
#             yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
#                              1.0 - 2.0 * (q.y**2 + q.z**2))

#             # Update global position
#             self.message_count += 1
#             latest_robot_position = {
#                 'x': x,
#                 'y': y,
#                 'yaw': math.degrees(yaw),
#                 'timestamp': time.time(),
#                 'message_count': self.message_count,
#                 'header_stamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
#                 'status': 'success'
#             }

#             # Print to terminal with colorful output
#             print(f"🎯 AMCL #{self.message_count:02d}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°")
            
#         except Exception as e:
#             print(f"❌ Error processing AMCL message: {e}")
#             latest_robot_position['status'] = 'error'

# def start_ros_node():
#     """Start ROS2 node in a separate thread"""
#     try:
#         rclpy.init()
#         node = AmclPoseListener()
        
#         print("🚀 Starting ROS2 AMCL Pose Listener...")
#         print("💡 Make sure your ROS bag is playing: ros2 bag play my_amcl_bag_0.db3")
#         print("📊 Waiting for AMCL pose data on /amcl_pose topic...")
        
#         # Use a simple spinner
#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.1)
            
#     except KeyboardInterrupt:
#         print("\n🛑 Shutting down ROS2 node...")
#     except Exception as e:
#         print(f"❌ ROS2 node error: {e}")
#     finally:
#         try:
#             node.destroy_node()
#             rclpy.shutdown()
#         except:
#             pass

# # Flask API Routes
# @app.route('/')
# def home():
#     return jsonify({
#         'message': 'ROS2 AMCL Position API',
#         'status': 'Ready to receive ROS2 data',
#         'instructions': 'Play your ROS2 bag: ros2 bag play my_amcl_bag_0.db3',
#         'endpoints': {
#             '/robot_position': 'GET - Get current robot position from ROS2',
#             '/status': 'GET - Get connection status',
#             '/health': 'GET - Health check',
#             '/debug': 'GET - Debug information'
#         }
#     })

# @app.route('/robot_position', methods=['GET'])
# def get_robot_position():
#     """Get the current robot position from ROS2"""
#     global latest_robot_position
    
#     # If no messages received yet
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
#             'message': 'Waiting for first AMCL message from ROS2... Make sure bag is playing.'
#         })
    
#     return jsonify({
#         'status': 'success',
#         'data': latest_robot_position,
#         'timestamp': time.time(),
#         'message': f'Received {latest_robot_position["message_count"]} AMCL messages'
#     })

# @app.route('/status', methods=['GET'])
# def get_status():
#     """Get detailed connection status"""
#     global latest_robot_position
    
#     status_info = {
#         'ros_connected': True,
#         'api_ready': True,
#         'messages_received': latest_robot_position['message_count'],
#         'last_position': latest_robot_position,
#         'last_update_age': time.time() - latest_robot_position['timestamp'] if latest_robot_position['timestamp'] > 0 else 'Never',
#         'topic': '/amcl_pose'
#     }
    
#     return jsonify(status_info)

# @app.route('/health', methods=['GET'])
# def health_check():
#     """Health check endpoint"""
#     return jsonify({
#         'status': 'healthy',
#         'service': 'ROS2 AMCL Position API',
#         'timestamp': time.time(),
#         'ros_available': True
#     })

# @app.route('/debug', methods=['GET'])
# def debug_info():
#     """Debug endpoint to see what's happening"""
#     global latest_robot_position
    
#     return jsonify({
#         'latest_position': latest_robot_position,
#         'system_time': time.time(),
#         'uptime_seconds': time.time() - app_start_time,
#         'instructions': [
#             '1. Make sure ROS2 environment is sourced',
#             '2. Play ROS2 bag: ros2 bag play my_amcl_bag_0.db3',
#             '3. Check if /amcl_pose topic has data: ros2 topic echo /amcl_pose',
#             '4. Verify Flask app is receiving data'
#         ]
#     })

# @app.route('/topics', methods=['GET'])
# def list_topics():
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

# if __name__ == '__main__':
#     app_start_time = time.time()
    
#     # Check if ROS2 is available
#     try:
#         import subprocess
#         result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
#         if result.returncode == 0:
#             print(f"✅ ROS2 found: {result.stdout.strip()}")
#         else:
#             print("❌ ROS2 not found. Make sure to source your ROS2 environment.")
#             print("   Example: source /opt/ros/humble/setup.bash")
#     except Exception as e:
#         print(f"❌ Error checking ROS2: {e}")
    
#     # Start ROS2 node in a separate thread
#     ros_thread = threading.Thread(target=start_ros_node, daemon=True)
#     ros_thread.start()
    
#     print("\n" + "="*60)
#     print("🎒 ROS2 AMCL POSITION READER")
#     print("="*60)
#     print("📍 Flask API Endpoints:")
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
#     print("="*60 + "\n")
    
#     # Start Flask app
#     app.run(debug=True, host='0.0.0.0', port=5001, use_reloader=False)


#!/usr/bin/env python3
"""
Flask API that reads AMCL pose data from ROS2 in real-time and stores it in database
Make sure ROS2 environment is sourced and bag is playing
"""

from flask import Flask, jsonify, request
from flask_cors import CORS
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import threading
import time
import os
from dataclasses import dataclass

app = Flask(__name__)
CORS(app)

# Database configuration (SQLite)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///robot_positions.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

# Database Model
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

# Create tables
with app.app_context():
    db.create_all()

# Global variable to store the latest robot position
latest_robot_position = {
    'x': 0.0,
    'y': 0.0,
    'yaw': 0.0,
    'timestamp': 0,
    'message_count': 0,
    'status': 'waiting'
}

class AmclPoseListener(Node):
    def __init__(self):
        super().__init__('amcl_pose_listener')
        
        # Subscribe to AMCL pose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        
        self.message_count = 0
        self.get_logger().info("🎯 AMCL Pose Listener Started")
        self.get_logger().info("📡 Waiting for ROS bag data on /amcl_pose...")

    def listener_callback(self, msg):
        global latest_robot_position
        
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
            latest_robot_position = {
                'x': x,
                'y': y,
                'yaw': yaw_degrees,
                'timestamp': time.time(),
                'message_count': self.message_count,
                'header_stamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
                'status': 'success'
            }

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

            # Print to terminal with colorful output
            print(f"🎯 AMCL #{self.message_count:02d}: x={x:.3f}, y={y:.3f}, yaw={yaw_degrees:.1f}°")
            
        except Exception as e:
            print(f"❌ Error processing AMCL message: {e}")
            latest_robot_position['status'] = 'error'

def start_ros_node():
    """Start ROS2 node in a separate thread"""
    try:
        rclpy.init()
        node = AmclPoseListener()
        
        print("🚀 Starting ROS2 AMCL Pose Listener...")
        print("💡 Make sure your ROS bag is playing: ros2 bag play my_amcl_bag_0.db3")
        print("📊 Waiting for AMCL pose data on /amcl_pose topic...")
        print("💾 Database ready: robot_positions.db")
        
        # Use a simple spinner
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        print("\n🛑 Shutting down ROS2 node...")
    except Exception as e:
        print(f"❌ ROS2 node error: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

# Flask API Routes
@app.route('/')
def home():
    return jsonify({
        'message': 'ROS2 AMCL Position API with Database',
        'status': 'Ready to receive ROS2 data',
        'database': 'robot_positions.db',
        'endpoints': {
            '/robot_position': 'GET - Get current robot position from ROS2',
            '/positions': 'GET - Get all stored positions from database',
            '/positions/<int:id>': 'GET - Get specific position by ID',
            '/positions/latest': 'GET - Get latest N positions',
            '/positions/count': 'GET - Get count of stored positions',
            '/positions/delete/<int:id>': 'DELETE - Delete position by ID',
            '/positions/clear': 'DELETE - Clear all positions',
            '/status': 'GET - Get connection status',
            '/health': 'GET - Health check',
            '/debug': 'GET - Debug information'
        }
    })

@app.route('/robot_position', methods=['GET'])
def get_robot_position():
    """Get the current robot position from ROS2"""
    global latest_robot_position
    
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
            'message': 'Waiting for first AMCL message from ROS2...'
        })
    
    return jsonify({
        'status': 'success',
        'data': latest_robot_position,
        'timestamp': time.time(),
        'message': f'Received {latest_robot_position["message_count"]} AMCL messages'
    })

# Database endpoints
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

# Existing routes (keep these)
@app.route('/status', methods=['GET'])
def get_status():
    """Get detailed connection status"""
    global latest_robot_position
    
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

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    try:
        db_count = RobotPosition.query.count()
        db_healthy = True
    except:
        db_count = 0
        db_healthy = False
    
    return jsonify({
        'status': 'healthy',
        'service': 'ROS2 AMCL Position API',
        'timestamp': time.time(),
        'ros_available': True,
        'database_available': db_healthy,
        'database_count': db_count
    })

@app.route('/debug', methods=['GET'])
def debug_info():
    """Debug endpoint to see what's happening"""
    global latest_robot_position
    
    try:
        db_count = RobotPosition.query.count()
        latest_10 = RobotPosition.query.order_by(RobotPosition.timestamp.desc()).limit(10).all()
    except Exception as e:
        db_count = 0
        latest_10 = []
    
    return jsonify({
        'latest_position': latest_robot_position,
        'database_count': db_count,
        'recent_positions': [pos.to_dict() for pos in latest_10],
        'system_time': time.time(),
        'uptime_seconds': time.time() - app_start_time,
        'instructions': [
            '1. Make sure ROS2 environment is sourced',
            '2. Play ROS2 bag: ros2 bag play my_amcl_bag_0.db3',
            '3. Check if /amcl_pose topic has data: ros2 topic echo /amcl_pose',
            '4. Positions are automatically saved to database'
        ]
    })

@app.route('/topics', methods=['GET'])
def list_topics():
    """List available ROS2 topics (for debugging)"""
    try:
        import subprocess
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
        topics = result.stdout.strip().split('\n') if result.stdout else []
        return jsonify({
            'topics': topics,
            'amcl_pose_available': '/amcl_pose' in topics
        })
    except Exception as e:
        return jsonify({'error': str(e), 'topics': []})

if __name__ == '__main__':
    app_start_time = time.time()
    
    # Check if ROS2 is available
    try:
        import subprocess
        result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ ROS2 found: {result.stdout.strip()}")
        else:
            print("❌ ROS2 not found. Make sure to source your ROS2 environment.")
            print("   Example: source /opt/ros/humble/setup.bash")
    except Exception as e:
        print(f"❌ Error checking ROS2: {e}")
    
    # Initialize database
    with app.app_context():
        db.create_all()
        count = RobotPosition.query.count()
        print(f"📊 Database initialized: {count} positions already stored")
    
    # Start ROS2 node in a separate thread
    ros_thread = threading.Thread(target=start_ros_node, daemon=True)
    ros_thread.start()
    
    print("\n" + "="*70)
    print("🎒 ROS2 AMCL POSITION READER WITH DATABASE")
    print("="*70)
    print("📍 Flask API Endpoints:")
    print("   http://localhost:5000/robot_position   - Get current robot position")
    print("   http://localhost:5000/positions        - Get all stored positions")
    print("   http://localhost:5000/positions/latest - Get latest positions")
    print("   http://localhost:5000/positions/count  - Get count of positions")
    print("   http://localhost:5000/status           - Connection status") 
    print("   http://localhost:5000/debug            - Debug information")
    print("   http://localhost:5000/health           - Health check")
    print("\n💾 Database: robot_positions.db")
    print("\n🎯 Next Steps:")
    print("   1. Keep this Flask app running")
    print("   2. Open a NEW terminal")
    print("   3. Source your ROS2 environment")
    print("   4. Play your ROS2 bag: ros2 bag play my_amcl_bag_0.db3")
    print("   5. Watch for position updates above!")
    print("   6. Positions are automatically saved to database")
    print("="*70 + "\n")
    
    # Start Flask app
    app.run(debug=True, host='0.0.0.0', port=5000, use_reloader=False)