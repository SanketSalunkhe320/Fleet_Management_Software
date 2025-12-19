# import websocket
# import threading
# import json
# import time

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# # ---------------- ROS2 Node ---------------- #
# class RobotMissionExecutor(Node):
#     def __init__(self):
#         super().__init__('mission_executor')
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.current_mission = None

#     def execute_task(self, task):
#         twist = Twist()
#         duration = 0

#         if task['type'] == 'MoveForward':
#             speed = float(task['config']['speed'])
#             distance = float(task['config']['distance'])
#             duration = distance / speed
#             twist.linear.x = speed
#         elif task['type'] == 'MoveReverse':
#             speed = float(task['config']['speed'])
#             distance = float(task['config']['distance'])
#             duration = distance / speed
#             twist.linear.x = -speed
#         elif task['type'] == 'RotateClockwise':
#             angle = float(task['config']['angle'])
#             duration = angle / 90  # simple approximation
#             twist.angular.z = -0.8
#         elif task['type'] == 'RotateAntiClockwise':
#             angle = float(task['config']['angle'])
#             duration = angle / 90
#             twist.angular.z = 0.8
#         elif task['type'] == 'Wait':
#             duration = float(task['config']['duration']) / 1000
#             time.sleep(duration)
#             return True

#         start = time.time()
#         while time.time() - start < duration:
#             self.cmd_pub.publish(twist)
#             time.sleep(0.1)

#         # stop after task
#         self.cmd_pub.publish(Twist())
#         time.sleep(0.2)
#         return True

#     def execute_mission(self, mission, ws, robot_id):
#         self.current_mission = mission
#         for task in mission['tasks']:
#             success = self.execute_task(task)
#             # Notify server task completed
#             ws.send(json.dumps({
#                 "type": "task_completed",
#                 "taskId": task['id'],
#                 "success": success
#             }))
#         # Notify mission completed
#         ws.send(json.dumps({
#             "type": "mission_completed",
#             "missionId": mission['id']
#         }))
#         self.current_mission = None


# # ---------------- WebSocket Client ---------------- #
# SERVER_URL = "ws://localhost:3002"  # HiWonder server
# executor_node = None
# robot_id = None

# def on_message(ws, message):
#     global executor_node, robot_id
#     data = json.loads(message)
#     msg_type = data.get("type")

#     if msg_type == "welcome":
#         robot_id = data.get("robotId")
#         print(f"✅ Connected to HiWonder server as {robot_id}")

#     elif msg_type == "mission_assignment":
#         mission = {
#             "id": data.get("missionId"),
#             "name": data.get("missionName"),
#             "tasks": data.get("tasks", [])
#         }
#         print(f"🎯 Received mission: {mission['name']}")
#         # Execute mission in background
#         threading.Thread(target=executor_node.execute_mission, args=(mission, ws, robot_id), daemon=True).start()

#     elif msg_type == "command":
#         command = data.get("command")
#         print(f"⚡ Received command: {command}")
#         # handle pause/resume/stop if needed
#         if command == "stop" or command == "emergency_stop":
#             executor_node.current_mission = None
#             executor_node.cmd_pub.publish(Twist())
#         elif command == "pause":
#             executor_node.cmd_pub.publish(Twist())
#         elif command == "resume" and executor_node.current_mission:
#             # optional: resume last task
#             pass

# def on_open(ws):
#     print("🔗 WebSocket connected, sending initial heartbeat")
#     def heartbeat():
#         while True:
#             if robot_id:
#                 ws.send(json.dumps({
#                     "type": "heartbeat",
#                     "status": "idle"
#                 }))
#             time.sleep(5)
#     threading.Thread(target=heartbeat, daemon=True).start()

# def run_ws():
#     ws = websocket.WebSocketApp(
#         SERVER_URL,
#         on_message=on_message,
#         on_open=on_open
#     )
#     ws.run_forever()

# # ---------------- Main ---------------- #
# def main():
#     global executor_node
#     rclpy.init()
#     executor_node = RobotMissionExecutor()
#     threading.Thread(target=lambda: rclpy.spin(executor_node), daemon=True).start()
#     run_ws()

# if __name__ == "__main__":
#     main()



import websocket
import threading
import json
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SERVER_URL = "ws://localhost:3002"

class RobotMissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_yaw = 0.0
        self.current_mission = None
        self.stop_requested = False

    def odom_callback(self, msg):
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def execute_task(self, task):
        self.stop_requested = False
        if task['type'] in ['MoveForward', 'MoveReverse']:
            self._execute_linear(task)
        elif task['type'] in ['RotateClockwise', 'RotateAntiClockwise']:
            self._execute_rotation(task)
        elif task['type'] == 'Wait':
            time.sleep(task['config']['duration'] / 1000.0)

        self.cmd_pub.publish(Twist())  # stop
        time.sleep(0.05)
        return True

    def _execute_linear(self, task):
        speed = float(task['config']['speed'])
        distance = float(task['config']['distance'])
        duration = abs(distance / speed)
        twist = Twist()
        twist.linear.x = speed if task['type'] == 'MoveForward' else -speed
        start = time.time()
        while time.time() - start < duration and not self.stop_requested:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

    def _execute_rotation(self, task):
        angle = math.radians(task['config']['angle'])
        clockwise = task['type'] == 'RotateClockwise'
        twist = Twist()
        twist.angular.z = -0.5 if clockwise else 0.5
        rotated = 0.0
        prev_yaw = self.current_yaw
        while rotated < angle and not self.stop_requested:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
            delta = self._normalize_angle(self.current_yaw - prev_yaw)
            rotated += abs(delta)
            prev_yaw = self.current_yaw
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

    def execute_mission(self, mission, ws):
       for task in mission['tasks']:
        if self.stop_requested:
            break
        success = self.execute_task(task)
        ws.send(json.dumps({
            "type": "task_completed",
            "taskId": task["id"],
            "success": success
        }))

       ws.send(json.dumps({
        "type": "mission_completed",
        "missionId": mission["id"]
    }))

       self.current_mission = None


executor_node = None
robot_id = None

def on_message(ws, message):
    global robot_id
    data = json.loads(message)
    if data['type'] == 'welcome':
        robot_id = data['robotId']
        print(f"✅ Connected as {robot_id}")
    elif data['type'] == 'mission_assignment':
        mission = {"id": data['missionId'], "tasks": data['tasks'], "loop": data.get("loop", 1)}
        threading.Thread(target=executor_node.execute_mission, args=(mission, ws), daemon=True).start()
    elif data['type'] == 'command' and data['command'] in ['stop', 'cancel']:
        executor_node.stop_requested = True
        executor_node.cmd_pub.publish(Twist())

def on_open(ws):
    print("🔗 WebSocket connected")
    def heartbeat():
        while True:
            if robot_id:
                ws.send(json.dumps({"type": "heartbeat", "status": "idle"}))
            time.sleep(5)
    threading.Thread(target=heartbeat, daemon=True).start()

def main():
    global executor_node
    rclpy.init()
    executor_node = RobotMissionExecutor()
    threading.Thread(target=lambda: rclpy.spin(executor_node), daemon=True).start()
    ws = websocket.WebSocketApp(SERVER_URL, on_message=on_message, on_open=on_open)
    ws.run_forever()

if __name__ == "__main__":
    main()
