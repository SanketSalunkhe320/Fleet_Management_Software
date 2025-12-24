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


# ============================ ROBOT NODE ============================
class RobotMissionExecutor(Node):

    def __init__(self):
        super().__init__('mission_executor')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_yaw = None
        self.stop_requested = False

    # -------------------- ODOM --------------------
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # -------------------- TASK EXECUTION --------------------
    def execute_task(self, task):
        self.stop_requested = False

        ttype = task.get("type")
        config = task.get("config", {})

        if ttype in ["MoveForward", "MoveReverse"]:
            self.execute_linear(ttype, config)

        elif ttype in ["RotateClockwise", "RotateAntiClockwise"]:
            self.execute_rotation(ttype, config)

        elif ttype == "Wait":
            time.sleep(config.get("duration", 0) / 1000.0)

        self.stop_robot()

    # -------------------- LINEAR --------------------
    def execute_linear(self, ttype, config):
        speed = float(config.get("speed", 0.0))
        distance = float(config.get("distance", 0.0))

        if speed == 0:
            return

        duration = abs(distance / speed)
        twist = Twist()
        twist.linear.x = speed if ttype == "MoveForward" else -speed

        start = time.time()
        while time.time() - start < duration and not self.stop_requested:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

    # -------------------- ROTATION (ODOM DEPENDENT) --------------------
    def execute_rotation(self, ttype, config):
        while self.current_yaw is None:
            time.sleep(0.01)

        angle_deg = float(config.get("angle", 0.0))
        angle_rad = math.radians(angle_deg)

        direction = -1.0 if ttype == "RotateClockwise" else 1.0

        start_yaw = self.current_yaw
        target_yaw = self.normalize_angle(start_yaw + direction * angle_rad)

        twist = Twist()

        while not self.stop_requested:
            error = self.normalize_angle(target_yaw - self.current_yaw)

            if abs(error) < math.radians(0.5):
                break

            twist.linear.x = 0.0
            twist.angular.z = max(0.1, min(0.4, abs(error))) * math.copysign(1, error)

            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.stop_robot()

    # -------------------- UTILS --------------------
    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.05)

    # -------------------- MISSION --------------------
    def execute_mission(self, mission, ws):
        mission_id = mission.get("id", "unknown")
        tasks = mission.get("tasks", [])
        loop = mission.get("loop", 1)

        self.stop_requested = False

        # 🔁 Infinite loop
        if loop == -1:
            while not self.stop_requested:
                for task in tasks:
                    if self.stop_requested:
                        break
                    self.execute_task(task)
            return

        # 🔁 Finite loops
        for _ in range(loop):
            if self.stop_requested:
                break
            for task in tasks:
                if self.stop_requested:
                    break
                self.execute_task(task)

        # ✅ Mission completed
        ws.send(json.dumps({
            "type": "mission_completed",
            "missionId": mission_id
        }))


# ============================ WEBSOCKET ============================
executor_node = None
robot_id = None


def on_message(ws, message):
    global robot_id
    data = json.loads(message)

    if data.get("type") == "welcome":
        robot_id = data.get("robotId")
        print(f"✅ Connected as {robot_id}")

    elif data.get("type") == "mission_assignment":
        mission = {
            "id": data.get("missionId", "unknown"),
            "tasks": data.get("tasks", []),
            "loop": data.get("loop", 1)
        }

        threading.Thread(
            target=executor_node.execute_mission,
            args=(mission, ws),
            daemon=True
        ).start()

    elif data.get("type") == "command" and data.get("command") in ["stop", "cancel"]:
        executor_node.stop_requested = True
        executor_node.stop_robot()


def on_open(ws):
    print("🔗 WebSocket connected")

    def heartbeat():
        while True:
            if robot_id:
                ws.send(json.dumps({
                    "type": "heartbeat",
                    "status": "idle"
                }))
            time.sleep(5)

    threading.Thread(target=heartbeat, daemon=True).start()


# ============================ MAIN ============================
def main():
    global executor_node

    rclpy.init()
    executor_node = RobotMissionExecutor()

    threading.Thread(
        target=lambda: rclpy.spin(executor_node),
        daemon=True
    ).start()

    ws = websocket.WebSocketApp(
        SERVER_URL,
        on_message=on_message,
        on_open=on_open
    )
    ws.run_forever()


if __name__ == "__main__":
    main()
