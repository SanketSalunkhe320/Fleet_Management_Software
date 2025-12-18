import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class JetAutoController(Node):
    def __init__(self):
        super().__init__('mission_executor')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)

        start = time.time()
        while time.time() - start < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        self.stop()

    def stop(self):
        self.cmd_pub.publish(Twist())
