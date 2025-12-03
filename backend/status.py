import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second

        # AGV-01 will toggle location between RUNNING / IDLE
        self.locations = ["RUNNING", "IDLE"]
        self.index = 0

    def timer_callback(self):
        msg = String()
        location = self.locations[self.index]
        self.index = (self.index + 1) % len(self.locations)

        # Status is always RUNNING
        msg.data = json.dumps({
            "agv_id": "AGV-01",
            "status": "RUNNING",
            "location": location
        })
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
