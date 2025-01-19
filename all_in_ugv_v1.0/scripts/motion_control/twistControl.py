import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# !!!!! GLOBAL PLANLAMAYA GÖRE ÇALIŞMAKTA SADECE
class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Publisher to cmd_vel
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers to distance and angle_diff topics
        self.distance_subscription = self.create_subscription(
            Float32,
            'distance',
            self.distance_callback,
            10
        )
        self.angle_diff_subscription = self.create_subscription(
            Float32,
            'angle_diff',
            self.angle_diff_callback,
            10
        )

        # Initialize variables
        self.distance = 0.0
        self.angle_diff = 0.0

        # Timer for sending navigation commands
        self.timer = self.create_timer(0.1, self.send_navigation_command)

    def distance_callback(self, msg):
        self.distance = msg.data
        self.get_logger().info(f"Received distance: {self.distance}")

    def angle_diff_callback(self, msg):
        self.angle_diff = msg.data
        self.get_logger().info(f"Received angle_diff: {self.angle_diff}")

    def send_navigation_command(self):
        # Create Twist message based on distance and angle_diff
        twist_msg = Twist()
        twist_msg.linear.x = 0.5 if self.distance > 1.0 else 0.0  # Move forward if distance > 1.0
        twist_msg.angular.z = 0.3 * self.angle_diff  # Adjust angular velocity proportional to angle_diff

        # Publish to cmd_vel
        self.velocity_publisher.publish(twist_msg)
        self.get_logger().info(f"Published Twist: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
