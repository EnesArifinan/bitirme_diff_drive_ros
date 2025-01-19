import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        # OccupancyGrid mesajına abonelik
        self.occupancy_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',  # OccupancyGrid yayınlanan topik
            self.occupancy_callback,
            10
        )
        self.occupancy_subscription  # Aboneliği aktif tutmak için saklıyoruz

        # Hedef nokta için abonelik
        self.target_subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',  # Hedef noktanın yayınlandığı topik
            self.target_callback,
            10
        )
        self.target_subscription  # Aboneliği aktif tutmak için saklıyoruz

        # Mesafe ve açı farkı için abonelik
        self.distance_subscription = self.create_subscription(
            Float64,
            '/distance',  # Mesafe verisinin yayınlandığı topik
            self.distance_callback,
            10
        )
        self.distance_subscription  # Aboneliği aktif tutmak için saklıyoruz

        self.angle_diff_subscription = self.create_subscription(
            Float64,
            '/angle_diff',  # Açı farkı verisinin yayınlandığı topik
            self.angle_diff_callback,
            10
        )
        self.angle_diff_subscription  # Aboneliği aktif tutmak için saklıyoruz

        # Yol yayını
        self.path_publisher = self.create_publisher(
            Path,
            '/planned_path',  # Planlanan yolun yayınlanacağı topik
            10
        )

        self.grid_map = None  # OccupancyGrid verisi
        self.robot_x = 0
        self.robot_y = 0

    def occupancy_callback(self, msg):
        """OccupancyGrid verisini alır ve günceller."""
        self.grid_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.get_logger().info('Occupancy grid received.')

    def target_callback(self, msg):
        """Hedef noktayı alır ve yol planlamayı başlatır."""
        if self.grid_map is None:
            self.get_logger().warning('Occupancy grid not available yet.')
            return

        target_x = int((msg.pose.position.x + (self.grid_map.shape[1] * msg.info.resolution / 2)) / msg.info.resolution)
        target_y = int((msg.pose.position.y + (self.grid_map.shape[0] * msg.info.resolution / 2)) / msg.info.resolution)

        # Mesafe ve açı farkı, başka bir node'dan alınıyor
        # self.distance ve self.angle_diff değerleri başka düğümlerden güncellenecek
        if hasattr(self, 'current_distance') and hasattr(self, 'current_angle_diff'):
            self.get_logger().info(f'Distance to target: {self.current_distance}, Angle difference: {self.current_angle_diff}')
        else:
            self.get_logger().warning('Distance and angle difference not available.')

        # Planlanan yolu oluştur
        self.plan_path(target_x, target_y)

    def distance_callback(self, msg):
        """Mesafe bilgisini alır."""
        self.current_distance = msg.data
        self.get_logger().info(f'Distance updated: {self.current_distance}')

    def angle_diff_callback(self, msg):
        """Açı farkını alır."""
        self.current_angle_diff = msg.data
        self.get_logger().info(f'Angle difference updated: {self.current_angle_diff}')

    def plan_path(self, target_x, target_y):
        """Hedef noktaya giden bir yolu planlar ve yayınlar."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        # Basit bir düz yol oluşturma
        for x in range(self.robot_x, target_x + 1):
            for y in range(self.robot_y, target_y + 1):
                if 0 <= x < self.grid_map.shape[1] and 0 <= y < self.grid_map.shape[0]:
                    if self.grid_map[y, x] != 100:  # Engel yoksa
                        pose = PoseStamped()
                        pose.header.stamp = path_msg.header.stamp
                        pose.header.frame_id = "map"
                        pose.pose.position.x = x * 0.1 - (self.grid_map.shape[1] * 0.1 / 2)
                        pose.pose.position.y = y * 0.1 - (self.grid_map.shape[0] * 0.1 / 2)
                        path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Planned path published.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
