import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np


class SimpleSLAMNode(Node):
    def __init__(self):
        super().__init__('simple_slam_node')

        # Lidar verisi için abonelik
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Lidar verisinin yayınlandığı ROS2 topik
            self.lidar_callback,
            10
        )
        self.lidar_subscription  # Aboneliği aktif tutmak için saklıyoruz

        # Occupancy Grid yayını
        self.occupancy_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',  # Harita yayın topiği
            10
        )

        # Basit bir grid map tanımlıyoruz (örnek 100x100)
        self.map_width = 100
        self.map_height = 100
        self.map_resolution = 0.1  # 10cm çözünürlük
        self.grid_map = np.zeros((self.map_width, self.map_height), dtype=np.int8)

        # Başlangıç pozisyonu
        self.robot_x = self.map_width // 2
        self.robot_y = self.map_height // 2

    def lidar_callback(self, msg):
        """Lidar verisini işleyip haritayı günceller."""
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, r in enumerate(ranges):
            if msg.range_min < r < msg.range_max:
                angle = angle_min + i * angle_increment
                obstacle_x = self.robot_x + int(r * np.cos(angle) / self.map_resolution)
                obstacle_y = self.robot_y + int(r * np.sin(angle) / self.map_resolution)

                if 0 <= obstacle_x < self.map_width and 0 <= obstacle_y < self.map_height:
                    self.grid_map[obstacle_y, obstacle_x] = 100  # Engel

        # Haritayı yayınla
        self.publish_map()

    def publish_map(self):
        """Occupancy grid haritasını yayınlar."""
        occupancy_msg = OccupancyGrid()
        occupancy_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_msg.header.frame_id = "map"

        occupancy_msg.info.resolution = self.map_resolution
        occupancy_msg.info.width = self.map_width
        occupancy_msg.info.height = self.map_height
        occupancy_msg.info.origin.position.x = -self.map_width * self.map_resolution / 2
        occupancy_msg.info.origin.position.y = -self.map_height * self.map_resolution / 2
        occupancy_msg.info.origin.position.z = 0.0
        occupancy_msg.info.origin.orientation.w = 1.0

        # 2D grid map'i 1D'ye çevir
        occupancy_msg.data = self.grid_map.flatten().tolist()
        self.occupancy_publisher.publish(occupancy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
