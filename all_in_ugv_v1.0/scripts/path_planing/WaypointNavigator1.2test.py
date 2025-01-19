import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
import math
from pyproj import Transformer

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # GPS and IMU subscribers
        self.gps_subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10) # !!!!!! /imu/data ı deiştirdim

        # Publishers for distance and angle_diff
        self.distance_publisher = self.create_publisher(Float32, 'distance', 10)
        self.angle_diff_publisher = self.create_publisher(Float32, 'angle_diff', 10)

        # Current position and orientation variables
        self.current_x = None
        self.current_y = None
        self.origin_x = None
        self.origin_y = None
        self.yaw = None

        # Coordinate transformation setup
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:32633")  # UTM zone 33N

        # Waypoints in GPS coordinates (latitude, longitude)
        self.gps_waypoints = [(34.052235, -118.243683),  # Example: LA
                              (40.748817, -73.985428),  # Example: NYC
                              (37.774929, -122.419416)]  # Example: SF
        self.waypoints = []  # Will store UTM-transformed waypoints
        self.current_waypoint_index = 0

        self.timer = self.create_timer(1.0, self.navigate)
        self.get_logger().info("Waypoint Navigator Node Initialized")

        self.setup_waypoints()

    def setup_waypoints(self):
        """Convert GPS waypoints to UTM coordinates relative to the initial origin."""
        for lat, lon in self.gps_waypoints:
            utm_x, utm_y = self.transformer.transform(lat, lon)
            self.waypoints.append((utm_x, utm_y))

    def gps_callback(self, msg):
        # Convert GPS latitude and longitude to UTM coordinates
        utm_x, utm_y = self.transformer.transform(msg.latitude, msg.longitude)

        if self.origin_x is None and self.origin_y is None:
            # Set the initial position as the origin
            self.origin_x = utm_x
            self.origin_y = utm_y

        # Update current position relative to the origin
        self.current_x = utm_x - self.origin_x
        self.current_y = utm_y - self.origin_y

    def imu_callback(self, msg):
        # Calculate yaw from IMU orientation quaternion
        orientation_q = msg.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def navigate(self):
        if self.current_x is None or self.current_y is None or self.yaw is None:
            self.get_logger().info("Waiting for GPS and IMU data...")
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!")
            return

        # Get the current waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_index]

        # Calculate the distance and angle to the waypoint
        dx = target_x - self.current_x
        dy = target_y - self.current_y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        target_angle = math.atan2(dy, dx)

        # Check if the waypoint is reached
        if distance < 1.0:  # Tolerance of 1 meter
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
            return

        # Calculate the angular difference
        angle_diff = target_angle - self.yaw

        # Publish distance and angle_diff
        distance_msg = Float32()
        distance_msg.data = distance
        self.distance_publisher.publish(distance_msg)

        angle_diff_msg = Float32()
        angle_diff_msg.data = angle_diff
        self.angle_diff_publisher.publish(angle_diff_msg)

        self.get_logger().info(f"Heading to waypoint {self.current_waypoint_index}: "
                               f"distance={distance:.2f} m, angle_diff={angle_diff:.2f} rad")

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
