"""Build an occupancy grid with laser endpoint voting and odometry poses."""

# 9.23 yangjm

from collections import deque
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


# 简易投票
class SimpleVotingMapper(Node):
    """Accumulate valid laser endpoints in an odometry-aligned grid."""

    def __init__(self):
        super().__init__('simple_voting_mapper')
        self.create_subscription(
            Odometry,
            '/encoder_imu_node/odom',
            self.odom_callback,
            50,
        )
        self.create_subscription(
            LaserScan,
            '/urg_node/scan',
            self.scan_callback,
            10,
        )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.path_publisher = self.create_publisher(Path, '/hw2/path', 10)
        self.create_timer(1.0, self.publish_map)

        self.resolution = 0.10
        self.laser_x = -0.05
        self.laser_y = 0.0
        self.odom_history = deque(maxlen=200)
        self.votes = {}
        self.path = Path()
        self.path.header.frame_id = 'map'

    def odom_callback(self, msg):
        """Store poses for timestamp matching and publish the driven path."""
        stamp = self.stamp_to_seconds(msg.header.stamp)
        yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_history.append((stamp, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp
        self.path_publisher.publish(self.path)

    def scan_callback(self, msg):
        """Vote for obstacle cells at valid laser-beam endpoints."""
        pose = self.closest_odom_pose(self.stamp_to_seconds(msg.header.stamp))
        if pose is None:
            return

        robot_x, robot_y, robot_yaw = pose
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        angle = msg.angle_min

        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min < distance < msg.range_max:
                local_x = self.laser_x + distance * math.cos(angle)
                local_y = self.laser_y + distance * math.sin(angle)
                world_x = robot_x + cos_yaw * local_x - sin_yaw * local_y
                world_y = robot_y + sin_yaw * local_x + cos_yaw * local_y
                cell = (math.floor(world_x / self.resolution), math.floor(world_y / self.resolution))
                self.votes[cell] = self.votes.get(cell, 0) + 1
            angle += msg.angle_increment

    def closest_odom_pose(self, stamp):
        """Return the odometry pose nearest to a laser timestamp."""
        if not self.odom_history:
            return None
        _, x, y, yaw = min(self.odom_history, key=lambda sample: abs(sample[0] - stamp))
        return x, y, yaw

    def publish_map(self):
        """Convert sparse votes into a dynamically sized OccupancyGrid."""
        if not self.votes:
            return

        cells_x = [cell[0] for cell in self.votes]
        cells_y = [cell[1] for cell in self.votes]
        margin = 10
        min_x, max_x = min(cells_x) - margin, max(cells_x) + margin
        min_y, max_y = min(cells_y) - margin, max(cells_y) + margin
        width = max_x - min_x + 1
        height = max_y - min_y + 1
        data = [-1] * (width * height)

        for (cell_x, cell_y), count in self.votes.items():
            index = (cell_y - min_y) * width + (cell_x - min_x)
            data[index] = min(100, count * 20)

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = min_x * self.resolution
        grid.info.origin.position.y = min_y * self.resolution
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        self.map_publisher.publish(grid)

    @staticmethod
    def stamp_to_seconds(stamp):
        """Convert a ROS time message to seconds."""
        return stamp.sec + stamp.nanosec * 1e-9

    @staticmethod
    def yaw_from_quaternion(quaternion):
        """Extract planar yaw without depending on tf_transformations."""
        numerator = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        denominator = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        return math.atan2(numerator, denominator)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVotingMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
