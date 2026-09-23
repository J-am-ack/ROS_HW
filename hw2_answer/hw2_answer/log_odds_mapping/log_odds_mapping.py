"""Build a log-odds occupancy grid from laser rays and odometry."""

from collections import deque
import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LogOddsMapper(Node):
    """Vote for free ray cells and occupied laser endpoints separately."""

    def __init__(self):
        super().__init__('log_odds_mapper')
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
        self.create_timer(1.0, self.publish_map)

        self.resolution = 0.10
        self.laser_x = -0.05
        self.laser_y = 0.0
        self.free_vote = -0.40
        self.occupied_vote = 0.85
        self.min_log_odds = -4.0
        self.max_log_odds = 4.0
        self.odom_history = deque(maxlen=200)
        self.log_odds = {}

    def odom_callback(self, msg):
        """Keep a short pose history for matching laser timestamps."""
        self.odom_history.append((
            self.stamp_to_seconds(msg.header.stamp),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.yaw_from_quaternion(msg.pose.pose.orientation),
        ))

    def scan_callback(self, msg):
        """Cast each hit beam: traversed cells are free, its endpoint occupied."""
        pose = self.closest_odom_pose(self.stamp_to_seconds(msg.header.stamp))
        if pose is None:
            return

        robot_x, robot_y, yaw = pose
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        laser_world_x = robot_x + cos_yaw * self.laser_x - sin_yaw * self.laser_y
        laser_world_y = robot_y + sin_yaw * self.laser_x + cos_yaw * self.laser_y
        start_cell = self.world_to_cell(laser_world_x, laser_world_y)
        angle = msg.angle_min

        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min < distance < msg.range_max:
                end_world_x = laser_world_x + distance * math.cos(yaw + angle)
                end_world_y = laser_world_y + distance * math.sin(yaw + angle)
                end_cell = self.world_to_cell(end_world_x, end_world_y)
                ray_cells = self.bresenham(start_cell, end_cell)
                for cell in ray_cells[:-1]:
                    self.add_vote(cell, self.free_vote)
                self.add_vote(end_cell, self.occupied_vote)
            angle += msg.angle_increment

    def add_vote(self, cell, vote):
        """Accumulate and clamp evidence so old readings cannot dominate forever."""
        value = self.log_odds.get(cell, 0.0) + vote
        self.log_odds[cell] = max(self.min_log_odds, min(self.max_log_odds, value))

    def closest_odom_pose(self, stamp):
        """Return the odometry sample closest to the scan timestamp."""
        if not self.odom_history:
            return None
        _, x, y, yaw = min(self.odom_history, key=lambda sample: abs(sample[0] - stamp))
        return x, y, yaw

    def world_to_cell(self, x, y):
        """Project a map-frame point onto an integer grid cell."""
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    @staticmethod
    def bresenham(start, end):
        """Return every grid cell touched by the integer line from start to end."""
        x0, y0 = start
        x1, y1 = end
        cells = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx + dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                return cells
            doubled_error = 2 * error
            if doubled_error >= dy:
                error += dy
                x0 += step_x
            if doubled_error <= dx:
                error += dx
                y0 += step_y

    def publish_map(self):
        """Publish the current evidence as a ROS OccupancyGrid."""
        if not self.log_odds:
            return

        cells_x = [cell[0] for cell in self.log_odds]
        cells_y = [cell[1] for cell in self.log_odds]
        margin = 10
        min_x, max_x = min(cells_x) - margin, max(cells_x) + margin
        min_y, max_y = min(cells_y) - margin, max(cells_y) + margin
        width = max_x - min_x + 1
        height = max_y - min_y + 1
        data = [-1] * (width * height)

        for (cell_x, cell_y), value in self.log_odds.items():
            probability = 1.0 / (1.0 + math.exp(-value))
            index = (cell_y - min_y) * width + (cell_x - min_x)
            data[index] = round(100.0 * probability)

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
        """Extract yaw without depending on tf_transformations."""
        numerator = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        denominator = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        return math.atan2(numerator, denominator)


def main(args=None):
    rclpy.init(args=args)
    node = LogOddsMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
