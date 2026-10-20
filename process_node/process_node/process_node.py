"""ROS2 integration for local corridor following and safe obstacle avoidance."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robotsdk_msgs.msg import AllSensor, Control

from .behaviors import BehaviorManager
from .config import NavigationConfig
from .controller import SteeringController
from .perception import PoseState, ScanPerception
from .scene_detector import SceneDetector
from .state_machine import NavigationFSM


class LaserWindowAnalyzer(Node):
    """Keep the existing ROS entry point while delegating navigation logic."""

    def __init__(self, window_size=10):
        super().__init__('laser_window_analyzer')
        defaults = NavigationConfig()
        self.declare_parameter('sensorTopic', '/sync_store_node/allsensor')
        self.declare_parameter('odomTopic', '/encoder_imu_node/odom')
        for name in defaults.__dataclass_fields__:
            if name.startswith('sector_'):
                continue
            self.declare_parameter(name, getattr(defaults, name))
        values = {}
        for name in defaults.__dataclass_fields__:
            if name.startswith('sector_'):
                continue
            values[name] = self.get_parameter(name).value
        self.config = NavigationConfig(**values)
        self.pose_state = PoseState()
        self.perception = ScanPerception(self.config)
        self.scene_detector = SceneDetector(self.config)
        self.fsm = NavigationFSM(self.config)
        self.behavior_manager = BehaviorManager(self.config)
        self.controller = SteeringController(self.config)
        self.last_log_time = float('-inf')
        self.last_state = self.fsm.context.state
        self.scan_info_logged = False
        self.subscription = self.create_subscription(
            AllSensor, self.get_parameter('sensorTopic').value,
            self.laser_scan_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, self.get_parameter('odomTopic').value,
            self.odom_callback, 10)
        self.publisher_ = self.create_publisher(Control, 'control', 10)
        self.get_logger().warn(
            'Verify front_angle_deg, steer_sign and speed_forward_sign '
            'against the vehicle before autonomous motion.')

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        values = (position.x, position.y, quaternion.x, quaternion.y,
                  quaternion.z, quaternion.w)
        if not all(math.isfinite(value) for value in values):
            self.pose_state.valid = False
            return
        norm = math.sqrt(sum(value * value for value in values[2:]))
        if norm < 1e-6:
            self.pose_state.valid = False
            return
        x, y, z, w = (value / norm for value in values[2:])
        yaw = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
        self.pose_state = PoseState(position.x, position.y, yaw, True)

    def laser_scan_callback(self, msg):
        steer, speed = self.process_core(msg.urg)
        command = Control()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = msg.urg.header.frame_id
        command.steer = steer
        command.speed = speed
        self.publisher_.publish(command)

    def process_core(self, urg):
        """Run sensing, classification, FSM, behavior, and control."""
        if not self.scan_info_logged:
            self.get_logger().info(
                'LaserScan angle_min=%.4f angle_max=%.4f angle_increment=%.4f '
                'len=%d front_angle_deg=%.1f' %
                (urg.angle_min, urg.angle_max, urg.angle_increment,
                 len(urg.ranges), self.config.front_angle_deg))
            self.scan_info_logged = True
        now = self.get_clock().now().nanoseconds * 1e-9
        perception = self.perception.update(urg, self.pose_state)
        scene = self.scene_detector.detect(perception)
        state = self.fsm.update(scene, perception, self.pose_state, now)
        target = self.behavior_manager.run(
            state, perception, self.pose_state, self.fsm.context)
        command = self.controller.update(target, state, perception, now)
        if state != self.last_state:
            self.get_logger().info('[FSM] %s -> %s' %
                                   (self.last_state.name, state.name))
            self.last_state = state
        if now - self.last_log_time >= self.config.log_period_seconds:
            self.get_logger().info(
                '[CTRL] scene=%s state=%s L=%.2f FL=%.2f F=%.2f '
                'FR=%.2f R=%.2f center=%.2f target=%.2f '
                'steer=%d speed=%d confidence=%.2f' %
                (scene.name, state.name, perception.left_distance,
                 perception.front_left_distance, perception.front_distance,
                 perception.front_right_distance, perception.right_distance,
                 perception.center_error, target.error, command.steer,
                 command.speed, perception.confidence))
            self.last_log_time = now
        return command.steer, command.speed


def main(args=None):
    rclpy.init(args=args)
    node = LaserWindowAnalyzer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
