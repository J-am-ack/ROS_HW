"""ROS2 integration for local corridor following and safe obstacle avoidance."""

import math
import os
import sys

# Ubuntu's apt OpenCV and ROS cv_bridge were built against the distro NumPy.
# Prefer that matching installation over an unrelated /usr/local NumPy 2.x.
_distro_python = '/usr/lib/python3/dist-packages'
if os.path.isdir(_distro_python) and 'numpy' not in sys.modules:
    sys.path.insert(0, _distro_python)

import rclpy
try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:
    cv2 = None
    CvBridge = None
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from robotsdk_msgs.msg import AllSensor, Control
from sensor_msgs.msg import Image

from .behaviors import BehaviorManager
from .config import NavigationConfig
from .controller import SteeringController
from .perception import PoseState, ScanPerception
from .scene_detector import SceneDetector
from .state_machine import NavigationFSM
from .visualization import (DecisionVisualizer, RiskRecorder,
                            check_path_risk, predict_path)


class LaserWindowAnalyzer(Node):
    """Keep the existing ROS entry point while delegating navigation logic."""

    def __init__(self, window_size=10):
        super().__init__('laser_window_analyzer')
        defaults = NavigationConfig()
        self.declare_parameter('sensorTopic', '/sync_store_node/allsensor')
        self.declare_parameter('odomTopic', '/encoder_imu_node/odom')
        self.declare_parameter('visualize', True)
        self.declare_parameter('map_image_path', '')
        self.declare_parameter('map_origin_x_px', 80.0)
        self.declare_parameter('map_origin_y_px', 90.0)
        self.declare_parameter('map_pixels_per_meter_x', 11.5)
        self.declare_parameter('map_pixels_per_meter_y', 11.2)
        self.declare_parameter('risk_log_path', '/tmp/process_node_risk_events.csv')
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
        self.steer_value = 0
        self.bridge = CvBridge() if CvBridge is not None else None
        self.visualize = self.get_parameter('visualize').value
        self.visualization_available = (
            self.visualize and cv2 is not None and self.bridge is not None)
        self.visualization_warned = False
        self.risk_recorder = RiskRecorder(self.get_parameter('risk_log_path').value)
        self.latest_risk = None
        self.latest_prediction = []
        self.visualizer = None
        if self.visualization_available:
            map_path = self.get_parameter('map_image_path').value
            if not map_path:
                map_path = os.path.join(
                    get_package_share_directory('process_node'),
                    'maps', 'course_map.png')
            self.visualizer = DecisionVisualizer(
                cv2, map_path,
                (self.get_parameter('map_origin_x_px').value,
                 self.get_parameter('map_origin_y_px').value),
                (self.get_parameter('map_pixels_per_meter_x').value,
                 self.get_parameter('map_pixels_per_meter_y').value))
            if self.visualizer.map_image is None:
                self.get_logger().warning(
                    'Map image unavailable: %s; showing blank map.' % map_path)
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
        self.image_publisher_ = self.create_publisher(Image, 'xtion_color', 10)
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
        if self.visualizer is not None:
            self.visualizer.add_pose(self.pose_state)

    def laser_scan_callback(self, msg):
        steer, speed = self.process_core(msg.urg)
        stamp = msg.urg.header.stamp
        now = (stamp.sec + stamp.nanosec * 1e-9 if stamp.sec > 0 else
               self.get_clock().now().nanoseconds * 1e-9)
        self.latest_prediction = predict_path(
            self.pose_state, steer, speed, self.config)
        self.latest_risk = check_path_risk(
            self.pose_state, msg.urg, self.latest_prediction, speed,
            self.config)
        scene = self.latest_scene.name
        if scene == 'OBSTACLE':
            self.latest_risk.kind = 'OBSTACLE'
        elif scene in ('LEFT_CORNER', 'RIGHT_CORNER', 'DEAD_END'):
            self.latest_risk.kind = 'WALL_OR_CORNER'
        else:
            self.latest_risk.kind = 'SURFACE'
        try:
            new_risk = self.risk_recorder.update(
                self.latest_risk, now, scene, self.fsm.context.state.name)
            if new_risk:
                self.get_logger().warning(
                    '[RISK] level=%s kind=%s clearance=%.2fm forward=%.2fm '
                    'state=%s total=%d' %
                    (self.latest_risk.level, self.latest_risk.kind,
                     self.latest_risk.clearance, self.latest_risk.forward,
                     self.fsm.context.state.name, self.risk_recorder.count))
        except OSError as error:
            self.get_logger().error('Risk CSV write failed: %s' % error)
        if self.visualizer is not None:
            self.visualizer.step_simulation(
                self.pose_state, steer, speed, now, self.config)
        self.show_camera(msg, steer, speed)
        # AllSensor carries the camera frame inside the same message.  Expose
        # it as a normal Image topic as well as the OpenCV window.
        if msg.xtion_color.width > 0 and msg.xtion_color.height > 0:
            self.image_publisher_.publish(msg.xtion_color)
        command = Control()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = msg.urg.header.frame_id
        command.steer = steer
        command.speed = speed
        self.publisher_.publish(command)

    def show_camera(self, msg, steer, speed):
        """Display the recorded Xtion frame like the original template."""
        if not self.visualize:
            return
        if not self.visualization_available:
            if not self.visualization_warned:
                self.get_logger().warning(
                    'OpenCV/CvBridge unavailable; cv2.imshow window disabled.')
                self.visualization_warned = True
            return
        if msg.xtion_color.width == 0 or msg.xtion_color.height == 0:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg.xtion_color, desired_encoding='bgr8')
            image = cv2.flip(image, 1)
            state = self.fsm.context.state.name
            prediction = self.latest_prediction
            camera = self.visualizer.camera_view(
                image, steer, speed, self.latest_scene.name, state,
                self.fsm.context)
            preview = self.visualizer.preview_view(
                self.pose_state, msg.urg, prediction, steer, speed, state,
                self.latest_risk, self.risk_recorder.count)
            course_map = self.visualizer.map_view(
                self.pose_state, prediction, state)
            cv2.imshow('Steering Visualization', camera)
            cv2.imshow('Decision Preview', preview)
            cv2.imshow('Course Map', course_map)
            cv2.waitKey(1)
        except Exception as error:
            self.get_logger().error('OpenCV visualization failed: %s' % error)
            self.visualization_available = False

    def draw_steering_wheel(self, image):
        """Draw the steering wheel overlay used by the original template."""
        center = (100, 100)
        radius = 80
        wheel = image.copy()
        cv2.circle(wheel, center, radius, (0, 255, 0), 2)
        angle = self.steer_value * 90.0 / 600.0
        end_point = (
            int(center[0] + radius * math.cos(math.radians(90 - angle))),
            int(center[1] - radius * math.sin(math.radians(90 - angle))),
        )
        cv2.line(wheel, center, end_point, (255, 0, 0), 2)
        cv2.putText(
            wheel, 'steer: %.2f' % self.steer_value,
            (center[0] - 60, center[1] - radius - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.addWeighted(wheel, 0.7, image, 0.3, 0, image)

    def process_core(self, urg):
        """Run sensing, classification, FSM, behavior, and control."""
        if not self.scan_info_logged:
            self.get_logger().info(
                'LaserScan angle_min=%.4f angle_max=%.4f angle_increment=%.4f '
                'len=%d front_angle_deg=%.1f' %
                (urg.angle_min, urg.angle_max, urg.angle_increment,
                 len(urg.ranges), self.config.front_angle_deg))
            self.scan_info_logged = True
        stamp = urg.header.stamp
        now = (stamp.sec + stamp.nanosec * 1e-9 if stamp.sec > 0 else
               self.get_clock().now().nanoseconds * 1e-9)
        perception = self.perception.update(urg, self.pose_state)
        scene = self.scene_detector.detect(perception)
        self.latest_scene = scene
        state = self.fsm.update(scene, perception, self.pose_state, now)
        target = self.behavior_manager.run(
            state, perception, self.pose_state, self.fsm.context)
        command = self.controller.update(target, state, perception, now)
        if state != self.last_state:
            reason = (' reason=%s' % self.fsm.context.stop_reason
                      if state.name == 'EMERGENCY_STOP' else '')
            self.get_logger().info('[FSM] %s -> %s%s' %
                                   (self.last_state.name, state.name, reason))
            self.last_state = state
        if now - self.last_log_time >= self.config.log_period_seconds:
            self.get_logger().info(
                '[CTRL] scene=%s state=%s L=%.2f FL=%.2f F=%.2f '
                'FR=%.2f R=%.2f center=%.2f target=%.2f '
                'steer=%d speed=%d confidence=%.2f right_turns=%d offset=%d '
                'avoids=%d tries=%d phase=%s role=%s yaw=%.2f' %
                (scene.name, state.name, perception.left_distance,
                 perception.front_left_distance, perception.front_distance,
                 perception.front_right_distance, perception.right_distance,
                 perception.center_error, target.error, command.steer,
                 command.speed, perception.confidence,
                 self.fsm.context.completed_right_turns,
                 self.fsm.context.offset_stage,
                 self.fsm.context.completed_avoids,
                 self.fsm.context.avoid_count,
                 (self.fsm.context.avoid_phase.name
                  if state.name.startswith('AVOID') else '-'),
                 self.fsm.context.active_turn_role,
                 self.pose_state.yaw))
            self.last_log_time = now
        return command.steer, command.speed


def main(args=None):
    rclpy.init(args=args)
    node = LaserWindowAnalyzer()
    try:
        rclpy.spin(node)
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
