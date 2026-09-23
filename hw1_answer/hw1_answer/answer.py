"""Publish planar dead-reckoning odometry from the homework sensor data."""


# finished 9.23 by yangjm

import math

import rclpy
from hw1_msgs.msg import Deadreckoning
from nav_msgs.msg import Odometry
from rclpy.node import Node



# 从Node类继承好之后，主要把航位推算写进来
class DeadReckoningNode(Node):
    """Convert heading and encoder displacement samples to odometry."""

    def __init__(self):
        super().__init__('answer')
        # 订阅和发布话题qwq
        self.subscription = self.create_subscription(
            Deadreckoning,
            '/hw1/deadreckoning_data',
            self.deadreckoning_callback,
            10,
        )
        self.odom_publisher = self.create_publisher(Odometry, '/hw1/odom', 10)

        # 航位推算状态：以 map 原点为起点，单位为米和弧度。
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def deadreckoning_callback(self, msg):
        """Integrate one encoder displacement using the IMU heading."""
        # cur_ori 是 IMU 给出的当前航向角；distance 是本采样周期的位移增量。
        self.theta = msg.cur_ori
        self.x += msg.distance * math.cos(self.theta)
        self.y += msg.distance * math.sin(self.theta)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # ROS 使用四元数描述姿态；平面运动只绕 Z 轴旋转。
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
