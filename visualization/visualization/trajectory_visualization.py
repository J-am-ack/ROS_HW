import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryVisualize(Node):
    def __init__(self):
        super().__init__('trajectory_visualization')
        
        # 订阅话题
        self.odom_sub = self.create_subscription(
            Odometry,
            '/hw1/odom',
            self.odom_callback,
            10)
        
        # 轨迹发布
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.path = Path()
        self.path.header.frame_id = 'map'
        
        


    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # 9.23 因为环境冲突先删掉
        # quat = msg.pose.pose.orientation
        # _, _, self.orientation = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualize()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
