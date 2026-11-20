#Processes /odom and /imu data into 2D pose and surge velocity

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D, Twist


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # Convert a 3D quaternion (x, y, z, w) into a 2D yaw/heading angle (radians)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SensorProcessingNode(Node):
    def __init__(self):
        super().__init__('sensor_processing_node')

        self.surge_vel = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.has_imu = False

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.pose_pub = self.create_publisher(Pose2D, '/state/pose', 10)
        self.vel_pub = self.create_publisher(Twist, '/state/velocity', 10)

        # Publish processed state at 50 Hz
        self.create_timer(0.02, self.publish_state)
        self.get_logger().info('Sensor Processing Node is running.')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.surge_vel = msg.twist.twist.linear.x

        # If /imu hasn't sent data yet, use odometry orientation as backup
        if not self.has_imu:
            q = msg.pose.pose.orientation
            self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def imu_callback(self, msg: Imu):
        self.has_imu = True
        q = msg.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def publish_state(self):
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.yaw
        self.pose_pub.publish(pose_msg)

        vel_msg = Twist()
        vel_msg.linear.x = self.surge_vel
        self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)         
    node = SensorProcessingNode()  
    rclpy.spin(node)              
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()