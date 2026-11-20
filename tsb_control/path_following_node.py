#Line-of-Sight (LOS) waypoint guidance node

import math
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32 
from geometry_msgs.msg import Pose2D, PoseArray

class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')

        # Parameters
        self.declare_parameter('cruise_velocity', 0.15)
        self.declare_parameter('acceptance_radius', 0.15)
        self.declare_parameter('waypoints_flat', [2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 0.0, 0.0])

        self.cruise_vel = self.get_parameter('cruise_velocity').value
        self.accept_radius = self.get_parameter('acceptance_radius').value

        # Pair flat list [x1, y1, x2, y2, ...] into [(x1, y1), (x2, y2), ...] 
        flat = self.get_parameter('waypoints_flat').value
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]

        self.current_wp_idx = 0
        self.x = 0.0
        self.y = 0.0


        self.create_subscription(Pose2D, '/state/pose', self.pose_cb, 10)
        self.create_subscription(PoseArray, '/waypoints', self.waypoints_cb, 10)

        self.vel_pub = self.create_publisher(Float32, '/desired_velocity', 10)
        self.head_pub = self.create_publisher(Float32, '/desired_heading', 10)

        self.create_timer(0.05, self.guidance_loop)
        self.get_logger().info(f'LOS Path Following started with {len(self.waypoints)} waypoints.')

    def pose_cb(self, msg: Pose2D):
        self.x = msg.x
        self.y = msg.y

    def waypoints_cb(self, msg: PoseArray):
        self.waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        self.current_wp_idx = 0
        self.get_logger().info(f'Received {len(self.waypoints)} new waypoints')


    def guidance_loop(self):
        # Stops if all waypoints have been visited
        if self.current_wp_idx >= len(self.waypoints):
            self.vel_pub.publish(Float32(data=0.0))
            return

        target_x, target_y = self.waypoints[self.current_wp_idx]
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)

        if distance <= self.accept_radius:
            self.get_logger().info(f'Reached waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}')
            self.current_wp_idx += 1
            return

        desired_heading = math.atan2(dy, dx)
        self.head_pub.publish(Float32(data=float(desired_heading)))
        self.vel_pub.publish(Float32(data=float(self.cruise_vel)))


def main(args=None):
     rclpy.init(args=args)
     node = PathFollowingNode()
     rclpy.spin(node)
     node.destroy_node()
     rclpy.shutdown()

if __name__ == '__main__':
     main()


