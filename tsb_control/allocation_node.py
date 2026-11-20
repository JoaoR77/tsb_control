# Allocates generalized forces (tau_u, tau_r) to thruster and velocity commands

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class AllocationNode(Node):
    def __init__(self):
        super().__init__('allocation_node')

        # Parameters for thruster geometry + turtlebot scaling
        self.declare_parameter('thruster_separation', 0.3)
        self.declare_parameter('force_to_vel_scale', 0.2)
        self.declare_parameter('torque_to_omega_scale', 0.5)

        self.L = self.get_parameter('thruster_separation').value
        self.k_v = self.get_parameter('force_to_vel_scale').value
        self.k_w = self.get_parameter('torque_to_omega_scale').value

        self.tau_u = 0.0
        self.tau_r = 0.0

        self.create_subscription(Float32, '/tau_u', self.tau_u_cb, 10)
        self.create_subscription(Float32, '/tau_r', self.tau_r_cb, 10)

        self.left_pub = self.create_publisher(Float32, '/current_left', 10)
        self.right_pub = self.create_publisher(Float32, '/current_right', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.02, self.allocate)
        self.get_logger().info('Allocation Node is running.')


    def tau_u_cb(self, msg: Float32):
        self.tau_u = msg.data

    def tau_r_cb(self, msg: Float32):
        self.tau_r = msg.data


    def allocate(self):
        # Differential thruster allocation
        f_left = 0.5 * self.tau_u - (self.tau_r / self.L)
        f_right = 0.5 * self.tau_u + (self.tau_r / self.L)

        self.left_pub.publish(Float32(data=float(f_left)))
        self.right_pub.publish(Float32(data=float(f_right)))

        # Turtlebot3 velocity limits
        cmd = Twist()
        cmd.linear.x = max(-0.22, min(0.22, self.k_v * self.tau_u))
        cmd.angular.z = max(-2.84, min(2.84, self.k_w * self.tau_r))
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AllocationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

