# Discrete 2-DOF PID controller for surge velocity and yaw heading

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, Twist


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, i_max: float, out_max: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_max = i_max
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float, dt: float):
        if dt <= 0.0:
            return 0.0
        # Integral (clamped to prevent windup)
        self.integral += error * dt
        self.integral = max(-self.i_max, min(self.i_max, self.integral))

        # Derivative
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # Sum P + I + D
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative) 
        return max(-self.out_max, min (self.out_max, output))

class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')


    # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
            ('kp_u', 1.5), ('ki_u', 0.2), ('kd_u', 0.05),
                ('kp_psi', 2.0), ('ki_psi', 0.1), ('kd_psi', 0.3),
                ('i_max', 1.0), ('tau_u_max', 5.0), ('tau_r_max', 3.0),
                ('control_rate', 50.0),
            ],
        )

        self.dt = 1.0 / self.get_parameter('control_rate').value

        self.surge_pid = PIDController(
            self.get_parameter('kp_u').value,
            self.get_parameter('ki_u').value,
            self.get_parameter('kd_u').value,
            self.get_parameter('i_max').value,
            self.get_parameter('tau_u_max').value,
        )

        self.heading_pid = PIDController(
            self.get_parameter('kp_psi').value,
            self.get_parameter('ki_psi').value,
            self.get_parameter('kd_psi').value,
            self.get_parameter('i_max').value,
            self.get_parameter('tau_r_max').value,
        )

        self.current_u = 0.0
        self.current_psi = 0.0
        self.desired_u = 0.0
        self.desired_psi = 0.0

        self.create_subscription(Twist, '/state/velocity', self.vel_cb, 10)
        self.create_subscription(Pose2D, '/state/pose', self.pose_cb, 10)
        self.create_subscription(Float32, '/desired_velocity', self.des_vel_cb, 10)
        self.create_subscription(Float32, '/desired_heading', self.des_head_cb, 10)

        self.tau_u_pub = self.create_publisher(Float32, '/tau_u', 10)
        self.tau_r_pub = self.create_publisher(Float32, '/tau_r', 10)

        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('PID node is running.')


    def vel_cb(self, msg: Twist):
        self.current_u = msg.linear.x

    def pose_cb(self, msg: Pose2D):
        self.current_psi = msg.theta

    def des_vel_cb(self, msg: Float32):
        self.desired_u = msg.data

    def des_head_cb(self, msg: Float32):
        self.desired_psi = msg.data

    def control_loop(self):
        e_u = self.desired_u - self.current_u

        # Wrap heading error to [-pi, pi] for the shortest turn
        raw_e_psi = self.desired_psi - self.current_psi
        e_psi = math.atan2(math.sin(raw_e_psi), math.cos(raw_e_psi))
       
        tau_u = self.surge_pid.compute(e_u, self.dt)
        tau_r = self.heading_pid.compute(e_psi, self.dt)
        
        self.tau_u_pub.publish(Float32(data=float(tau_u)))
        self.tau_r_pub.publish(Float32(data=float(tau_r)))


def main(args=None):
      rclpy.init(args=args)
      node = PIDNode()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
      main()