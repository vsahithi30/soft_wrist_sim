#!/usr/bin/env python3
"""
Single-DOF admittance controller for CB1.
Takes the estimated contact force (Fz) and computes a compliant joint
angle response using a virtual mass-spring-damper model:
    M * theta_ddot + D * theta_dot + K * theta = tau_ext
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

M = 1.0    # virtual inertia
D = 5.0    # virtual damping
K = 10.0   # virtual stiffness

FORCE_TO_TORQUE_GAIN = 0.1

THETA_MIN = -0.785
THETA_MAX = 0.785

DT = 0.02


class AdmittanceController(Node):
    def __init__(self):
        super().__init__('admittance_controller')
        self.theta = 0.0
        self.theta_dot = 0.0
        self.tau_ext = 0.0

        self.create_subscription(WrenchStamped, '/wrench_estimate', self.wrench_cb, 10)
        self.cmd_pub = self.create_publisher(JointTrajectory, '/soft_wrist_controller/joint_trajectory', 10)
        self.timer = self.create_timer(DT, self.update)
        self.get_logger().info('Admittance controller started for CB1.')

    def wrench_cb(self, msg):
        self.tau_ext = msg.wrench.force.z * FORCE_TO_TORQUE_GAIN

    def update(self):
        theta_ddot = (self.tau_ext - D * self.theta_dot - K * self.theta) / M
        self.theta_dot += theta_ddot * DT
        self.theta += self.theta_dot * DT
        self.theta = max(THETA_MIN, min(THETA_MAX, self.theta))

        msg = JointTrajectory()
        msg.joint_names = ['CB1', 'CB2', 'CB3']
        point = JointTrajectoryPoint()
        point.positions = [self.theta, 0.0, 0.0]
        point.time_from_start = Duration(sec=0, nanosec=int(DT * 1e9 * 2))
        msg.points = [point]
        self.cmd_pub.publish(msg)

        if abs(self.tau_ext) > 0.001:
            self.get_logger().info(
                f'tau_ext={self.tau_ext:.4f}  theta={self.theta:.4f}rad',
                throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = AdmittanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
