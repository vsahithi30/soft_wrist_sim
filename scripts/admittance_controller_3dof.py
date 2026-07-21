#!/usr/bin/env python3
"""
Admittance controller for CB1, CB2, CB3 - each an independent virtual
mass-spring-damper, all currently driven by the same estimated Fz
(single-point contact force estimate).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

M = 1.0
D = 5.0
K = 10.0
FORCE_TO_TORQUE_GAIN = 0.1

THETA_MIN = -0.785
THETA_MAX = 0.785

DT = 0.02


class Joint:
    """One independent virtual mass-spring-damper."""
    def __init__(self, name):
        self.name = name
        self.theta = 0.0
        self.theta_dot = 0.0

    def step(self, tau_ext):
        theta_ddot = (tau_ext - D * self.theta_dot - K * self.theta) / M
        self.theta_dot += theta_ddot * DT
        self.theta += self.theta_dot * DT
        self.theta = max(THETA_MIN, min(THETA_MAX, self.theta))


class AdmittanceController3DOF(Node):
    def __init__(self):
        super().__init__('admittance_controller_3dof')
        self.tau_ext = 0.0

        self.cb1 = Joint('CB1')
        self.cb2 = Joint('CB2')
        self.cb3 = Joint('CB3')

        self.create_subscription(WrenchStamped, '/wrench_estimate', self.wrench_cb, 10)
        self.cmd_pub = self.create_publisher(JointTrajectory, '/soft_wrist_controller/joint_trajectory', 10)
        self.timer = self.create_timer(DT, self.update)
        self.get_logger().info('3-DOF admittance controller started (CB1, CB2, CB3).')

    def wrench_cb(self, msg):
        self.tau_ext = msg.wrench.force.z * FORCE_TO_TORQUE_GAIN

    def update(self):
        self.cb1.step(self.tau_ext)
        self.cb2.step(self.tau_ext)
        self.cb3.step(self.tau_ext)

        msg = JointTrajectory()
        msg.joint_names = ['CB1', 'CB2', 'CB3']
        point = JointTrajectoryPoint()
        point.positions = [self.cb1.theta, self.cb2.theta, self.cb3.theta]
        point.time_from_start = Duration(sec=0, nanosec=int(DT * 1e9 * 2))
        msg.points = [point]
        self.cmd_pub.publish(msg)

        if abs(self.tau_ext) > 0.001:
            self.get_logger().info(
                f'tau_ext={self.tau_ext:.4f}  CB1={self.cb1.theta:.4f}  CB2={self.cb2.theta:.4f}  CB3={self.cb3.theta:.4f}',
                throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = AdmittanceController3DOF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
