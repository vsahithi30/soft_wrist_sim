#!/usr/bin/env python3
"""
Per-leg differentiated admittance controller for CB1, CB2, CB3.

Each leg's torque = baseline (symmetric push) + tilt term (based on how
aligned the peg's off-center offset is with that leg's mounting position).

tilt_signal_i = dot(peg_offset_xy, leg_position_xy)
This is a first-order, small-angle-tilt approximation - not the full
parallel-mechanism Jacobian, but physically motivated and directionally correct.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

M = 1.0
D = 5.0
K = 10.0

BASE_FORCE_TO_TORQUE_GAIN = 0.1
TILT_GAIN = 30.0

THETA_MIN = -0.785
THETA_MAX = 0.785
DT = 0.02

# Real geometry from soft_wrist_mechanism.urdf.xacro / STL analysis
PEG_OFFSET_XY = np.array([0.0248, -0.0018])
LEG_POSITIONS = {
    'CB1': np.array([0.02574, 0.01332]),
    'CB2': np.array([0.00149, -0.02868]),
    'CB3': np.array([-0.02275, 0.01332]),
}


class Joint:
    def __init__(self, name):
        self.name = name
        self.theta = 0.0
        self.theta_dot = 0.0
        self.tilt_signal = np.dot(PEG_OFFSET_XY, LEG_POSITIONS[name])

    def step(self, fz):
        tau_ext = fz * BASE_FORCE_TO_TORQUE_GAIN + fz * self.tilt_signal * TILT_GAIN
        theta_ddot = (tau_ext - D * self.theta_dot - K * self.theta) / M
        self.theta_dot += theta_ddot * DT
        self.theta += self.theta_dot * DT
        self.theta = max(THETA_MIN, min(THETA_MAX, self.theta))
        return tau_ext


class DifferentiatedAdmittance(Node):
    def __init__(self):
        super().__init__('admittance_controller_differentiated')
        self.fz = 0.0
        self.joints = {name: Joint(name) for name in LEG_POSITIONS}

        self.create_subscription(WrenchStamped, '/wrench_estimate', self.wrench_cb, 10)
        self.cmd_pub = self.create_publisher(JointTrajectory, '/soft_wrist_controller/joint_trajectory', 10)
        self.timer = self.create_timer(DT, self.update)

        for name, j in self.joints.items():
            self.get_logger().info(f'{name}: tilt_signal={j.tilt_signal:.6f}')
        self.get_logger().info('Differentiated 3-DOF admittance controller started.')

    def wrench_cb(self, msg):
        self.fz = msg.wrench.force.z

    def update(self):
        taus = {name: j.step(self.fz) for name, j in self.joints.items()}

        msg = JointTrajectory()
        msg.joint_names = ['CB1', 'CB2', 'CB3']
        point = JointTrajectoryPoint()
        point.positions = [self.joints['CB1'].theta, self.joints['CB2'].theta, self.joints['CB3'].theta]
        point.time_from_start = Duration(sec=0, nanosec=int(DT * 1e9 * 2))
        msg.points = [point]
        self.cmd_pub.publish(msg)

        if self.fz > 0.01:
            self.get_logger().info(
                f'CB1={self.joints["CB1"].theta:.4f}  CB2={self.joints["CB2"].theta:.4f}  CB3={self.joints["CB3"].theta:.4f}',
                throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = DifferentiatedAdmittance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
