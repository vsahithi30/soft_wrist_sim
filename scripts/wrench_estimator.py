#!/usr/bin/env python3
"""
Geometric wrench estimator for the soft wrist peg.
Tracks the peg tip's real-time world position via TF and estimates a
contact force whenever it penetrates the table surface (virtual spring model).
This is a Phase 1 stand-in for a true force-torque sensor.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from tf2_ros import Buffer, TransformListener
import numpy as np

# Peg tip location in sw_base's own local mesh frame (measured from STL)
PEG_TIP_LOCAL = np.array([0.0248, -0.0018, -0.1472])

# Table contact surface height in world Z (table top 0.36 + peg_hole_target 0.02 = 0.38)
TABLE_SURFACE_Z = 0.38

# Virtual spring stiffness (N/m) - how much force per meter of penetration
K_CONTACT = 500.0


def quat_to_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ])


class WrenchEstimator(Node):
    def __init__(self):
        super().__init__('wrench_estimator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(WrenchStamped, '/wrench_estimate', 10)
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz
        self.get_logger().info('Wrench estimator started, waiting for TF...')

    def update(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'sw_base', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}', throttle_duration_sec=2.0)
            return

        pos = np.array([t.transform.translation.x,
                         t.transform.translation.y,
                         t.transform.translation.z])
        q = t.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)

        peg_tip_world = pos + R @ PEG_TIP_LOCAL
        penetration = TABLE_SURFACE_Z - peg_tip_world[2]

        force_z = K_CONTACT * penetration if penetration > 0 else 0.0

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.wrench.force.z = float(force_z)
        self.pub.publish(msg)

        if force_z > 0.01:
            self.get_logger().info(
                f'Peg tip Z={peg_tip_world[2]:.4f}  penetration={penetration:.4f}m  Fz={force_z:.2f}N',
                throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = WrenchEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
