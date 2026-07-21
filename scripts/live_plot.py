#!/usr/bin/env python3
"""
Live plot: estimated contact force (Fz) vs CB1's resulting angle, over time.
Demonstrates the admittance control response for reports/demos.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque
import time

WINDOW_SECONDS = 20


class LivePlotNode(Node):
    def __init__(self):
        super().__init__('live_plot')
        self.t0 = time.time()
        self.times_f = deque()
        self.forces = deque()
        self.times_a = deque()
        self.angles = deque()

        self.create_subscription(WrenchStamped, '/wrench_estimate', self.wrench_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

    def wrench_cb(self, msg):
        t = time.time() - self.t0
        self.times_f.append(t)
        self.forces.append(msg.wrench.force.z)
        while self.times_f and self.times_f[0] < t - WINDOW_SECONDS:
            self.times_f.popleft()
            self.forces.popleft()

    def joint_cb(self, msg):
        if 'CB1' not in msg.name:
            return
        idx = msg.name.index('CB1')
        t = time.time() - self.t0
        self.times_a.append(t)
        self.angles.append(msg.position[idx])
        while self.times_a and self.times_a[0] < t - WINDOW_SECONDS:
            self.times_a.popleft()
            self.angles.popleft()


def main():
    rclpy.init()
    node = LivePlotNode()

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    fig.suptitle('Admittance Control Response - CB1')

    ax1.set_ylabel('Estimated Force Fz (N)')
    ax1.grid(True, alpha=0.3)
    ax1.ticklabel_format(useOffset=False, style='plain', axis='y')
    line1, = ax1.plot([], [], color='tab:red')

    ax2.set_ylabel('CB1 angle (rad)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, alpha=0.3)
    ax2.ticklabel_format(useOffset=False, style='plain', axis='y')
    line2, = ax2.plot([], [], color='tab:blue')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            if node.times_f:
                line1.set_data(node.times_f, node.forces)
                ax1.relim(); ax1.autoscale_view()
            if node.times_a:
                line2.set_data(node.times_a, node.angles)
                ax2.relim(); ax2.autoscale_view()

            fig.canvas.draw_idle()
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
