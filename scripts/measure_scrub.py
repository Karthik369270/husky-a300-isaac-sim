#!/usr/bin/env python3
"""
Skid-steer scrub measurement.

RESOURCES.md states Clearpath compensates with wheel_separation_multiplier: 1.75
and adds that "whether that value holds in your simulator is worth measuring
rather than assuming". This node measures it.

Method: command a series of constant angular velocities with zero linear
velocity, integrate the actual yaw achieved from /odom over a fixed window,
and solve for the multiplier that reconciles commanded with actual.

Run inside the `ros` service while the simulation is playing:
    ros2 run ... or:  python3 /workspace/scripts/measure_scrub.py
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

PHYSICAL_TRACK = 0.5468      # measured from the URDF chain, see docs/urdf_audit.md
TEST_RATES = [0.3, 0.6, 0.9, 1.2]   # rad/s commanded
SETTLE_S = 1.5
MEASURE_S = 4.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScrubTest(Node):
    def __init__(self):
        super().__init__("scrub_test")
        # use_sim_time is auto-declared by rclpy in Jazzy; set it, do not
        # re-declare. Timestamps come from /clock, published by Isaac Sim.
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.yaw = None
        self.stamp = None

    def on_odom(self, msg):
        self.yaw = yaw_of(msg.pose.pose.orientation)
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def spin_for(self, seconds):
        end = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.stamp is None:
                continue
            if end is None:
                end = self.stamp + seconds
            if self.stamp >= end:
                return

    def command(self, wz):
        t = Twist()
        t.angular.z = wz
        self.pub.publish(t)

    def run(self):
        self.get_logger().info("waiting for /odom ...")
        while rclpy.ok() and self.stamp is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        rows = []
        for wz in TEST_RATES:
            self.command(wz)
            self.spin_for(SETTLE_S)

            y0, t0 = self.yaw, self.stamp
            self.spin_for(MEASURE_S)
            y1, t1 = self.yaw, self.stamp

            self.command(0.0)
            self.spin_for(1.0)

            dyaw = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
            dt = t1 - t0
            actual = dyaw / dt if dt > 0 else float("nan")
            ratio = wz / actual if actual else float("nan")
            rows.append((wz, actual, ratio, PHYSICAL_TRACK * ratio))
            self.get_logger().info(
                f"commanded {wz:.2f} rad/s -> actual {actual:.3f} rad/s "
                f"| multiplier {ratio:.3f} | effective track {PHYSICAL_TRACK*ratio:.4f} m")

        print("\n| commanded (rad/s) | actual (rad/s) | multiplier | effective track (m) |")
        print("|---|---|---|---|")
        for wz, a, r, tr in rows:
            print(f"| {wz:.2f} | {a:.3f} | {r:.3f} | {tr:.4f} |")
        good = [r for _, _, r, _ in rows if r == r]
        if good:
            print(f"\nmean multiplier = {sum(good)/len(good):.3f} "
                  f"(Clearpath hardware value: 1.75)")


def main():
    rclpy.init()
    n = ScrubTest()
    try:
        n.run()
    finally:
        n.command(0.0)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
