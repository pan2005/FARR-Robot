#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuStats(Node):
    def __init__(self, topic: str, duration: float):
        super().__init__("farr_imu_stats")
        self.topic = topic
        self.duration = duration
        self.start = time.time()
        self.rows = []
        self.sub = self.create_subscription(Imu, topic, self.callback, 200)

    def callback(self, msg: Imu):
        self.rows.append(
            (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
        )

    def done(self) -> bool:
        return time.time() - self.start >= self.duration


def mean(values):
    return sum(values) / len(values)


def std(values, avg):
    return math.sqrt(sum((value - avg) ** 2 for value in values) / len(values))


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/livox/imu"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    rclpy.init()
    node = ImuStats(topic, duration)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    rows = node.rows
    node.destroy_node()
    rclpy.shutdown()

    if not rows:
        print("ERROR: no IMU samples received")
        return 2

    cols = list(zip(*rows))
    labels = ["gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z"]
    print(f"samples={len(rows)} duration={duration:.1f}s topic={topic}")
    for label, values in zip(labels, cols):
        avg = mean(values)
        print(f"{label}: mean={avg:.8f} std={std(values, avg):.8f} min={min(values):.8f} max={max(values):.8f}")

    gyro_norms = [math.sqrt(row[0] ** 2 + row[1] ** 2 + row[2] ** 2) for row in rows]
    acc_norms = [math.sqrt(row[3] ** 2 + row[4] ** 2 + row[5] ** 2) for row in rows]
    gyro_avg = mean(gyro_norms)
    acc_avg = mean(acc_norms)
    print(f"gyro_norm: mean={gyro_avg:.8f} std={std(gyro_norms, gyro_avg):.8f}")
    print(f"acc_norm: mean={acc_avg:.8f} std={std(acc_norms, acc_avg):.8f}")


if __name__ == "__main__":
    raise SystemExit(main())
