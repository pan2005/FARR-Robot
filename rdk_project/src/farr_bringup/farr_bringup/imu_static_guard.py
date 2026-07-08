#!/usr/bin/env python3
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuStaticGuard(Node):
    def __init__(self):
        super().__init__('farr_imu_static_guard')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('window_sec', 5.0)
        self.declare_parameter('required_stable_windows', 2)
        self.declare_parameter('max_gyro_mean_abs', 0.05)
        self.declare_parameter('max_gyro_std', 0.12)
        self.declare_parameter('max_acc_norm_std', 0.35)
        self.declare_parameter('min_acc_norm_mean', 0.6)
        self.declare_parameter('max_acc_norm_mean', 1.4)
        self.declare_parameter('min_samples', 300)
        self.declare_parameter('max_wait_sec', 0.0)
        self.declare_parameter('log_period_sec', 1.0)

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.window_sec = float(self.get_parameter('window_sec').value)
        self.required_stable_windows = int(self.get_parameter('required_stable_windows').value)
        self.max_gyro_mean_abs = float(self.get_parameter('max_gyro_mean_abs').value)
        self.max_gyro_std = float(self.get_parameter('max_gyro_std').value)
        self.max_acc_norm_std = float(self.get_parameter('max_acc_norm_std').value)
        self.min_acc_norm_mean = float(self.get_parameter('min_acc_norm_mean').value)
        self.max_acc_norm_mean = float(self.get_parameter('max_acc_norm_mean').value)
        self.min_samples = int(self.get_parameter('min_samples').value)
        self.max_wait_sec = float(self.get_parameter('max_wait_sec').value)
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)

        self.samples = deque()
        self.start_time = time.monotonic()
        self.last_log_time = 0.0
        self.stable_windows = 0
        self.ready = False

        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 100)
        self.timer = self.create_timer(0.2, self.check_window)
        self.get_logger().info(
            f'waiting for stable IMU on {self.imu_topic}: '
            f'{self.required_stable_windows}x{self.window_sec:.1f}s windows')

    def imu_callback(self, msg):
        now = time.monotonic()
        self.samples.append((
            now,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ))
        cutoff = now - self.window_sec
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def check_window(self):
        if self.ready:
            return

        now = time.monotonic()
        if self.max_wait_sec > 0.0 and now - self.start_time > self.max_wait_sec:
            self.get_logger().error('IMU static guard timed out before a stable window was observed')
            raise SystemExit(2)

        if len(self.samples) < self.min_samples:
            if now - self.last_log_time >= self.log_period_sec:
                self.last_log_time = now
                self.get_logger().info(f'collecting IMU samples: {len(self.samples)}/{self.min_samples}')
            return

        data = np.array(self.samples, dtype=np.float64)
        gyro = data[:, 1:4]
        acc = data[:, 4:7]
        gyro_mean_abs = np.abs(np.mean(gyro, axis=0))
        gyro_std = np.std(gyro, axis=0)
        acc_norm = np.linalg.norm(acc, axis=1)
        acc_norm_mean = float(np.mean(acc_norm))
        acc_norm_std = float(np.std(acc_norm))

        stable = (
            float(np.max(gyro_mean_abs)) <= self.max_gyro_mean_abs and
            float(np.max(gyro_std)) <= self.max_gyro_std and
            self.min_acc_norm_mean <= acc_norm_mean <= self.max_acc_norm_mean and
            acc_norm_std <= self.max_acc_norm_std
        )

        if stable:
            self.stable_windows += 1
        else:
            self.stable_windows = 0

        if now - self.last_log_time >= self.log_period_sec:
            self.last_log_time = now
            status = 'stable' if stable else 'not stable'
            self.get_logger().info(
                f'IMU {status}: stable_windows={self.stable_windows}/{self.required_stable_windows}, '
                f'gyro_mean_abs={gyro_mean_abs.round(4)}, gyro_std={gyro_std.round(4)}, '
                f'acc_norm_mean={acc_norm_mean:.3f}, acc_norm_std={acc_norm_std:.3f}')

        if self.stable_windows >= self.required_stable_windows:
            self.ready = True
            self.get_logger().info('IMU is stable; FAST-LIO can start now')
            raise SystemExit(0)


def main():
    rclpy.init()
    node = ImuStaticGuard()
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code) if isinstance(exc.code, int) else 0
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(code)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
