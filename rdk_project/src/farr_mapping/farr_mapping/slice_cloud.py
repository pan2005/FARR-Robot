#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class SliceCloud(Node):
    def __init__(self):
        super().__init__('farr_slice_cloud')
        self.declare_parameter('input_cloud_topic', '/cloud_registered')
        self.declare_parameter('output_cloud_topic', '/farr_slice_cloud')
        self.declare_parameter('z_min', -0.35)
        self.declare_parameter('z_max', 0.85)
        self.declare_parameter('min_range', 0.35)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('process_every_n_clouds', 3)
        self.declare_parameter('max_points', 12000)

        self.input_topic = self.get_parameter('input_cloud_topic').value
        self.output_topic = self.get_parameter('output_cloud_topic').value
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.process_every = max(1, int(self.get_parameter('process_every_n_clouds').value))
        self.max_points = max(100, int(self.get_parameter('max_points').value))
        self.count = 0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.callback, qos)
        self.get_logger().info(f'slice cloud: {self.input_topic} -> {self.output_topic}, z=[{self.z_min}, {self.z_max}]')

    def _offset(self, msg, name):
        for f in msg.fields:
            if f.name == name and f.datatype == PointField.FLOAT32:
                return f.offset
        return None

    def callback(self, msg):
        self.count += 1
        if self.count % self.process_every != 0:
            return
        ox, oy, oz = self._offset(msg, 'x'), self._offset(msg, 'y'), self._offset(msg, 'z')
        if ox is None or oy is None or oz is None or msg.point_step <= 0:
            return
        n = msg.width * msg.height
        endian = '>' if msg.is_bigendian else '<'
        data = memoryview(msg.data)
        x = np.ndarray((n,), dtype=endian + 'f4', buffer=data, offset=ox, strides=(msg.point_step,))
        y = np.ndarray((n,), dtype=endian + 'f4', buffer=data, offset=oy, strides=(msg.point_step,))
        z = np.ndarray((n,), dtype=endian + 'f4', buffer=data, offset=oz, strides=(msg.point_step,))
        d2 = x * x + y * y
        mask = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z) &
                (z >= self.z_min) & (z <= self.z_max) &
                (d2 >= self.min_range * self.min_range) &
                (d2 <= self.max_range * self.max_range))
        idx = np.flatnonzero(mask)
        if idx.size == 0:
            return
        if idx.size > self.max_points:
            step = int(np.ceil(idx.size / self.max_points))
            idx = idx[::step]
        xyz = np.column_stack((x[idx], y[idx], z[idx])).astype('<f4', copy=False)

        out = PointCloud2()
        out.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=msg.header.frame_id or 'camera_init')
        out.height = 1
        out.width = int(xyz.shape[0])
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = out.point_step * out.width
        out.is_dense = False
        out.data = xyz.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SliceCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
