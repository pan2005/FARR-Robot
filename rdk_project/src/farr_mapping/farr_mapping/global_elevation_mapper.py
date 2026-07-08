#!/usr/bin/env python3
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


FLOAT32 = PointField.FLOAT32


class GlobalElevationMapper(Node):
    def __init__(self):
        super().__init__('farr_global_elevation_mapper')

        self.declare_parameter('input_cloud_topic', '/cloud_registered')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('elevation_cloud_topic', '/farr/global_elevation_cloud')
        self.declare_parameter('elevation_grid_topic', '/farr/global_elevation_grid')
        self.declare_parameter('stats_topic', '/farr/global_elevation_stats')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('map_size_x', 24.0)
        self.declare_parameter('map_size_y', 24.0)
        self.declare_parameter('min_range', 0.35)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('self_filter_enabled', True)
        self.declare_parameter('self_filter_front', 0.46)
        self.declare_parameter('self_filter_rear', 0.46)
        self.declare_parameter('self_filter_left', 0.38)
        self.declare_parameter('self_filter_right', 0.38)
        self.declare_parameter('ground_quantile', 0.95)
        self.declare_parameter('ground_candidate_quantile', 0.65)
        self.declare_parameter('ground_plane_refine_distance', 0.10)
        self.declare_parameter('ground_plane_min_points', 100)
        self.declare_parameter('ground_inlier_ratio_min', 0.45)
        self.declare_parameter('ground_residual_std_max', 0.06)
        self.declare_parameter('min_height_clip', -0.20)
        self.declare_parameter('max_height_clip', 0.80)
        self.declare_parameter('min_points_per_cell', 2)
        self.declare_parameter('process_every_n_clouds', 2)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('save_directory', '/root/farr_maps')
        self.declare_parameter('save_name', 'global_elevation_map')

        self.input_cloud_topic = self.get_parameter('input_cloud_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.map_size_x = float(self.get_parameter('map_size_x').value)
        self.map_size_y = float(self.get_parameter('map_size_y').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.self_filter_enabled = bool(self.get_parameter('self_filter_enabled').value)
        self.self_filter_front = float(self.get_parameter('self_filter_front').value)
        self.self_filter_rear = float(self.get_parameter('self_filter_rear').value)
        self.self_filter_left = float(self.get_parameter('self_filter_left').value)
        self.self_filter_right = float(self.get_parameter('self_filter_right').value)
        self.ground_quantile = float(self.get_parameter('ground_quantile').value)
        self.ground_candidate_quantile = float(self.get_parameter('ground_candidate_quantile').value)
        self.ground_plane_refine_distance = float(
            self.get_parameter('ground_plane_refine_distance').value)
        self.ground_plane_min_points = int(self.get_parameter('ground_plane_min_points').value)
        self.ground_inlier_ratio_min = float(self.get_parameter('ground_inlier_ratio_min').value)
        self.ground_residual_std_max = float(self.get_parameter('ground_residual_std_max').value)
        self.min_height_clip = float(self.get_parameter('min_height_clip').value)
        self.max_height_clip = float(self.get_parameter('max_height_clip').value)
        self.min_points_per_cell = max(1, int(self.get_parameter('min_points_per_cell').value))
        self.process_every_n_clouds = max(1, int(self.get_parameter('process_every_n_clouds').value))
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.save_directory = Path(str(self.get_parameter('save_directory').value))
        self.save_name = str(self.get_parameter('save_name').value)

        self.width = int(round(self.map_size_x / self.resolution))
        self.height = int(round(self.map_size_y / self.resolution))
        self.origin_x = -0.5 * self.width * self.resolution
        self.origin_y = -0.5 * self.height * self.resolution
        self.height_grid = np.full((self.height, self.width), np.nan, dtype=np.float32)
        self.hit_count = np.zeros((self.height, self.width), dtype=np.uint16)
        self.last_update = np.zeros((self.height, self.width), dtype=np.float32)
        self.received_clouds = 0
        self.updated_clouds = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter('elevation_cloud_topic').value, 2)
        self.grid_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('elevation_grid_topic').value, 1)
        self.stats_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('stats_topic').value, 5)
        self.sub = self.create_subscription(PointCloud2, self.input_cloud_topic, self.cloud_callback, 5)
        self.reset_service = self.create_service(
            Trigger, '/reset_global_elevation_map', self.reset_callback)
        self.save_service = self.create_service(
            Trigger, '/save_global_elevation_map', self.save_callback)
        self.timer = self.create_timer(self.publish_period, self.publish_outputs)

        self.get_logger().info(
            f'global elevation mapper: {self.input_cloud_topic} -> '
            f'{self.get_parameter("elevation_cloud_topic").value}, '
            f'frame={self.map_frame}, size={self.map_size_x:.1f}x{self.map_size_y:.1f}m, '
            f'res={self.resolution:.2f}m')

    def _field_offset(self, msg, name):
        for field in msg.fields:
            if field.name == name and field.datatype == FLOAT32:
                return field.offset
        return None

    def _quat_to_matrix(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ], dtype=np.float32)

    def _transform_xyz_to_map_frame(self, msg, x, y, z):
        source_frame = msg.header.frame_id or self.map_frame
        if source_frame == self.map_frame:
            return x, y, z
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.20),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'skip cloud: no TF {self.map_frame} <- {source_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None, None, None
        rotation = self._quat_to_matrix(tf_msg.transform.rotation)
        translation = tf_msg.transform.translation
        tx = rotation[0, 0] * x + rotation[0, 1] * y + rotation[0, 2] * z + translation.x
        ty = rotation[1, 0] * x + rotation[1, 1] * y + rotation[1, 2] * z + translation.y
        tz = rotation[2, 0] * x + rotation[2, 1] * y + rotation[2, 2] * z + translation.z
        return tx, ty, tz

    def _self_mask(self, local_x, local_y):
        if not self.self_filter_enabled:
            return np.zeros_like(local_x, dtype=bool)
        return (
            (local_x >= -self.self_filter_rear) &
            (local_x <= self.self_filter_front) &
            (local_y >= -self.self_filter_right) &
            (local_y <= self.self_filter_left)
        )

    def _ground_debug(self, **kwargs):
        defaults = {
            'quality_ok': True,
            'candidate_count': 0,
            'inlier_ratio': 1.0,
            'residual_std': 0.0,
            'reason': 'ok',
        }
        defaults.update(kwargs)
        return defaults

    def _height_above_ground(self, x, y, z, base_mask):
        ground_z = float(np.quantile(z[base_mask], self.ground_quantile))
        valid_x = x[base_mask]
        valid_y = y[base_mask]
        valid_z = z[base_mask]
        if valid_z.size < self.ground_plane_min_points:
            return z - ground_z, self._ground_debug(
                quality_ok=False,
                candidate_count=int(valid_z.size),
                reason='too_few_ground_points',
            )

        candidate_limit = np.quantile(valid_z, 1.0 - self.ground_candidate_quantile)
        candidates = valid_z <= candidate_limit
        candidate_count = int(np.count_nonzero(candidates))
        if candidate_count < self.ground_plane_min_points:
            return z - ground_z, self._ground_debug(
                quality_ok=False,
                candidate_count=candidate_count,
                reason='too_few_plane_candidates',
            )

        try:
            design = np.column_stack((
                valid_x[candidates],
                valid_y[candidates],
                np.ones(candidate_count),
            ))
            coefficients = np.linalg.lstsq(design, valid_z[candidates], rcond=None)[0]
            estimated = coefficients[0] * valid_x + coefficients[1] * valid_y + coefficients[2]
            refine = np.abs(valid_z - estimated) <= self.ground_plane_refine_distance
            if np.count_nonzero(refine) >= self.ground_plane_min_points:
                design = np.column_stack((
                    valid_x[refine],
                    valid_y[refine],
                    np.ones(np.count_nonzero(refine)),
                ))
                coefficients = np.linalg.lstsq(design, valid_z[refine], rcond=None)[0]
            valid_ground = coefficients[0] * valid_x + coefficients[1] * valid_y + coefficients[2]
            residual = valid_z - valid_ground
            inliers = np.abs(residual) <= self.ground_plane_refine_distance
            inlier_ratio = float(np.count_nonzero(inliers) / valid_z.size)
            residual_std = float(np.std(residual[inliers] if np.any(inliers) else residual))
            quality_ok = (
                inlier_ratio >= self.ground_inlier_ratio_min and
                residual_std <= self.ground_residual_std_max
            )
            ground_plane = coefficients[0] * x + coefficients[1] * y + coefficients[2]
            return z - ground_plane, self._ground_debug(
                quality_ok=quality_ok,
                candidate_count=candidate_count,
                inlier_ratio=inlier_ratio,
                residual_std=residual_std,
                reason='ok' if quality_ok else 'poor_plane_fit',
            )
        except np.linalg.LinAlgError:
            return z - ground_z, self._ground_debug(
                quality_ok=False,
                candidate_count=candidate_count,
                reason='plane_solve_failed',
            )

    def cloud_callback(self, msg):
        self.received_clouds += 1
        if self.received_clouds % self.process_every_n_clouds != 0:
            return
        ox = self._field_offset(msg, 'x')
        oy = self._field_offset(msg, 'y')
        oz = self._field_offset(msg, 'z')
        if ox is None or oy is None or oz is None:
            self.get_logger().warn('PointCloud2 lacks float32 x/y/z fields; skip frame')
            return

        count = msg.width * msg.height
        if count <= 0 or msg.point_step <= 0:
            return
        endian = '>' if msg.is_bigendian else '<'
        data = memoryview(msg.data)
        try:
            x = np.ndarray((count,), dtype=endian + 'f4', buffer=data, offset=ox, strides=(msg.point_step,))
            y = np.ndarray((count,), dtype=endian + 'f4', buffer=data, offset=oy, strides=(msg.point_step,))
            z = np.ndarray((count,), dtype=endian + 'f4', buffer=data, offset=oz, strides=(msg.point_step,))
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse cloud: {exc}')
            return

        x, y, z = self._transform_xyz_to_map_frame(msg, x, y, z)
        if x is None:
            return

        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        dist2 = x * x + y * y
        base_mask = (
            finite &
            (dist2 >= self.min_range * self.min_range) &
            (dist2 <= self.max_range * self.max_range)
        )
        base_mask &= ~self._self_mask(x, y)
        if not np.any(base_mask):
            return

        height, ground_info = self._height_above_ground(x, y, z, base_mask)
        if not ground_info['quality_ok']:
            self.get_logger().warn(
                f'skip cloud: poor ground fit ({ground_info["reason"]}), '
                f'inlier={ground_info["inlier_ratio"]:.2f}, '
                f'std={ground_info["residual_std"]:.3f}m',
                throttle_duration_sec=2.0,
            )
            return

        height = np.clip(height, self.min_height_clip, self.max_height_clip)
        ix = np.floor((x[base_mask] - self.origin_x) / self.resolution).astype(np.int32)
        iy = np.floor((y[base_mask] - self.origin_y) / self.resolution).astype(np.int32)
        h = height[base_mask].astype(np.float32, copy=False)
        inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        if not np.any(inside):
            return

        ix = ix[inside]
        iy = iy[inside]
        h = h[inside]
        cell_ids = (iy * self.width + ix).astype(np.int64)
        unique_ids, counts = np.unique(cell_ids, return_counts=True)
        max_heights = np.full(unique_ids.shape, -np.inf, dtype=np.float32)
        inverse = np.searchsorted(unique_ids, cell_ids)
        np.maximum.at(max_heights, inverse, h)
        accepted = counts >= self.min_points_per_cell
        if not np.any(accepted):
            return

        accepted_ids = unique_ids[accepted]
        accepted_heights = max_heights[accepted]
        iy_cells = (accepted_ids // self.width).astype(np.int32)
        ix_cells = (accepted_ids % self.width).astype(np.int32)
        old = self.height_grid[iy_cells, ix_cells]
        update = np.isnan(old) | (accepted_heights > old)
        if np.any(update):
            self.height_grid[iy_cells[update], ix_cells[update]] = accepted_heights[update]
        np.add.at(self.hit_count, (iy_cells, ix_cells), 1)
        np.minimum(self.hit_count, np.iinfo(np.uint16).max, out=self.hit_count)
        self.last_update[iy_cells, ix_cells] = float(self.get_clock().now().nanoseconds) * 1e-9
        self.updated_clouds += 1

    def _make_grid_msg(self):
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='height', size=self.height, stride=self.height * self.width),
            MultiArrayDimension(label='width', size=self.width, stride=self.width),
        ]
        grid = np.nan_to_num(self.height_grid, nan=0.0).astype(np.float32, copy=False)
        msg.data = grid.reshape(-1).tolist()
        return msg

    def _make_cloud_msg(self):
        valid = np.isfinite(self.height_grid) & (self.hit_count >= self.min_points_per_cell)
        iy, ix = np.nonzero(valid)
        x = self.origin_x + (ix.astype(np.float32) + 0.5) * self.resolution
        y = self.origin_y + (iy.astype(np.float32) + 0.5) * self.resolution
        z = self.height_grid[iy, ix].astype(np.float32, copy=False)
        points = np.column_stack((x, y, z)).astype(np.float32, copy=False)

        out = PointCloud2()
        out.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map_frame)
        out.height = 1
        out.width = int(points.shape[0])
        out.fields = [
            PointField(name='x', offset=0, datatype=FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = out.point_step * out.width
        out.is_dense = False
        out.data = points.tobytes()
        return out

    def _make_stats_msg(self):
        valid = np.isfinite(self.height_grid) & (self.hit_count > 0)
        valid_count = int(np.count_nonzero(valid))
        total = int(self.height_grid.size)
        if valid_count > 0:
            values = self.height_grid[valid]
            max_height = float(np.max(values))
            mean_positive = float(np.mean(values[values > 0.02])) if np.any(values > 0.02) else 0.0
            obstacle_ratio = float(np.count_nonzero(values > 0.12) / total)
        else:
            max_height = 0.0
            mean_positive = 0.0
            obstacle_ratio = 0.0
        msg = Float32MultiArray()
        msg.data = [
            float(valid_count / total),
            max_height,
            mean_positive,
            obstacle_ratio,
            float(self.received_clouds),
            float(self.updated_clouds),
        ]
        return msg

    def publish_outputs(self):
        self.cloud_pub.publish(self._make_cloud_msg())
        self.grid_pub.publish(self._make_grid_msg())
        self.stats_pub.publish(self._make_stats_msg())

    def reset_callback(self, request, response):
        del request
        self.height_grid.fill(np.nan)
        self.hit_count.fill(0)
        self.last_update.fill(0.0)
        self.received_clouds = 0
        self.updated_clouds = 0
        response.success = True
        response.message = 'cleared FARR global elevation map'
        self.get_logger().info(response.message)
        return response

    def save_callback(self, request, response):
        del request
        self.save_directory.mkdir(parents=True, exist_ok=True)
        path = self.save_directory / f'{self.save_name}.npz'
        np.savez_compressed(
            path,
            height=self.height_grid,
            hit_count=self.hit_count,
            last_update=self.last_update,
            resolution=np.array([self.resolution], dtype=np.float32),
            origin=np.array([self.origin_x, self.origin_y], dtype=np.float32),
            frame=np.array([self.map_frame]),
        )
        response.success = True
        response.message = f'saved {path}'
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = GlobalElevationMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
