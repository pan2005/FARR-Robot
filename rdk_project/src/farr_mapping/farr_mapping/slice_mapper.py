#!/usr/bin/env python3
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


FLOAT32 = PointField.FLOAT32


class SliceMapper(Node):
    def __init__(self):
        super().__init__('farr_slice_mapper')

        self.declare_parameter('input_cloud_topic', '/cloud_registered')
        self.declare_parameter('map_topic', '/farr_2_5d_map')
        self.declare_parameter('obstacle_cloud_topic', '/farr_obstacle_cloud')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('map_size_x', 30.0)
        self.declare_parameter('map_size_y', 30.0)
        self.declare_parameter('z_min', -0.35)
        self.declare_parameter('z_max', 0.85)
        self.declare_parameter('mount_roll', 0.0)
        self.declare_parameter('mount_pitch', 0.0)
        self.declare_parameter('mount_yaw', 0.0)
        self.declare_parameter('use_relative_height', True)
        self.declare_parameter('ground_quantile', 0.05)
        self.declare_parameter('vertical_axis_sign', 1.0)
        self.declare_parameter('fit_ground_plane', True)
        self.declare_parameter('ground_candidate_quantile', 0.65)
        self.declare_parameter('ground_plane_refine_distance', 0.10)
        self.declare_parameter('ground_plane_min_points', 100)
        self.declare_parameter('ground_inlier_ratio_min', 0.45)
        self.declare_parameter('ground_residual_std_max', 0.06)
        self.declare_parameter('obstacle_min_height', 0.08)
        self.declare_parameter('obstacle_max_height', 0.45)
        self.declare_parameter('low_obstacle_height', 0.15)
        self.declare_parameter('low_obstacle_extra_points', 2)
        self.declare_parameter('min_range', 0.35)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('self_filter_enabled', True)
        self.declare_parameter('self_filter_front', 0.46)
        self.declare_parameter('self_filter_rear', 0.46)
        self.declare_parameter('self_filter_left', 0.38)
        self.declare_parameter('self_filter_right', 0.38)
        self.declare_parameter('self_filter_corner_radius', 0.05)
        self.declare_parameter('hit_threshold', 1)
        self.declare_parameter('max_hit_count', 20)
        self.declare_parameter('hit_decay_per_frame', 1)
        self.declare_parameter('min_points_per_cell', 2)
        self.declare_parameter('inflation_radius', 0.18)
        self.declare_parameter('debug_period', 2.0)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('process_every_n_clouds', 2)
        self.declare_parameter('unknown_as_free', True)
        self.declare_parameter('save_directory', '/root/farr_maps')
        self.declare_parameter('save_name', 'farr_2_5d_map')
        self.declare_parameter('transform_cloud_to_map_frame', True)

        self.input_cloud_topic = self.get_parameter('input_cloud_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.obstacle_cloud_topic = self.get_parameter('obstacle_cloud_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.map_size_x = float(self.get_parameter('map_size_x').value)
        self.map_size_y = float(self.get_parameter('map_size_y').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.mount_roll = float(self.get_parameter('mount_roll').value)
        self.mount_pitch = float(self.get_parameter('mount_pitch').value)
        self.mount_yaw = float(self.get_parameter('mount_yaw').value)
        self.mount_rotation = self._rotation_matrix(
            self.mount_roll, self.mount_pitch, self.mount_yaw)
        self.use_relative_height = bool(self.get_parameter('use_relative_height').value)
        self.ground_quantile = float(self.get_parameter('ground_quantile').value)
        self.vertical_axis_sign = float(self.get_parameter('vertical_axis_sign').value)
        if self.vertical_axis_sign not in (-1.0, 1.0):
            raise ValueError('vertical_axis_sign must be 1.0 for Z-up or -1.0 for Z-down')
        self.fit_ground_plane = bool(self.get_parameter('fit_ground_plane').value)
        self.ground_candidate_quantile = float(self.get_parameter('ground_candidate_quantile').value)
        self.ground_plane_refine_distance = float(
            self.get_parameter('ground_plane_refine_distance').value)
        self.ground_plane_min_points = int(self.get_parameter('ground_plane_min_points').value)
        self.ground_inlier_ratio_min = float(self.get_parameter('ground_inlier_ratio_min').value)
        self.ground_residual_std_max = float(self.get_parameter('ground_residual_std_max').value)
        self.obstacle_min_height = float(self.get_parameter('obstacle_min_height').value)
        self.obstacle_max_height = float(self.get_parameter('obstacle_max_height').value)
        self.low_obstacle_height = float(self.get_parameter('low_obstacle_height').value)
        self.low_obstacle_extra_points = max(0, int(self.get_parameter('low_obstacle_extra_points').value))
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.self_filter_enabled = bool(self.get_parameter('self_filter_enabled').value)
        self.self_filter_front = float(self.get_parameter('self_filter_front').value)
        self.self_filter_rear = float(self.get_parameter('self_filter_rear').value)
        self.self_filter_left = float(self.get_parameter('self_filter_left').value)
        self.self_filter_right = float(self.get_parameter('self_filter_right').value)
        self.self_filter_corner_radius = float(self.get_parameter('self_filter_corner_radius').value)
        self.hit_threshold = int(self.get_parameter('hit_threshold').value)
        self.max_hit_count = int(self.get_parameter('max_hit_count').value)
        self.hit_decay_per_frame = max(0, int(self.get_parameter('hit_decay_per_frame').value))
        self.min_points_per_cell = max(1, int(self.get_parameter('min_points_per_cell').value))
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.debug_period = max(0.1, float(self.get_parameter('debug_period').value))
        self.unknown_as_free = bool(self.get_parameter('unknown_as_free').value)
        self.process_every_n_clouds = max(1, int(self.get_parameter('process_every_n_clouds').value))
        self.save_directory = Path(str(self.get_parameter('save_directory').value))
        self.save_name = str(self.get_parameter('save_name').value)
        self.transform_cloud_to_map_frame = bool(self.get_parameter('transform_cloud_to_map_frame').value)

        self.width = int(round(self.map_size_x / self.resolution))
        self.height = int(round(self.map_size_y / self.resolution))
        self.origin_x = -0.5 * self.width * self.resolution
        self.origin_y = -0.5 * self.height * self.resolution
        self.hit_grid = np.zeros((self.height, self.width), dtype=np.uint16)
        self.traversed_free_grid = np.zeros((self.height, self.width), dtype=bool)
        self.last_map = None
        self.cloud_count = 0
        self.received_clouds = 0
        self.accepted_points = 0
        self.last_debug_time = self.get_clock().now()
        self.robot_pose = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.obstacle_cloud_publisher = self.create_publisher(
            PointCloud2, self.obstacle_cloud_topic, 5)
        self.subscription = self.create_subscription(PointCloud2, self.input_cloud_topic, self.cloud_callback, 5)
        self.odom_subscription = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.save_service = self.create_service(Trigger, '/save_2_5d_map', self.save_map_callback)
        self.reset_service = self.create_service(Trigger, '/reset_2_5d_map', self.reset_map_callback)
        self.timer = self.create_timer(float(self.get_parameter('publish_period').value), self.publish_map)

        self.get_logger().info(
            f'2.5D mapper: {self.input_cloud_topic} -> {self.map_topic}, '
            f'z=[{self.z_min:.2f}, {self.z_max:.2f}] m, size={self.map_size_x:.1f}x{self.map_size_y:.1f} m, '
            f'res={self.resolution:.2f} m, relative_height={self.use_relative_height}, '
            f'vertical_axis_sign={self.vertical_axis_sign:+.0f}, self_filter={self.self_filter_enabled}, '
            f'mount_rpy=[{self.mount_roll:.3f}, {self.mount_pitch:.3f}, {self.mount_yaw:.3f}] rad')


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
        if (not self.transform_cloud_to_map_frame) or source_frame == self.map_frame:
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

    def _rotation_matrix(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=np.float32)

    def _correct_points(self, x, y, z):
        if self.mount_roll == 0.0 and self.mount_pitch == 0.0 and self.mount_yaw == 0.0:
            return x, y, z
        cx = self.mount_rotation[0, 0] * x + self.mount_rotation[0, 1] * y + self.mount_rotation[0, 2] * z
        cy = self.mount_rotation[1, 0] * x + self.mount_rotation[1, 1] * y + self.mount_rotation[1, 2] * z
        cz = self.mount_rotation[2, 0] * x + self.mount_rotation[2, 1] * y + self.mount_rotation[2, 2] * z
        return cx, cy, cz

    def odom_callback(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_pose = (pose.position.x, pose.position.y, yaw)

    def _robot_local_xy(self, x, y):
        if self.robot_pose is None:
            return None, None
        px, py, yaw = self.robot_pose
        dx = x - px
        dy = y - py
        c = math.cos(yaw)
        s = math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def _self_mask(self, local_x, local_y):
        if local_x is None:
            return np.zeros(0, dtype=bool)
        radius = max(0.0, self.self_filter_corner_radius)
        inside_core = (
            (local_x >= -self.self_filter_rear) &
            (local_x <= self.self_filter_front) &
            (local_y >= -self.self_filter_right) &
            (local_y <= self.self_filter_left)
        )
        if radius <= 0.0:
            return inside_core
        dx = np.maximum.reduce([
            -self.self_filter_rear - local_x,
            local_x - self.self_filter_front,
            np.zeros_like(local_x),
        ])
        dy = np.maximum.reduce([
            -self.self_filter_right - local_y,
            local_y - self.self_filter_left,
            np.zeros_like(local_y),
        ])
        return inside_core | (dx * dx + dy * dy <= radius * radius)

    def _clear_robot_footprint(self):
        if not self.self_filter_enabled or self.robot_pose is None:
            return
        px, py, yaw = self.robot_pose
        reach = max(
            self.self_filter_front, self.self_filter_rear,
            self.self_filter_left, self.self_filter_right) + self.self_filter_corner_radius
        ix0 = max(0, int(math.floor((px - reach - self.origin_x) / self.resolution)))
        ix1 = min(self.width, int(math.ceil((px + reach - self.origin_x) / self.resolution)) + 1)
        iy0 = max(0, int(math.floor((py - reach - self.origin_y) / self.resolution)))
        iy1 = min(self.height, int(math.ceil((py + reach - self.origin_y) / self.resolution)) + 1)
        if ix0 >= ix1 or iy0 >= iy1:
            return
        xs = self.origin_x + (np.arange(ix0, ix1) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(iy0, iy1) + 0.5) * self.resolution
        grid_x, grid_y = np.meshgrid(xs, ys)
        dx = grid_x - px
        dy = grid_y - py
        c = math.cos(yaw)
        s = math.sin(yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        footprint = self._self_mask(local_x, local_y)
        view = self.hit_grid[iy0:iy1, ix0:ix1]
        view[footprint] = 0
        traversed_view = self.traversed_free_grid[iy0:iy1, ix0:ix1]
        traversed_view[footprint] = True

    def _field_offset(self, msg, name):
        for field in msg.fields:
            if field.name == name and field.datatype == FLOAT32:
                return field.offset
        return None

    def _publish_obstacle_cloud(self, source_msg, x, y, z, mask):
        points = np.column_stack((x[mask], y[mask], z[mask])).astype(np.float32, copy=False)
        out = PointCloud2()
        out.header = Header(stamp=source_msg.header.stamp, frame_id=self.map_frame)
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
        self.obstacle_cloud_publisher.publish(out)

    def _ground_debug(self, **kwargs):
        defaults = {
            'quality_ok': True,
            'used_plane': False,
            'reason': 'ok',
            'candidate_count': 0,
            'inlier_ratio': 1.0,
            'residual_std': 0.0,
        }
        defaults.update(kwargs)
        return defaults

    def _height_above_ground(self, x, y, z, base_mask):
        ground_z = float(np.quantile(z[base_mask], self.ground_quantile))
        if not self.fit_ground_plane:
            return self.vertical_axis_sign * (z - ground_z), self._ground_debug(reason='quantile_only')

        valid_x = x[base_mask]
        valid_y = y[base_mask]
        valid_z = z[base_mask]
        if valid_z.size < self.ground_plane_min_points:
            return self.vertical_axis_sign * (z - ground_z), self._ground_debug(
                quality_ok=False,
                reason='too_few_ground_points',
                candidate_count=int(valid_z.size),
            )

        if self.vertical_axis_sign < 0.0:
            candidate_limit = np.quantile(valid_z, self.ground_candidate_quantile)
            candidates = valid_z >= candidate_limit
        else:
            candidate_limit = np.quantile(valid_z, 1.0 - self.ground_candidate_quantile)
            candidates = valid_z <= candidate_limit

        candidate_count = int(np.count_nonzero(candidates))
        if candidate_count < self.ground_plane_min_points:
            return self.vertical_axis_sign * (z - ground_z), self._ground_debug(
                quality_ok=False,
                reason='too_few_plane_candidates',
                candidate_count=candidate_count,
            )

        try:
            design = np.column_stack((
                valid_x[candidates],
                valid_y[candidates],
                np.ones(np.count_nonzero(candidates)),
            ))
            coefficients = np.linalg.lstsq(design, valid_z[candidates], rcond=None)[0]
            estimated = (
                coefficients[0] * valid_x +
                coefficients[1] * valid_y +
                coefficients[2]
            )
            refine = np.abs(valid_z - estimated) <= self.ground_plane_refine_distance
            if np.count_nonzero(refine) >= self.ground_plane_min_points:
                design = np.column_stack((
                    valid_x[refine],
                    valid_y[refine],
                    np.ones(np.count_nonzero(refine)),
                ))
                coefficients = np.linalg.lstsq(design, valid_z[refine], rcond=None)[0]
            ground_plane = coefficients[0] * x + coefficients[1] * y + coefficients[2]
            valid_ground = coefficients[0] * valid_x + coefficients[1] * valid_y + coefficients[2]
            residual = valid_z - valid_ground
            inliers = np.abs(residual) <= self.ground_plane_refine_distance
            inlier_ratio = float(np.count_nonzero(inliers) / valid_z.size)
            residual_std = float(np.std(residual[inliers] if np.any(inliers) else residual))
            quality_ok = (
                inlier_ratio >= self.ground_inlier_ratio_min and
                residual_std <= self.ground_residual_std_max
            )
            return self.vertical_axis_sign * (z - ground_plane), self._ground_debug(
                quality_ok=quality_ok,
                used_plane=True,
                reason='ok' if quality_ok else 'poor_plane_fit',
                candidate_count=candidate_count,
                inlier_ratio=inlier_ratio,
                residual_std=residual_std,
            )
        except np.linalg.LinAlgError:
            return self.vertical_axis_sign * (z - ground_z), self._ground_debug(
                quality_ok=False,
                reason='plane_solve_failed',
                candidate_count=candidate_count,
            )

    def _maybe_log_mapping_debug(self, count, base_points, ground_info, obstacle_points, written_cells):
        now = self.get_clock().now()
        if (now - self.last_debug_time).nanoseconds < int(self.debug_period * 1e9):
            return
        self.last_debug_time = now
        self.get_logger().info(
            '2.5D frame: '
            f'input={count}, base={base_points}, '
            f'ground_candidates={ground_info.get("candidate_count", 0)}, '
            f'ground_inlier={ground_info.get("inlier_ratio", 0.0):.2f}, '
            f'ground_std={ground_info.get("residual_std", 0.0):.3f}m, '
            f'obstacle_points={obstacle_points}, written_cells={written_cells}'
        )

    def _decay_observed_cells(self, x, y, base_mask, hit_cell_ids):
        if self.hit_decay_per_frame <= 0:
            return

        ix = np.floor((x[base_mask] - self.origin_x) / self.resolution).astype(np.int32)
        iy = np.floor((y[base_mask] - self.origin_y) / self.resolution).astype(np.int32)
        inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        if not np.any(inside):
            return

        observed_ids = np.unique((iy[inside] * self.width + ix[inside]).astype(np.int64))
        if hit_cell_ids.size > 0:
            observed_ids = np.setdiff1d(observed_ids, hit_cell_ids, assume_unique=False)
        if observed_ids.size <= 0:
            return

        iy_obs = (observed_ids // self.width).astype(np.int32)
        ix_obs = (observed_ids % self.width).astype(np.int32)
        current = self.hit_grid[iy_obs, ix_obs].astype(np.int32)
        decayed = np.maximum(0, current - self.hit_decay_per_frame).astype(np.uint16)
        self.hit_grid[iy_obs, ix_obs] = decayed

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
        x, y, z = self._correct_points(x, y, z)
        x, y, z = self._transform_xyz_to_map_frame(msg, x, y, z)
        if x is None:
            return

        local_x, local_y = self._robot_local_xy(x, y)
        if local_x is None:
            range_x, range_y = x, y
        else:
            range_x, range_y = local_x, local_y
        dist2 = range_x * range_x + range_y * range_y
        base_mask = (
            np.isfinite(x) & np.isfinite(y) & np.isfinite(z) &
            (dist2 >= self.min_range * self.min_range) &
            (dist2 <= self.max_range * self.max_range)
        )
        if self.self_filter_enabled and local_x is not None:
            base_mask &= ~self._self_mask(local_x, local_y)
        if not np.any(base_mask):
            self.cloud_count += 1
            return

        if self.use_relative_height:
            height_above_ground, ground_info = self._height_above_ground(x, y, z, base_mask)
            if not ground_info['quality_ok']:
                self._decay_observed_cells(x, y, base_mask, np.array([], dtype=np.int64))
                self.cloud_count += 1
                self._maybe_log_mapping_debug(
                    count,
                    int(np.count_nonzero(base_mask)),
                    ground_info,
                    0,
                    0,
                )
                self.get_logger().warn(
                    f'skip cloud: poor ground fit ({ground_info["reason"]}), '
                    f'inlier={ground_info["inlier_ratio"]:.2f}, '
                    f'std={ground_info["residual_std"]:.3f}m',
                    throttle_duration_sec=2.0,
                )
                return
            mask = (
                base_mask &
                (height_above_ground >= self.obstacle_min_height) &
                (height_above_ground <= self.obstacle_max_height)
            )
        else:
            ground_info = self._ground_debug(reason='absolute_z')
            height_above_ground = z
            mask = base_mask & (z >= self.z_min) & (z <= self.z_max)

        if not np.any(mask):
            self._decay_observed_cells(x, y, base_mask, np.array([], dtype=np.int64))
            self.cloud_count += 1
            self._maybe_log_mapping_debug(
                count,
                int(np.count_nonzero(base_mask)),
                ground_info,
                0,
                0,
            )
            return

        self._publish_obstacle_cloud(msg, x, y, z, mask)

        ix = np.floor((x[mask] - self.origin_x) / self.resolution).astype(np.int32)
        iy = np.floor((y[mask] - self.origin_y) / self.resolution).astype(np.int32)
        inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        if not np.any(inside):
            self._decay_observed_cells(x, y, base_mask, np.array([], dtype=np.int64))
            self.cloud_count += 1
            self._maybe_log_mapping_debug(
                count,
                int(np.count_nonzero(base_mask)),
                ground_info,
                int(np.count_nonzero(mask)),
                0,
            )
            return

        iy_inside = iy[inside]
        ix_inside = ix[inside]
        height_inside = height_above_ground[mask][inside]
        cell_ids = (iy_inside * self.width + ix_inside).astype(np.int64)
        unique_ids, counts = np.unique(cell_ids, return_counts=True)
        max_heights = np.full(unique_ids.shape, -np.inf, dtype=np.float32)
        inverse = np.searchsorted(unique_ids, cell_ids)
        np.maximum.at(max_heights, inverse, height_inside.astype(np.float32, copy=False))
        required_counts = np.full(unique_ids.shape, self.min_points_per_cell, dtype=np.int32)
        required_counts[max_heights < self.low_obstacle_height] += self.low_obstacle_extra_points
        accepted_cells = unique_ids[counts >= required_counts]

        self._decay_observed_cells(x, y, base_mask, accepted_cells)
        if accepted_cells.size <= 0:
            self.cloud_count += 1
            self._maybe_log_mapping_debug(
                count,
                int(np.count_nonzero(base_mask)),
                ground_info,
                int(np.count_nonzero(mask)),
                0,
            )
            return

        iy_cells = (accepted_cells // self.width).astype(np.int32)
        ix_cells = (accepted_cells % self.width).astype(np.int32)
        np.add.at(self.hit_grid, (iy_cells, ix_cells), 1)
        np.minimum(self.hit_grid, self.max_hit_count, out=self.hit_grid)
        self._clear_robot_footprint()
        self.cloud_count += 1
        self.accepted_points += int(accepted_cells.size)
        self._maybe_log_mapping_debug(
            count,
            int(np.count_nonzero(base_mask)),
            ground_info,
            int(np.count_nonzero(mask)),
            int(accepted_cells.size),
        )

    def _inflated_occupancy(self):
        occupied = self.hit_grid >= self.hit_threshold
        if self.inflation_radius <= 0.0 or not np.any(occupied):
            return occupied

        radius_cells = int(math.ceil(self.inflation_radius / self.resolution))
        inflated = occupied.copy()
        h, w = occupied.shape
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx == 0 and dy == 0:
                    continue
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                src_y0 = max(0, -dy)
                src_y1 = min(h, h - dy)
                dst_y0 = max(0, dy)
                dst_y1 = min(h, h + dy)
                src_x0 = max(0, -dx)
                src_x1 = min(w, w - dx)
                dst_x0 = max(0, dx)
                dst_x1 = min(w, w + dx)
                inflated[dst_y0:dst_y1, dst_x0:dst_x1] |= occupied[src_y0:src_y1, src_x0:src_x1]
        return inflated

    def build_map_msg(self):
        occupied = self._inflated_occupancy()
        occupied &= ~self.traversed_free_grid
        grid = np.full((self.height, self.width), 0 if self.unknown_as_free else -1, dtype=np.int8)
        grid[occupied] = 100

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info = MapMetaData()
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).astype(np.int8).tolist()
        self.last_map = msg
        return msg

    def publish_map(self):
        self._clear_robot_footprint()
        msg = self.build_map_msg()
        self.publisher.publish(msg)

    def reset_map_callback(self, request, response):
        del request
        self.hit_grid.fill(0)
        self.traversed_free_grid.fill(False)
        self.last_map = None
        self.cloud_count = 0
        self.received_clouds = 0
        self.accepted_points = 0
        response.success = True
        response.message = 'cleared FARR 2.5D occupancy history'
        self.get_logger().info(response.message)
        return response

    def save_map_callback(self, request, response):
        del request
        msg = self.last_map if self.last_map is not None else self.build_map_msg()
        self.save_directory.mkdir(parents=True, exist_ok=True)
        pgm_path = self.save_directory / f'{self.save_name}.pgm'
        yaml_path = self.save_directory / f'{self.save_name}.yaml'

        grid = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        image = np.full_like(grid, 205, dtype=np.uint8)
        image[grid == 0] = 254
        image[grid >= 65] = 0
        image = np.flipud(image)

        with pgm_path.open('wb') as f:
            f.write(f'P5\n# FARR 2.5D sliced map\n{msg.info.width} {msg.info.height}\n255\n'.encode('ascii'))
            f.write(image.tobytes())

        yaml_data = {
            'image': pgm_path.name,
            'mode': 'trinary',
            'resolution': float(msg.info.resolution),
            'origin': [float(msg.info.origin.position.x), float(msg.info.origin.position.y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }
        yaml_path.write_text(yaml.safe_dump(yaml_data, sort_keys=False), encoding='utf-8')
        response.success = True
        response.message = f'saved {yaml_path} ({self.cloud_count} clouds, {self.accepted_points} accepted points)'
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = SliceMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
