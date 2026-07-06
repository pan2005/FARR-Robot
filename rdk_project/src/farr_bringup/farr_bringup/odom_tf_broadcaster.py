#!/usr/bin/env python3

import copy
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quat_normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    vx, vy, vz = v
    rotated = quat_multiply(quat_multiply(q, (vx, vy, vz, 0.0)), quat_conjugate(q))
    return rotated[:3]


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return quat_normalize((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def transform_inverse(t, q):
    qi = quat_conjugate(q)
    ti = quat_rotate(qi, tuple(-v for v in t))
    return ti, qi


def transform_multiply(t_ab, q_ab, t_bc, q_bc):
    rotated = quat_rotate(q_ab, t_bc)
    t_ac = (t_ab[0] + rotated[0], t_ab[1] + rotated[1], t_ab[2] + rotated[2])
    q_ac = quat_normalize(quat_multiply(q_ab, q_bc))
    return t_ac, q_ac


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('farr_odom_tf_broadcaster')
        self.source_frame = self.declare_parameter('source_frame', 'camera_init').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.child_frame = self.declare_parameter('child_frame', 'base_link').value
        self.sensor_frame = self.declare_parameter('sensor_frame', 'laser_link').value
        self.odom_topic = self.declare_parameter('odom_topic', '/Odometry').value
        self.output_odom_topic = self.declare_parameter('output_odom_topic', '/farr_base_odom').value
        self.publish_hz = float(self.declare_parameter('publish_hz', 30.0).value)

        laser_xyz = self.declare_parameter('base_to_sensor_xyz', [0.03169, -0.05418, 0.35690]).value
        laser_rpy = self.declare_parameter(
            'base_to_sensor_rpy',
            [3.141592653589793, 0.0, -1.570796326794897],
        ).value
        body_to_laser_xyz = self.declare_parameter('fastlio_body_to_laser_xyz', [0.0, 0.0, 0.0]).value
        body_to_laser_rpy = self.declare_parameter('fastlio_body_to_laser_rpy', [0.0, 0.0, 0.0]).value

        self.base_to_sensor_translation = tuple(float(v) for v in laser_xyz)
        self.base_to_sensor_rotation = quat_from_rpy(*(float(v) for v in laser_rpy))
        self.sensor_to_base_translation, self.sensor_to_base_rotation = transform_inverse(
            self.base_to_sensor_translation,
            self.base_to_sensor_rotation,
        )
        self.body_to_laser_translation = tuple(float(v) for v in body_to_laser_xyz)
        self.body_to_laser_rotation = quat_from_rpy(*(float(v) for v in body_to_laser_rpy))

        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.last_odom = None
        self.t_camera_odom = None
        self.q_camera_odom = None
        self.sent_first = False
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1.0), self.publish_tf)
        self.get_logger().info(
            f'converting FAST-LIO {self.odom_topic} into horizontal {self.odom_frame}->{self.child_frame}; '
            f'also publishing {self.odom_frame}->{self.source_frame} for point cloud transforms')

    def on_odom(self, msg: Odometry):
        self.last_odom = msg

    def camera_to_base(self, msg):
        p_cbdy = msg.pose.pose.position
        q_cbdy_msg = msg.pose.pose.orientation
        t_cbdy = (p_cbdy.x, p_cbdy.y, p_cbdy.z)
        q_cbdy = quat_normalize((q_cbdy_msg.x, q_cbdy_msg.y, q_cbdy_msg.z, q_cbdy_msg.w))
        t_cl, q_cl = transform_multiply(
            t_cbdy,
            q_cbdy,
            self.body_to_laser_translation,
            self.body_to_laser_rotation,
        )
        return transform_multiply(t_cl, q_cl, self.sensor_to_base_translation, self.sensor_to_base_rotation)

    def send_tf(self, stamp, parent, child, t, q):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = t[0]
        tf_msg.transform.translation.y = t[1]
        tf_msg.transform.translation.z = t[2]
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self.br.sendTransform(tf_msg)

    def publish_tf(self):
        if self.last_odom is None:
            return
        msg = self.last_odom
        stamp = self.get_clock().now().to_msg()
        source = msg.header.frame_id or self.source_frame

        t_camera_base, q_camera_base = self.camera_to_base(msg)
        if self.t_camera_odom is None:
            self.t_camera_odom = t_camera_base
            self.q_camera_odom = q_camera_base
            self.get_logger().info('initialized horizontal odom from first base_link pose')

        t_odom_camera, q_odom_camera = transform_inverse(self.t_camera_odom, self.q_camera_odom)
        t_odom_base, q_odom_base = transform_multiply(
            t_odom_camera,
            q_odom_camera,
            t_camera_base,
            q_camera_base,
        )

        self.send_tf(stamp, self.odom_frame, source, t_odom_camera, q_odom_camera)
        self.send_tf(stamp, self.odom_frame, self.child_frame, t_odom_base, q_odom_base)

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.child_frame
        out.pose = copy.deepcopy(msg.pose)
        out.pose.pose.position.x = t_odom_base[0]
        out.pose.pose.position.y = t_odom_base[1]
        out.pose.pose.position.z = t_odom_base[2]
        out.pose.pose.orientation.x = q_odom_base[0]
        out.pose.pose.orientation.y = q_odom_base[1]
        out.pose.pose.orientation.z = q_odom_base[2]
        out.pose.pose.orientation.w = q_odom_base[3]
        out.twist = copy.deepcopy(msg.twist)
        self.odom_pub.publish(out)

        if not self.sent_first:
            self.sent_first = True
            self.get_logger().info(f'first TF sent: {self.odom_frame}->{source} and {self.odom_frame}->{self.child_frame}')


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
