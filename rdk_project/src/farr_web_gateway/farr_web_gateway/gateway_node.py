#!/usr/bin/env python3
import asyncio
import json
import math
import random
import signal
import struct
import threading
import time
from datetime import datetime, timezone

import cv2
import numpy as np
from aiohttp import web

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from nav2_msgs.action import NavigateToPose
except Exception:
    NavigateToPose = None


def iso_now():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='milliseconds')


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    half = 0.5 * yaw
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half),
        'w': math.cos(half),
    }


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def field_offsets(msg):
    return {field.name: field.offset for field in msg.fields}


class FarrWebGateway(Node):
    def __init__(self):
        super().__init__('farr_web_gateway')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('status_hz', 5.0)
        self.declare_parameter('pointcloud_hz', 3.0)
        self.declare_parameter('max_points', 5000)
        self.declare_parameter('manual_timeout', 0.5)
        self.declare_parameter('max_linear', 1.2)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('video_topic', '/person_pose/result_image')
        self.declare_parameter('detections_topic', '/person_pose/detections')
        self.declare_parameter('vision_status_topic', '/person_pose/status')
        self.declare_parameter('pointcloud_topic', '/farr_obstacle_cloud')
        self.declare_parameter('fallback_pointcloud_topic', '/cloud_registered')
        self.declare_parameter('vision_stale_sec', 3.0)
        self.declare_parameter('navigate_action', '/navigate_to_pose')

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.status_hz = float(self.get_parameter('status_hz').value)
        self.pointcloud_hz = float(self.get_parameter('pointcloud_hz').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.mode = 'standby'
        self.navigation_status = 'standby'
        self.last_log = {'level': 'info', 'message': 'gateway started'}
        self.last_cmd_time = 0.0
        self.pending_goal = None
        self.nav_goal_handle = None
        self.emergency_locked = False
        self.trajectory = []
        self.max_trajectory = 300
        self.last_pose = None
        self.last_speed = {'linear': 0.0, 'angular': 0.0}
        self.last_cmd_vel = {'linear': 0.0, 'angular': 0.0}
        self.last_stm32_status = ''
        self.last_vision_detections = {'timestamp': iso_now(), 'frame_id': 'hik_camera', 'detections': []}
        self.last_vision_status = {}
        self.last_victims = []
        self.vision_stale_sec = float(self.get_parameter('vision_stale_sec').value)
        self.last_times = {}
        self.latest_jpeg = None
        self.latest_pcv1 = None
        self.latest_pcv1_source = None
        self.pcv1_seq = 0
        self.pcv1_lock = threading.Lock()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/farr_base_odom', self.odom_cb, 20)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 20)
        self.create_subscription(String, '/stm32_status_text', self.stm32_cb, 10)
        self.create_subscription(String, self.get_parameter('detections_topic').value, self.detections_cb, 10)
        self.create_subscription(String, self.get_parameter('vision_status_topic').value, self.vision_status_cb, 10)
        self.create_subscription(Path, '/plan', self.plan_cb, 5)
        self.create_subscription(Image, self.get_parameter('video_topic').value, self.image_cb, 1)
        self.create_subscription(
            PointCloud2, self.get_parameter('pointcloud_topic').value, self.obstacle_cloud_cb, 2)
        self.create_subscription(
            PointCloud2, self.get_parameter('fallback_pointcloud_topic').value, self.cloud_registered_cb, 2)
        self.save_map_client = self.create_client(Trigger, '/save_2_5d_map')
        self.reset_map_client = self.create_client(Trigger, '/reset_2_5d_map')
        self.nav_client = (
            ActionClient(self, NavigateToPose, self.get_parameter('navigate_action').value)
            if NavigateToPose is not None else None
        )
        self.watchdog_timer = self.create_timer(0.05, self.manual_watchdog)
        self.last_path = []

    def touch(self, key):
        self.last_times[key] = time.monotonic()

    def device_state(self, key, max_age=2.0, running=False):
        age = time.monotonic() - self.last_times.get(key, 0.0)
        if age <= max_age:
            return 'running' if running else 'online'
        return 'offline'

    def odom_cb(self, msg):
        self.touch('odom')
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        pose = {'x': float(p.x), 'y': float(p.y), 'yaw': float(yaw_from_quat(q))}
        if abs(pose['x']) > 1000.0 or abs(pose['y']) > 1000.0:
            self.set_log('warn', 'ignored abnormal odometry pose')
            return
        self.last_pose = pose
        self.last_speed = {
            'linear': float(msg.twist.twist.linear.x),
            'angular': float(msg.twist.twist.angular.z),
        }
        if not self.trajectory or math.hypot(
                pose['x'] - self.trajectory[-1]['x'],
                pose['y'] - self.trajectory[-1]['y']) > 0.03:
            self.trajectory.append({'x': pose['x'], 'y': pose['y']})
            self.trajectory = self.trajectory[-self.max_trajectory:]

    def cmd_vel_cb(self, msg):
        self.touch('cmd_vel')
        self.last_cmd_vel = {'linear': float(msg.linear.x), 'angular': float(msg.angular.z)}

    def stm32_cb(self, msg):
        self.touch('chassis')
        self.last_stm32_status = msg.data

    def parse_json_string(self, msg, label):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('top-level JSON must be an object')
            return data
        except Exception as exc:
            self.set_log('warn', f'ignored invalid {label}: {exc}')
            self.get_logger().warn(f'ignored invalid {label}: {exc}')
            return None

    def detections_cb(self, msg):
        payload = self.parse_json_string(msg, '/person_pose/detections')
        if payload is None:
            return
        detections = payload.get('detections', [])
        if not isinstance(detections, list):
            self.set_log('warn', 'ignored detections JSON: detections is not an array')
            return

        timestamp = str(payload.get('timestamp') or iso_now())
        victims = []
        for index, det in enumerate(detections):
            if not isinstance(det, dict):
                continue
            victim_id = str(det.get('id') or f'Victim-{index + 1:02d}')
            label = str(det.get('label') or 'person')
            try:
                confidence = float(det.get('confidence', 0.0))
            except (TypeError, ValueError):
                confidence = 0.0
            victim = {
                'id': victim_id,
                'label': label,
                'confidence': max(0.0, min(1.0, confidence)),
                'timestamp': timestamp,
            }
            for optional_key in ('track_id', 'bbox', 'center', 'status', 'robot_relative_pose', 'map_pose'):
                if optional_key in det:
                    victim[optional_key] = det[optional_key]
            victims.append(victim)

        self.touch('detections')
        self.last_vision_detections = payload
        self.last_victims = victims

    def vision_status_cb(self, msg):
        payload = self.parse_json_string(msg, '/person_pose/status')
        if payload is None:
            return
        self.touch('vision_status')
        self.last_vision_status = payload

    def plan_cb(self, msg):
        self.touch('plan')
        self.last_path = [
            {'x': float(pose.pose.position.x), 'y': float(pose.pose.position.y)}
            for pose in msg.poses[:200]
        ]

    def image_cb(self, msg):
        self.touch('video')
        image = self.image_to_bgr(msg)
        if image is None:
            return
        ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            self.latest_jpeg = encoded.tobytes()

    def image_to_bgr(self, msg):
        try:
            h, w = int(msg.height), int(msg.width)
            enc = msg.encoding.lower()
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if enc in ('bgr8', '8uc3'):
                return data.reshape((h, w, 3)).copy()
            if enc == 'rgb8':
                return cv2.cvtColor(data.reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
            if enc in ('mono8', '8uc1'):
                return cv2.cvtColor(data.reshape((h, w)), cv2.COLOR_GRAY2BGR)
            if enc == 'bgra8':
                return cv2.cvtColor(data.reshape((h, w, 4)), cv2.COLOR_BGRA2BGR)
            if enc == 'rgba8':
                return cv2.cvtColor(data.reshape((h, w, 4)), cv2.COLOR_RGBA2BGR)
        except Exception as exc:
            self.get_logger().warn(f'failed to convert image: {exc}')
        return None

    def obstacle_cloud_cb(self, msg):
        self.touch('pointcloud')
        self.update_pcv1(msg, preferred=True)

    def cloud_registered_cb(self, msg):
        self.touch('cloud_registered')
        if self.latest_pcv1_source == 'obstacle' and time.monotonic() - self.last_times.get('pointcloud', 0.0) < 1.0:
            return
        self.update_pcv1(msg, preferred=False)

    def update_pcv1(self, msg, preferred):
        try:
            packed = self.make_pcv1(msg)
        except Exception as exc:
            self.get_logger().warn(f'failed to pack pointcloud: {exc}')
            return
        with self.pcv1_lock:
            self.latest_pcv1 = packed
            self.latest_pcv1_source = 'obstacle' if preferred else 'cloud_registered'

    def make_pcv1(self, msg):
        offsets = field_offsets(msg)
        if not {'x', 'y', 'z'} <= set(offsets):
            raise ValueError('PointCloud2 has no x/y/z fields')
        count = int(msg.width) * int(msg.height)
        if count <= 0:
            payload = b''
        else:
            step = max(1, math.ceil(count / max(1, self.max_points)))
            start = random.randrange(step) if step > 1 else 0
            payload_buf = bytearray()
            data = msg.data
            point_step = int(msg.point_step)
            ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
            written = 0
            for i in range(start, count, step):
                base = i * point_step
                if base + max(ox, oy, oz) + 4 > len(data):
                    break
                x = struct.unpack_from('<f', data, base + ox)[0]
                y = struct.unpack_from('<f', data, base + oy)[0]
                z = struct.unpack_from('<f', data, base + oz)[0]
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    payload_buf.extend(struct.pack('<fff', x, y, z))
                    written += 1
                    if written >= self.max_points:
                        break
            payload = bytes(payload_buf)

        point_count = len(payload) // 12
        frame_id = (msg.header.frame_id or 'unknown')[:16].encode('ascii', errors='ignore')
        schema = b'x:0,y:4,z:8'
        timestamp_ns = stamp_to_ns(msg.header.stamp)
        self.pcv1_seq = (self.pcv1_seq + 1) & 0xFFFFFFFF
        header = struct.pack(
            '<4sHHIIQIHHIBB16s26s',
            b'PCV1',
            1,
            0,
            self.pcv1_seq,
            0,
            timestamp_ns,
            point_count,
            12,
            0,
            len(payload),
            len(frame_id),
            len(schema),
            frame_id.ljust(16, b'\0'),
            schema.ljust(26, b'\0'),
        )
        return header + payload

    def manual_watchdog(self):
        if self.mode == 'manual' and time.monotonic() - self.last_cmd_time > self.manual_timeout:
            self.publish_stop()

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def set_log(self, level, message):
        self.last_log = {'level': level, 'message': message}

    def vision_devices(self):
        status_age = time.monotonic() - self.last_times.get('vision_status', 0.0)
        if status_age > self.vision_stale_sec or not self.last_vision_status:
            fallback = self.device_state('video')
            return {
                'camera': fallback,
                'yolo': self.device_state('video', running=True),
            }

        camera = self.last_vision_status.get('camera', {})
        inference = self.last_vision_status.get('inference', {})
        camera_online = bool(camera.get('online', False)) if isinstance(camera, dict) else False
        yolo_error = ''
        yolo_running = False
        if isinstance(inference, dict):
            yolo_error = str(inference.get('last_error') or '')
            yolo_running = bool(inference.get('running', False))
        if yolo_error:
            yolo_state = 'error'
        elif yolo_running:
            yolo_state = 'running'
        else:
            yolo_state = 'offline'
        return {
            'camera': 'online' if camera_online else 'offline',
            'yolo': yolo_state,
        }

    def current_victims(self):
        age = time.monotonic() - self.last_times.get('detections', 0.0)
        if age > self.vision_stale_sec:
            return []
        return self.last_victims

    def status_payload(self):
        pose = self.last_pose or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        vision_devices = self.vision_devices()
        return {
            'timestamp': iso_now(),
            'mode': self.mode,
            'battery_percent': None,
            'robot_pose': pose,
            'speed': self.last_speed,
            'devices': {
                'camera': vision_devices['camera'],
                'lidar': 'online' if (
                    self.device_state('pointcloud') == 'online' or
                    self.device_state('cloud_registered') == 'online'
                ) else 'offline',
                'yolo': vision_devices['yolo'],
                'slam': self.device_state('odom', running=True),
                'navigation': 'unknown',
                'chassis': self.device_state('chassis'),
                'arm': 'unknown',
            },
            'victims': self.current_victims(),
            'vision': {
                'detections': self.last_vision_detections,
                'status': self.last_vision_status,
            },
            'navigation_status': self.navigation_status,
            'path': self.last_path,
            'trajectory': self.trajectory,
            'log': self.last_log,
        }

    async def handle_control_message(self, ws, data):
        command = data.get('command', '')
        request_id = data.get('request_id', '')
        accepted = True
        message = 'ok'
        if self.emergency_locked and command not in ('set_mode', 'emergency_stop'):
            accepted = False
            message = 'emergency locked'
        elif command == 'set_mode':
            mode = data.get('mode')
            if mode not in ('standby', 'manual', 'mapping', 'navigation', 'emergency'):
                accepted = False
                message = 'invalid mode'
            else:
                self.publish_stop()
                self.mode = mode
                if mode != 'emergency':
                    self.emergency_locked = False
                self.navigation_status = 'standby' if mode != 'navigation' else self.navigation_status
                message = f'mode set to {mode}'
        elif command == 'manual_velocity':
            if self.mode != 'manual':
                accepted = False
                message = 'manual_velocity rejected: not in manual mode'
            else:
                vel = data.get('velocity', {})
                linear = max(-self.max_linear, min(self.max_linear, float(vel.get('linear', 0.0))))
                angular = max(-self.max_angular, min(self.max_angular, float(vel.get('angular', 0.0))))
                msg = Twist()
                msg.linear.x = linear
                msg.angular.z = angular
                self.cmd_pub.publish(msg)
                self.last_cmd_time = time.monotonic()
                message = 'manual velocity published'
        elif command == 'emergency_stop':
            self.publish_stop()
            self.mode = 'emergency'
            self.navigation_status = 'canceled'
            self.emergency_locked = True
            message = 'emergency stop applied'
        elif command in ('pause_navigation', 'cancel_navigation'):
            self.publish_stop()
            self.navigation_status = 'paused' if command == 'pause_navigation' else 'canceled'
            if command == 'cancel_navigation' and self.nav_goal_handle is not None:
                self.nav_goal_handle.cancel_goal_async()
                self.nav_goal_handle = None
                self.mode = 'standby'
            message = f'{command} applied as stop command'
        elif command == 'set_goal':
            self.pending_goal = data.get('goal')
            accepted, message = self.send_nav_goal(self.pending_goal, 'set_goal')
        elif command == 'start_navigation':
            if not self.pending_goal:
                accepted = False
                message = 'no goal has been set'
            else:
                accepted, message = self.send_nav_goal(self.pending_goal, 'start_navigation')
        elif command == 'save_map':
            accepted, message = await self.call_trigger(self.save_map_client, 'save map')
        elif command == 'reset_mapping':
            accepted, message = await self.call_trigger(self.reset_map_client, 'reset mapping')
        elif command in ('start_mapping', 'stop_mapping', 'arm_action'):
            accepted = False
            message = f'{command} not implemented in gateway v1'
        else:
            accepted = False
            message = f'unknown command: {command}'

        self.set_log('command' if accepted else 'warn', message)
        await ws.send_json({
            'request_id': request_id,
            'command': command,
            'accepted': accepted,
            'message': message,
            'timestamp': iso_now(),
        })

    def send_nav_goal(self, goal, command):
        if self.nav_client is None:
            return False, 'nav2_msgs is not available'
        if not isinstance(goal, dict):
            return False, 'goal must be an object'
        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            return False, 'Nav2 action server /navigate_to_pose is not available'

        try:
            x = float(goal.get('x', 0.0))
            y = float(goal.get('y', 0.0))
            yaw = float(goal.get('yaw', 0.0))
        except (TypeError, ValueError):
            return False, 'goal x/y/yaw must be numeric'

        self.publish_stop()
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = str(goal.get('frame_id') or 'map')
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        quat = quat_from_yaw(yaw)
        msg.pose.pose.orientation.x = quat['x']
        msg.pose.pose.orientation.y = quat['y']
        msg.pose.pose.orientation.z = quat['z']
        msg.pose.pose.orientation.w = quat['w']

        future = self.nav_client.send_goal_async(msg, feedback_callback=self.nav_feedback_cb)
        future.add_done_callback(self.nav_goal_response_cb)
        self.mode = 'navigation'
        self.navigation_status = 'planning'
        self.set_log('command', f'{command}: goal sent to Nav2')
        return True, 'goal sent to Nav2'

    def nav_goal_response_cb(self, future):
        try:
            goal_handle = future.result()
            self.nav_goal_handle = goal_handle
            if not goal_handle.accepted:
                self.navigation_status = 'failed'
                self.set_log('warn', 'Nav2 goal rejected')
                return
            self.navigation_status = 'moving'
            self.set_log('info', 'Nav2 goal accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_cb)
        except Exception as exc:
            self.navigation_status = 'failed'
            self.set_log('error', f'Nav2 goal response failed: {exc}')

    def nav_feedback_cb(self, feedback):
        self.navigation_status = 'moving'

    def nav_result_cb(self, future):
        try:
            status = future.result().status
            self.navigation_status = 'reached' if status == 4 else 'failed'
            self.set_log('info', f'Nav2 result status={status}')
        except Exception as exc:
            self.navigation_status = 'failed'
            self.set_log('error', f'Nav2 result failed: {exc}')

    async def call_trigger(self, client, label):
        if not client.wait_for_service(timeout_sec=0.2):
            return False, f'{label} service not available'
        future = client.call_async(Trigger.Request())
        start = time.monotonic()
        while not future.done() and time.monotonic() - start < 3.0:
            await asyncio.sleep(0.02)
        if not future.done():
            return False, f'{label} service timeout'
        resp = future.result()
        return bool(resp.success), resp.message or label


def blank_jpeg():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(image, 'FARR: no video frame', (120, 190), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (80, 220, 255), 2, cv2.LINE_AA)
    ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    return encoded.tobytes() if ok else b''


async def create_app(node):
    app = web.Application()

    async def health(request):
        return web.Response(text='farr_web_gateway ok\n', content_type='text/plain')

    async def index(request):
        html = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>FARR Web Gateway</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 24px; background: #101418; color: #eef3f7; }
    code, pre { background: #1b232b; border-radius: 6px; padding: 2px 6px; }
    .ok { color: #76e39b; }
    .bad { color: #ff8d8d; }
    img { max-width: 720px; width: 100%; border: 1px solid #2b3a45; }
  </style>
</head>
<body>
  <h1>FARR Web Gateway</h1>
  <p class="ok">Gateway is running.</p>
  <p>Use these frontend endpoints:</p>
  <ul>
    <li><code>ws://HOST:8080/ws/status</code></li>
    <li><code>ws://HOST:8080/ws/pointcloud</code></li>
    <li><code>http://HOST:8080/video/stream</code></li>
  </ul>
  <h2>Status</h2>
  <pre id="status">connecting...</pre>
  <h2>Video Stream</h2>
  <img src="/video/stream" alt="video stream">
  <script>
    const statusBox = document.getElementById('status');
    const ws = new WebSocket(`ws://${location.host}/ws/status`);
    ws.onmessage = (event) => {
      statusBox.textContent = JSON.stringify(JSON.parse(event.data), null, 2);
    };
    ws.onerror = () => { statusBox.textContent = 'status websocket error'; statusBox.className = 'bad'; };
    ws.onclose = () => { statusBox.textContent += '\\nstatus websocket closed'; };
  </script>
</body>
</html>"""
        return web.Response(text=html, content_type='text/html')

    async def status_ws(request):
        ws = web.WebSocketResponse(heartbeat=10.0)
        await ws.prepare(request)
        period = 1.0 / max(node.status_hz, 1.0)
        try:
            while not ws.closed:
                try:
                    await ws.send_json(node.status_payload())
                except Exception:
                    break
                await asyncio.sleep(period)
        finally:
            pass
        return ws

    async def control_ws(request):
        ws = web.WebSocketResponse(heartbeat=10.0)
        await ws.prepare(request)
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except json.JSONDecodeError:
                        await ws.send_json({
                            'request_id': '',
                            'command': '',
                            'accepted': False,
                            'message': 'invalid json',
                            'timestamp': iso_now(),
                        })
                        continue
                    await node.handle_control_message(ws, data)
        finally:
            node.publish_stop()
        return ws

    async def pointcloud_ws(request):
        ws = web.WebSocketResponse(heartbeat=10.0)
        await ws.prepare(request)
        period = 1.0 / max(node.pointcloud_hz, 1.0)
        try:
            while not ws.closed:
                with node.pcv1_lock:
                    frame = node.latest_pcv1
                if frame:
                    try:
                        await ws.send_bytes(frame)
                    except Exception:
                        break
                await asyncio.sleep(period)
        finally:
            pass
        return ws

    async def video_stream(request):
        response = web.StreamResponse(
            status=200,
            reason='OK',
            headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame'},
        )
        await response.prepare(request)
        fallback = blank_jpeg()
        while True:
            frame = node.latest_jpeg or fallback
            chunk = (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                + f'Content-Length: {len(frame)}\r\n\r\n'.encode()
                + frame
                + b'\r\n'
            )
            try:
                await response.write(chunk)
            except (ConnectionResetError, asyncio.CancelledError):
                break
            await asyncio.sleep(0.15)
        return response

    app.router.add_get('/', index)
    app.router.add_get('/health', health)
    app.router.add_get('/ws/status', status_ws)
    app.router.add_get('/ws/control', control_ws)
    app.router.add_get('/ws/pointcloud', pointcloud_ws)
    app.router.add_get('/video/stream', video_stream)
    return app


def spin_ros(node, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


async def async_main(node):
    app = await create_app(node)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, node.host, node.port)
    await site.start()
    node.get_logger().info(f'FARR web gateway listening on http://{node.host}:{node.port}')
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop.set)
        except NotImplementedError:
            pass
    await stop.wait()
    await runner.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = FarrWebGateway()
    stop_event = threading.Event()
    ros_thread = threading.Thread(target=spin_ros, args=(node, stop_event), daemon=True)
    ros_thread.start()
    try:
        asyncio.run(async_main(node))
    finally:
        stop_event.set()
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
