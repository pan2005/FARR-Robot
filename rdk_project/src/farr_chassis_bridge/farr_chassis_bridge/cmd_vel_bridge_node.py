#!/usr/bin/env python3
import struct
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
import serial

from farr_chassis_bridge.keyboard_control_node import (
    CMD_ID_RDK_CONTROL,
    CMD_ID_STM32_STATUS,
    RDK_CONTROL_FLAG_ENABLE,
    FrameParser,
    pack_frame,
)


@dataclass
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    enabled: bool = True


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('farr_cmd_vel_bridge')
        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_hz', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('linear_scale', 1000.0)   # m/s -> STM32 mm/s-like units
        self.declare_parameter('angular_scale', 1000.0)  # rad/s -> STM32 yaw units
        self.declare_parameter('max_vx', 450.0)
        self.declare_parameter('max_vy', 450.0)
        self.declare_parameter('max_w', 650.0)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.send_hz = float(self.get_parameter('send_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_w = float(self.get_parameter('max_w').value)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.0)
        self.parser = FrameParser()
        self.seq = 0
        self.cmd = VelocityCommand(enabled=False)
        self.last_cmd_time = 0.0
        self.last_status = 'no status yet'

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.status_pub = self.create_publisher(String, 'stm32_status_text', 10)
        self.timer = self.create_timer(1.0 / self.send_hz, self.tick)
        self.get_logger().info(
            f'cmd_vel bridge opened {self.port} @ {self.baud}, max vx/vy/w=' 
            f'{self.max_vx}/{self.max_vy}/{self.max_w}')

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def cmd_cb(self, msg: Twist):
        self.cmd.vx = self.clamp(msg.linear.x * self.linear_scale, self.max_vx)
        self.cmd.vy = self.clamp(msg.linear.y * self.linear_scale, self.max_vy)
        self.cmd.w = self.clamp(-msg.angular.z * self.angular_scale, self.max_w)
        self.cmd.enabled = True
        self.last_cmd_time = time.monotonic()

    def read_status(self):
        data = self.ser.read(512)
        if not data:
            return
        for cmd_id, payload in self.parser.feed(data):
            if cmd_id == CMD_ID_STM32_STATUS and len(payload) == 21:
                echoed_seq, uptime_ms, rx_count, ok_count, err_count, online = struct.unpack('<IIIIIB', payload)
                self.last_status = (
                    f'echo={echoed_seq} uptime={uptime_ms}ms rx={rx_count} '
                    f'ok={ok_count} err={err_count} online={online}'
                )
                msg = String()
                msg.data = self.last_status
                self.status_pub.publish(msg)

    def send_control(self):
        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            self.cmd = VelocityCommand(enabled=False)
        flags = RDK_CONTROL_FLAG_ENABLE if self.cmd.enabled else 0
        payload = struct.pack('<IfffffB', self.seq, self.cmd.vx, -self.cmd.vy, self.cmd.w, 0.0, 0.0, flags)
        self.ser.write(pack_frame(CMD_ID_RDK_CONTROL, payload))
        self.seq += 1

    def tick(self):
        self.read_status()
        self.send_control()

    def close(self):
        try:
            self.cmd = VelocityCommand(enabled=False)
            self.send_control()
            time.sleep(0.05)
        finally:
            self.ser.close()


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
