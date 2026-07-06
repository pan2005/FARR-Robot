#!/usr/bin/env python3
import select
import struct
import sys
import termios
import time
import tty
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

FRAME_HEADER = 0xA5
FRAME_TAIL = 0x5A
CMD_ID_STM32_STATUS = 0x11
CMD_ID_RDK_CONTROL = 0x12
RDK_CONTROL_FLAG_ENABLE = 0x01


@dataclass
class CommandState:
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    front_arm_delta: float = 0.0
    rear_arm_delta: float = 0.0
    enabled: bool = True


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def pack_frame(cmd_id: int, payload: bytes) -> bytes:
    frame = bytearray([FRAME_HEADER, cmd_id & 0xFF, len(payload) & 0xFF])
    frame.extend(payload)
    frame.append(checksum(frame))
    frame.append(FRAME_TAIL)
    return bytes(frame)


class FrameParser:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []
        while True:
            while self.buf and self.buf[0] != FRAME_HEADER:
                self.buf.pop(0)
            if len(self.buf) < 5:
                break
            payload_len = self.buf[2]
            frame_len = payload_len + 5
            if len(self.buf) < frame_len:
                break
            raw = bytes(self.buf[:frame_len])
            del self.buf[:frame_len]
            if raw[-1] != FRAME_TAIL:
                continue
            if raw[-2] != checksum(raw[:-2]):
                continue
            frames.append((raw[1], raw[3:3 + payload_len]))
        return frames


class KeyboardRawMode:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('farr_keyboard_control')
        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_hz', 20.0)
        self.declare_parameter('vx_step', 300.0)
        self.declare_parameter('vy_step', 300.0)
        self.declare_parameter('w_step', 500.0)
        self.declare_parameter('vx_max', 3000.0)
        self.declare_parameter('vy_max', 3000.0)
        self.declare_parameter('w_max', 3000.0)
        self.declare_parameter('arm_delta', 0.01)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.send_hz = float(self.get_parameter('send_hz').value)
        self.vx_step = float(self.get_parameter('vx_step').value)
        self.vy_step = float(self.get_parameter('vy_step').value)
        self.w_step = float(self.get_parameter('w_step').value)
        self.vx_max = float(self.get_parameter('vx_max').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.arm_delta_mag = float(self.get_parameter('arm_delta').value)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.0)
        self.parser = FrameParser()
        self.cmd = CommandState()
        self.seq = 0
        self.last_status = 'no status yet'
        self.status_pub = self.create_publisher(String, 'stm32_status_text', 10)
        self.timer = self.create_timer(1.0 / self.send_hz, self.tick)

        self.get_logger().info(f'Opened {self.port} @ {self.baud}')
        self.print_help()

    def print_help(self):
        print('')
        print('Keyboard control:')
        print('  w/s : vx forward/backward')
        print('  a/d : vy left/right')
        print('  z/c : w left/right turn')
        print('  x   : zero chassis speed')
        print('  i/k : front arms up/down while key is pressed')
        print('  o/l : rear arms up/down while key is pressed')
        print('  space : disable command and stop')
        print('  e : enable command')
        print('  q : quit')
        print('')

    def clamp(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def handle_key(self, key: str):
        if key == 'w':
            self.cmd.vx = self.clamp(self.cmd.vx + self.vx_step, -self.vx_max, self.vx_max)
            self.cmd.enabled = True
        elif key == 's':
            self.cmd.vx = self.clamp(self.cmd.vx - self.vx_step, -self.vx_max, self.vx_max)
            self.cmd.enabled = True
        elif key == 'a':
            self.cmd.vy = self.clamp(self.cmd.vy - self.vy_step, -self.vy_max, self.vy_max)
            self.cmd.enabled = True
        elif key == 'd':
            self.cmd.vy = self.clamp(self.cmd.vy + self.vy_step, -self.vy_max, self.vy_max)
            self.cmd.enabled = True
        elif key == 'z':
            self.cmd.w = self.clamp(self.cmd.w + self.w_step, -self.w_max, self.w_max)
            self.cmd.enabled = True
        elif key == 'c' or key == 'C':
            self.cmd.w = self.clamp(self.cmd.w - self.w_step, -self.w_max, self.w_max)
            self.cmd.enabled = True
        elif key == 'x':
            self.cmd.vx = 0.0
            self.cmd.vy = 0.0
            self.cmd.w = 0.0
        elif key == 'i':
            self.cmd.front_arm_delta = self.arm_delta_mag
            self.cmd.enabled = True
        elif key == 'k':
            self.cmd.front_arm_delta = -self.arm_delta_mag
            self.cmd.enabled = True
        elif key == 'o':
            self.cmd.rear_arm_delta = self.arm_delta_mag
            self.cmd.enabled = True
        elif key == 'l':
            self.cmd.rear_arm_delta = -self.arm_delta_mag
            self.cmd.enabled = True
        elif key == ' ':
            self.cmd = CommandState(enabled=False)
        elif key == 'e':
            self.cmd.enabled = True
        elif key == 'q':
            raise KeyboardInterrupt

    def send_control(self):
        flags = RDK_CONTROL_FLAG_ENABLE if self.cmd.enabled else 0
        payload = struct.pack(
            '<IfffffB',
            self.seq,
            self.cmd.vx,
            -self.cmd.vy,
            self.cmd.w,
            self.cmd.front_arm_delta,
            self.cmd.rear_arm_delta,
            flags,
        )
        self.ser.write(pack_frame(CMD_ID_RDK_CONTROL, payload))
        self.seq += 1
        self.cmd.front_arm_delta = 0.0
        self.cmd.rear_arm_delta = 0.0

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

    def tick(self):
        self.read_status()
        self.send_control()
        print(
            f'cmd enable={int(self.cmd.enabled)} vx={self.cmd.vx:.0f} vy={self.cmd.vy:.0f} w={self.cmd.w:.2f} '
            f'status: {self.last_status}',
            end=chr(13),
            flush=True,
        )

    def close(self):
        try:
            self.cmd = CommandState(enabled=False)
            self.send_control()
            time.sleep(0.05)
        finally:
            self.ser.close()


def main():
    rclpy.init()
    node = KeyboardControlNode()
    try:
        with KeyboardRawMode() as keyboard:
            while rclpy.ok():
                key = keyboard.read_key()
                if key is not None:
                    node.handle_key(key)
                rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        print('\nStopping keyboard control')
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
