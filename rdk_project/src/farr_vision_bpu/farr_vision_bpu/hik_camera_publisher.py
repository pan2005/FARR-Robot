#!/usr/bin/env python3

import sys
import ctypes

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sys.path.append('/opt/MVS/Samples/aarch64/Python/MvImport')
from MvCameraControl_class import *


class HikCameraPublisher(Node):
    def __init__(self):
        super().__init__('hik_camera_publisher')

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.cam = MvCamera()
        self.open_camera()

        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def check(self, ret, msg):
        if ret != 0:
            raise RuntimeError(f'{msg}, ret=0x{ret:x}')

    def open_camera(self):
        device_list = MV_CC_DEVICE_INFO_LIST()

        ret = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, device_list)
        self.check(ret, 'Enum devices failed')

        if device_list.nDeviceNum == 0:
            raise RuntimeError('No USB Hikrobot camera found')

        self.get_logger().info(f'Found {device_list.nDeviceNum} camera(s)')

        device_info = ctypes.cast(
            device_list.pDeviceInfo[0],
            ctypes.POINTER(MV_CC_DEVICE_INFO)
        ).contents

        ret = self.cam.MV_CC_CreateHandle(device_info)
        self.check(ret, 'Create handle failed')

        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        self.check(ret, 'Open device failed')

        self.cam.MV_CC_SetEnumValue('TriggerMode', MV_TRIGGER_MODE_OFF)

        self.cam.MV_CC_SetEnumValue('ExposureAuto', 0)
        self.cam.MV_CC_SetFloatValue('ExposureTime', 20000.0)

        self.cam.MV_CC_SetEnumValue('GainAuto', 0)
        self.cam.MV_CC_SetFloatValue('Gain', 12.0)
        ret = self.cam.MV_CC_StartGrabbing()
        self.check(ret, 'Start grabbing failed')

        self.get_logger().info('Camera started')

    def publish_frame(self):
        frame = MV_FRAME_OUT()
        ctypes.memset(ctypes.byref(frame), 0, ctypes.sizeof(frame))

        ret = self.cam.MV_CC_GetImageBuffer(frame, 1000)
        if ret != 0:
            self.get_logger().warning(f'Get image buffer failed: 0x{ret:x}')
            return

        try:
            width = frame.stFrameInfo.nWidth
            height = frame.stFrameInfo.nHeight
            pixel_type = frame.stFrameInfo.enPixelType
            frame_len = frame.stFrameInfo.nFrameLen

            src_data = ctypes.string_at(frame.pBufAddr, frame_len)

            dst_size = width * height * 3
            dst_buf = (ctypes.c_ubyte * dst_size)()

            convert_param = MV_CC_PIXEL_CONVERT_PARAM()
            ctypes.memset(ctypes.byref(convert_param), 0, ctypes.sizeof(convert_param))

            src_buf = (ctypes.c_ubyte * frame_len).from_buffer_copy(src_data)

            convert_param.nWidth = width
            convert_param.nHeight = height
            convert_param.pSrcData = src_buf
            convert_param.nSrcDataLen = frame_len
            convert_param.enSrcPixelType = pixel_type
            convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed
            convert_param.pDstBuffer = dst_buf
            convert_param.nDstBufferSize = dst_size

            ret = self.cam.MV_CC_ConvertPixelType(convert_param)
            if ret != 0:
                self.get_logger().warning(f'Convert pixel type failed: 0x{ret:x}')
                return

            img = np.frombuffer(dst_buf, dtype=np.uint8).reshape((height, width, 3))

            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'hik_camera'
            self.pub.publish(msg)

        finally:
            self.cam.MV_CC_FreeImageBuffer(frame)

    def destroy_node(self):
        try:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = HikCameraPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


print(f'file loaded, __name__={__name__}', flush=True)
main()