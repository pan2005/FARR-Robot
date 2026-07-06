#!/usr/bin/env python3

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class SavePoseResult(Node):
    def __init__(self):
        super().__init__('save_pose_result')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/person_pose/result_image',
            self.callback,
            10
        )

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('/root/farr_output/person_pose_result.jpg', image)
        self.get_logger().info('saved /root/farr_output/person_pose_result.jpg')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SavePoseResult()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

