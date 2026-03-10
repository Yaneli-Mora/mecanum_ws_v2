#!/usr/bin/env python3
# camera_node.py — publishes color camera frames using OpenCV directly
# Bypasses usb_cam driver entirely, works on any V4L2 camera
#
# Usage:
#   python3 camera_node.py
#
# Install deps if needed:
#   pip3 install opencv-python --break-system-packages

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_forward')

        self.declare_parameter('device_id',    1)
        self.declare_parameter('width',        640)
        self.declare_parameter('height',       480)
        self.declare_parameter('fps',          30.0)
        self.declare_parameter('topic',        '/camera/forward/image_raw')

        device_id = self.get_parameter('device_id').value
        width     = self.get_parameter('width').value
        height    = self.get_parameter('height').value
        fps       = self.get_parameter('fps').value
        topic     = self.get_parameter('topic').value

        self.pub = self.create_publisher(Image, topic, 10)

        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open /dev/video{device_id}')
            return

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Camera opened: {actual_w}x{actual_h} @ {actual_fps:.1f}fps → {topic}')

        period = 1.0 / fps
        self.timer = self.create_timer(period, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # frame is BGR from OpenCV — convert to RGB for ROS
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_forward'
        msg.height          = frame_rgb.shape[0]
        msg.width           = frame_rgb.shape[1]
        msg.encoding        = 'rgb8'
        msg.is_bigendian    = False
        msg.step            = frame_rgb.shape[1] * 3
        msg.data            = frame_rgb.tobytes()

        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
