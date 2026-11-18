"""Publish extracted files for separate-frame RViz inspection only."""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from .data_io import load_xyz


class FilePreview(Node):
    def __init__(self):
        super().__init__('lincoln_file_preview')
        self.declare_parameter('scan_file', '')
        self.declare_parameter('reference_file', '')
        self.clouds = []
        for parameter, frame, topic in (
            ('scan_file', 'extracted_scan_unverified', '~/scan'),
            ('reference_file', 'unity_pcd_unverified', '~/reference'),
        ):
            path = self.get_parameter(parameter).value
            if not path:
                raise ValueError('Set ' + parameter + ' to an absolute input path')
            points = load_xyz(path)
            points = points[np.isfinite(points).all(axis=1)]
            if not len(points):
                raise ValueError('No finite points in ' + path)
            header = Header()
            header.frame_id = frame
            msg = point_cloud2.create_cloud_xyz32(header, points.astype(np.float32).tolist())
            self.clouds.append((self.create_publisher(PointCloud2, topic, 10), msg))
        self.create_timer(1.0, self.publish)
        self.get_logger().warning(
            'PREVIEW ONLY: stamps are publication time, not original measurement time. '
            'Frames are unverified and disconnected. No TF is published.')

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        for publisher, msg in self.clouds:
            msg.header.stamp = stamp
            publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FilePreview()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
