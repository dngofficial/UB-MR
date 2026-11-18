"""Capture one scan in Autoware's map frame using measurement-time TF."""
import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from .math3d import transform_xyz
from .cloud_io import xyz


class LocalizationSnapshot(Node):
    def __init__(self, **kwargs):
        super().__init__('localization_snapshot', **kwargs)
        self.declare_parameter('cloud_topic', '/sensing/lidar/concatenated/pointcloud')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('expected_scan_frame', '')
        self.declare_parameter('max_scan_age_sec', 2.0)
        self.declare_parameter('radius_m', 20.0)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan = None
        self.scan_sub = self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value, self.receive_scan, qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '~/map_cloud', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '~/sensor_pose', 10)
        self.service = self.create_service(Trigger, '~/capture', self.capture)
        self.get_logger().info('Ready. Confirm localization and stationary vehicle before calling ~/capture.')

    def receive_scan(self, msg):
        self.scan = msg

    def capture(self, request, response):
        try:
            msg = self.scan
            if msg is None:
                raise ValueError('No scan received')
            frame = msg.header.frame_id
            expected = self.get_parameter('expected_scan_frame').value
            if not frame or not expected or frame != expected:
                raise ValueError('Set expected_scan_frame to the actual scan frame; received ' + repr(frame))
            stamp = Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                raise ValueError('Scan has zero timestamp; exact-time TF is required')
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            max_age = self.get_parameter('max_scan_age_sec').value
            if not math.isfinite(max_age) or max_age <= 0 or age < -0.1 or age > max_age:
                raise ValueError(f'Scan age {age:.3f}s is outside allowed range; check /clock')
            target = self.get_parameter('map_frame').value
            if not target or target == frame:
                raise ValueError('map_frame must be distinct from the sensor frame')
            # No latest-TF fallback: it can pair a scan with the wrong vehicle pose.
            tf = self.tf_buffer.lookup_transform(target, frame, stamp)
            tr, rot = tf.transform.translation, tf.transform.rotation
            points = xyz(msg)
            radius = self.get_parameter('radius_m').value
            if radius <= 0 or not math.isfinite(radius):
                raise ValueError('radius_m must be positive and finite')
            points = points[np.linalg.norm(points, axis=1) <= radius]
            if len(points) == 0:
                raise ValueError('No finite points within radius')
            mapped = transform_xyz(points, [tr.x, tr.y, tr.z], [rot.x, rot.y, rot.z, rot.w])
            header = Header()
            header.frame_id = target
            header.stamp = msg.header.stamp
            out = point_cloud2.create_cloud_xyz32(header, mapped.astype(np.float32).tolist())
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = tr.x, tr.y, tr.z
            pose.pose.orientation = rot
            self.pose_pub.publish(pose)
            self.cloud_pub.publish(out)
            response.success = True
            response.message = f'Published {len(points)} points and {target} <- {frame} pose at scan time; verify localization independently.'
        except (ValueError, TypeError, KeyError, TransformException) as e:
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationSnapshot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
