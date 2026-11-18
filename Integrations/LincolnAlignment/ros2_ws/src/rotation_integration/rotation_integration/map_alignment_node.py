"""One-shot map alignment using the existing Autoware localization pipeline."""
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from .cloud_io import xyz
from .localization_gate import LocalizationGate
from .map_registration import align_maps, rigid


class MapAlignment(Node):
    def __init__(self, **kwargs):
        super().__init__('map_alignment', **kwargs)
        parameters = {
            'autoware_map_topic': '/map/pointcloud_map',
            'unity_map_topic': '/unity/reference_cloud_ros',
            'localization_topic': '/localization/kinematic_state',
            'config_file': '', 'output_report': '', 'max_pose_age_sec': 2.,
            'max_linear_speed': .1, 'max_angular_speed': .05,
            'stationary_duration_sec': 2., 'max_pose_gap_sec': .5,
            'max_position_stddev_m': 2., 'max_yaw_stddev_deg': 15.,
            'max_position_drift_m': .2, 'max_rotation_drift_deg': 2.,
            'autoware_transient_local': True, 'unity_transient_local': False,
            'require_initialized': True,
            'initialization_state_topic': '/api/localization/initialization_state',
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)
        path = self.value('config_file')
        if not path:
            raise ValueError('Set config_file; the two map origins cannot be assumed equal')
        self.config = json.loads(Path(path).read_text())
        if self.config.get('confirmed') is not True:
            raise ValueError('Confirm map axes, units, frame names and approximate map transform first')
        rigid(self.config['initial_autoware_from_unity_ros'])
        for name in ('autoware_frame', 'unity_ros_frame', 'body_frame'):
            if not isinstance(self.config.get(name), str) or not self.config[name].strip():
                raise ValueError('Missing frame in config: ' + name)
        if self.config['autoware_frame'] == self.config['unity_ros_frame']:
            raise ValueError('The two map frames must have distinct names')
        self.gate = LocalizationGate(
            dwell_sec=self.value('stationary_duration_sec'), max_gap_sec=self.value('max_pose_gap_sec'),
            max_age_sec=self.value('max_pose_age_sec'), max_linear_speed=self.value('max_linear_speed'),
            max_angular_speed=self.value('max_angular_speed'),
            max_position_stddev=self.value('max_position_stddev_m'),
            max_yaw_stddev_deg=self.value('max_yaw_stddev_deg'),
            max_position_drift=self.value('max_position_drift_m'),
            max_rotation_drift_deg=self.value('max_rotation_drift_deg'))
        self.maps, self.pose, self.finished = {}, None, False
        self.initialized = not self.value('require_initialized')
        self.subs = []
        if self.value('require_initialized'):
            try:
                from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
            except ImportError as exc:
                raise ValueError('Source the Autoware workspace with autoware_adapi_v1_msgs. '
                                 'For synthetic checks only, set require_initialized:=false.') from exc
            self.initialized_value = LocalizationInitializationState.INITIALIZED
            self.subs.append(self.create_subscription(
                LocalizationInitializationState, self.value('initialization_state_topic'),
                self.receive_initialization, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for key in ('autoware', 'unity'):
            durable = self.value(key + '_transient_local')
            qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE if durable else ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)
            self.subs.append(self.create_subscription(
                PointCloud2, self.value(key + '_map_topic'),
                lambda msg, key=key: self.maps.update({key: msg}), qos))
        self.subs.append(self.create_subscription(
            Odometry, self.value('localization_topic'), self.receive_pose, qos_profile_sensor_data))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.transform_pub = self.create_publisher(TransformStamped, '~/candidate_transform', latched)
        self.cloud_pub = self.create_publisher(PointCloud2, '~/aligned_unity_cloud', latched)
        self.report_pub = self.create_publisher(String, '~/report', latched)
        self.srv = self.create_service(Trigger, '~/align_once', self.align)
        self.get_logger().info('Ready. Call ~/align_once after initialized, stationary localization and both reference clouds arrive.')

    def value(self, name):
        return self.get_parameter(name).value

    def receive_initialization(self, msg):
        self.initialized = msg.state == self.initialized_value
        if not self.initialized:
            self.gate.reset('Autoware localization is not initialized')

    def receive_pose(self, msg):
        self.pose = msg
        if not self.initialized:
            self.gate.reset('Autoware localization is not initialized')
            return
        if msg.header.frame_id != self.config['autoware_frame'] or msg.child_frame_id != self.config['body_frame']:
            self.gate.reset('Localization must describe the configured body in the Autoware map frame')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.gate.observe(Time.from_msg(msg.header.stamp).nanoseconds / 1e9,
                          self.get_clock().now().nanoseconds / 1e9,
                          [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
                          [v.x, v.y, v.z], [w.x, w.y, w.z], msg.pose.covariance)

    def align(self, request, response):
        try:
            if self.finished:
                raise ValueError('Already completed; restart the node to align another map pair')
            if self.pose is None or set(self.maps) != {'autoware', 'unity'}:
                raise ValueError('Need both map clouds and Autoware kinematic_state')
            if not self.initialized:
                raise ValueError('Autoware localization is not initialized')
            stationary_sec = self.gate.require_ready(self.get_clock().now().nanoseconds / 1e9)
            pose, a, u = self.pose, self.maps['autoware'], self.maps['unity']
            if a.header.frame_id != self.config['autoware_frame'] or u.header.frame_id != self.config['unity_ros_frame']:
                raise ValueError('Cloud frame IDs disagree with the configured map frames')
            if Time.from_msg(u.header.stamp).nanoseconds == 0:
                raise ValueError('Unity reference needs a nonzero, fixed snapshot stamp')
            position = pose.pose.pose.position
            report, aligned = align_maps(xyz(u), xyz(a), self.config['initial_autoware_from_unity_ros'],
                                          [position.x, position.y, position.z], **self.config.get('registration', {}))
            report.update({
                'parent_frame': a.header.frame_id, 'child_frame': u.header.frame_id,
                'pose_stamp_sec': pose.header.stamp.sec, 'pose_stamp_nanosec': pose.header.stamp.nanosec,
                'unity_snapshot_stamp_sec': u.header.stamp.sec,
                'unity_snapshot_stamp_nanosec': u.header.stamp.nanosec,
                'stationary_duration_sec': stationary_sec,
                'initialization_state_required': self.value('require_initialized'),
            })
            serialized = json.dumps(report, allow_nan=False)
            output = self.value('output_report')
            if output:
                path = Path(output).expanduser()
                path.parent.mkdir(parents=True, exist_ok=True)
                temporary = path.with_suffix(path.suffix + '.tmp')
                temporary.write_text(serialized + '\n')
                temporary.replace(path)
            if report['accepted']:
                from scipy.spatial.transform import Rotation
                matrix = np.asarray(report['autoware_from_unity_ros'])
                q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
                msg = TransformStamped()
                msg.header.frame_id, msg.child_frame_id = a.header.frame_id, u.header.frame_id
                # Bind the candidate to the exact frozen Unity snapshot. This topic is NOT TF.
                msg.header.stamp = u.header.stamp
                msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = matrix[:3, 3].tolist()
                msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q.tolist()
                header = Header(frame_id=a.header.frame_id, stamp=pose.header.stamp)
                cloud = point_cloud2.create_cloud_xyz32(header, aligned.astype(np.float32).tolist())
                self.cloud_pub.publish(cloud)
                self.transform_pub.publish(msg)
                self.finished = True
            self.report_pub.publish(String(data=serialized))
            response.success = report['accepted']
            response.message = ('Candidate passed numerical checks; inspect map overlay before applying in Unity'
                                if response.success else 'Rejected; see ~/report')
        except (ValueError, KeyError, TypeError, RuntimeError, ImportError, OSError, AssertionError) as exc:
            response.success, response.message = False, str(exc)
            self.get_logger().warning('Alignment refused: ' + str(exc))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapAlignment()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
