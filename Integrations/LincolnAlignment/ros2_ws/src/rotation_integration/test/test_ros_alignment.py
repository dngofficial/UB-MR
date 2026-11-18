"""Opt-in real DDS/service integration test, using synthetic maps, never vehicle inputs."""
import os
import json
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np

RUN = os.environ.get('RUN_ROS_INTEGRATION') == '1'
if RUN:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
    from std_srvs.srv import Trigger
    from rotation_integration.map_alignment_node import MapAlignment
    from rotation_integration.map_registration import transform
    from rotation_integration.cloud_io import xyz
    from rotation_integration.snapshot_node import LocalizationSnapshot
    from tf2_ros import StaticTransformBroadcaster
    from rclpy.time import Time
    import subprocess
    import yaml
    from ament_index_python.packages import get_package_prefix
    from rotation_integration.frame_chain import replay_description
    from rotation_integration.math3d import transform_xyz


@unittest.skipUnless(RUN, 'Set RUN_ROS_INTEGRATION=1 in an isolated ROS_DOMAIN_ID to run DDS checks')
class ROSAlignmentTests(unittest.TestCase):
    def test_native_robot_state_publisher_applies_jake_rotation_and_final_scan_joint(self):
        # This is an isolated synthetic fixture, not confirmation of any real calibration.
        root = Path(__file__).resolve().parents[4]
        profile = json.loads((root / 'config/calibration_candidates.json').read_text())['profiles']['jake_with_installed_scan_joint_hypothesis']
        profile['confirmed_for_bag'] = True
        profile['source'] = 'Synthetic native-TF fixture only'
        rclpy.init(args=[])
        executor = SingleThreadedExecutor()
        nodes, process = [], None
        try:
            with tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / 'fixture.json'
                config.write_text(json.dumps({'profiles': {'fixture': profile}}))
                params = Path(directory) / 'params.yaml'
                params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
                    'robot_description': replay_description(config, 'fixture')}}}))
                executable = Path(get_package_prefix('robot_state_publisher')) / 'lib/robot_state_publisher/robot_state_publisher'
                with (Path(directory) / 'robot_state_publisher.log').open('w+') as log:
                    process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(params)],
                                               stdout=log, stderr=subprocess.STDOUT)
                    fixture = rclpy.create_node('jake_mount_fixture')
                    snapshot = LocalizationSnapshot(namespace='/jake_mount_test', parameter_overrides=[
                        Parameter('map_frame', value='base_link'),
                        Parameter('expected_scan_frame', value='velodyne_top')])
                    nodes = [fixture, snapshot]
                    for node in nodes:
                        executor.add_node(node)
                    deadline = time.monotonic() + 5
                    while not snapshot.tf_buffer.can_transform('base_link', 'velodyne_top', Time()) and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.02)
                    log.flush()
                    self.assertTrue(snapshot.tf_buffer.can_transform('base_link', 'velodyne_top', Time()),
                                    'Native robot_state_publisher failed to provide the complete sensor chain')
                    tf = snapshot.tf_buffer.lookup_transform('base_link', 'velodyne_top', Time())
                    t, q = tf.transform.translation, tf.transform.rotation
                    np.testing.assert_allclose([t.x, t.y, t.z], [.9009933613391747, .0000299794790477413, 2.066102529713171], atol=1e-10)
                    # An independent expected transformed point for the supplied RPY and optical offset.
                    mapped = transform_xyz([[1, 0, 0]], [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
                    np.testing.assert_allclose(mapped, [[1.9002185311113027, -.036357888943765244, 2.051103092206843]], rtol=0, atol=1e-10)
                    scan = point_cloud2.create_cloud_xyz32(
                        Header(frame_id='velodyne_top', stamp=fixture.get_clock().now().to_msg()), [[1., 0., 0.]])
                    snapshot.receive_scan(scan)
                    response = snapshot.capture(Trigger.Request(), Trigger.Response())
                    self.assertTrue(response.success, response.message)
        finally:
            for node in reversed(nodes):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            if process is not None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)

    def test_snapshot_uses_full_mount_chain_and_exact_scan_time(self):
        rclpy.init(args=[])
        executor = SingleThreadedExecutor()
        nodes = []
        try:
            fixture = rclpy.create_node('lincoln_tf_fixture')
            snapshot = LocalizationSnapshot(namespace='/lincoln_tf_test', parameter_overrides=[
                Parameter('cloud_topic', value='/lincoln_tf_test/scan'),
                Parameter('expected_scan_frame', value='test_lidar')])
            nodes = [fixture, snapshot]
            for node in nodes:
                executor.add_node(node)
            publisher = fixture.create_publisher(PointCloud2, '/lincoln_tf_test/scan', qos_profile_sensor_data)
            output = []
            fixture.create_subscription(PointCloud2, '/lincoln_tf_test/localization_snapshot/map_cloud', output.append, 10)
            broadcaster = StaticTransformBroadcaster(fixture)
            map_from_body = TransformStamped(header=Header(frame_id='map'), child_frame_id='test_base')
            map_from_body.transform.translation.x, map_from_body.transform.translation.y = 10., 20.
            map_from_body.transform.rotation.z = map_from_body.transform.rotation.w = float(np.sqrt(.5))
            body_from_lidar = TransformStamped(header=Header(frame_id='test_base'), child_frame_id='test_lidar')
            body_from_lidar.transform.translation.x, body_from_lidar.transform.translation.z = 1., 2.
            body_from_lidar.transform.rotation.w = 1.
            broadcaster.sendTransform([map_from_body, body_from_lidar])
            deadline = time.monotonic() + 5
            while not snapshot.tf_buffer.can_transform('map', 'test_lidar', Time()) and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            self.assertTrue(snapshot.tf_buffer.can_transform('map', 'test_lidar', Time()))
            scan = point_cloud2.create_cloud_xyz32(
                Header(frame_id='test_lidar', stamp=fixture.get_clock().now().to_msg()), [[1., 0., 0.], [0., 1., 0.]])
            deadline = time.monotonic() + 5
            while snapshot.scan is None and time.monotonic() < deadline:
                publisher.publish(scan)
                executor.spin_once(timeout_sec=.02)
            client = fixture.create_client(Trigger, '/lincoln_tf_test/localization_snapshot/capture')
            self.assertTrue(client.wait_for_service(timeout_sec=5.))
            future = client.call_async(Trigger.Request())
            executor.spin_until_future_complete(future, timeout_sec=5.)
            self.assertTrue(future.done())
            self.assertTrue(future.result().success, future.result().message)
            deadline = time.monotonic() + 5
            while not output and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            self.assertTrue(output)
            np.testing.assert_allclose(xyz(output[-1]), [[10, 22, 2], [9, 21, 2]], atol=1e-5)
            self.assertEqual(output[-1].header.stamp, scan.header.stamp)
            scan.header.stamp = Time().to_msg()
            snapshot.receive_scan(scan)
            self.assertFalse(snapshot.capture(Trigger.Request(), Trigger.Response()).success)
            # Only a newer dynamic transform exists. Exact-time lookup must fail rather than
            # silently substitute the latest pose for an older scan.
            dynamic = TransformStamped(header=Header(frame_id='map', stamp=fixture.get_clock().now().to_msg()),
                                       child_frame_id='test_dynamic_lidar')
            dynamic.transform.rotation.w = 1.
            snapshot.tf_buffer.set_transform(dynamic, 'synthetic_test')
            snapshot.set_parameters([Parameter('expected_scan_frame', value='test_dynamic_lidar')])
            scan.header.frame_id = 'test_dynamic_lidar'
            scan.header.stamp = Time(nanoseconds=Time.from_msg(dynamic.header.stamp).nanoseconds - 1000000000).to_msg()
            result = snapshot.capture(Trigger.Request(), Trigger.Response())
            self.assertFalse(result.success)
            self.assertIn('extrapolation', result.message.lower())
        finally:
            for node in reversed(nodes):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()

    def test_service_qos_registration_snapshot_identity_and_one_shot(self):
        rclpy.init(args=[])
        executor = SingleThreadedExecutor()
        nodes = []
        try:
            fixture = rclpy.create_node('lincoln_alignment_fixture', namespace='/lincoln_test')
            nodes.append(fixture)
            executor.add_node(fixture)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            map_pub = fixture.create_publisher(PointCloud2, '/lincoln_test/autoware', latched)
            unity_pub = fixture.create_publisher(PointCloud2, '/lincoln_test/unity', 1)
            pose_pub = fixture.create_publisher(Odometry, '/lincoln_test/pose', qos_profile_sensor_data)
            clouds, reports = [], []
            fixture.create_subscription(PointCloud2, '/lincoln_test/map_alignment/aligned_unity_cloud', clouds.append, latched)
            fixture.create_subscription(String, '/lincoln_test/map_alignment/report', reports.append, latched)
            rng = np.random.default_rng(25)
            source = rng.uniform([-9, -7, -2], [9, 7, 5], (1800, 3))
            truth = np.eye(4)
            angle = np.radians(14.)
            truth[:3, :3] = [[np.cos(angle), -np.sin(angle), 0],
                              [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
            truth[:3, 3] = [100., 200., 3.]
            initial = truth.copy()
            initial[:3, 3] += [.2, -.1, .05]
            autoware = point_cloud2.create_cloud_xyz32(Header(frame_id='map'), transform(source, truth).tolist())
            unity = point_cloud2.create_cloud_xyz32(Header(frame_id='unity_map_ros'), source.tolist())
            unity.header.stamp.sec, unity.header.stamp.nanosec = 1234, 98765
            # Publish Autoware's map before the service subscribes, exercising transient-local QoS.
            map_pub.publish(autoware)
            with tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / 'seed.json'
                output = Path(directory) / 'report.json'
                config.write_text(json.dumps({
                    'confirmed': True, 'autoware_frame': 'map', 'unity_ros_frame': 'unity_map_ros',
                    'body_frame': 'base_link', 'initial_autoware_from_unity_ros': initial.tolist()}))
                overrides = dict(config_file=str(config), require_initialized=False, use_sim_time=False,
                                 autoware_map_topic='/lincoln_test/autoware', unity_map_topic='/lincoln_test/unity',
                                 localization_topic='/lincoln_test/pose', stationary_duration_sec=.2,
                                 output_report=str(output))
                service = MapAlignment(namespace='/lincoln_test',
                                       parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()])
                nodes.append(service)
                executor.add_node(service)
                client = fixture.create_client(Trigger, '/lincoln_test/map_alignment/align_once')
                self.assertTrue(client.wait_for_service(timeout_sec=5.))

                def pump(duration=.4, moving=False):
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        unity_pub.publish(unity)
                        pose = Odometry(header=Header(frame_id='map', stamp=fixture.get_clock().now().to_msg()),
                                        child_frame_id='base_link')
                        pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = truth[:3, 3]
                        pose.pose.pose.orientation.w = 1.
                        pose.pose.covariance = np.diag([.01, .01, .01, .001, .001, .001]).ravel().tolist()
                        pose.twist.twist.linear.x = .5 if moving else 0.
                        pose_pub.publish(pose)
                        executor.spin_once(timeout_sec=.01)
                        time.sleep(.02)
                    for _ in range(8):
                        executor.spin_once(timeout_sec=.01)

                def call():
                    future = client.call_async(Trigger.Request())
                    executor.spin_until_future_complete(future, timeout_sec=15.)
                    self.assertTrue(future.done(), 'alignment service timed out')
                    return future.result()

                pump(.6, moving=True)
                self.assertFalse(call().success, 'moving vehicle must be refused')
                pump(.5)
                unity.header.frame_id = 'wrong_frame'
                pump(.1)
                rejected = call()
                self.assertFalse(rejected.success)
                self.assertIn('frame', rejected.message)
                unity.header.frame_id = 'unity_map_ros'
                pump(.2)
                accepted = call()
                self.assertTrue(accepted.success, accepted.message)
                # A late subscriber must receive the accepted transform exactly once from the cache.
                candidates = []
                fixture.create_subscription(TransformStamped, '/lincoln_test/map_alignment/candidate_transform',
                                            candidates.append, latched)
                deadline = time.monotonic() + 5
                while not (clouds and reports and candidates) and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.05)
                self.assertTrue(candidates, 'late subscriber missed transient-local candidate')
                self.assertTrue(clouds)
                self.assertTrue(reports)
                candidate = candidates[-1]
                self.assertEqual((candidate.header.stamp.sec, candidate.header.stamp.nanosec), (1234, 98765))
                self.assertEqual((candidate.header.frame_id, candidate.child_frame_id), ('map', 'unity_map_ros'))
                report = json.loads(output.read_text())
                np.testing.assert_allclose(report['autoware_from_unity_ros'], truth, atol=.015)
                np.testing.assert_allclose(xyz(clouds[-1]), transform(source, truth), atol=.02)
                self.assertIn('Already completed', call().message)
        finally:
            for node in reversed(nodes):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
