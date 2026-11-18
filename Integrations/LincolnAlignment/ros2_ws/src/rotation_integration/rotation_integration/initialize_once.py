"""Call Autoware's AD API once; do not implement another localizer."""
import argparse
import json
import sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseWithCovarianceStamped
from .data_io import validate_seed


def main(args=None):
    argv = sys.argv if args is None else ['initialize_once'] + list(args)
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--seed', help='Verified base_link pose in the Autoware map frame')
    source.add_argument('--use-gnss', action='store_true', help='Ask existing Autoware pipeline to use its configured GNSS pose')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--body-frame', default='base_link')
    parser.add_argument('--service', default='/api/localization/initialize')
    parser.add_argument('--timeout', type=float, default=30.0)
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not 0 < options.timeout <= 300:
        parser.error('--timeout must be between 0 and 300 seconds')
    seed = None
    if options.seed:
        try:
            seed = validate_seed(json.loads(Path(options.seed).read_text()), options.map_frame, options.body_frame)
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
    try:
        from autoware_adapi_v1_msgs.srv import InitializeLocalization
    except ImportError:
        parser.error('Source the Autoware workspace containing autoware_adapi_v1_msgs. '
                     'This executable uses the AD API, not an internal version-specific NDT service.')
    rclpy.init(args=argv[1:])
    node = Node('initialize_localization_once')
    exit_code = 1
    try:
        client = node.create_client(InitializeLocalization, options.service)
        if not client.wait_for_service(timeout_sec=options.timeout):
            raise RuntimeError('AD API service unavailable; check ros2 service list -t and your sourced workspace')
        req = InitializeLocalization.Request()
        if seed is not None:
            p, q, covariance = seed
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = options.map_frame
            now = node.get_clock().now()
            if node.get_parameter('use_sim_time').value:
                import time
                deadline = time.monotonic() + options.timeout
                while now.nanoseconds == 0 and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    now = node.get_clock().now()
            if now.nanoseconds == 0:
                raise RuntimeError('No simulation clock; play the bag with --clock')
            pose.header.stamp = now.to_msg()
            pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = p.tolist()
            pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = q.tolist()
            pose.pose.covariance = covariance.tolist()
            req.pose = [pose]
        else:
            req.pose = []
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=options.timeout)
        if not future.done():
            raise RuntimeError('Request timed out; server outcome is UNKNOWN. Inspect initialization state before retrying.')
        result = future.result()
        if result is None:
            raise RuntimeError('No service response')
        print(json.dumps({'service': options.service, 'success': bool(result.status.success),
                          'code': int(result.status.code), 'message': result.status.message,
                          'note': 'Check initialization state, localization diagnostics, TF and map overlay next.'}, indent=2))
        exit_code = 0 if result.status.success else 1
    except (RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)
