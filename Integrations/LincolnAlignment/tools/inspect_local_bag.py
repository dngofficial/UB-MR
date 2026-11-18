#!/usr/bin/env python3
"""Read the extracted Oakley SQLite bag without replaying it or publishing ROS data."""
import argparse
import json
from pathlib import Path
import sqlite3
import sys
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/rotation_integration'))
from rotation_integration.data_io import load_xyz


def inspect(bag_dir, scan_offset):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, NavSatFix
    from diagnostic_msgs.msg import DiagnosticArray
    from rcl_interfaces.msg import Log
    from rotation_integration.cloud_io import xyz

    info = yaml.safe_load((bag_dir / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    files = info['relative_file_paths']
    if len(files) != 1 or Path(files[0]).name != files[0]:
        raise ValueError('This audit expects one SQLite file directly under bag_dir')
    database = (bag_dir / files[0]).resolve()
    with sqlite3.connect(database.as_uri() + '?mode=ro&immutable=1', uri=True) as connection:
        topics = {name: {'id': tid, 'type': typ} for tid, name, typ in
                  connection.execute('SELECT id,name,type FROM topics')}
        start = connection.execute('SELECT timestamp FROM messages ORDER BY timestamp LIMIT 1').fetchone()[0]
        low = start + round(scan_offset * 1e9)

        def first(topic):
            tid = topics[topic]['id']
            row = connection.execute(
                'SELECT timestamp,data FROM messages WHERE timestamp>=? AND timestamp<? '
                'AND topic_id=? ORDER BY timestamp LIMIT 1', (low, low + 1000000000, tid)).fetchone()
            if row is None:
                raise ValueError('No message in requested one-second scan window: ' + topic)
            return row

        scans = []
        reference_csv = load_xyz(ROOT / 'current_alignment/lincoln_lidar_10s_30m.csv')
        for name, topic in topics.items():
            if topic['type'] != 'sensor_msgs/msg/PointCloud2':
                continue
            timestamp, blob = first(name)
            msg = deserialize_message(blob, PointCloud2)
            points = xyz(msg)
            radii = np.linalg.norm(points, axis=1)
            cropped = points[radii <= 30]
            matches = cropped.shape == reference_csv.shape and np.array_equal(cropped, reference_csv)
            scans.append({'topic': name, 'frame_id': msg.header.frame_id,
                          'bag_offset_sec': (timestamp - start) / 1e9,
                          'stamp_sec': msg.header.stamp.sec, 'stamp_nanosec': msg.header.stamp.nanosec,
                          'finite_points': len(points), 'points_within_20m': int((radii <= 20).sum()),
                          'points_within_30m': len(cropped), 'exact_xyz_match_to_uploaded_csv': bool(matches)})
            channel = next((f for f in msg.fields if f.name in ('ring', 'channel') and f.datatype == 4), None)
            if channel is not None:
                values = np.ndarray((msg.height, msg.width), dtype='>u2' if msg.is_bigendian else '<u2',
                                    buffer=msg.data, offset=channel.offset,
                                    strides=(msg.row_step, msg.point_step))
                scans[-1]['recorded_channel_ids'] = np.unique(values).astype(int).tolist()

        odom_topic = '/sensing/gnss/novatel/oem7/odom'
        _, blob = first(odom_topic)
        msg = deserialize_message(blob, Odometry)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odometry = {'topic': odom_topic, 'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id,
                    'position': [p.x, p.y, p.z], 'orientation_xyzw': [q.x, q.y, q.z, q.w],
                    'stamp_sec': msg.header.stamp.sec, 'stamp_nanosec': msg.header.stamp.nanosec,
                    'is_verified_map_frame_body_pose': False}
        history = []
        for timestamp, blob in connection.execute(
                'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (topics[odom_topic]['id'],)):
            msg = deserialize_message(blob, Odometry)
            p, v = msg.pose.pose.position, msg.twist.twist.linear
            history.append([(timestamp - start) / 1e9, p.x, p.y, p.z, np.linalg.norm([v.x, v.y, v.z])])
        history = np.asarray(history)
        window = history[np.abs(history[:, 0] - scan_offset) <= 2]
        moving = history[history[:, 4] > .1]
        motion = {'source': odom_topic, 'observations': len(history),
                  'speed_threshold_mps': .1,
                  'first_reported_speed_above_threshold_bag_offset_sec': float(moving[0, 0]) if len(moving) else None,
                  'inspection_window_bag_offset_sec': [scan_offset - 2, scan_offset + 2],
                  'window_max_reported_speed_mps': float(window[:, 4].max()),
                  'window_position_peak_to_peak_m': np.ptp(window[:, 1:4], axis=0).tolist(),
                  'physical_stationarity_independently_verified': False,
                  'note': 'GNSS pose jumps coexist with low reported speed. Check vehicle signals and scan consistency; do not feed this odometry to the map alignment gate as localization.'}

        fix_topic = '/sensing/gnss/novatel/oem7/fix'
        _, blob = first(fix_topic)
        msg = deserialize_message(blob, NavSatFix)
        fix = {'topic': fix_topic, 'frame_id': msg.header.frame_id,
               'latitude': msg.latitude, 'longitude': msg.longitude, 'altitude': msg.altitude}
        counts = {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
        models, source_switches = set(), []
        if '/diagnostics' in topics:
            row = connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1',
                                     (topics['/diagnostics']['id'],)).fetchone()
            if row is not None:
                for status in deserialize_message(row[0], DiagnosticArray).status:
                    if 'lidar' in status.name.lower() and status.hardware_id:
                        models.add(status.hardware_id.split(':', 1)[0].strip())
        if '/rosout' in topics:
            for timestamp, blob in connection.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                                                     (topics['/rosout']['id'],)):
                log = deserialize_message(blob, Log)
                if 'GPSFix position source=' in log.msg:
                    source_switches.append({'bag_offset_sec': (timestamp - start) / 1e9,
                                            'node': log.name, 'message': log.msg})
        return {'bag_dir': str(bag_dir), 'database': str(database),
                'recorded_ros_schema': connection.execute('SELECT * FROM schema').fetchall(),
                'duration_sec': info['duration']['nanoseconds'] / 1e9,
                'start_bag_timestamp_ns': start, 'message_count_from_metadata': info['message_count'],
                'topic_message_counts': counts, 'scan_candidates': scans,
                'gnss_odometry_at_requested_offset': odometry, 'gnss_fix_at_requested_offset': fix,
                'motion_observations': motion,
                'lidar_models_from_recorded_diagnostics': sorted(models),
                'gnss_position_source_switch_logs': source_switches,
                'recorded_tf_topics': [name for name in topics if name in ('/tf', '/tf_static')],
                'recorded_autoware_localization_topics': [name for name in topics if name.startswith('/localization/')],
                'note': 'No registration, frame relabeling, TF generation, or vehicle/Autoware launch was performed.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag-dir', type=Path, default=ROOT / '.local-data/oakley_test_bag')
    parser.add_argument('--scan-offset', type=float, default=10.)
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/oakley_bag_audit.json')
    args = parser.parse_args()
    if not np.isfinite(args.scan_offset) or args.scan_offset < 2:
        parser.error('--scan-offset must be finite and at least 2 seconds for the motion inspection window')
    report = inspect(args.bag_dir.resolve(), args.scan_offset)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print('Wrote', args.output)
    print(json.dumps({k: v for k, v in report.items() if k not in ('topic_message_counts',)}, indent=2))


if __name__ == '__main__':
    main()
