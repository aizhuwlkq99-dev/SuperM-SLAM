#!/usr/bin/env python3
"""Replay a bag and check finite odometry, timing, filter counts and saved outputs.
Run after sourcing this workspace's install/setup.bash. No FAST-LIVO2 environment is used.
"""
import argparse
import bisect
from datetime import datetime
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import time
import yaml

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt64MultiArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--log-dir', type=Path,
                        help='Log directory; defaults to log/validation/<run>_<timestamp>')
    parser.add_argument('--dynamic', action='store_true')
    parser.add_argument('--before-observe', action='store_true')
    parser.add_argument('--dataset', choices=['retail', 'nclt'], default='retail')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--duration', type=float, help='Optional short smoke replay duration in seconds')
    parser.add_argument('--params-file', type=Path, help='Override the installed dataset configuration')
    parser.add_argument('--param', action='append', default=[], metavar='NAME:=VALUE',
                        help='Additional ROS parameter override; can be repeated')
    args = parser.parse_args()
    if args.rate <= 0 or (args.duration is not None and args.duration <= 0):
        parser.error('rate and duration must be positive')
    config_name = 'nclt_20130110.yaml' if args.dataset == 'nclt' else 'retail_street.yaml'
    config = args.params_file or Path(get_package_share_directory('super_lio')) / 'config' / config_name
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']['lio']
    metadata = yaml.safe_load((args.bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    duration = metadata['duration']['nanoseconds'] * 1e-9
    lidar_topic, imu_topic = params['ros']['lidar_topic'], params['ros']['imu_topic']
    # rosbag2 topic selection must use the exact stored spelling, which may be relative.
    topics = [row['topic_metadata']['name'] for row in metadata['topics_with_message_count']
              if '/' + row['topic_metadata']['name'].lstrip('/') in
              ('/' + lidar_topic.lstrip('/'), '/' + imu_topic.lstrip('/'))]
    lidar_count = sum(row['message_count'] for row in metadata['topics_with_message_count']
                      if '/' + row['topic_metadata']['name'].lstrip('/') == '/' + lidar_topic.lstrip('/'))
    imu_count = sum(row['message_count'] for row in metadata['topics_with_message_count']
                    if '/' + row['topic_metadata']['name'].lstrip('/') == '/' + imu_topic.lstrip('/'))
    if len(topics) != 2 or lidar_count == 0:
        parser.error('bag must contain both configured sensor topics')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    log_root = Path(os.environ.get('SUPERM_LOG_DIR', str(Path(__file__).resolve().parents[1] / 'log')))
    log_dir = (args.log_dir or log_root / 'validation' /
               f'{output.name}_{datetime.now():%Y%m%d_%H%M%S_%f}_{os.getpid()}').resolve()
    log_dir.mkdir(parents=True, exist_ok=False)
    # Configure logging before creating the monitor or its child processes.
    os.environ['ROS_LOG_DIR'] = str(log_dir / 'ros')
    runtime_env = dict(os.environ, GLOG_log_dir=str(log_dir / 'glog'))
    exe = Path(get_package_prefix('super_lio')) / 'lib/super_lio/super_lio_node'
    # Use a domain separate from the usual domain 0; callers can choose one explicitly.
    os.environ.setdefault('ROS_DOMAIN_ID', str(20 + os.getpid() % 60))
    rclpy.init()
    monitor = rclpy.create_node('superm_validation')
    stats = dict(odometry=0, cloud_frames=0, filter_frames=0, dynamic_points=0,
                 static_points=0, count_errors=0, invalid_odometry=0,
                 stamp_regressions=0, max_gap_s=0.0, peak_rss_mib=0.0,
                 first_stamp=None, last_stamp=None, final_position=None,
                 max_processing_ms=0.0, mean_processing_ms=0.0)
    stats.update(invalid_clouds=0, mapping_count_errors=0, rss_after_warmup_mib=None,
                 final_rss_mib=0.0, log_dir=str(log_dir),
                 node_log=str(log_dir / 'node.log'), bag_log=str(log_dir / 'bag.log'))
    mapping_counts = {}
    steady_counts = {}

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not all(math.isfinite(x) for x in vals) or abs(sum(x*x for x in vals[3:]) - 1) > 0.01:
            stats['invalid_odometry'] += 1
        if stats['last_stamp'] is not None:
            dt = stamp - stats['last_stamp']
            if dt <= 0:
                stats['stamp_regressions'] += 1
            stats['max_gap_s'] = max(stats['max_gap_s'], dt)
        else:
            stats['first_stamp'] = stamp
        stats['last_stamp'] = stamp
        stats['final_position'] = vals[:3]
        stats['odometry'] += 1

    def on_cloud(msg):
        stats['cloud_frames'] += 1
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        mapping_counts[stamp] = msg.width * msg.height
        if (msg.height != 1 or len(msg.data) != msg.row_step * msg.height or
                msg.row_step < msg.width * msg.point_step or msg.header.frame_id != 'world'):
            stats['invalid_clouds'] += 1

    def on_steady(msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        steady_counts[stamp] = msg.width * msg.height
        if msg.height != 1 or len(msg.data) != msg.row_step * msg.height:
            stats['invalid_clouds'] += 1

    def on_counts(msg):
        stats['filter_frames'] += 1
        if len(msg.data) != 3 or msg.data[0] != sum(msg.data[1:]):
            stats['count_errors'] += 1
            return
        stats['dynamic_points'] += msg.data[1]
        stats['static_points'] += msg.data[2]

    from geometry_msgs.msg import PoseStamped
    def on_time(msg):
        stats['max_processing_ms'] = max(stats['max_processing_ms'], msg.pose.position.x)
        stats['mean_processing_ms'] = msg.pose.position.y

    subscriptions = [
        monitor.create_subscription(Odometry, '/lio/odom', on_odom, 1000),
        monitor.create_subscription(PointCloud2, '/lio/cloud_world', on_cloud, 100),
        monitor.create_subscription(UInt64MultiArray, '/lio/mdet/counts', on_counts, 1000),
        monitor.create_subscription(PointCloud2, '/lio/mdet/steady_cloud', on_steady, 100),
        monitor.create_subscription(PoseStamped, '/lio/processing_time', on_time, 1000),
    ]
    node_log = (log_dir / 'node.log').open('w')
    bag_log = (log_dir / 'bag.log').open('w')
    node_cmd = [str(exe), '--ros-args', '--params-file', str(config),
                '-p', 'use_sim_time:=true', '-p', f'dyn_obj.en:={str(args.dynamic).lower()}',
                '-p', f'lio.kf.dyn_filter_before_observe:={str(args.before_observe).lower()}',
                '-p', f'lio.map.save_map_dir:={output}/map',
                '-p', f'lio.evo.pose_output_dir:={output}/trajectory']
    for override in args.param:
        node_cmd.extend(['-p', override])
    node = subprocess.Popen(node_cmd, stdout=node_log, stderr=subprocess.STDOUT, env=runtime_env)
    bag = None
    failure = None
    start = time.monotonic()
    try:
        # Wait for subscription discovery and dynamic-filter allocation before playback.
        deadline = start + 30
        while time.monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.1)
            if node.poll() is not None:
                raise RuntimeError(f'Node exited during startup: {node.returncode}')
            if monitor.count_subscribers(lidar_topic) and monitor.count_subscribers(imu_topic) and time.monotonic() - start > 3:
                break
        else:
            raise RuntimeError('Sensor subscriptions were not discovered')
        bag_cmd = ['ros2', 'bag', 'play', str(args.bag), '--clock', '--rate', str(args.rate),
                   '--topics', *topics]
        if args.dataset == 'nclt':
            bag_cmd.extend(['--wait-for-all-acked', '5000'])
        bag = subprocess.Popen(bag_cmd, stdout=bag_log, stderr=subprocess.STDOUT, env=runtime_env)
        playback_start = time.monotonic()
        while bag.poll() is None:
            rclpy.spin_once(monitor, timeout_sec=0.01)
            if node.poll() is not None:
                raise RuntimeError(f'Node exited during playback: {node.returncode}')
            status = Path(f'/proc/{node.pid}/status')
            if status.exists():
                for line in status.read_text().splitlines():
                    if line.startswith('VmRSS:'):
                        rss = int(line.split()[1]) / 1024
                        stats['peak_rss_mib'] = max(stats['peak_rss_mib'], rss)
                        stats['final_rss_mib'] = rss
                        if time.monotonic() - playback_start > 10 and stats['rss_after_warmup_mib'] is None:
                            stats['rss_after_warmup_mib'] = rss
            if args.duration and time.monotonic() - playback_start >= args.duration / args.rate:
                bag.send_signal(signal.SIGINT)
                break
            if time.monotonic() - playback_start > duration / args.rate + 90:
                raise RuntimeError('Bag playback timed out')
        bag.wait(timeout=15)
        if bag.returncode not in (0, 130, -signal.SIGINT):
            raise RuntimeError(f'Bag player exited with {bag.returncode}')
        # Accelerated playback can leave work queued in the mapping process.
        # Finish when odometry becomes quiet, rather than truncating after 5 s.
        deadline = time.monotonic() + 120
        quiet_since = time.monotonic()
        last_count = stats['odometry']
        while time.monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.05)
            if stats['odometry'] != last_count:
                last_count = stats['odometry']
                quiet_since = time.monotonic()
            if time.monotonic() - quiet_since > 3:
                break
        else:
            raise RuntimeError('Mapping did not drain queued scans within 120 seconds')
        if node.poll() is not None:
            raise RuntimeError(f'Node exited before shutdown: {node.returncode}')
    except Exception as exc:
        failure = str(exc)
    finally:
        if bag is not None and bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try:
                bag.wait(timeout=15)
            except subprocess.TimeoutExpired:
                bag.kill()
                bag.wait()
        if node.poll() is None:
            node.send_signal(signal.SIGINT)
            try:
                # Include final map merging/downsampling in the memory peak.
                deadline = time.monotonic() + 120
                while node.poll() is None:
                    if time.monotonic() > deadline:
                        raise subprocess.TimeoutExpired(node.args, 120)
                    try:
                        for line in Path(f'/proc/{node.pid}/status').read_text().splitlines():
                            if line.startswith(('VmRSS:', 'VmHWM:')):
                                stats['peak_rss_mib'] = max(stats['peak_rss_mib'], int(line.split()[1]) / 1024)
                    except FileNotFoundError:
                        pass
                    time.sleep(0.05)
            except subprocess.TimeoutExpired:
                node.kill()
                node.wait()
                failure = failure or 'Node failed to save outputs and stop within 120 seconds'
        node_log.close()
        bag_log.close()
        monitor.destroy_node()
        rclpy.shutdown()
    stats['node_exit_code'] = node.returncode
    stats['wall_seconds'] = time.monotonic() - start
    stats['dynamic_enabled'] = args.dynamic
    stats['before_observe'] = args.before_observe
    stats['dataset'] = args.dataset
    stats['bag_duration_s'] = duration
    stats['bag_lidar_frames'] = lidar_count
    stats['bag_imu_messages'] = imu_count
    # Filter clouds use scan-end time; odometry/mapping clouds use the last
    # integrated IMU time (normally less than 5 ms earlier). Match nearby stamps.
    stamp_seconds = lambda stamp: stamp[0] + stamp[1] * 1e-9
    steady_rows = sorted((stamp_seconds(k), v) for k, v in steady_counts.items())
    steady_times = [row[0] for row in steady_rows]
    stats['matched_mapping_frames'] = 0
    for stamp, count in mapping_counts.items():
        seconds = stamp_seconds(stamp)
        index = bisect.bisect_left(steady_times, seconds)
        candidates = steady_rows[max(0, index-1):index+1]
        if candidates:
            matched = min(candidates, key=lambda row: abs(row[0] - seconds))
            if abs(matched[0] - seconds) < 0.02:
                stats['matched_mapping_frames'] += 1
                stats['mapping_count_errors'] += count != matched[1]
    trajectory = output / 'trajectory' / (params['evo']['seq_name'] + '.txt')
    map_file = output / 'map/map.pcd'
    stats['map_bytes'] = map_file.stat().st_size if map_file.exists() else 0
    node_text = (log_dir / 'node.log').read_text()
    stats['voxel_index_overflow'] = 'indices would overflow' in node_text.lower()
    summary = re.search(r'Sensor summary: received_lidar=(\d+) queued_lidar=(\d+) '
                        r'synced_scans=(\d+) empty_imu_scans=(\d+) pending_lidar=(\d+)', node_text)
    if summary:
        stats['sensor_summary'] = dict(zip(
            ['received_lidar', 'queued_lidar', 'synced_scans', 'empty_imu_scans', 'pending_lidar'],
            map(int, summary.groups())))
        imu_summary = re.search(r' pending_lidar=\d+ received_imu=(\d+)', node_text)
        if imu_summary:
            stats['sensor_summary']['received_imu'] = int(imu_summary.group(1))
        reorder_summary = re.search(r' reordered_imu=(\d+) late_imu=(\d+)', node_text)
        if reorder_summary:
            stats['sensor_summary']['reordered_imu'] = int(reorder_summary.group(1))
            stats['sensor_summary']['late_imu'] = int(reorder_summary.group(2))
    stats['map_points'] = 0
    if map_file.exists():
        with map_file.open('rb') as stream:
            for line in stream:
                if line.startswith(b'POINTS '):
                    stats['map_points'] = int(line.split()[1])
                if line.startswith(b'DATA '):
                    break
    stats['trajectory_poses'] = sum(1 for row in trajectory.read_text().splitlines() if row and not row.startswith('#')) if trajectory.exists() else 0
    if not failure:
        checks = [node.returncode == 0, stats['odometry'] > 20, stats['cloud_frames'] > 20,
                  stats['invalid_odometry'] == 0, stats['stamp_regressions'] == 0,
                  stats['max_gap_s'] < 1.0, stats['map_bytes'] > 1000,
                  stats['trajectory_poses'] > 20, stats['count_errors'] == 0]
        checks += [stats['invalid_clouds'] == 0, stats['mapping_count_errors'] == 0,
                   not stats['voxel_index_overflow']]
        if args.dynamic:
            checks += [stats['filter_frames'] > 20, stats['dynamic_points'] > 0,
                       stats['static_points'] > 0, stats['matched_mapping_frames'] > 20]
        else:
            checks += [stats['filter_frames'] == 0]
        if not args.duration:
            checks += [stats['odometry'] > lidar_count * 0.95,
                       stats['first_stamp'] is not None and
                       stats['last_stamp'] - stats['first_stamp'] > duration * 0.95]
            if summary:
                checks += [stats['sensor_summary']['received_lidar'] >= lidar_count * 0.995,
                           stats['sensor_summary']['pending_lidar'] <= 2]
                if 'received_imu' in stats['sensor_summary']:
                    checks += [stats['sensor_summary']['received_imu'] >= imu_count * 0.995]
                if 'late_imu' in stats['sensor_summary']:
                    checks += [stats['sensor_summary']['late_imu'] == 0]
        if not all(checks):
            failure = 'One or more validation checks failed; inspect report and logs'
    stats['passed'] = failure is None
    stats['failure'] = failure
    (output / 'report.json').write_text(json.dumps(stats, indent=2) + '\n')
    print(json.dumps(stats, indent=2), flush=True)
    return 0 if failure is None else 1


if __name__ == '__main__':
    raise SystemExit(main())
