#!/usr/bin/env python3
"""Replay a short bag through the public large-scale launch and verify shutdown products."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml
from super_lio_interfaces.srv import ExportMap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--dataset', choices=['nclt', 'retail'], default='nclt')
    parser.add_argument('--duration', type=float, default=25)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error('Choose a new output directory')
    log = Path('log/validation') / output.name
    log.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(log.resolve() / 'ros')
    os.environ.setdefault('ROS_DOMAIN_ID', '67')
    rclpy.init()
    monitor = rclpy.create_node('large_scale_launch_check')
    frontend = (log / 'launch.log').open('w')
    bag_log = (log / 'bag.log').open('w')
    launch = subprocess.Popen([
        'ros2', 'launch', 'super_lio_backend', 'large_scale.launch.py',
        f'dataset:={args.dataset}', f'output_dir:={output}', 'shutdown_export:=both'],
        stdout=frontend, stderr=subprocess.STDOUT)
    bag = None
    try:
        client = monitor.create_client(ExportMap, '/slam/save_map')
        assert client.wait_for_service(timeout_sec=20), 'Backend failed to start'
        config = yaml.safe_load((output / 'parameters.yaml').read_text())['/**']['ros__parameters']
        lidar = '/' + config['lio.ros.lidar_topic'].lstrip('/')
        imu = '/' + config['lio.ros.imu_topic'].lstrip('/')
        deadline = time.monotonic() + 20
        while not (monitor.count_subscribers(lidar) and monitor.count_subscribers(imu)):
            assert launch.poll() is None, 'Launch exited before sensor discovery'
            assert time.monotonic() < deadline, 'Sensor discovery failed'
            rclpy.spin_once(monitor, timeout_sec=.1)
        bag = subprocess.Popen([
            'ros2', 'bag', 'play', str(args.bag.resolve()), '--clock', '--rate', '1',
            '--delay', '2'], stdout=bag_log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + args.duration + 2
        while time.monotonic() < deadline and bag.poll() is None:
            assert launch.poll() is None, 'Launch exited during playback'
            rclpy.spin_once(monitor, timeout_sec=.2)
        if bag.poll() is None:
            bag.send_signal(signal.SIGINT)
        bag.wait(timeout=20)
        time.sleep(2)
        launch.send_signal(signal.SIGINT)
        assert launch.wait(timeout=150) == 0, 'Launch shutdown failed'
        store = json.loads((output / 'map/store/manifest.json').read_text())
        graph = json.loads((output / 'graph/report.json').read_text())
        raw = json.loads((output / 'maps/raw/manifest.json').read_text())
        opt = json.loads((output / 'maps/optimized/manifest.json').read_text())
        assert store['complete'] and store['durable_frames'] > 100
        assert store['durable_frames'] == store['submitted_frames'] == raw['frame_count']
        assert raw['frame_count'] == opt['frame_count'] == graph['frames']
        assert graph['accepted_loops'] == 0, 'Default launch unexpectedly enabled loops'
        assert raw['points'] == opt['points'], 'No-loop exports differ'
        assert abs(raw['leaf'] - (.1 if args.dataset == 'nclt' else .05)) < 1e-8
        report = {'passed': True, 'dataset': args.dataset, 'frames': raw['frame_count'],
                  'map_points': raw['points'], 'default_loops': graph['accepted_loops']}
        (output / 'launch_check.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report))
    finally:
        for process in (bag, launch):
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        monitor.destroy_node()
        rclpy.shutdown()
        frontend.close()
        bag_log.close()


if __name__ == '__main__':
    main()
