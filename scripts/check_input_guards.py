#!/usr/bin/env python3
"""Verify invalid mapping modes and reordered IMU overload fail explicitly."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    logs = Path('log/validation') / args.output.name
    logs.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('ROS_DOMAIN_ID', '70')
    os.environ['ROS_LOG_DIR'] = str(logs.resolve() / 'ros')
    exe = Path(get_package_prefix('super_lio')) / 'lib/super_lio/super_lio_node'
    config = Path(get_package_share_directory('super_lio')) / 'config/nclt_20130110.yaml'
    common = [str(exe), '--ros-args', '--params-file', str(config),
              '-p', 'lio.map.save_map:=false', '-p', 'lio.evo.pose_output_en:=false',
              '-p', 'lio.ros.imu_reorder_window:=0.1',
              '-p', f'lio.evo.pose_output_dir:={args.output.resolve()}/trajectory']
    with (logs / 'invalid_mode.log').open('w') as output:
        process = subprocess.run(common + ['-p', 'lio.map.mode:=invalid'], stdout=output,
                                 stderr=subprocess.STDOUT, timeout=20)
    assert process.returncode != 0, 'Unknown map mode silently fell back to legacy'
    assert 'must be legacy or streamed' in (logs / 'invalid_mode.log').read_text()
    rclpy.init()
    monitor = rclpy.create_node('input_guard_check')
    publisher = monitor.create_publisher(Imu, '/imu_raw', 10)
    with (logs / 'reordered_overload.log').open('w') as output:
        process = subprocess.Popen(common + ['-p', 'lio.ros.max_imu_queue:=1'], stdout=output,
                                   stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 20
            while not publisher.get_subscription_count():
                assert process.poll() is None
                assert time.monotonic() < deadline, 'IMU discovery timed out'
                rclpy.spin_once(monitor, timeout_sec=.1)
            first = Imu()
            first.header.stamp.sec = 100
            first.linear_acceleration.z = 9.81
            publisher.publish(first)
            time.sleep(.3)
            second = Imu()
            second.header.stamp.sec = 99
            second.header.stamp.nanosec = 990000000
            second.linear_acceleration.z = 9.81
            publisher.publish(second)
            assert process.wait(timeout=15) != 0, 'Reordered queue overflow did not fail'
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=20)
    monitor.destroy_node()
    rclpy.shutdown()
    assert 'IMU queue limit exceeded during reordering' in (
        logs / 'reordered_overload.log').read_text()
    result = {'invalid_mode_rejected': True, 'reordered_imu_budget_enforced': True}
    (args.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
