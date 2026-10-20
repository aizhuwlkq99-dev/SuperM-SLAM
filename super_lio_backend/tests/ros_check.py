#!/usr/bin/env python3
"""Exercise asynchronous export and bounded tile visualization against a real ROS graph."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import PointCloud2
from super_lio_interfaces.srv import ExportMap
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('store', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('ROS_DOMAIN_ID', '49')
    rclpy.init()
    monitor = rclpy.create_node('superm_backend_check')
    prefix = Path(get_package_prefix('super_lio_backend')) / 'lib/super_lio_backend'
    logfile = (args.output / 'backend.log').open('w')
    backend = subprocess.Popen([
        str(prefix / 'superm_backend_node'), '--ros-args',
        '-p', f'store_path:={args.store.resolve()}',
        '-p', f'graph_directory:={args.output.resolve()}/graph',
        '-p', 'loop_closure:=false'], stdout=logfile, stderr=subprocess.STDOUT)
    viewer = None
    try:
        client = monitor.create_client(ExportMap, '/slam/save_map')
        assert client.wait_for_service(timeout_sec=15), 'Export service discovery failed'
        time.sleep(2)
        request = ExportMap.Request()
        request.output_directory = str(args.output.resolve() / 'map')
        request.leaf_size = .1
        request.single_pcd = True
        future = client.call_async(request)
        rclpy.spin_until_future_complete(monitor, future, timeout_sec=20)
        assert future.done() and future.result().accepted, 'Export request was rejected'
        manifest = args.output / 'map/manifest.json'
        deadline = time.monotonic() + 40
        while not manifest.exists() and time.monotonic() < deadline:
            assert backend.poll() is None, 'Backend exited during export'
            rclpy.spin_once(monitor, timeout_sec=.1)
        result = json.loads(manifest.read_text())
        assert result['complete'] and result['frame_count'] == 81, 'Snapshot lost committed frames'
        undo = monitor.create_client(Trigger, '/slam/reject_last_loop')
        assert undo.wait_for_service(timeout_sec=5)
        future = undo.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(monitor, future, timeout_sec=5)
        assert future.done() and not future.result().success, 'Empty-loop revocation should fail cleanly'
        clouds = []
        subscription = monitor.create_subscription(PointCloud2, '/slam/map_tiles', clouds.append, 1)
        viewer = subprocess.Popen([
            str(prefix / 'superm_map_viewer'), '--ros-args',
            '-p', f'manifest:={manifest.resolve()}', '-p', 'max_points:=1000'],
            stdout=logfile, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 10
        while not clouds and time.monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=.1)
        assert clouds and 0 < clouds[0].width <= 1000, 'Tile preview exceeded its point budget or was empty'
        assert clouds[0].header.frame_id == 'map_view', 'Tile origins were not handled in a local viewing frame'
        print(json.dumps(dict(export_frames=result['frame_count'], points=result['points'],
                              preview_points=clouds[0].width, passed=True)))
    finally:
        for process in (viewer, backend):
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=30)
                assert process.returncode == 0, f'Node failed to stop cleanly: {process.returncode}'
        logfile.close()
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
