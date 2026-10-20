"""Run streamed mapping with independent, optional loop optimization in Humble."""
import hashlib
import json
import os
import subprocess
from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    dataset = LaunchConfiguration('dataset').perform(context)
    if dataset not in ('nclt', 'retail'):
        raise ValueError('dataset must be nclt or retail')

    def enabled(name):
        return LaunchConfiguration(name).perform(context).lower() == 'true'

    output = Path(LaunchConfiguration('output_dir').perform(context)).expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError('output_dir must be a new or empty directory')
    output.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory('super_lio'))
    backend = Path(get_package_share_directory('super_lio_backend'))
    config = share / 'config' / ('nclt_20130110.yaml' if dataset == 'nclt' else 'retail_street.yaml')
    params_file = LaunchConfiguration('params_file').perform(context) or str(config)
    # Resolve once so node-name matching cannot silently skip parameters in upstream YAML.
    document = yaml.safe_load(Path(params_file).read_text())

    def flatten(values, prefix=''):
        result = {}
        for key, value in values.items():
            name = prefix + key
            if isinstance(value, dict):
                result.update(flatten(value, name + '.'))
            else:
                result[name] = value
        return result

    params = flatten(next(iter(document.values()))['ros__parameters'])
    sim_time = enabled('use_sim_time')
    params.update({
        'use_sim_time': sim_time,
        'dyn_obj.en': enabled('dyn_filter'),
        'lio.map.save_map': True,
        'lio.map.mode': 'streamed',
        'lio.map.save_map_dir': str(output / 'map'),
        'lio.evo.pose_output_dir': str(output / 'trajectory'),
    })
    (output / 'parameters.yaml').write_text(
        yaml.safe_dump({'/**': {'ros__parameters': params}}, sort_keys=True))
    # Explicit paths support root in Docker reading a repository owned by the host user.
    workspace = Path(os.environ.get('SUPERM_WORKSPACE', Path.cwd())).resolve()
    git = ['git', f'--git-dir={workspace / ".git"}', f'--work-tree={workspace}']
    revision = subprocess.run(git + ['rev-parse', 'HEAD'], capture_output=True, text=True)
    changes = subprocess.run(git + ['status', '--porcelain'], capture_output=True, text=True)
    (output / 'provenance.json').write_text(json.dumps({
        'dataset': dataset,
        'git_revision': revision.stdout.strip() if revision.returncode == 0 else 'unknown',
        'working_tree_dirty': bool(changes.stdout.strip()) if changes.returncode == 0 else None,
        'source_configuration': str(params_file),
        'configuration_sha256': hashlib.sha256(Path(params_file).read_bytes()).hexdigest(),
        'coordinate_frame': 'body/IMU; original actual deskew reference time',
        'loop_closure': enabled('loop_closure'),
        'map_leaf': 0.1 if dataset == 'nclt' else 0.05,
        'parameters': 'parameters.yaml',
    }, indent=2) + '\n')
    return [
        Node(package='super_lio', executable='super_lio_node', output='screen',
             parameters=[params], sigterm_timeout='120', sigkill_timeout='30'),
        Node(package='super_lio_backend', executable='superm_backend_node', output='screen',
             parameters=[{
                 'use_sim_time': sim_time,
                 'store_path': str(output / 'map' / 'store'),
                 'graph_directory': str(output / 'graph'),
                 'loop_closure': enabled('loop_closure'),
                 'map_leaf': 0.1 if dataset == 'nclt' else 0.05,
                 'shutdown_export': LaunchConfiguration('shutdown_export').perform(context),
                 'shutdown_map_directory': str(output / 'maps'),
             }], sigterm_timeout='600', sigkill_timeout='30'),
    ] + ([Node(package='rviz2', executable='rviz2', output='screen',
               arguments=['-d', str(backend / 'config' / 'mapping.rviz')],
               parameters=[{'use_sim_time': sim_time}])] if enabled('rviz') else [])


def generate_launch_description():
    repo = Path(os.environ.get('SUPERM_WORKSPACE', Path.cwd()))
    run = repo / 'outputs' / ('large_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    log_root = Path(os.environ.get('SUPERM_LOG_DIR', str(repo / 'log')))
    log = Path(os.environ.get('ROS_LOG_DIR', str(log_root / 'ros')))
    log.mkdir(parents=True, exist_ok=True)
    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', str(log)),
        DeclareLaunchArgument('dataset', default_value='nclt'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=str(run)),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dyn_filter', default_value='true'),
        DeclareLaunchArgument('loop_closure', default_value='false'),
        DeclareLaunchArgument('shutdown_export', default_value='raw',
                              description='none, raw, or both (raw plus optimized)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_start),
    ])
