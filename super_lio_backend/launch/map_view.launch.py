"""Display a bounded region of full-resolution map tiles."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('super_lio_backend'))
    repo = Path(os.environ.get('SUPERM_WORKSPACE', Path.cwd()))
    log_root = Path(os.environ.get('SUPERM_LOG_DIR', str(repo / 'log')))
    log = Path(os.environ.get('ROS_LOG_DIR', str(log_root / 'ros')))
    log.mkdir(parents=True, exist_ok=True)
    defaults = {'manifest': '', 'center_x': '0.0', 'center_y': '0.0', 'center_z': '0.0',
                'radius': '100.0', 'max_points': '200000', 'rviz': 'true'}
    params = {'manifest': LaunchConfiguration('manifest')}
    for key in ('center_x', 'center_y', 'center_z', 'radius', 'max_points'):
        params[key] = ParameterValue(LaunchConfiguration(key),
                                     value_type=int if key == 'max_points' else float)
    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', str(log)),
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        Node(package='super_lio_backend', executable='superm_map_viewer', output='screen',
             parameters=[params]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(share / 'config' / 'tiles.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
