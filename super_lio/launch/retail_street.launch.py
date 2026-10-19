"""Run SuperM-LIO in ROS 2 Humble with an isolated output directory."""
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('super_lio'))
    args = [
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/retail_street.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dyn_filter', default_value='true'),
        DeclareLaunchArgument('filter_before_observe', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=str(
            Path.cwd() / 'outputs' / datetime.now().strftime('%Y%m%d_%H%M%S_%f'))),
    ]
    params = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'dyn_obj.en': ParameterValue(LaunchConfiguration('dyn_filter'), value_type=bool),
        'lio.kf.dyn_filter_before_observe': ParameterValue(
            LaunchConfiguration('filter_before_observe'), value_type=bool),
        'lio.map.save_map_dir': PathJoinSubstitution([LaunchConfiguration('output_dir'), 'map']),
        'lio.evo.pose_output_dir': PathJoinSubstitution([LaunchConfiguration('output_dir'), 'trajectory']),
    }
    return LaunchDescription(args + [
        Node(package='super_lio', executable='super_lio_node', output='screen',
             parameters=[LaunchConfiguration('params_file'), params], sigterm_timeout='120'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(share / 'rviz/retail_street.rviz')],
             parameters=[{'use_sim_time': params['use_sim_time']}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
