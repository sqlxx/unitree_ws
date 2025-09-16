import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  log_level = LaunchConfiguration('log_level')

  lifecycle_nodes = ['map_saver']

  param_substitutions = {
    'use_sim_time': use_sim_time
  }

  configured_params = ParameterFile(
    RewrittenYaml(
      source_file=os.path.join(get_package_share_directory('tb3_navigation'), 'config', 'slam_config.yaml'),
      param_rewrites=param_substitutions,
      convert_types=True
    ), allow_substs=True
  )

  return LaunchDescription([
    Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='lifecycle_manager_slam',
      output='screen',
      arguments=['--ros-args', '--log-level', log_level],
      parameters=[{'use_sim_time': use_sim_time}, {'node_names': lifecycle_nodes}, {'autostart': 'true'}]
    ),
    Node(
      package='nav2_map_server',
      executable='map_saver_server',
      output='screen',
      respawn=True,
      respawn_delay=2.0,
      arguments=['--ros-args', '--log-level', log_level],
      parameters=[configured_params]
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('tb3_slam'), 'launch', 'slam_toolbox_sync.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time, 'log_level': log_level}.items()
    )
  ])
