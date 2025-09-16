from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_composition = LaunchConfiguration('use_composition')
  map_yaml_file = LaunchConfiguration('map')
  log_level = LaunchConfiguration('log_level')
  use_amcl = LaunchConfiguration('use_amcl')
  container_name = 'unitree_container'

  declare_map_yaml_cmd = DeclareLaunchArgument('map', description='Full path to map yaml file to load')
  declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True')
  declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='False')
  declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info')
  declare_use_amcl_cmd = DeclareLaunchArgument('use_amcl', description='use amcl if True, use slam_toolbox if False', default_value='False')

  bringup_node_group = GroupAction([
    Node(
      condition=IfCondition(use_composition),
      name=container_name,
      package='rclcpp_components',
      executable='component_container_isolated',
      parameters=[{'autostart': 'true'}],
      arguments=['--ros-args', '--log-level', log_level],
      output='screen'
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('unitree_navigation'), 'launch', 'tour_nav.launch.py']),
      launch_arguments={
        'use_sim_time': use_sim_time, 
        'map': map_yaml_file, 
        'log_level': log_level, 
        'use_composition': use_composition, 
        'container_name': container_name,
        'use_amcl': use_amcl}.items()
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('unitree_rosbridge'), 'launch', 'rosbridge.launch.py'])
    )

  ])
 
  ld = LaunchDescription()
  ld.add_action(declare_map_yaml_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_composition_cmd)
  ld.add_action(declare_log_level_cmd)
  ld.add_action(declare_use_amcl_cmd)

  ld.add_action(bringup_node_group)

  return ld
