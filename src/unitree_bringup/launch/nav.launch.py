import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


######### mid360 config values ######
xfer_format = 2
multi_topic = 0
data_src = 0
publish_freq = 10.0
output_type = 1
frame_id = 'mid360_link'
# lvx_file_path = '/'
# cmdline_bd_code = 'livox0000000001'

#####################################


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_composition = LaunchConfiguration('use_composition')
  map_yaml_file = LaunchConfiguration('map')
  log_level = LaunchConfiguration('log_level')
  use_amcl = LaunchConfiguration('use_amcl')
  container_name = 'unitree_container'

  livox_ros2_params = [
    {'xfer_format': xfer_format},
    {'multi_topic': multi_topic},
    {'data_src': data_src},
    {'publish_freq': publish_freq},
    {'output_type': output_type},
    {'frame_id': frame_id},
    # {'lvx_file_path': lvx_file_path},
    # {'cmdline_input_bd_code': cmdline_bd_code},
    {'user_config_path': os.path.join(get_package_share_directory('unitree_bringup'), 'config', 'mid360_config.yaml')}
  ]

  declare_map_yaml_cmd = DeclareLaunchArgument('map', description='Full path to map yaml file to load')
  declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='False')
  declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='False')
  declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info')
  declare_use_amcl_cmd = DeclareLaunchArgument('use_amcl', description='use amcl if True, use slam_toolbox if False', default_value='False')

  bringup_node_group = GroupAction([
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'robot_description': os.path.join(get_package_share_directory('unitree_bringup'), 'urdf', 'g1_23dof_mode_10.urdf')}]
    ),
    Node(
      package='livox_ros_driver2',
      executable='livox_ros_driver2_node',
      name='livox_lidar_publisher',
      output='screen',
      parameters=livox_ros2_params
    ),
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
