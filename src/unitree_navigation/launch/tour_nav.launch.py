import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  map_yaml_file = LaunchConfiguration('map')
  log_level = LaunchConfiguration('log_level')
  use_amcl = LaunchConfiguration('use_amcl')
  use_slam = LaunchConfiguration('use_slam')
  use_composition = LaunchConfiguration('use_composition')
  container_name = LaunchConfiguration('container_name')

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      description='Use simulation time if true'
    ),
    DeclareLaunchArgument(
      'map',
      description="Path to map yaml file"
    ),
    DeclareLaunchArgument(
      'log_level',
      description='log level',
      default_value='info'
    ),
    DeclareLaunchArgument(
      'use_amcl',
      description='Use amcl if true, use slam toolbox if false',
      default_value='False'
    ),
    DeclareLaunchArgument(
      'use_slam',
      description='Use SLAM if true',
      default_value='False'
    ),
    DeclareLaunchArgument(
      'use_composition',
      description='Use container for nav nodes if true',
      default_value='False'
    ),
    DeclareLaunchArgument(
      'container_name',
      description='the name of the container that nodes will load in to if use_composition is true',
      default_value='nav_container'
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('tb3_navigation'), 'launch', 'slam.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time, 'log_level': log_level}.items(),
      condition=IfCondition(use_slam)
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('tb3_navigation'), 'launch', 'localization.launch.py']),
      launch_arguments={
        'use_sim_time': use_sim_time, 
        'log_level': log_level, 
        'map': map_yaml_file, 
        'use_composition': use_composition, 
        'container_name': container_name,
        'use_amcl': use_amcl}.items(),
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('tb3_navigation'), 'launch', 'navigation.launch.py']),
      launch_arguments={
        'use_sim_time': use_sim_time, 
        'log_level': log_level, 
        'use_composition': use_composition, 
        'container_name': container_name}.items(),
    )
  ])
