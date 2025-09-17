import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
  pkg_dir = get_package_share_directory('unitree_slam')
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_dir, 'config/sync_online_params.yaml'))

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='true',
      description='use simulation clock if true'
    ),
    DeclareLaunchArgument(
      'params_file',
      default_value=params_file,
      description='Full path to the ROS2 parameters file to use'
    ),
    Node(
      package="slam_toolbox",
      executable='sync_slam_toolbox_node',
      name='slam_toolbox',
      output='screen',
      parameters=[params_file, {'use_sim_time': use_sim_time}],
      remappings=[
        ('/scan', '/scan')
      ]
    )
  ])