import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  params_file = os.path.join(get_package_share_directory('unitree_bringup'), 'config', 'common.yaml')

  declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='False')

  return LaunchDescription([
    declare_use_sim_time_cmd,
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
      parameters=[params_file]
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('unitree_slam'), 'launch', 'slam_sync.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items()
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('unitree_rosbridge'), 'launch', 'rosbridge.launch.py'])
    )
    
  ])