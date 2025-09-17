import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, DeclareLaunchArgument
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
      parameters=livox_ros2_params
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('unitree_slam'), 'launch', 'slam_sync.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items()
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('unitree_rosbridge'), 'launch', 'rosbridge.launch.py'])
    )
    
  ])