from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  return LaunchDescription([
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('unitree_slam'), 'launch', 'slam_toolbox_sync.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items()
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('unitree_rosbridge'), 'launch', 'rosbridge.launch.py'])
    )
    
  ])