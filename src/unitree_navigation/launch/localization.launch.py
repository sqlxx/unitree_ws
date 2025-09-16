import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile, ComposableNode
from launch_ros.substitutions import FindPackageShare 
from launch_ros.actions import Node, LoadComposableNodes 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  map_yaml_file = LaunchConfiguration('map')
  log_level = LaunchConfiguration('log_level')
  use_composition = LaunchConfiguration('use_composition')
  container_name = LaunchConfiguration('container_name')
  use_amcl = LaunchConfiguration('use_amcl')

  param_substitutions = {
    'use_sim_time': use_sim_time,
    'yaml_filename': map_yaml_file
  }

  configured_params = ParameterFile(
    RewrittenYaml(
      source_file=os.path.join(get_package_share_directory('tb3_navigation'), 'config', 'localization_config.yaml'),
      param_rewrites=param_substitutions,
      convert_types=True
    ), allow_substs=True
  )
  
  lifecycle_nodes = ['map_server']
  # lifecycle_nodes = ['map_server', 'amcl']

  nodes = GroupAction(
    condition=IfCondition(PythonExpression(['not ', use_composition])),
    actions=[    
      Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time}, {'node_names': lifecycle_nodes}, {'autostart': True}]
      ),
      Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]
      ),
      Node(
        condition=IfCondition(use_amcl),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]
      ),
    ]
  )

  composite_nodes = LoadComposableNodes(
    condition=IfCondition(use_composition),
    target_container=container_name,
    composable_node_descriptions=[
      ComposableNode(
        package='nav2_lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': use_sim_time}, {'node_names': lifecycle_nodes}, {'autostart': True}]        
      ),
      ComposableNode(
        package='nav2_map_server',
        plugin='nav2_map_server::MapServer',
        name='map_server',
        parameters=[configured_params],
      ),
      ComposableNode(
        package='nav2_amcl',
        plugin='nav2_amcl::AmclNode',
        name='amcl',
        parameters=[configured_params],
        condition=IfCondition(use_amcl)
      ),

    ]
  )

  declare_use_composition_cmd = DeclareLaunchArgument(
    'use_composition',
    description='Use container for nav nodes if true',
    default_value='False'
  )

  declare_container_name_cmd = DeclareLaunchArgument(
    'container_name',
    description='the name of the container that nodes will load in to if use_composition is true',
    default_value='nav_container'
  )

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    description='Use simulation time if true',
    default_value='True'
  )

  declare_use_amcl_cmd = DeclareLaunchArgument(
    'use_amcl',
    description='Use amcl if true, use slam toolbox if false',
    default_value='False'
  )

  slam_toolbox_localization_cmd = IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('tb3_slam'), 'launch', 'slam_toolbox_localization.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time, 'log_level': log_level}.items(),
      condition=IfCondition(PythonExpression(['not ', use_amcl]))
  )

  ld = LaunchDescription()
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_composition_cmd)
  ld.add_action(declare_container_name_cmd)
  ld.add_action(declare_use_amcl_cmd)
  ld.add_action(nodes)
  ld.add_action(composite_nodes)
  ld.add_action(slam_toolbox_localization_cmd)

  return ld
                          
  


