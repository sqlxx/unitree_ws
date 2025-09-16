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
  log_level = LaunchConfiguration('log_level')
  use_composition = LaunchConfiguration('use_composition')
  container_name = LaunchConfiguration('container_name')

  param_substitutions = {
    'use_sim_time': use_sim_time
  }

  configured_params = ParameterFile(
    RewrittenYaml(
      source_file=os.path.join(get_package_share_directory('tb3_navigation'), 'config', 'navigation_config.yaml'),
      param_rewrites=param_substitutions,
      convert_types=True
    ), allow_substs=True
  )
  
  lifecycle_nodes = ['controller_server',
                     'smoother_server', 
                     'planner_server', 
                     'behavior_server', 
                     'bt_navigator', 
                     'waypoint_follower', 
                     'velocity_smoother']

  nodes_cmd = GroupAction(
    condition=IfCondition(PythonExpression(['not ', use_composition])),
    actions=[    
      Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time}, {'node_names': lifecycle_nodes}, {'autostart': True}]
      ),
      Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
      ),
      Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
      ),
      Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]
      ),
      Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]
      ),
      Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]
      ),
      Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]
      ),
      Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        respawn=True,
        respawn_delay=2.0,
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]
      )
    ]
  )

  composite_nodes_cmd = LoadComposableNodes(
    condition=IfCondition(use_composition),
    target_container=container_name,
    composable_node_descriptions=[
      ComposableNode(
        package='nav2_lifecycle_manager',
        plugin='nav2_lifecycle_manager::LifecycleManager',
        name='lifecycle_manager_navigation',
        parameters=[{'use_sim_time': use_sim_time}, {'node_names': lifecycle_nodes}, {'autostart': True}]        
      ),
      ComposableNode(
        package='nav2_controller',
        plugin='nav2_controller::ControllerServer',
        name='controller_server',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
      ),
      ComposableNode(
        package='nav2_velocity_smoother',
        plugin='nav2_velocity_smoother::VelocitySmoother',
        name='velocity_smoother',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
      ),
      ComposableNode(
        package='nav2_planner',
        plugin='nav2_planner::PlannerServer',
        name='planner_server',
        parameters=[configured_params]
      ),
      ComposableNode(
        package='nav2_smoother',
        plugin='nav2_smoother::SmootherServer',
        name='smoother_server',
        parameters=[configured_params]
      ),
      ComposableNode(
        package='nav2_behaviors',
        plugin='behavior_server::BehaviorServer',
        name='behavior_server',
        parameters=[configured_params]
      ),
      ComposableNode(
        package='nav2_bt_navigator',
        plugin='nav2_bt_navigator::BtNavigator',
        name='bt_navigator',
        parameters=[configured_params]
      ), 
      ComposableNode(
        package='nav2_waypoint_follower',
        plugin='nav2_waypoint_follower::WaypointFollower',
        name='waypoint_follower',
        parameters=[configured_params]
      )
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

  declare_log_level_cmd = DeclareLaunchArgument(
    'log_level',
    description='log level',
    default_value='info'
  )

  ld = LaunchDescription()
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_composition_cmd)
  ld.add_action(declare_container_name_cmd)
  ld.add_action(declare_log_level_cmd)
  ld.add_action(nodes_cmd)
  ld.add_action(composite_nodes_cmd)

  return ld

