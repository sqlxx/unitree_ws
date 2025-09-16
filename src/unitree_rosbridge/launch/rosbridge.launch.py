from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package="rosbridge_server",
      executable="rosbridge_websocket",
      output="screen",
      name="rosbridge_websocket_server",
      parameters=[{
        "port": 9090,
        "retry_startup_delay": 5.0,
        "call_services_in_new_thread": True,
        "send_action_goals_in_new_thread": True,
        "default_call_service_timeout": 5.0
      }]
    ),
  ])