from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    params_file = os.path.join(
        os.getenv('COLCON_CURRENT_PREFIX', ''), 'share', 'dwa_local_planner', 'config', 'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='dwa_local_planner',
            executable='dwa_planner',
            name='dwa_planner',
            output='screen',
            parameters=[params_file],
        )
    ])
