from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the inner control loop launch file
    geometric_controller_path = os.path.join(
        get_package_share_directory('geometric_controller'),  
        'launch',
        'geometric_controller_launch.py'  
    )

    geometric_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(geometric_controller_path)
    )

    planner_node = Node(
        package='trajectory_planner_py',
        executable='planner_node',
        name='planner_node'
    )

    return LaunchDescription([
        geometric_controller_launch,
        TimerAction(period=5.0, actions=[planner_node]),  
    ])
