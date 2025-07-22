from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    inner_node = Node(
        package='geometric_controller',
        executable='inner_loop_node',
        name='inner_loop_node'
    )

    outer_node = Node(
        package='geometric_controller',
        executable='outer_loop_node',
        name='outer_loop_node'
    )

    offboard_node = Node(
        package='geometric_controller',
        executable='off_board_node',
        name='off_board_node'
    )

    return LaunchDescription([
        offboard_node,
        TimerAction(period=2.0, actions=[inner_node]),  # Launch after 2 seconds
        TimerAction(period=4.0, actions=[outer_node])  # Launch after 4 seconds
    ])
