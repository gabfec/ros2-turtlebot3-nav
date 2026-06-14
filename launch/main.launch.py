from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_nav',
            executable='find_wall_server',
            name='find_wall_node'
        ),
        Node(
            package='turtlebot3_nav',
            executable='wall_follower',
            name='wall_follower_node'
        )
    ])
