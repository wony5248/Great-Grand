from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            node_name='talker',
            node_executable='talker',
            output='screen'
        ),
        Node(
            package='my_package',
            node_name='listener',
            node_executable='listener',
            name='screen'
        )
    ])