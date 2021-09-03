from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='security_service',
            node_executable='odom',
            node_name='odom'
        ),
        
        Node(
            package='security_service',
            node_executable='make_path',
            node_name='make_path'
        ),

        # Node(
        #     package='security_service',
        #     node_executable='patrol_client',
        #     node_name='patrol_client'
        # )
    ])



