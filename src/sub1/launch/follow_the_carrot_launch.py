from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sub1',
            node_executable='odom',
            node_name='udp_to_pub'
        ),
        Node(
            package='sub1',
            node_executable='path_pub',
            node_name='path_pub'
        ),

        Node(
            package='sub1',
            node_executable='path_tracking',
            node_name='path_tracking'
        ) 
    ])



