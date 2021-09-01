from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ssafy_bridge',
            node_executable='udp_to_pub',
            node_name='udp_to_pub'
        ),
        Node(
            package='ssafy_bridge',
            node_executable='sub_to_udp',
            node_name='sub_to_udp'
        ),
        Node(
            package='ssafy_bridge',
            node_executable='udp_to_cam',
            node_name='udp_to_cam'
        ),

        Node(
            package='ssafy_bridge',
            node_executable='udp_to_laser',
            node_name='udp_to_laser'
        ) 
    ])



