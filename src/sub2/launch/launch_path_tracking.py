from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sub2',
            node_executable='odom',
            node_name='odom'
        ),
        Node(
            package='sub2',
            node_executable='load_map',
            node_name='load_map'
        ),
        Node(
            package='sub2',
            node_executable='a_star',
            node_name='a_star'
        ),
        Node(
            package='sub2',
            node_executable='a_star_local_path',
            node_name='a_star_local_path'
        ),
        Node(
            package='sub2',
            node_executable='path_tracking',
            node_name='path_tracking'
        ),
    ])



