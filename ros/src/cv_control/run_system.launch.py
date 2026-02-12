from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cv_control',
            executable='yolo_node',
            name='yolo_detector',
            output='screen',
            parameters=[
                {'model': 'yolo26n.pt'} 
            ]
        ),
        Node(
            package='cv_control',
            executable='hand_node',
            name='hand_detector',
            output='screen',
            # Parameters for config paths are defaults in the node code, 
            # but can be overridden here if needed.
        ),
    ])
