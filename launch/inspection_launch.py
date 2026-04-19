from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project_2',
            executable='inspection_robot_node',
            name='motion_control_node',
            output='screen'
        ),
        Node(
            package='project_2',
            executable='battery_life_node',
            name='battery_life_node',
            output='screen'
        ),
        Node(
            package='project_2',
            executable='image_processing_node',
            name='image_processing_node',
            output='screen'
        ),
         Node(
            package='project_2',
            executable='robot_location_manager_node',
            name='robot_location_manager_node',
            output='screen'
        )
        
    ])

