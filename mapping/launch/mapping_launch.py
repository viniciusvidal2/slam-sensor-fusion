from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'map_data_path',
            default_value=os.path.join(os.path.expanduser('~'), 'Desktop', 'map_data'),
            description='The path to the map data'
        ),
        DeclareLaunchArgument(
            'enable_debug',
            default_value='False',
            description='Enable debug mode'
        ),
        # Launch the localization node
        Node(
            package='mapping',
            executable='map_data_save_node',
            name='mapping_node',
            output='screen',
            parameters=[{
                'map_data_path': LaunchConfiguration('map_data_path'),
                'enable_debug': LaunchConfiguration('enable_debug'),
            }]
        )
    ])
