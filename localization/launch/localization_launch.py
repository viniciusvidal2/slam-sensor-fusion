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
            'map_name',
            default_value='map',
            description='The map name'
        ),
        DeclareLaunchArgument(
            'enable_debug',
            default_value='False',
            description='Enable debug mode'
        ),
        DeclareLaunchArgument(
            'max_map_optimization_poses',
            default_value='50.0',
            description='How many poses to use for map optimization, maximum value'
        ),
        
        # Launch the localization node
        Node(
            package='localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[{
                'map_data_path': LaunchConfiguration('map_data_path'),
                'map_name': LaunchConfiguration('map_name'),
                'enable_debug': LaunchConfiguration('enable_debug'),
                'max_map_optimization_poses': LaunchConfiguration('max_map_optimization_poses')
            }]
        )
    ])
