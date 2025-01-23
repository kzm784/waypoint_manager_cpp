import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Set the path to the waypoint CSV
    navigation_data_dir = os.getenv('NAVIGATION_DATA_DIR')
    navigation_data_name = os.getenv('NAVIGATION_DATA_NAME')
    waypoints_csv_path = os.path.join(
        navigation_data_dir,
        navigation_data_name,
        f"{navigation_data_name}_wp.csv"
    )

    # Set the path to the waypoint_manager_cpp config
    waypoint_manager_cpp_config = launch.substitutions.LaunchConfiguration(
        'waypoint_manager_cpp_config',
        default=os.path.join(
            get_package_share_directory('waypoint_manager_cpp'),
                'config',
                'config_waypoint_manager_cpp.yaml'
        )
    )

    waypoint_manager_cpp_component = Node(
        package='waypoint_manager_cpp',
        executable='waypoint_manager_cpp_node',
        name='waypoint_manager_cpp_node',
        output='screen',
        parameters=[waypoint_manager_cpp_config, {
                'waypoints_csv': waypoints_csv_path
        }]
    )
    
    ld.add_action(waypoint_manager_cpp_component)

    return ld