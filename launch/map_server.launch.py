from ament_index_python import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from launch import LaunchDescription
from launch_ros.actions import Node

config = os.path.join(get_package_share_directory('particle_filter'),'config','map_server_params.yaml')
map = os.path.join(get_package_share_directory('maps'), 'icra_ext' ,'map.yaml')
autostart = True
lifecycle_nodes = ['map_server']

def generate_launch_description():
    ld = LaunchDescription()
    map_server = Node(
        package="nav2_map_server",
        namespace="",
        executable="map_server",
        name="map_server",
        parameters=[{"yaml_filename": map}],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        name="lifecycle_manager_navigation",
        executable="lifecycle_manager",
        output='screen',
        parameters=[{'autostart': autostart},
                {'node_names': lifecycle_nodes}],
    )

    ld.add_action(map_server)
    ld.add_action(lifecycle)
    return ld
