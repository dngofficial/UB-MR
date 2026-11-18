"""Start the one-shot map service; Autoware and Unity run separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [DeclareLaunchArgument('config_file', description='Confirmed map-to-map seed JSON')]
    defaults = {
        'use_sim_time': 'true', 'autoware_map_topic': '/map/pointcloud_map',
        'unity_map_topic': '/unity/reference_cloud_ros',
        'localization_topic': '/localization/kinematic_state', 'output_report': '',
        'require_initialized': 'true', 'autoware_transient_local': 'true',
        'unity_transient_local': 'false',
    }
    arguments += [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    params = {name: LaunchConfiguration(name) for name in ('config_file', *defaults)}
    for name in ('use_sim_time', 'require_initialized', 'autoware_transient_local', 'unity_transient_local'):
        params[name] = ParameterValue(LaunchConfiguration(name), value_type=bool)
    params['output_report'] = ParameterValue(LaunchConfiguration('output_report'), value_type=str)
    return LaunchDescription(arguments + [Node(package='rotation_integration', executable='align_maps',
                                               output='screen', parameters=[params])])
