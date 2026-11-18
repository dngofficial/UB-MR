from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [('cloud_topic', '/sensing/lidar/concatenated/pointcloud'),
            ('map_frame', 'map'), ('expected_scan_frame', ''), ('use_sim_time', 'false')]
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value) for name, value in args] + [
        Node(package='rotation_integration', executable='snapshot', output='screen',
             parameters=[{'cloud_topic': LaunchConfiguration('cloud_topic'),
                          'map_frame': LaunchConfiguration('map_frame'),
                          'expected_scan_frame': LaunchConfiguration('expected_scan_frame'),
                          'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])])
