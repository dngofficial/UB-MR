"""Publish verified recorded-sensor fixed joints using the existing ROS robot_state_publisher."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rotation_integration.frame_chain import replay_description


def start_frames(context):
    description = replay_description(LaunchConfiguration('calibration_file').perform(context),
                                     LaunchConfiguration('profile').perform(context))
    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='lincoln_recorded_sensor_frames', output='screen',
                 parameters=[{'robot_description': description,
                              'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', description='Calibration candidate JSON with verified recording provenance'),
        DeclareLaunchArgument('profile', description='Complete profile confirmed for this bag'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=start_frames)])
