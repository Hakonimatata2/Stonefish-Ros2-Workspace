from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    package_name = 'stonefish_bluerov2'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'),
                    'launch',
                    'stonefish_simulator.launch.py'
                ])
            ]),
            launch_arguments = {
                'simulation_data' : PathJoinSubstitution([FindPackageShare(package_name), 'data']),
                'scenario_desc' : PathJoinSubstitution([FindPackageShare(package_name), 'scenarios', 'scenario1.scn']),
                'simulation_rate' : '300.0',
                'window_res_x' : '1200',
                'window_res_y' : '800',
                'rendering_quality' : 'high'
            }.items()
        )
    ])