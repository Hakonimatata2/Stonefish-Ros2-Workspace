from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='listener',
            executable='dvl_logger',
            name='dvl_logger',
            output='screen',
            parameters=[{
                'dvl_topic': '/dvl',
                'best_effort_qos': True,
                'log_to_csv': False,
                'csv_path': '/tmp/dvl_log.csv',
                'log_beams_as_json': True,
            }]
        )
    ])
