from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sniffi_power',
            executable='power_node',
            name='sniffi_power',
            parameters=[{'low_battery_soc': 20.0, 'publish_hz': 1.0}]
        ),
        Node(
            package='sniffi_safety',
            executable='safety_node',
            name='sniffi_safety',
            parameters=[{'watchdog_timeout_s': 3.0}]
        ),
    ])
