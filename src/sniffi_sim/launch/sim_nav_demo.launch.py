from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('sniffi_sim')
    urdf = os.path.join(pkg, 'urdf', 'sniffi_sim.urdf')
    world = os.path.join(pkg, 'worlds', 'sniffi.world')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    rtab_params = os.path.join(pkg, 'config', 'rtabmap_params.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'nav_demo.rviz')

    return LaunchDescription([
        # Gazebo Classic
        ExecuteProcess(cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'], output='screen'),

        # Robot state publisher (sim time)
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'use_sim_time': True}], arguments=[urdf]),

        # Spawn robot
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-file', urdf, '-entity', 'sniffi', '-x', '0', '-y', '0', '-z', '0.15']),

        # Safety + Power (from your Week-2 packages)
        Node(package='sniffi_safety', executable='safety_node', name='sniffi_safety',
             parameters=[{'watchdog_timeout_s': 3.0}]),
        Node(package='sniffi_power', executable='power_node', name='sniffi_power',
             parameters=[{'low_battery_soc': 20.0, 'publish_hz': 1.0}]),

        # RTAB-Map (lidar-only)
        Node(package='rtabmap_ros', executable='rtabmap', name='rtabmap', output='screen',
             parameters=[{'frame_id': 'base_link', 'subscribe_depth': False, 'subscribe_rgb': False,
                          'use_sim_time': True}, rtab_params],
             remappings=[('scan', '/scan'), ('odom', '/odom')]),

        # Nav2 servers
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['controller_server','planner_server','bt_navigator',
                                         'behavior_server','smoother_server']}]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_params]),
        Node(package='nav2_behavior_tree', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[nav2_params]),

        # RViz (nice for the video)
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg]),
    ])
