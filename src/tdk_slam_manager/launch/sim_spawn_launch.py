import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    localization_pkg = get_package_share_directory('tdk_slam_manager')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    localization_mode = LaunchConfiguration('localization_mode', default='mapping')
    
    xacro_file = os.path.join(localization_pkg, 'urdf', 'sensors.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'tdk_robot'],
        output='screen'
    )

    filter_front = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='filter_front',
        parameters=[os.path.join(localization_pkg, 'config', 'laser_filter_params.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/front/scan'), ('scan_filtered', '/front/scan_filtered')]
    )

    filter_rear = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='filter_rear',
        parameters=[os.path.join(localization_pkg, 'config', 'laser_filter_params.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/rear/scan'), ('scan_filtered', '/rear/scan_filtered')]
    )


    merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laser_merger',
        parameters=[os.path.join(localization_pkg, 'config', 'laser_merger_params.yaml'),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    mapping_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'mapping'"])),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(localization_pkg, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    localization_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'slam_toolbox'"])),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(localization_pkg, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('localization_mode', default_value='mapping'),
        
        robot_state_publisher,
        spawn_entity,
        # filter_front,
        # filter_rear,  
        merger_node,
        mapping_node,
        localization_node
    ])