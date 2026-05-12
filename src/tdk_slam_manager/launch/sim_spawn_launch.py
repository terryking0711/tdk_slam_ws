import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    localization_pkg = os.path.join('/home/ted/tdk_slam_ws/src/tdk_slam_manager')
    
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
        arguments=['-topic', 'robot_description',
                    '-entity', 'tdk_robot',
                    '-x', '0.425',
                    '-y', '1.0',  
                    ],
        output='screen'
    )

    filter_front = Node(
        package='tdk_slam_manager',
        executable='laser_angle_filter_node',
        name='filter_front',
        parameters=[{
            'lower_angle': -3.1415,
            'upper_angle': -1.5708,
            'input_topic': '/front/scan',
            'output_topic': '/front/scan_filtered'
        }]
    )

    filter_rear = Node(
        package='tdk_slam_manager',
        executable='laser_angle_filter_node',
        name='filter_rear',
        parameters=[{
            'lower_angle': -3.1415,
            'upper_angle': -1.5708,
            'input_topic': '/rear/scan',
            'output_topic': '/rear/scan_filtered'
        }]
    )

    merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laser_merger',
        parameters=[PathJoinSubstitution([FindPackageShare('tdk_slam_manager'), 'config', 'laser_merger_params.yaml']),
            {'use_sim_time': use_sim_time}],
        output='screen'
    )

    mapping_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'mapping'"])),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('tdk_slam_manager'), 'config', 'mapper_params_online_async.yaml']),
            {'use_sim_time': use_sim_time}]
    )

    localization_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'slam_toolbox'"])),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('tdk_slam_manager'), 'config', 'slam_toolbox_params.yaml']),
            {
                'mode': PythonExpression(["'mapping' if '", localization_mode, "' == 'mapping' else 'localization'"]),
                'use_sim_time': use_sim_time
            }
        ]
    )

    # Cartographer mapping
    cartographer_mapping_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'carto_mapping'"])),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(localization_pkg, 'cartographer_config'),
            '-configuration_basename', 'cartographer_2d.lua'
        ]
        # ,remappings=[
        #     ('/scan', '/scan'),
        #     ('/odom', '/odom')
        # ]
    )
    # Convert Submap to OccupancyGrid
    occupancy_grid_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' in ['carto_mapping', 'cartographer']"])),
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05']
    )
    # Cartographer localization
    cartographer_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'cartographer'"])),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(localization_pkg, 'cartographer_config'),
            '-configuration_basename', 'localization.lua',
            '-load_state_filename', os.path.join(localization_pkg, 'maps', 'carto_map_0.pbstream')
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('localization_mode', default_value='cartographer'),

        robot_state_publisher,
        spawn_entity,
        filter_front,
        filter_rear,  
        merger_node,
        mapping_node,
        localization_node,

        cartographer_mapping_node,
        occupancy_grid_node,
        cartographer_node
    ])