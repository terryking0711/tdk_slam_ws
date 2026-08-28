import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition

# ============================================================================
# TF 樹（實機）：
#   world --(static, 本檔 world_tf_pub)--> map
#   map   --(cartographer pure localization)--> odom
#   odom  --(ekf_filter_node, robot_localization)--> base_footprint
#   base_footprint --(robot_state_publisher, URDF)--> base_link --> laser_*
#
# world(0,0)   = 真實比賽場地最左下角
# map(0,0)     = 建圖原點（= 比賽出發點）
# world -> map = 平移 (0.425, 1.0)，無旋轉
# ============================================================================

WORLD_TO_MAP_X = 0.425
WORLD_TO_MAP_Y = 1.0


def generate_launch_description():
    localization_pkg = get_package_share_directory('tdk_slam_manager')
    sllidar_pkg = get_package_share_directory('rplidar_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    localization_mode = LaunchConfiguration('localization_mode', default='mapping')

    xacro_file = os.path.join(localization_pkg, 'urdf', 'sensors.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # ------------------------------------------------------------------
    # world -> map 靜態轉換（架構圖中的 world_to_map_publisher）
    # ------------------------------------------------------------------
    world_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_publisher',
        output='screen',
        arguments=[
            '--x', str(WORLD_TO_MAP_X),
            '--y', str(WORLD_TO_MAP_Y),
            '--z', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'map'
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # ------------------------------------------------------------------
    # EKF：取代 odom_tf_broadcaster，發布 odom -> base_footprint TF
    # 與 /odometry/filtered（目前只吃 /odom，之後加 IMU 只改 ekf_config.yaml）
    # ------------------------------------------------------------------
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[
    #         PathJoinSubstitution([FindPackageShare('tdk_slam_manager'), 'config', 'ekf_config.yaml']),
    #         {'use_sim_time': use_sim_time}
    #     ]
    # )

    odom_tf_broadcaster_node = Node(
        package='tdk_slam_manager',
        executable='odom_tf_broadcaster_node',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'use_sim_time': use_sim_time
        }]
    ) 

# start RPLiDAR S3 (Front)
    lidar_front = GroupAction([
        PushRosNamespace('front'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sllidar_pkg, 'launch', 'rplidar_s3_launch.py')),
            launch_arguments={
                'serial_port': '/dev/rplidar_front',
                'frame_id': 'laser_front',
                'inverted': 'false'
            }.items()
        )
    ])

    # start RPLiDAR S3 (Rear)
    lidar_rear = GroupAction([
        PushRosNamespace('rear'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sllidar_pkg, 'launch', 'rplidar_s3_launch.py')),
            launch_arguments={
                'serial_port': '/dev/rplidar_rear',
                'frame_id': 'laser_rear',
                'inverted': 'false'
            }.items()
        )
    ])
    
    filter_front = Node(
        package='tdk_slam_manager',
        executable='laser_angle_filter_node',
        name='filter_front',
        parameters=[{
            'lower_angle': 0.0,
            'upper_angle': 1.5707,
            'input_topic': '/front/scan',
            'output_topic': '/front/scan_filtered'
        }]
    )

    filter_rear = Node(
        package='tdk_slam_manager',
        executable='laser_angle_filter_node',
        name='filter_rear',
        parameters=[{
            'lower_angle': 0.0,
            'upper_angle': 1.5707,
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
    # remap odom -> /odometry/filtered：cartographer 的 odom topic 輸入
    # 與 EKF 發的 odom->base_footprint TF 必須是同一份資料，否則會互相打架。
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
        ],
        remappings=[
            ('odom', '/odom')
        ]
    )
    # Convert Submap to OccupancyGrid — remapped to /carto_map so it doesn't
    # conflict with nav2_map_server which publishes the authoritative /map
    occupancy_grid_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' in ['carto_mapping', 'cartographer']"])),
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05'],
        remappings=[('/map', '/carto_map')]
        # /carto_map here is only for visualization in foxglove and rviz2, the real map it is using is /map from nav_launch.py, which is published by map_server
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
            '-load_state_filename', os.path.join(localization_pkg, 'maps', 'real_map_0.pbstream')
        ],
        remappings=[
            ('odom', '/odom')
        ]
    )

    map_yaml_file = os.path.join(localization_pkg, 'maps', 'real_map_0.yaml')

    # map_server
    map_server_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time}
        ]
    )
    # Lifecycle Manager: activate map server
    lifecycle_manager_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    # AMCL
    amcl_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(localization_pkg, 'config', 'amcl_params.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    # ------------------------------------------------------------------
    # robot_pose_publisher：world -> base_footprint 全局座標 /robot_pose
    # 只在 localization 模式需要（給 localization_manager 驗證用）
    # ------------------------------------------------------------------
    robot_pose_publisher_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' in ['slam_toolbox', 'cartographer']"])),
        package='tdk_slam_manager',
        executable='robot_pose_publisher_node',
        name='robot_pose_pub',
        output='screen',
        parameters=[{
            'parent_frame': 'world',
            'child_frame': 'base_footprint',
            'use_sim_time': use_sim_time
        }]
    )

    # ------------------------------------------------------------------
    # localization_manager：接收 FSM /init_pose_cmd，重啟 cartographer
    # trajectory 並驗證，回報 /init_pose_status
    # ------------------------------------------------------------------
    localization_manager_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' in ['slam_toolbox', 'cartographer']"])),
        package='tdk_slam_manager',
        executable='localization_manager_node',
        name='localization_manager',
        output='screen',
        parameters=[{
            'slam_type': localization_mode,
            'world_to_map_x': WORLD_TO_MAP_X,
            'world_to_map_y': WORLD_TO_MAP_Y,
            'carto_config_dir': os.path.join(localization_pkg, 'cartographer_config'),
            'carto_config_basename': 'localization.lua',
            'tolerance_dist': 0.15,
            'tolerance_yaw': 0.15,
            'verify_timeout_sec': 5.0,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization_mode', default_value='mapping'),

        world_tf_pub,
        robot_state_publisher,
        odom_tf_broadcaster_node,
        # ekf_node,
        lidar_front,
        lidar_rear,
        filter_front,
        filter_rear,
        merger_node,
        mapping_node,
        localization_node,

        cartographer_mapping_node,
        occupancy_grid_node,
        cartographer_node,
        map_server_node,
        lifecycle_manager_node,
        amcl_node,

        robot_pose_publisher_node,
        localization_manager_node
    ])
    