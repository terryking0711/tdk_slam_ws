import os

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


FIELD_CONFIG_FILE = '/workspaces/field_config.yaml'


def resolve_map(context):
    # map:=<yaml> 有給就直接用；沒給就依 field_side 從 field_config.yaml 選地圖
    if LaunchConfiguration('map').perform(context):
        return []
    with open(FIELD_CONFIG_FILE) as f:
        field_config = yaml.safe_load(f)
    field_side = LaunchConfiguration('field_side').perform(context) or field_config['default_side']
    if field_side not in field_config['sides']:
        raise RuntimeError(
            f"field_side '{field_side}' not in {FIELD_CONFIG_FILE} sides {list(field_config['sides'])}")
    map_yaml = os.path.join(
        field_config['maps_dir'], field_config['sides'][field_side]['map_name'] + '.yaml')
    return [
        LogInfo(msg=f'[nav_launch] field_side = {field_side}, map = {map_yaml}'),
        SetLaunchConfiguration('map', map_yaml),
    ]


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tdk_nav2_dir = get_package_share_directory('tdk_nav2_manager')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')

    params_file = os.path.join(
        tdk_nav2_dir,
        'config',
        'tdk_nav2_params.yaml'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        }]
    )

    # Separate lifecycle manager for map_server so it activates independently
    # from the main nav2 lifecycle manager
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # Nav2 Humble remaps controller_server to cmd_vel_nav, but leaves the
    # behavior_server recovery plugins publishing directly to cmd_vel. Route
    # every raw Nav2 velocity command through velocity_smoother so cmd_vel has
    # one effective producer: velocity_smoother (cmd_vel_smoothed -> cmd_vel).
    nav2_group = GroupAction(actions=[
        SetRemap(src='cmd_vel', dst='cmd_vel_nav'),
        nav2_launch,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'field_side',
            default_value='',
            description='normal / mirror（空白 = field_config.yaml 的 default_side）'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml（空白 = 依 field_side 從 field_config.yaml 選）'
        ),
        OpaqueFunction(function=resolve_map),
        map_server_node,
        lifecycle_manager_map,
        nav2_group,
    ])
