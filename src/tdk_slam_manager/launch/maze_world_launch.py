import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('tdk_slam_manager')

    models_path = os.path.join(pkg_path, 'sim', 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[models_path]
    )

    world_path = os.path.join(pkg_path, 'sim', 'worlds', 'maze.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    return LaunchDescription([
        set_gazebo_model_path, 
        gazebo                 
    ])