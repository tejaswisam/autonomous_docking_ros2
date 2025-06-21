from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    docking_sim_dir = get_package_share_directory('turtlebot3_docking_sim')
    world_path = os.path.join(docking_sim_dir, 'worlds', 'aruco_docking_world.world')
    model_path = os.path.join(docking_sim_dir, 'models')

    return LaunchDescription([
        # Set model path so Gazebo finds ArUco box
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),

        # Set TB3 model to waffle
        DeclareLaunchArgument(
            'model',
            default_value='waffle',
            description='Turtlebot3 model type [burger, waffle, waffle_pi]'
        ),

        # Launch TurtleBot3 Gazebo simulation with custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        )
    ])
