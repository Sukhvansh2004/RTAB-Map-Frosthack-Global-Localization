import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    gazebo_launch_dir = os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')
    state_publisher_dir = os.path.join(get_package_share_directory('tortoisebot_description'), 'launch')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    state_publisher_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(state_publisher_dir, 'state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )    

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'
        ),
        state_publisher_nodes,
        gazebo_launch_cmd,
    ])