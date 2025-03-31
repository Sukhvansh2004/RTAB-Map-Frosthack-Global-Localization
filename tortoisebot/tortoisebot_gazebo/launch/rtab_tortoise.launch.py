# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) Add
#          <joint name="camera_rgb_optical_joint" type="fixed">
#            <parent>camera_rgb_frame</parent>
#            <child>camera_rgb_optical_frame</child>
#            <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
#            <axis>
#              <xyz>0 0 1</xyz>
#            </axis>
#          </joint> 
#     3) Rename <link name="camera_rgb_frame"> to <link name="camera_rgb_optical_frame">
#     4) Add <link name="camera_rgb_frame"/>
#     5) Change <sensor name="camera" type="camera"> to <sensor name="camera" type="depth">
#     6) Change image width/height from 1920x1080 to 640x480
# Example:
#   $ ros2 launch rtabmap_demos turtlebot3_sim_rgbd_demo.launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import os

def launch_setup(context, *args, **kwargs):
    
    use_sim_time=LaunchConfiguration('use_sim_time')
    # Directories
    pkg_tortoisebot_gazebo = get_package_share_directory(
        'tortoisebot_bringup')
    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')
    pkg_rtabmap = get_package_share_directory(
        'tortoisebot_gazebo')

    # world = LaunchConfiguration('world').perform(context)
    
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('tortoisebot_gazebo'), 'config', 'tortoise_nav2_param.yaml']
    )

    # Paths
    # gazebo_launch = PathJoinSubstitution(
        # [pkg_tortoisebot_gazebo, 'launch', 'spawn_bot_in_world.launch.py'])
    
    gazebo_launch = PathJoinSubstitution(
        [pkg_tortoisebot_gazebo, 'launch', 'bringup.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])
    rtabmap_launch = PathJoinSubstitution(
        [pkg_rtabmap, 'launch', 'tortoise_rgbd.launch.py'])

    # Includes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('use_sim_time', use_sim_time)
        ]
    )
    
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('params_file', nav2_params_file)
        ]
    )
    
    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch])
    )
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', use_sim_time)
        ]
    )
    
    return [
        # Nodes to launch
        nav2,
        rviz,
        rtabmap,
        gazebo
    ]

def generate_launch_description():
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='False',
            description='Launch in localization mode.'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Launch in simulation mode.'),

        OpaqueFunction(function=launch_setup)
    ])
