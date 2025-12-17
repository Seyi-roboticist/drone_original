from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # world launch parameter
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value= 'flight_sim.sdf',
        description='Path to Ignition Gazebo world file (empty.sdf, flight_sim.sdf)'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value= 'false',
        description='Launch RVIZ'
    )

    declare_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value= 'true',
        description='Launch Gazebo SIM with Ardupilot, SITL, MavProxy, and MAVROS'
    )

    declare_real_arg = DeclareLaunchArgument(
        'use_real',
        default_value= 'false',
        description='Launch Gazebo SIM with Ardupilot, SITL, MavProxy, and MAVROS'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    use_sim = LaunchConfiguration('use_sim')
    use_real = LaunchConfiguration('use_real')

    
    gz_launch_path = PathJoinSubstitution([get_package_share_directory('x4_bringup'), 'launch', "x4_gazebo.launch.py"])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(use_sim)
    )

    rviz_launch_path = PathJoinSubstitution([get_package_share_directory('x4_bringup'), 'launch', "x4_rviz.launch.py"])
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path),
        condition=IfCondition(use_rviz)
    )

    ardupilot_launch_path = PathJoinSubstitution([get_package_share_directory('x4_bringup'), 'launch', "x4_ardupilot.launch.py"])
    ardupilot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ardupilot_launch_path),
        launch_arguments={
            'use_sim': use_sim,
            'use_real': use_real,
        }.items(),
    )

    return LaunchDescription([
        declare_world_arg,
        declare_rviz_arg,
        declare_real_arg,
        declare_sim_arg,
        gazebo_launch,
        rviz_launch,
        ardupilot_launch
    ])
