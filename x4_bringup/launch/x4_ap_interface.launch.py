from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    agent_profile = PathJoinSubstitution([get_package_share_directory('x4_bringup'), 'config', "dds_xrce_profile.xml"])
    return LaunchDescription([
        # Launch ArduPilot with sim_vehicle.py
        ExecuteProcess(
            cmd=[
                "./Tools/autotest/sim_vehicle.py",
                "-w",
                "-v", "ArduCopter",
                "--console",
                # "--enable-dds",
                "--map",
                "-DG"
            ],
            cwd="./src/ardupilot",
            shell=True,
            output="screen"
        ),

        # Launch micro-ROS agent
        # ExecuteProcess(
        #     cmd=[
        #         "ros2", "run", "micro_ros_agent", "micro_ros_agent",
        #         "udp4", "-p", "2019", "-r", agent_profile
        #     ],
        #     output="screen"
        # ),
    ])
