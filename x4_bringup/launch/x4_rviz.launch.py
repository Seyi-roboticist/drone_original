from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("x4_description"), "urdf", "x4.urdf.xacro"]),
    ])

    robot_description = {
    	"robot_description": ParameterValue(
        	robot_description_content, value_type=str
        )
    }


    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
        )
    
    rviz_dir = os.path.join(get_package_share_directory('x4_description'), 'rviz', 'x4.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_dir]
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz_node
    ])
