from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from ament_index_python.packages import get_package_share_directory
#import os

def generate_launch_description():

    # world launch parameter
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value= 'flight_sim.sdf',
        description='Path to Ignition Gazebo world file (empty.sdf, flight_sim.sdf)'
    )
    
    gazebo_envvar= SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value= PathJoinSubstitution([FindPackageShare('x4_description')]),
    )
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("x4_description"), "sdf", "x4.sdf.xacro"]),
    ])
    
    gz_launch_path = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    gz_args = TextSubstitution(text='-r -v 4 ')  # Flags
    world_path = PathJoinSubstitution([FindPackageShare('x4_gazebo'), 'world', LaunchConfiguration('world')])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': [gz_args, world_path]}.items()
    )

    create_x4 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments = ["-string", robot_description_content, "-z", "0.314"]
    )

    ign_bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['camera_rx0ii/image@sensor_msgs/msg/Image[ignition.msgs.Image'],
        output='screen'
    )

    ign_bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/flight_sim/model/AureliaX4/link/drone_link/sensor/imu_1/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
        output='screen'
    )

    ign_bridge_gps = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['gps_1/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'],
        output='screen'
    )

    ign_bridge_magnetometer = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['magnetometer_1/data@sensor_msgs/msg/MagneticField[ignition.msgs.Magnetometer'],
        output='screen'
    )

    ign_bridge_altimeter = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['altimeter_1/data@ros_gz_interfaces/msg/Altimeter[ignition.msgs.Altimeter'],
        output='screen'
    )

    ign_bridge_rangefinder = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['rangefinder_1/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
        output='screen'
    )
    sensor_node_bridges = [
        ign_bridge_camera,
        ign_bridge_imu,
        ign_bridge_gps,
        ign_bridge_magnetometer,
        ign_bridge_altimeter,
        ign_bridge_rangefinder
    ]

    return LaunchDescription([
        gazebo_envvar,
        declare_world_arg,
        gazebo_launch,
        create_x4,
    ]
    + sensor_node_bridges
    )
