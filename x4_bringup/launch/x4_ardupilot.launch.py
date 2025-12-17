from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_x4_gazebo = get_package_share_directory("x4_gazebo")
    #pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    #pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

    declare_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value= 'true',
        description='Launch MAVROS and MAVProxy for simulation'
    )

    declare_real_arg = DeclareLaunchArgument(
        'use_real',
        default_value='false',
        description='Use real hardware over serial'
    )

    use_sim= LaunchConfiguration('use_sim')
    use_real = LaunchConfiguration('use_real')

    # ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
    micro_ros_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '-p', '2019'],
    )

    # sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --enable-dds --map -DG
    launch_sitl = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '-v', 'ArduCopter',
            '-f', 'gazebo-iris',
            '--model', 'JSON',
            '--console',
            '--enable-dds',
            '--map',
            '-DG'
        ],
        output='screen'
    )

    # mavproxy.py --master udp:127.0.0.1:14550  --console --map
    launch_mavproxy = ExecuteProcess(
        cmd=[
            'xterm', '-e',
            'mavproxy.py --master udp:127.0.0.1:14551',
        ],
        condition=IfCondition(use_sim),
    )

    #may be better in the future but limited functionality right now
    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl_dds_udp.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "transport": "udp4",
            "port": "2019",
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "defaults": os.path.join(
                pkg_x4_gazebo,
                "config",
                "gazebo-x4.parm",
            )
            + ","
                + os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "dds_udp.parm",
            ),
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )

    sitl_mavproxy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl_mavproxy.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "console": "True",
            "defaults": os.path.join(
                pkg_x4_gazebo,
                "config",
                "gazebo-x4.parm",
            ),
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
        condition=IfCondition(use_sim),
    )

    sitl_mavproxy_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl_mavproxy.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "console": "True",
            "defaults": os.path.join(
                pkg_x4_gazebo,
                "config",
                "gazebo-x4.parm",
            ),
            "sim_address": "127.0.0.1",
            "master": "/dev/ttyACM0,115200",
            "sitl": "127.0.0.1:5501",
        }.items(),
        condition=IfCondition(use_real),
    )

    #ros2 launch mavros apm.launch fcu_url:=tcp://localhost fcu_url:=udp://127.0.0.1:14550:@14555
    launch_mavros_sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
                    [
                        FindPackageShare("mavros"),
                        "launch",
                        "apm.launch",
                    ]
                ),),
        launch_arguments={
            'fcu_url': 'udp://127.0.0.1:14550@14555',
            #'gcs_url': 'udp://@localhost:14550',
            #'system_id': '255',
            #'component_id': '240',
            'target_system_id': '1',
            'target_component_id': '1'
            }.items(),
        condition=IfCondition(use_sim),
    )

    #ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200
    launch_mavros_real = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("mavros"), "launch", "apm.launch"
        ])),
        launch_arguments={
            #'fcu_url': 'serial:///dev/ttyACM0:115200',
            'fcu_url': 'udp://127.0.0.1:14550@14555',
            'system_id': '255',
            'component_id': '240',
            'target_system_id': '1',
            'target_component_id': '1'
        }.items(),
        condition=IfCondition(use_real),
    )

    delayed_mavros_sim = TimerAction(
        period=2.0,
        actions=[launch_mavros_sim]
    )

    delayed_mavros_serial = TimerAction(
        period=2.0,
        actions=[launch_mavros_real]
    )

    launch_gui = Node(
        package='x4_interfaces',
        executable='gui.py',
        name='x4_gui',
        output='screen',
        condition=IfCondition(use_sim),
    )
        
    return LaunchDescription([
        declare_real_arg,
        declare_sim_arg,
        launch_mavproxy,
        sitl_mavproxy,
        sitl_mavproxy_real,
        delayed_mavros_sim,
        delayed_mavros_serial,
        #launch_gui, diasbles takeoff for some reason
    ])