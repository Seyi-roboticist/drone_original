from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Camera calibration file (YAML)
    calibration_file = PathJoinSubstitution([
        FindPackageShare("x4_vision"),
        "config",
        "camera_calibration.yaml"
    ])

    # RViz configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("x4_vision"),
        "rviz",
        "x4_vision.rviz"
    ])

    # Declare launch arguments
    calibration_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=calibration_file,
        description='Path to camera calibration file'
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Path to video device (e.g., /dev/video0, /dev/video2)'
    )

    # v4l2_camera driver node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='rx0_camera',
        parameters=[
            {'device': LaunchConfiguration('camera_device')},
            {'image_size': [640, 480]},
            {'camera_frame_id': 'rx0_frame'},
            {'camera_info_url': ['file://', LaunchConfiguration('calibration_file')]}
        ],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ]
    )

    # image_transport republisher (optional compression pipeline)
    republish_node = Node(
        package='image_transport',
        executable='republish',
        arguments=['raw', 'compressed'],
        remappings=[('in', '/camera/image_raw'), ('out', '/camera/image_raw/compressed')]
    )

    # ArUco detection node
    aruco_node = Node(
        package='x4_vision',
        executable='aruco_detector_node',
        name='aruco_detector',
        output='screen',
        parameters=[{'marker_size': 0.15}]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        calibration_arg,
        camera_device_arg,
        camera_node,
        republish_node,
        aruco_node,
        rviz_node
    ])

