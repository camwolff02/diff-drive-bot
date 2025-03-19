from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gobilda_robot'),
                    'launch',
                    'gobilda.launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros'),
                    'launch',
                    'view_rplidar_a1_launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('depthai_ros_driver'),
                    'launch',
                    'camera.launch.py'
                ])
            ]),
        ),
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            parameters=[
                {'image_topic': '/oak/rgb/image_raw'},
                {'camera_info_topic': '/oak/rgb/camera_info'},
            ],
            output='screen'
        )
    ])