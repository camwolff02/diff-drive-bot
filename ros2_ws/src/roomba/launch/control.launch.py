from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("foxglove_bridge"),
                                "launch",
                                "foxglove_bridge_launch.xml",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "port": "6006",
                }.items(),
            ),
            Node(
                package="roomba",
                executable="run_roomba",
                remappings=[("odom", "/gobilda_base_controller/odom")],
                parameters=[
                    {"forward_speed": 0.2},
                    {"turn_speed": 0.1},
                    {"backup_time": 1.0},
                    {"stop_dist": 0.3048},
                    {"obstacle_width": 0.0635},
                    {"turn_radians": 0.7},
                ],
            ),
        ]
    )
