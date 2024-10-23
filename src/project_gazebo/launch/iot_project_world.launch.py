import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_gazebo = get_package_share_directory("project_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution(
                    [pkg_project_gazebo, "worlds", "iot_project_world.sdf"],
                )
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gz_bridge_ctrl_world = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/iot_project_world/control@ros_gz_interfaces/srv/ControlWorld"
        ],
    )

    gz_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/iot_project_world/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock"
        ],
    )

    return LaunchDescription(
        [
            gz_sim,
            gz_bridge_ctrl_world,
            gz_bridge_clock,
        ]
    )
