from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup_nodes(context: LaunchContext):
    field_width = int(LaunchConfiguration("field_width").perform(context))
    field_height = int(LaunchConfiguration("field_height").perform(context))
    balloons = int(LaunchConfiguration("balloons").perform(context))
    sensors = int(LaunchConfiguration("sensors").perform(context))
    sensors_range = int(LaunchConfiguration("sensors_range").perform(context))
    cache_type = LaunchConfiguration("cache_type").perform(context)
    cache_size = int(LaunchConfiguration("cache_size").perform(context))
    cache_expiration = int(LaunchConfiguration("cache_expiration").perform(context))
    query_rate = int(LaunchConfiguration("query_rate").perform(context))

    return [
        Node(
            package="project_main",
            executable="simulation_manager",
            parameters=[
                {"balloons": balloons},
                {"sensors": sensors},
                {"sensors_range": sensors_range},
            ],
        ),
        Node(
            package="project_main",
            executable="movement_coordinator",
            parameters=[
                {"balloons": balloons},
                {"sensors": sensors},
                {"sensors_range": sensors_range},
                {"field_width": field_width},
                {"field_height": field_height},
            ],
        ),
        Node(
            package="project_main",
            executable="base_station_controller",
            parameters=[
                {"balloons": balloons},
                {"sensors": sensors},
                {"cache_type": cache_type},
                {"cache_size": cache_size},
                {"cache_expiration": cache_expiration},
                {"query_rate": query_rate},
            ],
        ),
    ]


def generate_launch_description():
    # Declare the launch arguments
    balloons_launch_arg = DeclareLaunchArgument(
        "balloons", default_value="3", description="Number of balloons"
    )
    sensors_launch_arg = DeclareLaunchArgument(
        "sensors", default_value="3", description="Number of sensors"
    )
    sensors_range_launch_arg = DeclareLaunchArgument(
        "sensors_range", default_value="50", description="Range of sensors"
    )

    ld = LaunchDescription()

    # Add the launch arguments to the launch description
    ld.add_action(balloons_launch_arg)
    ld.add_action(sensors_launch_arg)
    ld.add_action(sensors_range_launch_arg)

    # Add the opaque function to the launch description
    ld.add_action(OpaqueFunction(function=_setup_nodes))

    return ld
