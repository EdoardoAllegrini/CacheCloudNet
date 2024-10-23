from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_project_gazebo = FindPackageShare("project_gazebo")
    pkg_project_main = FindPackageShare("project_main")

    field_width = LaunchConfiguration("field_width", default="100")
    field_height = LaunchConfiguration("field_height", default="100")
    bs_outside = LaunchConfiguration("bs_outside", default="")
    balloons = LaunchConfiguration("balloons", default="3")
    sensors = LaunchConfiguration("sensors", default="3")
    sensors_range = LaunchConfiguration("sensors_range", default="50")
    cache_type = LaunchConfiguration("cache_type", default="FIFO")
    cache_size = LaunchConfiguration("cache_size", default="10")
    cache_expiration = LaunchConfiguration("cache_expiration", default="10")
    query_rate = LaunchConfiguration("query_rate", default="10")

    # Declare the launch arguments
    bs_outside_launch_arg = DeclareLaunchArgument(
        "bs_outside",
        default_value="",
        description="Place the base station outside the field",
    )

    balloons_launch_arg = DeclareLaunchArgument(
        "balloons", default_value="3", description="Number of balloons"
    )

    sensors_launch_arg = DeclareLaunchArgument(
        "sensors", default_value="3", description="Number of sensors"
    )

    sensors_range_launch_arg = DeclareLaunchArgument(
        "sensors_range", default_value="50", description="Range of sensors"
    )

    field_width_launch_arg = DeclareLaunchArgument(
        "field_width", default_value="100", description="Field width"
    )

    field_height_launch_arg = DeclareLaunchArgument(
        "field_height", default_value="100", description="Field height"
    )

    cache_type_launch_arg = DeclareLaunchArgument(
        "cache_type",
        default_value="FIFO",
        description="Balloon cache type (FIFO, LRU, LFU, MRU)",
    )

    cache_size_launch_arg = DeclareLaunchArgument(
        "cache_size", default_value="10", description="Balloon cache size"
    )

    cache_expiration_launch_arg = DeclareLaunchArgument(
        "cache_expiration", default_value="10", description="Balloon cache expiration seconds"
    )

    query_rate_launch_arg = DeclareLaunchArgument(
        "query_rate", default_value="10", description="Base Station query rate (seconds)"
    )

    world_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_project_gazebo, "launch", "iot_project_world.launch.py"]
            )
        ),
    )

    robots_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_main, "launch", "robots.launch.py"])
        ),
        launch_arguments={
            "balloons": balloons,
            "bs_outside": bs_outside,
            "field_width": field_width,
            "field_height": field_height,
            "sensors": sensors,
            "sensors_range": sensors_range,
            "cache_type": cache_type,
            "cache_size": cache_size,
            "cache_expiration": cache_expiration,
            "query_rate": query_rate,
        }.items(),
    )

    managers_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_project_main, "launch", "managers.launch.py"])
        ),
        launch_arguments={
            "balloons": balloons,
            "sensors": sensors,
            "sensors_range": sensors_range,
            "field_width": field_width,
            "field_height": field_height,
            "cache_type": cache_type,
            "cache_size": cache_size,
            "cache_expiration": cache_expiration,
            "query_rate": query_rate,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(bs_outside_launch_arg)
    ld.add_action(balloons_launch_arg)
    ld.add_action(sensors_launch_arg)
    ld.add_action(sensors_range_launch_arg)
    ld.add_action(field_width_launch_arg)
    ld.add_action(field_height_launch_arg)
    ld.add_action(cache_type_launch_arg)
    ld.add_action(cache_size_launch_arg)
    ld.add_action(cache_expiration_launch_arg)
    ld.add_action(query_rate_launch_arg)

    # Add the launch description actions
    ld.add_action(world_launch_include)
    ld.add_action(robots_launch_include)
    ld.add_action(managers_launch_include)

    return ld
