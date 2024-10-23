import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from project_main.sim_utils import (
    base_station_placement,
    balloons_placement,
    compute_area_coverage,
    sensors_placement,
    spawn_sdf,
    create_node,
)

SENSOR_MARGIN_RANGE = 5


def _setup_robots(context: LaunchContext, *args, **kwargs):
    pkg_project_description = get_package_share_directory("project_description")
    base_station_sdf_file = os.path.join(
        pkg_project_description, "models", "base_station", "model.sdf"
    )
    balloon_sdf_file = os.path.join(
        pkg_project_description, "models", "balloon", "model.sdf"
    )
    active_sensor_sdf_file = os.path.join(
        pkg_project_description, "models", "active_sensor", "model.sdf"
    )

    field_width = int(LaunchConfiguration("field_width").perform(context))
    field_height = int(LaunchConfiguration("field_height").perform(context))

    bs_outside = bool(LaunchConfiguration("bs_outside").perform(context))

    balloons = int(LaunchConfiguration("balloons").perform(context))
    cache_type = LaunchConfiguration("cache_type").perform(context)
    cache_size = int(LaunchConfiguration("cache_size").perform(context))
    cache_expiration = int(LaunchConfiguration("cache_expiration").perform(context))

    sensors = int(LaunchConfiguration("sensors").perform(context))
    sensors_range = int(LaunchConfiguration("sensors_range").perform(context))

    # Compute the initial placement of the base station.
    base_station_coords = base_station_placement(
        field_width, field_height, outside=bs_outside
    )

    # Compute the initial placement of the balloons.
    balloons_coords = balloons_placement(
        balloons, field_width, field_height, sensors_range - SENSOR_MARGIN_RANGE
    )

    # Compute the area coverage of the field.
    grid_coverage, grid_x_coords, grid_y_coords = compute_area_coverage(
        field_width, field_height, balloons_coords, sensors_range
    )

    # Compute the initial placement of the sensors.
    sensors_coords = sensors_placement(
        sensors, grid_coverage, grid_x_coords, grid_y_coords
    )

    # Define the list of nodes to be spawned.
    nodes = []

    # Spawn the base station and its bridge.
    # nodes.append(
    #     create_node(
    #         base_station_sdf_file,
    #         "BaseStation",
    #         "iot_project_world",
    #         *base_station_coords,
    #     )
    # )

    nodes.append(spawn_sdf(base_station_sdf_file, None, base_station_coords, "iot_project_world"))
    nodes.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/BaseStation/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
            ],
        )
    )

    # # # TODO: Add base sation node controller.
    # nodes.append(
    #     Node(
    #         package="project_main",
    #         executable="base_station_controller",
    #         namespace="BaseStation",
    #         name="BaseStationController",
    #         parameters=[
    #             {"balloons": balloons},
    #             {"sensors": sensors},
    #         ],
    #     )
    # )
    
    # Spawn the balloons and their related topics (cmd_vel, odometry)
    for i, coord in enumerate(balloons_coords):
        nodes.append(spawn_sdf(balloon_sdf_file, i, coord, "iot_project_world"))
        nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    f"/Balloon_{i}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
                ],
            )
        )
        nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    f"/Balloon_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
                ],
            )
        )

        # TODO: Review KeyboardInterrupt error when uncommenting this block.
        nodes.append(
            Node(
                package="project_main",
                executable="balloon_controller",
                namespace=f"Balloon_{i}",
                name=f"BalloonController{i}",
                parameters=[
                    {"id": i},
                    {"cache_type": cache_type},
                    {"cache_size": cache_size},
                    {"cache_expiration": cache_expiration},
                ],
            )
        )

    # Spawn the sensors and their related topics (cmd_vel, odometry)
    for i, coord in enumerate(sensors_coords):
        nodes.append(spawn_sdf(active_sensor_sdf_file, i, coord, "iot_project_world"))
        nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    f"/ActiveSensor_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
                ],
            )
        )
        nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    f"/ActiveSensor_{i}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
                ],
            )
        )
        nodes.append(
            Node(
                package="project_main",
                executable="sensor_controller",
                namespace=f"ActiveSensor_{i}",
                parameters=[
                    {"id": i},
                ],
            )
        )

    return nodes


def generate_launch_description():
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
        "cache_expiration", default_value="10", description="Balloon cache size"
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

    # Add the opaque function for setting up the robots
    ld.add_action(OpaqueFunction(function=_setup_robots))

    return ld
