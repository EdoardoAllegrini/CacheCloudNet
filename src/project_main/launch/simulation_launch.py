# from random import randint
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution
# from project_main.deployment_planner import DeploymentPlanner
# from project_main.sim_utils import spawn_sdf
# import pickle
# import base64
# import numpy as np

# WORLD_NAME = "iot_project_world"

# NUMBER_OF_BALLOONS = 4
# NUMBER_OF_SENSORS = 1
# SENSORS_RANGE = 50

# MARGIN = 5
# d_planner = DeploymentPlanner(NUMBER_OF_BALLOONS, NUMBER_OF_SENSORS, SENSORS_RANGE - MARGIN)
# map_width, map_height = 100, 100
# base_station_position, balloons_position, sensors_position, coverage, X, Y = d_planner.compute(map_width, map_height)
# #-----------------------------------------------------------------------------------------------
# # Launch file for the IoT Project. Launches all the nodes required to start the final solution
# # -----------------------------------------------------------------------------------------------
# def generate_launch_description():
#     # ------------------- Launch Gazebo here, with the iot_project_world world --------------------
#     targets_to_spawn = [
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 PathJoinSubstitution([
#                     FindPackageShare('ros_gz_sim'),
#                     'launch/'
#                     'gz_sim.launch.py',
#                 ])
#             ]),
#             launch_arguments={
#                 'gz_args': f"resources/{WORLD_NAME}.sdf"
#             }.items()
#         )
#     ]

#     # -------------------------------- Bridge for the world control -------------------------------
#     targets_to_spawn.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             arguments=[
#                 f"/world/{WORLD_NAME}/control@ros_gz_interfaces/srv/ControlWorld"
#             ]
#         )
#     )

#     # ------------------------------ Bridge for the simulation clock ------------------------------
#     targets_to_spawn.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             arguments=[
#                 f"/world/{WORLD_NAME}/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock"
#             ]
#         )
#     )

#     # -------------------------- Spawn balloons and bridge their topics ---------------------------
#     for i in range(NUMBER_OF_BALLOONS):
#         targets_to_spawn.append(spawn_sdf("resources/balloon/balloon.sdf", id = i, pos = balloons_position[i]))

#         # Spawn bridge for cmd_vel and odometry for each of the spawned object
#         targets_to_spawn.append(
#             Node(
#                 package="ros_gz_bridge",
#                 executable="parameter_bridge",
#                 arguments=[
#                     f"/Balloon_{i}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
#                 ]
#             )
#         )

#         targets_to_spawn.append(
#             Node(
#                 package="ros_gz_bridge",
#                 executable="parameter_bridge",
#                 arguments=[
#                     f"/Balloon_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
#                 ]
#             )
#         )

#         targets_to_spawn.append(
#             Node(
#                 package="project_main",
#                 executable="balloon_controller",
#                 namespace=f"Balloon_{i}",
#                 name=f"BalloonController{i}",
#                 parameters=[
#                     {'id': i}
#                 ]
#             )
#         )

#     #-------------------------- Spawn Active Sensors and bridge their topics ---------------------------
#     for i in range(NUMBER_OF_SENSORS):
#         targets_to_spawn.append(spawn_sdf("resources/active_sensor/active_sensor.sdf", id = i, pos = sensors_position[i]))

#         targets_to_spawn.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             arguments=[
#             f"/ActiveSensor_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
#             ]
#         )
#         )

#         targets_to_spawn.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             arguments=[
#             f"/ActiveSensor_{i}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
#             ]
#         )
#         )

#         targets_to_spawn.append(
#             Node(
#                 package="project_main",
#                 executable="sensor_controller",
#                 namespace=f"ActiveSensor_{i}",
#                 parameters=[
#                     {'id': i},
#                     {'coverage': base64.b64encode(pickle.dumps(coverage)).decode('utf-8')},
#                     {'X': base64.b64encode(pickle.dumps(X)).decode('utf-8')},
#                     {'Y': base64.b64encode(pickle.dumps(Y)).decode('utf-8')}
#                 ]
#             )
#         )

#     #------------------------------------ Spawn base station -------------------------------------
#     targets_to_spawn.append(spawn_sdf("resources/base_station/base_station.sdf", pos = base_station_position))
#     targets_to_spawn.append(
#         Node(
#             package="ros_gz_bridge",
#             executable="parameter_bridge",
#             arguments=[
#                 "/base_station/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
#             ]
#         )
#     )

#     targets_to_spawn.append(
#         Node(
#             package="project_main",
#             executable="simulation_manager",
#             arguments=[
#                 f"{NUMBER_OF_BALLOONS}",
#                 f"{NUMBER_OF_SENSORS}",
#                 f"{SENSORS_RANGE}"
#             ]
#         )
#     )

#     return LaunchDescription(targets_to_spawn)
