from time import sleep
from threading import Thread
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from project_interfaces.action import Patrol
from project_main.sim_utils import compute_area_coverage
from project_main.sim_utils import sensors_placement
from geometry_msgs.msg import Pose, Point, Quaternion


class MovementCoordinator(Node):
    """
    Movement Coordinator class, used the manage the whole fleet of Sensors.
    """

    def __init__(self):
        super().__init__("movement_coordinator")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("balloons", 3),
                ("sensors", 3),
                ("sensors_range", 50),
                ("field_width", 100),
                ("field_height", 100),
            ],
        )
        self.balloons = (
            self.get_parameter("balloons").get_parameter_value().integer_value
        )
        self.sensors = self.get_parameter("sensors").get_parameter_value().integer_value
        self.sensors_range = (
            self.get_parameter("sensors_range").get_parameter_value().integer_value
        )
        self.field_width = (
            self.get_parameter("field_width").get_parameter_value().integer_value
        )
        self.field_height = (
            self.get_parameter("field_height").get_parameter_value().integer_value
        )

        self.sensor_action_clients = {}
        self.sensor_positions = {}
        self.sensor_states = {}
        self.balloon_positions = {}

        for i in range(self.sensors):
            self.sensor_action_clients[i] = ActionClient(
                self, Patrol, f"/ActiveSensor_{i}/patrol"
            )
            self.sensor_states[i] = SensorState.IDLE

        for i in range(self.sensors):
            self.create_subscription(
                Odometry,
                f"ActiveSensor_{i}/odometry",
                lambda msg, id=i: self.store_sensor_position(id, msg),
                10,
            )

        self.odometry_subs = []

        for i in range(self.balloons):
            self.odometry_subs.append(
                self.create_subscription(
                    Odometry,
                    f"Balloon_{i}/odometry",
                    lambda odometry_msg, balloon_id=i: (
                        self.store_balloon_position(balloon_id, odometry_msg)
                    ),
                    10,
                )
            )

    def store_balloon_position(self, balloon_id, position: Odometry):
        self.balloon_positions[balloon_id] = position.pose.pose.position
        self.get_logger().info(
            f"Balloon_{balloon_id} position: {self.balloon_positions[balloon_id]}"
        )
        if len(self.balloon_positions) >= self.balloons:
            for sub in self.odometry_subs:
                self.destroy_subscription(sub)

            self.coverage_grid, self.grid_x, self.grid_y = compute_area_coverage(
                self.field_width,
                self.field_height,
                [(pos.x, pos.y, pos.z) for pos in self.balloon_positions.values()],
                self.sensors_range,
            )
            self.get_logger().info("Coverage grid computed")

    def patrol_targets(self):
        """
        Method used to keep the fleet of Sensors constantly moving in a random mode.
        """

        def patrol_targets_inner():
            while True:
                for i in range(self.sensors):
                    # Do not resubmit tasks to already moving sensors
                    if not self.sensor_states[i] is SensorState.MOVING:
                        self.submit_task(i)

        # Start this function in another thread, so that the node can start spinning immediately after
        # this function has finished
        Thread(target=patrol_targets_inner).start()

    def submit_task(self, uav_id: int):
        # Wait for the action server to go online
        while (
            not self.sensor_action_clients[uav_id].wait_for_server(1)
            or (len(self.sensor_positions) < self.sensors and len(self.balloon_positions) < self.balloons)
        ):
            self.get_logger().info(
                "Waiting for action server to come online and sensors + balloons to announce their position"
            )
            sleep(3)

        # Set the Sensors to moving state
        self.sensor_states[uav_id] = SensorState.MOVING
        goal = Patrol.Goal()
        goal.targets = []

        target_coords = sensors_placement(
            1, self.coverage_grid, self.grid_x, self.grid_y
        )[0]
        actual_target = Point()
        actual_target.x = float(target_coords[0])
        actual_target.y = float(target_coords[1])
        actual_target.z = float(target_coords[2])

        pose_msg = Pose()
        # Assign the point values to the position field
        pose_msg.position = actual_target

        pose_msg.orientation = Quaternion()
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0  # Identity quaternion

        goal.targets.append(actual_target)

        self.get_logger().info(
            f"Submitting task for Sensor {uav_id}, target {goal.targets[0]}"
        )

        # Submit the task here and add a callback for when the submission is accepted
        patrol_future = self.sensor_action_clients[uav_id].send_goal_async(goal)
        patrol_future.add_done_callback(
            lambda future, uav_id=uav_id: self.patrol_submitted_callback(uav_id, future)
        )

    def patrol_submitted_callback(self, uav_id, future):

        # Check if the patrol action was accepted
        goal_handle = future.result()

        if not goal_handle.accepted:
            # If not, set the sensor back to sensing, and return
            self.get_logger().info("Task has been refused by the action server")
            self.sensor_states[uav_id] = SensorState.SENSING
            return

        result_future = goal_handle.get_result_async()

        # Add a callback for when the action is completed
        result_future.add_done_callback(
            lambda future, uav_id=uav_id: self.patrol_completed_callback(uav_id, future)
        )

    def patrol_completed_callback(self, uav_id, future):

        self.get_logger().info(
            f"Patrolling action for Sensor {uav_id} has been completed. It is going idle"
        )
        self.sensor_states[uav_id] = SensorState.SENSING

    def store_sensor_position(self, id, msg: Odometry):
        self.sensor_positions[id] = msg.pose.pose.position


class SensorState(Enum):
    IDLE = 1
    SENSING = 2
    MOVING = 3


def main():

    rclpy.init()

    executor = MultiThreadedExecutor()
    movement_coordinator = MovementCoordinator()

    executor.add_node(movement_coordinator)
    movement_coordinator.patrol_targets()

    executor.spin()

    executor.shutdown()
    movement_coordinator.destroy_node()
    rclpy.shutdown()
