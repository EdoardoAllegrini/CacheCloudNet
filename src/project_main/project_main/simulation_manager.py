import math_utils
import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from project_interfaces.action import Query
from project_interfaces.msg import MultiStringArray
import random

class SimulationManager(Node):
    def __init__(self):
        super().__init__("simulation_manager")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("balloons", 3),
                ("sensors", 3),
                ("sensors_range", 50),
            ],
        )
        self.balloons = (
            self.get_parameter("balloons").get_parameter_value().integer_value
        )
        self.sensors = self.get_parameter("sensors").get_parameter_value().integer_value
        self.sensors_range = (
            self.get_parameter("sensors_range").get_parameter_value().integer_value
        )
        self.sensor_positions = {}
        self.balloon_positions = {}

        for i in range(self.sensors):
            self.create_subscription(
                Odometry,
                f"ActiveSensor_{i}/odometry",
                lambda odometry_msg, sensor_id=i: (
                    self.store_sensor_position(sensor_id, odometry_msg)
                ),
                10,
            )

            self.create_subscription(
                String,
                f"ActiveSensor_{i}/tx_data",
                lambda string_msg, sensor_id=i: (
                    self.forward_data(sensor_id, string_msg)
                ),
                10,
            )

        self.balloons_rx = {}
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

            self.balloons_rx[i] = self.create_publisher(
                String, f"Balloon_{i}/rx_data", 10
            )


    def store_sensor_position(self, sensor_id, position: Odometry):
        self.sensor_positions[sensor_id] = position.pose.pose.position

    def store_balloon_position(self, balloon_id, position: Odometry):
        self.balloon_positions[balloon_id] = position.pose.pose.position
        self.get_logger().info(
            f"Balloon_{balloon_id} position: {self.balloon_positions[balloon_id]}"
        )
        if len(self.balloon_positions) >= self.balloons:
            for sub in self.odometry_subs:
                self.destroy_subscription(sub)

    def forward_data(self, sensor_id, msg: String):
        for i in range(self.balloons):
            if sensor_id in self.sensor_positions and i in self.balloon_positions:
                pt_distance = math_utils.point_distance(
                    self.sensor_positions[sensor_id], self.balloon_positions[i]
                )
                if pt_distance < self.sensors_range:
                    self.balloons_rx[i].publish(msg)


def main():
    rclpy.init()
    simulationManager = SimulationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(simulationManager)
    executor.spin()
    executor.shutdown()
    simulationManager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
