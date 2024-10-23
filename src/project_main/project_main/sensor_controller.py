import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sim_utils import EventScheduler
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry
from threading import Lock
from project_interfaces.action import Patrol
from project_main.sensor_data import SensorData

import math_utils
import math
import random
import json
import time

WORLD_NAME = ""
SENSING_RATE = (5, 20)

class SensorController(Node):
    def __init__(self):
        super().__init__("sensor_controller")
        self.tx_topic = self.create_publisher(String, "tx_data", 10)
        self.id = self.declare_parameter("id", -1)
        self.generated_data = 0

        self.position = Point(x=-1.0, y=0.0, z=0.0)
        self.yaw = 0

        self.stop_msg = Twist()
        self.stop_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        self.stop_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)

        self.event_scheduler = EventScheduler()

        self.clock_topic = self.create_subscription(
            Clock, f"/world/iot_project_world/clock", self.event_scheduler.routine, 10
        )

        self.create_subscription(Odometry, "odometry", self.store_position, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.patrol_action_server = ActionServer(
            self, Patrol, "patrol", self.execute_patrol_action
        )

        self.simulation_started = False

        # Create a timer to check for simulation start
        self.check_simulation_start_timer = self.create_timer(
            0.1, self.check_simulation_start
        )

        self.simple_publish_timer = None

        self.patrol_thread = None

    def check_simulation_start(self):
        if self.simulation_started:
            if self.simple_publish_timer is None:
                self.get_logger().info("Starting simple publish timer.")
                self.simple_publish_timer = self.create_timer(random.randint(SENSING_RATE[0], SENSING_RATE[1]), self.publish_sensor_data)

            # Destroy the check_simulation_start timer
            self.check_simulation_start_timer.cancel()
            self.get_logger().info("Stopping check_simulation_start timer.")

    def publish_sensor_data(self):
        """Publishes randomly generated sensor data as a ROS2 message."""
        sensor_data = self.generate_random_sensor_data()

        sensor_id = self.id.get_parameter_value().integer_value

        sensor_data_dict = sensor_data.to_dict()
        sensor_data_dict['sensor_id'] = sensor_id
        
        sensor_data_serialized = json.dumps(sensor_data_dict) 

        # Publish the sensor data
        msg = String()
        msg.data = sensor_data_serialized
        self.tx_topic.publish(msg)
        self.get_logger().info(f"Published SensorData: {str(sensor_data)}")

    def generate_random_sensor_data(self):
        """Generate random sensor data for temperature, blood pressure, and heartbeat."""
        # Randomly generate sensor readings within realistic ranges
        temperature = round(random.uniform(35.0, 40.0), 1)  # Temperature in °C
        blood_pressure = (random.randint(90, 140), random.randint(60, 90))  # BP (systolic, diastolic)
        heartbeat = random.randint(60, 100)  # Heartbeat in bpm

        # Create SensorData object
        sensor_data = SensorData(temperature=temperature, blood_pressure=blood_pressure, heartbeat=heartbeat, misuration_ts=time.time())
        return sensor_data

    def store_position(self, odometry_msg: Odometry):
        self.position = odometry_msg.pose.pose.position
        self.yaw = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w,
        )
        if not self.simulation_started and self.position.x != -1.0:
            self.simulation_started = True

    def execute_patrol_action(self, goal: ServerGoalHandle):
        command_goal: Patrol.Goal = goal.request
        self.get_logger().info(
            f"Action requested. Performing movement to targets:\n\t{command_goal.targets}"
        )

        target = command_goal.targets[0]
        self.rotate_to_target(target)
        self.move_to_target(target)
        self.get_logger().info(f"Movement to target {target} completed!")
        goal.succeed()

        result = Patrol.Result()
        result.result = "Movement completed"

        return result

    def rotate_to_target(self, target, eps=0.5):

        target_angle = math_utils.angle_between_points(self.position, target)
        angle_to_rotate = target_angle - self.yaw

        angle_to_rotate = (angle_to_rotate + math.pi) % (2 * math.pi) - math.pi

        rotation_dir = 1 if angle_to_rotate < 0 else -1

        # Prepare the cmd_vel message
        move_msg = Twist()
        move_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        move_msg.angular = Vector3(x=0.0, y=0.0, z=0.5 * rotation_dir)
        self.cmd_vel_publisher.publish(move_msg)

        # Publish the message until the correct rotation is reached (accounting for some eps error)
        # Note that here the eps also helps us stop the drone and not overshoot the target, as
        # the drone will keep moving for a while after it receives a stop message
        # Also note that rotating the drone too fast will make it loose altitude.
        # You can account for that by also giving some z linear speed to the rotation movement.
        while abs(angle_to_rotate) > eps:
            angle_to_rotate = target_angle - self.yaw

            # self.get_logger().info(f"Rotation: {self.yaw}")
            # No sleep here. We don't want to miss the angle by sleeping too much. Even 0.1 seconds
            # could make us miss the given epsilon interval

        # When done, send a stop message to be sure that the drone doesn't
        # overshoot its target
        self.cmd_vel_publisher.publish(self.stop_msg)

    def move_to_target(self, target, eps=0.5, angle_eps=0.02):
        # Save the target position and compute the distance
        distance = math_utils.point_distance(self.position, target)

        # Keep publishing movement while the distance is greater than the given EPS
        while distance > eps:

            # Compute the move vector with the given position and target
            mv = math_utils.move_vector(self.position, target)

            twist_msg = Twist()
            twist_msg.linear.x = mv[0]
            twist_msg.linear.z = mv[1]

            # Check if Balloon is still facing the target correctly, otherwise add angular
            # velocity to the Twist msg
            target_angle = math_utils.angle_between_points(self.position, target)

            if not (target_angle - angle_eps < self.yaw < target_angle + angle_eps):
                angle_diff = self.yaw - target_angle
                twist_msg.angular = Vector3(x=0.0, y=0.0, z=math.sin(angle_diff))

            # Publish msg
            self.cmd_vel_publisher.publish(twist_msg)

            # Update position and distance after finishing
            distance = math_utils.point_distance(self.position, target)

        # After reaching the target, publish a stop msg
        self.cmd_vel_publisher.publish(self.stop_msg)


def main():
    rclpy.init()

    executor = MultiThreadedExecutor()
    sensor_controller = SensorController()
    executor.add_node(sensor_controller)
    executor.spin()
    executor.shutdown()
    sensor_controller.destroy_node()

    rclpy.shutdown()
