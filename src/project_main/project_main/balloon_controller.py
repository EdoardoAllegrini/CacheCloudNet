import rclpy
import math_utils

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from project_main.cache_strategy import CacheType, CacheFactory, CacheExpiredError, KeyNotFoundError
from rclpy.action import ActionServer
from project_interfaces.action import Query
import json
from project_main.sensor_data import SensorData
# from project_interfaces.msg import MultiStringArray

class BalloonController(Node):
    def __init__(self):
        super().__init__("balloon_controller")
        self.id = self.declare_parameter('id', -1)
        self._init_cache()
        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.yaw = 0
        self.stop_msg = Twist()
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.odometry_subscrber = self.create_subscription(
            Odometry,
            'odometry',
            self.store_position,
            10
        )

        self.rx_data = self.create_subscription(
            String,
            'rx_data',
            self.rx_callback,
            10
        )
        self._action_server = ActionServer(
            self,
            Query,
            f"/Balloon_{self.id.get_parameter_value().integer_value}/query_sensors",
            self.execute_query_callback)
        
    def execute_query_callback(self, goal_handle):
        
        id = goal_handle.request.id

        sensor_id = goal_handle.request.sensor_id
        parameters = goal_handle.request.parameters
        # self.get_logger().info(f'sensor_id={sensor_id}, parameters={parameters}')
        
        response = self.search_response(id, sensor_id, parameters)
        
        goal_handle.succeed()

        result = Query.Result()
        result.balloon_id = self.id.get_parameter_value().integer_value

        result.sensor_response = response
        return result

    def search_response(self, query_id, sensor_id, parameters):
        # self.get_logger().info(f'Searching on Balloon_{self.id.get_parameter_value().integer_value}')
        self.get_logger().info(f'Cache -> {self.cache}')

        basic_response = {"sensor_id": sensor_id}
        try:
            value = self.cache.__getitem__(sensor_id)
            dict_converted = value.to_dict()
            basic_response["data"] = dict_converted
        except CacheExpiredError:
            basic_response["data"] = "CacheExpiredError"
        except KeyNotFoundError:
            basic_response["data"] = "KeyNotFoundError"

        response = json.dumps(basic_response)
        return response


    def _init_cache(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('cache_type', CacheType.FIFO.name),
                ('cache_size', 10),
                ('cache_expiration', 10),
            ]
        )
        cache_type = self.get_parameter('cache_type').get_parameter_value().string_value
        cache_size = self.get_parameter('cache_size').get_parameter_value().integer_value
        cache_expiration = self.get_parameter('cache_expiration').get_parameter_value().integer_value

        self.cache = CacheFactory.create(cache_type, cache_size, cache_expiration)
        self.get_logger().info(
            f"Cache of type '{cache_type}' with size {cache_size} initialized successfully."
        )

    def rx_callback(self, msg : String):
        sensor_data_dict = json.loads(msg.data)
        sensor_id = sensor_data_dict.get("sensor_id", "Unknown")
        sensor_data = self.convert_dict_to_sensordata(sensor_data_dict)

        # self.get_logger().info(f"Balloon #{self.id.get_parameter_value().integer_value}; Sensor_{sensor_id} -> {sensor_data.to_dict()}")

        self.cache[sensor_id] = sensor_data

    def convert_dict_to_sensordata(self, sensor_data_dict: dict):
        sensor_data = SensorData(
            temperature=sensor_data_dict['temperature'],
            blood_pressure=(
                sensor_data_dict['blood_pressure']['systolic'],
                sensor_data_dict['blood_pressure']['diastolic']
            ),
            heartbeat=sensor_data_dict['heartbeat'],
            misuration_ts=sensor_data_dict['timestamp']
        )
        return sensor_data

    def store_position(self, odometry_msg: Odometry):
        self.position = odometry_msg.pose.pose.position
        self.yaw = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        )


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    balloon = BalloonController()
    executor.add_node(balloon)
    executor.spin()
    balloon.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()