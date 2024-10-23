import rclpy
import random
from threading import Thread

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from project_interfaces.action import Query
from project_interfaces.action import Patrol

# from project_interfaces.msg import MultiStringArray
import time
from rosgraph_msgs.msg import Clock
from project_main.sensor_data import SensorData
import json
from project_main.cache_strategy import CacheType

DATA = ["temperature", "blood pressure", "heartbeat"]


class BaseStationController(Node):
    def __init__(self):
        super().__init__("base_station_controller")
        self.goal_id = 0

        self.declare_parameters(
            namespace="",
            parameters=[
                ("balloons", 3),
                ("sensors", 3),
                ('cache_type', CacheType.FIFO.name),
                ('cache_size', 10),
                ('cache_expiration', 10),
                ('query_rate', 10),
            ],
        )
        self.balloons = (
            self.get_parameter("balloons").get_parameter_value().integer_value
        )
        self.sensors = self.get_parameter("sensors").get_parameter_value().integer_value
        self.cache_type = self.get_parameter('cache_type').get_parameter_value().string_value
        self.cache_size = self.get_parameter('cache_size').get_parameter_value().integer_value
        self.cache_expiration = self.get_parameter('cache_expiration').get_parameter_value().integer_value
        self.query_rate = self.get_parameter('query_rate').get_parameter_value().integer_value

        self.action_clients = {}
        for i in range(self.balloons):
            action_client = ActionClient(
                self,
                Query,
                f"/Balloon_{i}/query_sensors",
            )
            self.action_clients[f"Balloon_{i}"] = action_client

        self.sensor_positions = {}

        for idx in range(self.sensors):
            self.create_subscription(
                Odometry,
                f"ActiveSensor_{idx}/odometry",
                lambda msg, id=idx: self.store_sensor_position(id, msg),
                10,
            )

        self.query_responses = {}
        self.active_query = {"status": False, "timestamp": None}
        self.last_query_time = time.time()
        self.out_path = f"/home/intou/Desktop/IoT/IoT-Project-2024/src/output_tests/new/{time.time()}.json"
        with open(self.out_path, "w") as f:
            json.dump({"sensors": self.sensors, "balloons": self.balloons, "cache_type": self.cache_type, "cache_expiration": self.cache_expiration, "cache_size": self.cache_size, "query_rate": self.query_rate, "queries": []}, f)



    def store_sensor_position(self, id, msg: Odometry):
        self.sensor_positions[id] = msg.pose.pose.position

    def send_queries(self):
        """
        Method used to keep the Base Station constantly (or as needed) sending queries to Balloons.
        """

        def send_queries_inner():
            i = 0
            while True:
                if not self.active_query["status"] and time.time() - self.last_query_time >= self.query_rate:
                    self.start_querying()

        # Start this function in another thread, so that the node can start spinning immediately after
        # this function has finished
        Thread(target=send_queries_inner).start()

    def start_querying(self):
        # Wait for the action server to go online and sensors to start moving
        while (
            not (sum([action_client.wait_for_server(1) for action_client in self.action_clients.values()]) == len(self.action_clients.values()))
            or len(self.sensor_positions) < self.sensors
        ):
            self.get_logger().info(
                f"Waiting for Query action server to come online and sensors to announce their position"
            )
            time.sleep(3)

        self.active_query["status"] = True
        self.active_query["timestamp"] = time.time()

        sensor_id, parameters = self.compute_next_goal()
        self.goal_id += 1

        self.send_goal(sensor_id, parameters)

    def send_goal(self, queried_sensor, queried_parameters):
        goal_msg = Query.Goal()
        goal_msg.id = self.goal_id
        goal_msg.sensor_id = queried_sensor

        goal_msg.parameters = queried_parameters

        self.get_logger().info(f'Sending Goal_{self.goal_id}: sensor_id={queried_sensor}, parameters={queried_parameters}')
        self.responses = {}
        
        for ball in range(self.balloons):
            self._send_goal_future = self.action_clients[f"Balloon_{ball}"].send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except RuntimeError:
            self.get_logger().info("Actual query was handled too quick")
            return self.conclude_query()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal_{self.goal_id} rejected :(')
            return

        self.get_logger().info(f'Goal_{self.goal_id} accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        response = json.loads(result.sensor_response)
        if response["data"] != "KeyNotFoundError" and response["data"] != "CacheExpiredError":
            result_data = self.convert_dict_to_sensordata(response["data"])
            self.get_logger().info(f'Result: sensor_response={result_data}, balloon_id={result.balloon_id}')
            self.query_responses[f"Balloon_{result.balloon_id}"] = response
        else:
            self.get_logger().info(f'Result: sensor_response={response["data"]}, balloon_id={result.balloon_id}')
            self.query_responses[f"Balloon_{result.balloon_id}"] = response

        if len(self.query_responses) == self.balloons:
            self.conclude_query()

    def conclude_query(self):
        self.store_result()
        # Send the next goal automatically
        self.active_query["status"] = False
        self.active_query["timestamp"] = None

        self.last_query_time = time.time()
        self.query_responses = {}
        return


    def compute_next_goal(self):
        sensor_id = random.sample(list(range(self.sensors)), 1)
        parameters = [DATA[0], DATA[2]]
        return sensor_id[0], parameters
    
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
    
    def store_result(self):
        serialized = {"timestamp_request_start": self.active_query["timestamp"]}
        for k, v in self.query_responses.items():
            if type(v) == SensorData:
                serialized[k] = v.to_dict()
            else:
                serialized[k] = v

        with open(self.out_path, "r") as f:
            to_update = json.load(f)

        with open(self.out_path, "w") as f:
            to_update["queries"].append(serialized)
            json.dump(to_update, f)
        return
    
def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    bs = BaseStationController()
    executor.add_node(bs)
    bs.send_queries()

    executor.spin()

    executor.shutdown()
    bs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()