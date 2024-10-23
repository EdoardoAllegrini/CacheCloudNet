import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sim_utils import EventScheduler


class DummySensor(Node):

    def __init__(self):
        super().__init__('dummy_sensor')
        self.tx_topic = self.create_publisher(
            String,
            'tx_data',
            10
        )

        self.id = self.declare_parameter('id', -1)
        self.generated_data = 0
        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/iot_project_world/clock',
            self.event_scheduler.routine,
            10
        )

        self.event_scheduler.schedule_event(1, self.simple_publish)


    def simple_publish(self):
        id = self.id.get_parameter_value().integer_value
        msg = String()
        msg.data = f"Sensor data: {id}_{self.generate_data()}!"
        self.tx_topic.publish(msg) 


    def generate_data(self):
        self.generated_data += 1
        return self.generated_data


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    sensor = DummySensor()
    executor.add_node(sensor)
    executor.spin()
    sensor.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
