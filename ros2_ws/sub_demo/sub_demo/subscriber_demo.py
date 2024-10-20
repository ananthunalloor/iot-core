import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from awscrt import io, mqtt, auth, http
from awsiot import mqtt_connection_builder
import json
from sub_demo.connection_helper import ConnectionHelper

class TelemetrySubscriber(Node):
    def __init__(self):
        super().__init__('telemetry_subscriber')
        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)

        path_for_config = self.get_parameter("path_for_config").get_parameter_value().string_value
        discover_endpoints = self.get_parameter("discover_endpoints").get_parameter_value().bool_value
        self.connection_helper = ConnectionHelper(self.get_logger(), path_for_config, discover_endpoints)

        self.subscription = self.create_subscription(
            String, 
            'ros2_mock_publish_topic',
            self.listener_callback,
            10  # QoS (Quality of Service) profile
        )
        self.subscription
        self.init_subs()
        
    def init_subs(self):
        """Subscribe to mock ros2_mock_publish_topic topic"""
        self.subscription = self.create_subscription(
            String,
            'ros2_mock_publish_topic',
            self.listener_callback,
            10
        ) 

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message from ros2_mock_publish_topic: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    telemetry_subscriber = TelemetrySubscriber()
    rclpy.spin(telemetry_subscriber)
    telemetry_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()