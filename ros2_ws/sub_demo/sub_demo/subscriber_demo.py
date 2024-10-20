import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Change this to the correct message type used in ros2_mock_telemetry_topic

class TelemetrySubscriber(Node):
    def __init__(self):
        super().__init__('telemetry_subscriber')
        self.subscription = self.create_subscription(
            String,  # Change this to the correct message type used in your topic
            'ros2_mock_publish_topic',  # The name of the topic to subscribe to
            self.listener_callback,
            10  # QoS (Quality of Service) profile
        )
        self.subscription  # Prevent unused variable warning

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