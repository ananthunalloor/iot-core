import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from awscrt import io, mqtt, auth, http
from awsiot import mqtt_connection_builder
import json
import logging
logging.basicConfig(level=logging.DEBUG)

class TelemetrySubscriber(Node):
    def __init__(self):
        super().__init__('telemetry_subscriber')
        self.declare_parameter("path_for_config", "")
        self.declare_parameter("aws_topic", "ros2_mock_publish_topic")

        path_for_config = self.get_parameter("path_for_config").get_parameter_value().string_value
        self.aws_topic = self.get_parameter("aws_topic").get_parameter_value().string_value

        try:
            with open(path_for_config, 'r') as f:
                cert_data = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Configuration file not found: {path_for_config}")
            return
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse configuration file as JSON.")
            return

        self.mqtt_connection = self.create_mqtt_connection(cert_data)
        self.init_subs()

    def create_mqtt_connection(self, cert_data):
        """Create an MQTT connection to AWS IoT."""
        logger = self.get_logger()
        try:
            mqtt_connection = mqtt_connection_builder.mtls_from_path(
                endpoint=cert_data["endpoint"],
                port=cert_data.get("port"),
                cert_filepath=cert_data["certificatePath"],
                pri_key_filepath=cert_data["privateKeyPath"],
                ca_filepath=cert_data["rootCAPath"],
                client_id=cert_data.get("clientID"),
            )
            connect_future = mqtt_connection.connect()
            connect_future.result()
            logger.info("Connected to AWS IoT")
            return mqtt_connection
        except Exception as e:
            logger.error(f"Failed to connect to AWS IoT: {e}")
            return None

    def init_subs(self):
        """Subscribe to the AWS IoT topic."""
        if not self.mqtt_connection:
            self.get_logger().error("MQTT connection is not established.")
            return

        logger = self.get_logger()
        logger.info(f"Subscribing to topic: {self.aws_topic}")
        try:
            subscribe_future, packet_id = self.mqtt_connection.subscribe(
                topic=self.aws_topic,
                qos=mqtt.QoS.AT_LEAST_ONCE,
                callback=self.aws_topic_callback
            )
            subscribe_result = subscribe_future.result()
            logger.info(f"Subscribed with QoS: {subscribe_result['qos']} to topic: {self.aws_topic} with packet_id: {packet_id}")
        except Exception as e:
            logger.error(f"Failed to subscribe to topic {self.aws_topic}: {e}")

    def aws_topic_callback(self, topic, payload, **kwargs):
        """Callback function for AWS IoT messages."""
        logger = self.get_logger()
        try:
            message = json.loads(payload)
            logger.info(f'Received message from AWS topic "{topic}": {message}')
        except json.JSONDecodeError:
            logger.error(f"Failed to decode message payload: {payload}")
        except Exception as e:
            logger.error(f"Error in callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    telemetry_subscriber = TelemetrySubscriber()
    try:
        rclpy.spin(telemetry_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        if telemetry_subscriber.mqtt_connection:
            telemetry_subscriber.mqtt_connection.disconnect().result()
        telemetry_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()