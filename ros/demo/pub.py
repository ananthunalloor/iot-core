import rclpy
import json
import random
from rclpy.node import Node
from std_msgs.msg import String
import time
from awscrt import mqtt
from awsiot import mqtt_connection_builder


TOPIC = "rover_iot_thing/telemetry_topic"

class TelemetryPublisher(Node):

    def __init__(self):
        super().__init__('mock_telemetry_publisher')
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mqtt_connection = self.configure_mqtt_client()

        self.init_pubs()
    

    def init_pubs(self):
        logger = self.get_logger()

        logger.info("Connecting to AWS IoT")
        self.mqtt_connection = self.configure_mqtt_client()
        self.mqtt_connection.connect()
        logger.info("Connected to AWS IoT")

    def configure_mqtt_client(self):
        with open('/home/ros/cert/iot_config.json') as f:
            cert_data = json.load(f)

        mqtt_connection = mqtt_connection_builder.mtls_from_path(
            endpoint=cert_data["endpoint"],
            port=8883,
            cert_filepath=cert_data["certificatePath"],
            pri_key_filepath=cert_data["privateKeyPath"],
            ca_filepath=cert_data["rootCAPath"],
            client_id=cert_data.get("clientID"),
    )
        return mqtt_connection

    def timer_callback(self):
        msg = String()

        mock_telemetry = {}
        mock_telemetry["battery"] = round(random.uniform(85, 90), 2)
        mock_telemetry["velocity"] = round(random.uniform(3, 4), 2)
        mock_telemetry["timestamp"] = time.time()
        msg.data = json.dumps(mock_telemetry)

        self.mqtt_connection.publish(
            TOPIC,
            msg.data,
            mqtt.QoS.AT_LEAST_ONCE,
        )

        logger = self.get_logger()
        logger.info(f"Published message to topic {TOPIC}: {msg.data}")
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TelemetryPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()