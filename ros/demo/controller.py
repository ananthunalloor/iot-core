import rclpy
import json
import random
from rclpy.node import Node
from std_msgs.msg import String
import time
from awscrt import mqtt
from awsiot import mqtt_connection_builder
from geometry_msgs.msg import Twist


TOPIC = "rover_iot_thing/publish_topic"

class TurtleSimController(Node):

    def __init__(self):
        super().__init__('turtle_sim_controller')
        self.mqtt_connection = self.configure_mqtt_client()
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 3)

        self.init_pubs()

    def publish_cmd_vel(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.get_logger().info(f"Publishing cmd_vel: {twist}")
        self.publisher.publish(twist)
    

    def init_pubs(self):
        logger = self.get_logger()
        logger.info("Connecting to AWS IoT")
        self.mqtt_connection = self.configure_mqtt_client()
        self.mqtt_connection.connect()
        logger.info("Connected to AWS IoT")
        subscribe_future, _ = self.mqtt_connection.subscribe(
            TOPIC,
            mqtt.QoS.AT_LEAST_ONCE,
            self.on_message_received
        )
        subscribe_future.result()
        logger.info("Subscribed successfully")

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
    

    def on_message_received(self, topic, payload, **kwargs):
        logger = self.get_logger()
        logger.info(f"Received message from topic '{topic}': {payload.decode('utf-8')}")

        message = json.loads(payload)
# The lines `linear_x = float(message.get("linear", {}).get("x", 0.0))` and
# `angular_z = float(message.get("angular", {}).get("z", 0.0))` are extracting the
# values of linear and angular velocities from a JSON message payload received
# over MQTT.
        logger.info(f"Received message from topic '{topic}': {message}")
        linear_x = float(message.get("linear", {}).get("x", 0.0))
        angular_z = float(message.get("angular", {}).get("z", 0.0))


        self.publish_cmd_vel(linear_x, angular_z)
        # logger.info(f"Published cmd_vel: linear_x={linear_x}, angular_z={angular_z}")   

    # def timer_callback(self):
    #     msg = String()

    #     mock_telemetry = {}
    #     mock_telemetry["battery"] = round(random.uniform(85, 90), 2)
    #     mock_telemetry["velocity"] = round(random.uniform(3, 4), 2)
    #     mock_telemetry["timestamp"] = time.time()
    #     msg.data = json.dumps(mock_telemetry)

    #     self.mqtt_connection.publish(
    #         TOPIC,
    #         msg.data,
    #         mqtt.QoS.AT_LEAST_ONCE,
    #     )

    #     logger = self.get_logger()
    #     logger.info(f"Published message to topic {TOPIC}: {msg.data}")
        

def main(args=None):
    rclpy.init(args=args)

    node = TurtleSimController()

    rclpy.spin(node)

    # Destroy the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()