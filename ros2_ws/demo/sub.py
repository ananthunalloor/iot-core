import json
import logging
import time
from awscrt import mqtt
from awsiot import mqtt_connection_builder

# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger()

TOPIC = "ros2_mock_publish_topic"

def configure_mqtt_client():
    with open('/root/aws-iot-robot-connectivity-samples-ros2/iot_certs_and_config/iot_config.json') as f:
        cert_data = json.load(f)

    mqtt_connection = mqtt_connection_builder.mtls_from_path(
        endpoint=cert_data["endpoint"],
        port=8883,
        cert_filepath=cert_data["certificatePath"],
        pri_key_filepath=cert_data["privateKeyPath"],
        ca_filepath=cert_data["rootCAPath"],
        client_id=cert_data.get("clientID", "test_ros2_thing"),
        # client_id='test_ros2_thing', 
    )

    return mqtt_connection

def on_message_received(topic, payload, **kwargs):
    logger.info(f"Received message from topic '{topic}': {payload.decode('utf-8')}")


def main():
    mqtt_connection = configure_mqtt_client()

    try:
        connect_future = mqtt_connection.connect()
        connect_future.result()
        logger.info("Connected to AWS IoT")
    except Exception as e:
        logger.error(f"Failed to connect to AWS IoT: {e}")

    logger.info(f"Subscribing to topic: {TOPIC}")
    try:   
        subscribe_future, _ = mqtt_connection.subscribe(
            TOPIC,
            mqtt.QoS.AT_LEAST_ONCE,
            on_message_received
        )
        subscribe_future.result()
        logger.info("Subscribed successfully")

    except Exception as e:
        logger.error(f"Failed to subscribe to topic {TOPIC}: {e}")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Disconnecting from AWS IoT Core")
        mqtt_connection.disconnect()

if __name__ == '__main__':
    main()
