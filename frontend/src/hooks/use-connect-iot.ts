import { iot, mqtt } from "aws-iot-device-sdk-v2";
import { useEffect, useState } from "react";
import { useAwsCredentials } from "./use-get-aws-credentials";

export type useAwsIotMqttProps = {
    setMessageHandler: (topic: string, message: string) => void;
};

export const useAwsIotMqtt = () => {
    const [connection, setConnection] = useState<mqtt.MqttClientConnection | null>(null);
    const credentials = useAwsCredentials();
  
    useEffect(() => {
    if (!credentials) return;

      const connectToAwsIot = async () => {
        const { accessKeyId, secretAccessKey, sessionToken } = credentials;
        
        if (!accessKeyId || !secretAccessKey || !sessionToken) {
          console.error('Invalid AWS credentials');
          return;
        }
  
        const configBuilder = iot.AwsIotMqttConnectionConfigBuilder.new_with_websockets()
          .with_clean_session(true)
          .with_client_id(`mqtt-client-${Math.floor(Math.random() * 1000)}`)
          .with_endpoint(import.meta.env.VITE_ENDPOINT)
          .with_credentials(
            import.meta.env.VITE_REGION,
            accessKeyId,
            secretAccessKey,
            sessionToken
          );
  
        const mqttClientConfig = configBuilder.build();
        const client = new mqtt.MqttClient();
        const connection = client.new_connection(mqttClientConfig);
  
        try {
          await connection.connect().then  (() => {
              
              console.log('Connected to AWS IoT');
          });
          setConnection(connection);

        } catch (error) {
          console.error('Error connecting:', error);
        }
      };
  
      connectToAwsIot();
  
      return () => {
        if (connection) {
            console.log('Disconnecting from AWS IoT');
            connection.disconnect();
        }
      };
    }, [credentials, setConnection]);
  
    return connection;
  };