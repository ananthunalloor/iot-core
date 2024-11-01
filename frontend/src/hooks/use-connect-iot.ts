import { iot, mqtt } from "aws-iot-device-sdk-v2";
import { useEffect, useState } from "react";
import { CredentialsType } from "./use-get-aws-credentials";


export const useAwsIotMqtt = (credentials: CredentialsType | undefined) => {
  const [connection, setConnection] =
    useState<mqtt.MqttClientConnection | null>(null);

  useEffect(() => {
    if (!credentials) return;

    const connectToAwsIot = async () => {
      const { accessKeyId, secretAccessKey, sessionToken } = credentials;

      if (!accessKeyId || !secretAccessKey || !sessionToken) {
        console.error("Invalid AWS credentials");
        return;
      }

      const configBuilder =
        iot.AwsIotMqttConnectionConfigBuilder.new_with_websockets()
          .with_clean_session(true)
          .with_client_id(`mqtt-client`)
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
        await connection.connect().then(() => {
          console.log("Connected to AWS IoT");
        });
        setConnection(connection);
      } catch (error) {
        console.error("Error connecting:", error);
      }
    };

    connectToAwsIot();

    return () => {
      if (connection) {
        console.log("Disconnecting from AWS IoT");
        connection.disconnect();
      }
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [credentials]);

  return connection;
};
