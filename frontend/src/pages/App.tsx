import { useCallback, useEffect, useState } from "react";
import { useAwsIotMqtt } from "../hooks/use-connect-iot";
import { mqtt } from "aws-iot-device-sdk-v2";

export const App=()=> {

  const [messages, setMessages] = useState<{topic: string, message: string}[]>([]);
  const [latestMessage, setLatestMessage] = useState<{battery: number, velocity: number}>({
    battery: 0,
    velocity: 0
  });
  const [publishMessage, setPublishMessage] = useState<string>('Message');

  const setMessageHandler = (topic: string, message: string) => {
    setMessages((prevMessages) => [...prevMessages, { topic, message }]);
    const data = JSON.parse(message);
    setLatestMessage({ battery: data.battery, velocity: data.velocity });
  };

const connection = useAwsIotMqtt();

  useEffect(() => {
    if (connection) {
      connection.connect();
      connection.subscribe('ros2_mock_telemetry_topic', mqtt.QoS.AtLeastOnce, (topic, payload) => {
        const message = new TextDecoder('utf-8').decode(new Uint8Array(payload));
      setMessageHandler(topic, message);
      });
    }
  }, [connection]);

  const onPublish = useCallback(() => {
    if (!connection) {
      return;
    }
    connection.publish('ros2_mock_publish_topic', JSON.stringify(publishMessage), mqtt.QoS.AtLeastOnce).then(() => {
      setPublishMessage('');
    });
  } , [connection, publishMessage]);

  return (
    <div>
      <h1 className="text-2xl font-bold mb-6">AWS IoT Core MQTT Messages</h1>
      <div className="flex flex-col gap-4 m-6">
        <div className="flex gap-4">
            <div className="bg-white shadow-md rounded-lg p-6 flex flex-col items-center min-w-[140px]">
                <h2 className="text-gray-500 font-medium text-sm">Battery</h2>
                <p className="text-2xl font-bold">{latestMessage.battery}</p>
            </div>

            <div className="bg-white shadow-md rounded-lg p-6 flex flex-col items-center min-w-[140px]">
                <h2 className="text-gray-500 font-medium text-sm">Velocity</h2>
                <p className="text-2xl font-bold">{latestMessage.velocity}</p>
            </div>
            <div className="bg-white shadow-md rounded-lg p-6 flex flex-col items-center min-w-[140px]">
                <h2 className="text-gray-500 font-medium text-sm">Total Message Count</h2>
                <p className="text-2xl font-bold">{messages.length}</p>
            </div>
        </div>
    </div>
    <div className="flex">
    <div className="w-[480px] rounded-lg p-8 flex flex-col shadow-md">
      <h2 className="text-lg mb-1 font-medium title-font">Publish Message</h2>
      <div className="mb-4">
        <label htmlFor="message" className="leading-7 text-sm">Message</label>
        <textarea value={publishMessage} onChange={(e) => setPublishMessage(e.target.value)} id="message" name="message" className="w-full bg-white rounded border border-gray-300 h-32 text-base outline-none py-1 px-3 resize-none leading-6"/>
      </div>
      <button onClick={onPublish} className="text-white bg-indigo-500 border-0 py-2 px-6 rounded text-lg">Button</button>
      </div>
  </div>
    </div>
  );
}

