import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useAwsIotMqtt } from "../hooks/use-connect-iot";
import { mqtt } from "aws-iot-device-sdk-v2";
import { Joystick } from "react-joystick-component";
import { useKinesisViewer } from "../hooks/useKinesisViewer";

// import { AWSWebRTCViewer } from "../components/aws-webtrc-viewer";
// import { AWSWebRTCConfig } from "../hooks/use-connect-kns";

type JoystickDirection = "FORWARD" | "RIGHT" | "LEFT" | "BACKWARD";
export interface IJoystickUpdateEvent {
  type: "move" | "stop" | "start";
  x: number | null;
  y: number | null;
  direction: JoystickDirection | null;
  distance: number | null;
}

export const App = () => {
  const [messages, setMessages] = useState<
    { topic: string; message: string }[]
  >([]);
  const [latestMessage, setLatestMessage] = useState<{
    battery: number;
    velocity: number;
    timestamp?: number;
  }>({
    battery: 0,
    velocity: 0,
    timestamp: 0,
  });
  const [publishMessage, setPublishMessage] = useState<string>("Message");
  const [joystickData, setJoystickData] = useState<IJoystickUpdateEvent | null>(
    null
  );
  const videoRef = useRef<HTMLVideoElement>(null);

  const setMessageHandler = (topic: string, message: string) => {
    setMessages((prevMessages) => [...prevMessages, { topic, message }]);
    const data = JSON.parse(message);
    setLatestMessage({
      battery: data.battery,
      velocity: data.velocity,
      timestamp: data.timestamp,
    });
  };

  const connection = useAwsIotMqtt();
  // const config = AWSWebRTCConfig()
  const signaling = useKinesisViewer(videoRef);



  useEffect(() => {
    if (connection) {
      connection.subscribe(
        "ros2_mock_telemetry_topic",
        mqtt.QoS.AtLeastOnce,
        (topic, payload) => {
          const message = new TextDecoder("utf-8").decode(
            new Uint8Array(payload)
          );
          setMessageHandler(topic, message);
        }
      );
    }
  }, [connection]);

  const onPublish = useCallback(() => {
    if (!connection) {
      return;
    }
    connection
      .publish(
        "ros2_mock_publish_topic",
        JSON.stringify(publishMessage),
        mqtt.QoS.AtLeastOnce
      )
      .then(() => {
        setPublishMessage("");
      });
  }, [connection, publishMessage]);

  const messageDelay = useMemo(() => {
    return latestMessage.timestamp
      ? (Date.now() - latestMessage.timestamp * 1000).toFixed(2)
      : 0;
  }, [latestMessage.timestamp]);

  const connectToAwsKinesis = useCallback(() => {
    if (!signaling) return;
    signaling.open();
  }, [signaling]);

  const disconnectFromAwsKinesis = useCallback(() => {
    if (!signaling) return;
    signaling.close();
  }, [signaling]);

  const handleMove = (event: IJoystickUpdateEvent) => {
    setJoystickData(event);
  };

  const handleStop = (event: IJoystickUpdateEvent) => {
    setJoystickData({
      ...event,
      x: 0,
      y: 0,
    });
  };

  useEffect(() => {
    if (joystickData) {
      if (!connection) return;
      console.log(joystickData);
      connection.publish(
        "ros2_mock_publish_topic",
        JSON.stringify(joystickData),
        mqtt.QoS.AtMostOnce
      );
    }
  }, [joystickData, connection]);

  return (
    <div className="flex flex-col gap-6 p-6">
      <h1 className="mb-6 text-2xl font-bold">AWS IoT Core MQTT Messages</h1>
      <div className="flex flex-row flex-wrap gap-4">
        <div>
          <div className="flex flex-col gap-4">
            <div className="flex gap-4">
              <div className="bg-white shadow-md rounded-lg p-6 flex flex-col items-center min-w-[140px]">
                <h2 className="text-sm font-medium text-gray-500">Battery</h2>
                <p className="text-2xl font-bold">{latestMessage.battery}</p>
              </div>

              <div className="bg-white shadow-md rounded-lg p-6 flex flex-col items-center min-w-[140px]">
                <h2 className="text-sm font-medium text-gray-500">Velocity</h2>
                <p className="text-2xl font-bold">{latestMessage.velocity}</p>
              </div>
              <div className="bg-white shadow-md rounded-lg p-6 flex flex-col items-center min-w-[140px]">
                <h2 className="text-sm font-medium text-gray-500">
                  Total Message Count
                </h2>
                <p className="text-2xl font-bold">{messages.length}</p>
              </div>
              <div className="bg-white shadow-md rounded-lg p-6 flex flex-col items-center min-w-[180px]">
                <h2 className="text-sm font-medium text-gray-500">Delay</h2>
                <p className="text-2xl font-bold">{`${messageDelay} ms`}</p>
              </div>
            </div>
          </div>
          <div className="flex gap-6">
            <div className="w-[480px] rounded-lg p-8 flex flex-col shadow-md">
              <h2 className="mb-1 text-lg font-medium title-font">
                Publish Message
              </h2>
              <div className="mb-4">
                <label htmlFor="message" className="text-sm leading-7">
                  Message
                </label>
                <textarea
                  value={publishMessage}
                  onChange={(e) => setPublishMessage(e.target.value)}
                  id="message"
                  name="message"
                  className="w-full h-32 px-3 py-1 text-base leading-6 bg-white border border-gray-300 rounded outline-none resize-none"
                />
              </div>
              <button
                onClick={onPublish}
                className="px-6 py-2 text-lg text-white bg-indigo-500 border-0 rounded"
              >
                Publish
              </button>
            </div>
            <div className="flex flex-col items-center p-6 rounded-lg shadow-md">
              <h2 className="mb-1 text-lg font-medium title-font">
                Send joystick commands
              </h2>
              <div className="m-auto">
                <Joystick
                  size={120}
                  throttle={500}
                  move={handleMove}
                  stop={handleStop}
                />
              </div>
              <p className="text-sm text-gray-500">
                move the joystick to send commands
              </p>
              <div className="flex items-center gap-4">
                <p className="text-gray-500">{`X: ${joystickData?.x?.toFixed(
                  4
                )}`}</p>
                <p className="text-gray-500">{`Y: ${joystickData?.y?.toFixed(
                  4
                )}`}</p>
              </div>
            </div>
          </div>
        </div>
        <div className="w-[854px] h-[480px] bg-gray-100 rounded-lg flex flex-col items-center justify-center ">
          <video height='100%' width='100%' autoPlay ref={videoRef} />
          {/* {
            !config?.channelARN ? <p>loading...</p> :

              <AWSWebRTCViewer config={config} />
          } */}
          <div className="flex flex-row gap-4">

            <button className="px-6 py-2 text-lg text-white bg-indigo-500 border-0 rounded" onClick={connectToAwsKinesis}>Connect</button>
            {/* <button className="px-6 py-2 text-lg text-white bg-indigo-500 border-0 rounded" onClick={disconnectFromAwsKinesis}>Disconnect</button> */}
          </div>
        </div>
      </div>
    </div>
  );
};
