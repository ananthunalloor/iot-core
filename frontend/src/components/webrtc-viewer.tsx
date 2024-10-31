import {  useRef } from "react";
import { useAwsCredentials } from "../hooks/use-get-aws-credentials";
import { useKinesisVideoClient } from "../hooks/kvs-hooks/useKinesisVideoClient";
import { useChannelARN } from "../hooks/kvs-hooks/useChannelARN";
import { useSignalingEndpoints } from "../hooks/kvs-hooks/useSignalingEndpoints";
import { useIceServers } from "../hooks/kvs-hooks/useIceServers";
import { useSignalingClient } from "../hooks/kvs-hooks/useSignalingClient";
// import { useKinesisViewer } from "../hooks/useKinesisViewer";

export const WebRTCViewer = () => {
  const videoRef = useRef<HTMLVideoElement>(null);

  // const signaling = useKinesisViewer(videoRef);

  const credentials = useAwsCredentials();
  const kinesisVideoClient = useKinesisVideoClient({ credentials });
  const channelARN = useChannelARN({ kinesisVideoClient });
  const endpoint = useSignalingEndpoints(kinesisVideoClient, channelARN);
  const iceServers  = useIceServers({ credentials, channelARN, endpoint });
  const {  connectToAwsKinesis : connectKVS, disconnectFromAwsKinesis: disconnectKVS  } = useSignalingClient({
    ref: videoRef,
    channelARN,
    credentials,
    endpoint,
    iceServers,
  });

  const connectToAwsKinesis = () => {
    console.log("connectToAwsKinesis");
    connectKVS();
    // signalingClient?.open();
  };

  const disconnectFromAwsKinesis = () => {
    console.log("disconnectFromAwsKinesis");
    disconnectKVS();
    // signalingClient?.close();
  };

  return (
    <div className="w-full h-full bg-gray-100 rounded-lg flex flex-col items-center justify-center ">
      <video className="w-full h-full" playsInline ref={videoRef} />
      <div className="flex flex-row gap-4">
        <button
          className="px-6 py-2 text-lg text-white bg-blue-500 border-0 rounded"
          onClick={connectToAwsKinesis}
        >
          Connect
        </button>
        <button
          className="px-6 py-2 text-lg text-white bg-red-500 border-0 rounded"
          onClick={disconnectFromAwsKinesis}
        >
          Disconnect
        </button>
      </div>
    </div>
  );
};