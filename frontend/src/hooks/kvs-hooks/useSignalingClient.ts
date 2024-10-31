import { useCallback, useEffect, useState } from "react";
import {
  SignalingClient,
  SigV4RequestSigner,
} from "amazon-kinesis-video-streams-webrtc";

import { ResourceEndpoint } from "./useSignalingEndpoints";
import { Uri } from "aws-sdk/clients/kinesisvideosignalingchannels";
import { Role } from "amazon-kinesis-video-streams-webrtc";
import { CredentialsType } from "../use-get-aws-credentials";

export interface useSignalingClientProps {
  ref: React.RefObject<HTMLVideoElement>;
  channelARN: string | null;
  credentials?: CredentialsType;
  endpoint: ResourceEndpoint | null;
  iceServers: Array<{ urls: Uri; username?: string; credential?: string }>;
}

export const useSignalingClient = ({
  ref,
  channelARN,
  credentials,
  endpoint,
  iceServers,
}: useSignalingClientProps) => {
  const [signalingClient, setSignalingClient] =
    useState<SignalingClient | null>(null);
  const [peerConnectionState, setPeerConnection] =
    useState<RTCPeerConnection | null>(null);
    const [connect, setConnection] = useState<boolean>(false);



  useEffect(() => {
    if(!connect) return;
    if (!credentials || !endpoint || !channelARN || !iceServers.length) return;

    if (!credentials?.accessKeyId || !credentials?.secretAccessKey) return;


    if (peerConnectionState) return;
    console.log("Connecting to signaling server...");
    
    const peerConnection = new RTCPeerConnection({
      iceServers,
      iceTransportPolicy: "all",
    });
    const client = new SignalingClient({
      channelARN,
      channelEndpoint: endpoint.WSS,
      clientId: "iot_user",
      role: Role.VIEWER,
      region: import.meta.env.VITE_REGION,
      credentials: {
        accessKeyId: credentials.accessKeyId,
        secretAccessKey: credentials.secretAccessKey,
        sessionToken: credentials.sessionToken,
      },
    });

    client.on("open", async () => {
        if (peerConnection.signalingState === "closed") return;
      const offer = await peerConnection.createOffer({
        offerToReceiveAudio: true,
        offerToReceiveVideo: true,
        // iceRestart: true,
      });
      await peerConnection.setLocalDescription(offer);
      if (peerConnection.localDescription)
        client.sendSdpOffer(peerConnection.localDescription);
    });

    client.on("sdpAnswer", (answer) =>{
        if (!peerConnection.remoteDescription) {
            peerConnection.setRemoteDescription(answer);
          }}
    );

    peerConnection.onicecandidate = (event) => {
      console.log("Local ICE candidate: ", event.candidate);
      if (event.candidate) {
        client.sendIceCandidate(event.candidate);
      
        if (peerConnection.localDescription) {
          client.sendSdpAnswer(peerConnection.localDescription);
        }
      }
    };

    peerConnection.ontrack = ({ streams }) => {
      if (ref.current?.srcObject) {
        return;
      }
      ref.current!.srcObject = streams[0];
      ref.current!.play();
      ref
        .current!.play()
        .catch((error) => console.error("Error playing video:", error));
    };

    peerConnection.removeEventListener("icecandidate", (event) => {
      console.log("Local ICE candidate: ", event.candidate);
      if (event.candidate) {
        client.sendIceCandidate(event.candidate);
      
        if (peerConnection.localDescription) {
          client.sendSdpAnswer(peerConnection.localDescription);
        }
      }
    });

    setPeerConnection(peerConnection);
    setSignalingClient(client);

    return () => {
      console.log("Cleaning up signaling client and peer connection");
      client.close();
    };
  }, [channelARN, credentials, endpoint, iceServers, ref, connect, peerConnectionState, signalingClient]);

  const connectToAwsKinesis = useCallback(() => {
    setConnection(true)
  }, [setConnection]);

  const disconnectFromAwsKinesis =  useCallback(() => {
    setConnection(false)
  }, [setConnection]);

  useEffect(() => {
    if (!connect) {
      signalingClient?.close();

    } else {
      signalingClient?.open();
    }

  }, [connect, signalingClient]);

  return { signalingClient, peerConnection : peerConnectionState, connectToAwsKinesis, disconnectFromAwsKinesis };
};
