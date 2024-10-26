import { useEffect, useState } from "react";
import { SignalingClient, SigV4RequestSigner } from "amazon-kinesis-video-streams-webrtc";

import { ResourceEndpoint } from "./useSignalingEndpoints";
import { Uri } from "aws-sdk/clients/kinesisvideosignalingchannels";
import { Role } from 'amazon-kinesis-video-streams-webrtc';
import { CredentialsType } from "../use-get-aws-credentials";

export interface useSignalingClientProps {
    ref: React.RefObject<HTMLVideoElement>;
    channelARN: string | null;
    credentials?: CredentialsType;
    endpoint: ResourceEndpoint | null;
    iceServers: Array<{ urls: Uri; username?: string; credential?: string }>;
}

export const useSignalingClient = (
    { ref, channelARN, credentials, endpoint, iceServers }: useSignalingClientProps
) => {
    const [signalingClient, setSignalingClient] = useState<SignalingClient | null>(null);
    const [peerConnection, setPeerConnection] = useState<RTCPeerConnection | null>(null);

    useEffect(() => {
        if (!credentials || !endpoint || !channelARN || !iceServers.length) return;

        if(!credentials.accessKeyId || !credentials.secretAccessKey) return;

        const peerConnection = new RTCPeerConnection({ iceServers });
        const client = new SignalingClient({
            channelARN,
            channelEndpoint: endpoint.WSS,
            clientId: "iot_user",
            role: Role.VIEWER,
            region: import.meta.env.VITE_REGION,
            credentials :{
                accessKeyId: credentials.accessKeyId,
                secretAccessKey: credentials.secretAccessKey,
                sessionToken: credentials.sessionToken
            },
            requestSigner: new SigV4RequestSigner(import.meta.env.VITE_REGION, {
                accessKeyId: credentials.accessKeyId,
                secretAccessKey: credentials.secretAccessKey,
                sessionToken: credentials.sessionToken
            }),
        });

        client.on("open", async () => {
            const offer = await peerConnection.createOffer({ offerToReceiveAudio: true, offerToReceiveVideo: true });
            await peerConnection.setLocalDescription(offer);
            if(peerConnection.localDescription)
            client.sendSdpOffer(peerConnection.localDescription);
        });

        client.on("sdpAnswer", answer => peerConnection.setRemoteDescription(answer));
        client.on("iceCandidate", candidate => peerConnection.addIceCandidate(candidate));

        peerConnection.onicecandidate = event => {
            if (event.candidate) {
                client.sendIceCandidate(event.candidate);
                if 
                (peerConnection.localDescription) {
                    client.sendSdpAnswer(peerConnection.localDescription);
                }
            }
        };

        peerConnection.addEventListener('track', event => {
            console.log("Remote stream received event");
            if (ref.current?.srcObject) {
                return;
            }
            ref.current!.srcObject = event.streams[0];
            ref.current!.play();
        } )



        peerConnection.ontrack = ({ streams }) => {
            console.log("Remote stream received on track");
            console.log("Remote stream received:", streams[0]);
            if (ref.current?.srcObject) {
                return;
            }
            ref.current!.srcObject = streams[0];
            ref.current!.play();
            ref.current!.play().catch(error => console.error("Error playing video:", error));
        };

        setPeerConnection(peerConnection);
        setSignalingClient(client);
   

        return () => {
            console.log("Cleaning up signaling client and peer connection");
            client.close();
            peerConnection.close();
        };
    }, [channelARN, credentials, endpoint, iceServers, ref]);

    return { signalingClient, peerConnection };
};
