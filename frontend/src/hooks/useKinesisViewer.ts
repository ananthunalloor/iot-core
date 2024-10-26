import { RefObject, useEffect, useState } from "react";
import { CredentialsType, useAwsCredentials } from "./use-get-aws-credentials";
import { KinesisVideo, KinesisVideoSignalingChannels } from "aws-sdk";
import { ChannelProtocol, ChannelRole, ResourceEndpointListItem } from "aws-sdk/clients/kinesisvideo";
import { Uri } from "aws-sdk/clients/kinesisvideosignalingchannels";
import { Role, SignalingClient, SigV4RequestSigner } from 'amazon-kinesis-video-streams-webrtc';

export type ResourceEndpoint = {
    "HTTPS": string
    "WSS": string
}
export const useKinesisViewer = (ref: RefObject<HTMLVideoElement>) => {
    const cred = useAwsCredentials();

    const [credentials, setCredentials] = useState<CredentialsType | null>(null);
    const [kinesisVideoClient, setKinesisVideoClient] =
        useState<KinesisVideo | null>(null);
    const [channelARN, setChannelARN] = useState<string | null>(null);
    const [endpoint, setEndpoint] = useState<ResourceEndpoint | null>(null);
    const [iceServers, setIceServers] = useState<{ urls: Uri, username?: string, credential?: string }[]>([
        { urls: `stun:stun.kinesisvideo.${import.meta.env.VITE_REGION}.amazonaws.com:443` },
    ]);

    const [signalingClient, setSignalingClient] = useState<SignalingClient | null>(null);



    // Set credentials when available
    useEffect(() => {
        if (cred) {
            setCredentials(cred);
        }
    }, [cred]);

    // Initialize Kinesis Video client when credentials are set
    useEffect(() => {
        if (credentials) {
            setKinesisVideoClient(
                new KinesisVideo({
                    region: import.meta.env.VITE_REGION,
                    accessKeyId: credentials.accessKeyId,
                    secretAccessKey: credentials.secretAccessKey,
                    sessionToken: credentials.sessionToken,
                    correctClockSkew: true,
                })
            );
        }
    }, [credentials]);

    // Fetch channel ARN once Kinesis Video client is available
    useEffect(() => {
        if (!kinesisVideoClient) return;

        const fetchChannelARN = async () => {
            try {
                const describeSignalingChannelResponse = await kinesisVideoClient
                    .describeSignalingChannel({
                        ChannelName: "test_ros2_thing",
                    })
                    .promise();

                const arn = describeSignalingChannelResponse?.ChannelInfo?.ChannelARN;
                if (arn) {
                    setChannelARN(arn);
                }
            } catch (error) {
                console.error("Error fetching channel ARN:", error);
            }
        };

        fetchChannelARN();
    }, [kinesisVideoClient]);


    useEffect(() => {
        if (!kinesisVideoClient || !channelARN) return;

        const getSignalingChannelEndpointResponse = async () => {

            const getSignalingChannelEndpointResponse = await kinesisVideoClient
                .getSignalingChannelEndpoint({
                    ChannelARN: channelARN,
                    SingleMasterChannelEndpointConfiguration: {
                        Protocols: ['WSS', 'HTTPS'] as ChannelProtocol[],
                        Role: 'VIEWER' as ChannelRole,
                    },
                })
                .promise();



            const endpointsByProtocol = getSignalingChannelEndpointResponse.ResourceEndpointList?.reduce((endpoints: ResourceEndpoint,
                endpoint: ResourceEndpointListItem) => {
                if (!endpoint.Protocol || !endpoint.ResourceEndpoint) return endpoints;

                endpoints[endpoint.Protocol as "WSS" | "HTTPS"] = endpoint.ResourceEndpoint;
                return endpoints;
            }, <ResourceEndpoint>{});

            if (endpointsByProtocol) {
                const endpoint = endpointsByProtocol.WSS || endpointsByProtocol.HTTPS;
                if (endpoint) {
                    console.log("Signaling endpoint:", endpoint);
                    console.log("Signaling protocol:", endpointsByProtocol);
                    setEndpoint(endpointsByProtocol as ResourceEndpoint);
                }
            }

        };
        getSignalingChannelEndpointResponse();
    }, [kinesisVideoClient, channelARN, setEndpoint]);


    useEffect(() => {
        if (!kinesisVideoClient || !channelARN || !endpoint || !credentials) return;

        const kinesisVideoSignalingChannelsClient = new KinesisVideoSignalingChannels({
            region: import.meta.env.VITE_REGION,
            accessKeyId: credentials.accessKeyId,
            secretAccessKey: credentials.secretAccessKey,
            sessionToken: credentials.sessionToken,
            endpoint: endpoint.HTTPS,
            correctClockSkew: true,
        });

        const getIceServerConfigResponse = async () => {


            const getIceServerConfigResponse = await kinesisVideoSignalingChannelsClient
                .getIceServerConfig({
                    ChannelARN: channelARN,
                })
                .promise();

            getIceServerConfigResponse.IceServerList?.forEach(iceServer => {
                setIceServers(prev => {
                    return [...prev, {
                        urls: (iceServer.Uris || '') as Uri,
                        username: iceServer.Username,
                        credential: iceServer.Password,
                    }]
                })
                return iceServer
            }
            );

        }

        getIceServerConfigResponse();


    }, [kinesisVideoClient, channelARN, endpoint, credentials]);


    useEffect(() => {
        if (!iceServers || !kinesisVideoClient || !channelARN || !endpoint || !credentials?.accessKeyId || !credentials?.secretAccessKey) return;
        const peerConnection = new RTCPeerConnection({ iceServers, iceTransportPolicy: "all", });

        const signalingClient = new SignalingClient({
            channelARN,
            channelEndpoint: endpoint.WSS,
            clientId: 'test_ros2_thing',
            role: Role.VIEWER,
            region: import.meta.env.VITE_REGION,
            credentials: {
                accessKeyId: credentials.accessKeyId,
                secretAccessKey: credentials.secretAccessKey,
                sessionToken: credentials.sessionToken
            },
            requestSigner: {
                getSignedURL: async function (signalingEndpoint, queryParams, date) {

                    const signer = new SigV4RequestSigner(
                        import.meta.env.VITE_REGION, {
                        accessKeyId: credentials.accessKeyId || '',
                        secretAccessKey: credentials.secretAccessKey || '',
                        sessionToken: credentials.sessionToken
                    });
                    const retVal = await signer.getSignedURL(signalingEndpoint, queryParams, date);

                    return retVal;
                },
            },
            systemClockOffset: kinesisVideoClient.config.systemClockOffset,
        });

        const cleanup = () => {
            signalingClient.close();
            peerConnection.close();
        };

        signalingClient.on('open', async () => {
            console.log("Signaling client opened");
            try {

                await peerConnection.setLocalDescription(
                    await peerConnection.createOffer({
                        offerToReceiveAudio: true,
                        offerToReceiveVideo: true,
                    }),
                );
                console.log("Offer", peerConnection.localDescription);
                if (peerConnection.localDescription)
                    signalingClient.sendSdpOffer(peerConnection.localDescription);
            } catch (error) {
                console.error("Error creating offer:", error);
            }
        });

        signalingClient.on('sdpAnswer', async answer => {
            console.log("Answer", answer);
            await peerConnection.setRemoteDescription(answer);
        });

        // When an ICE candidate is received from the master, add it to the peer connection.
        signalingClient.on('iceCandidate', candidate => {

            console.log("Candidate", candidate);
            peerConnection.addIceCandidate(candidate);
        });

        // signalingClient.on('close', () => {
        //     console.log("Signaling client closed");
        //     // Handle client closures
        // });

        // signalingClient.on('error', error => {
        //     console.error("Signaling client error:", error);
        //     // Handle client errors
        // });

        peerConnection.addEventListener('icecandidate', ({ candidate }) => {
            console.log("Candidate", candidate);
            if (candidate) {
                signalingClient.sendIceCandidate(candidate);
                if (peerConnection.localDescription)
                    signalingClient.sendSdpOffer(peerConnection.localDescription)
            }
        });

        // peerConnection.onicegatheringstatechange = (event) => {
        //     if (peerConnection.iceGatheringState === 'gathering') {
        //         console.log("iceGatheringState", peerConnection.iceGatheringState);
        //     } else if (peerConnection.iceGatheringState === 'complete') {
        //         console.log("iceGatheringState", peerConnection.iceGatheringState);
        //     }
        // }

        peerConnection.onconnectionstatechange = (event) => {
            if (peerConnection.connectionState === 'new' || peerConnection.connectionState === 'connecting') {
                console.log("connectionState", peerConnection.connectionState);
            }
            if (peerConnection.connectionState === 'connected') {
                console.log("connectionState", peerConnection.connectionState);
            }
        };

        // peerConnection.oniceconnectionstatechange = (event) => {
        //     if (peerConnection.iceConnectionState === 'connected') {
        //         peerConnection.getStats().then(stats => {
        //             stats.forEach(report => {
        //                 if (report.type === 'candidate-pair') {
        //                     console.log("candidate-pair", report);
        //                 }
        //             });
        //         });
        //     }
        // };

        peerConnection.ondatachannel = event => {
            // Callback for the data channel created by master
            event.channel.onmessage = event => {
                console.log("Data channel message received:", event.data);
            }
        };

        // As remote tracks are received, add them to the remote view
        peerConnection.addEventListener('track', event => {
            console.log("Remote stream received:", event.streams[0]);
            if (ref.current?.srcObject) {
                return;
            }
            ref.current!.srcObject = event.streams[0];
            ref.current!.play();
            

        });

        setSignalingClient(signalingClient);
        // signalingClient.open();

        return cleanup

    }, [channelARN, credentials, endpoint, iceServers, kinesisVideoClient, ref]);



    return signalingClient
}
