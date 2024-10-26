import { useEffect, useState } from "react";
import { KinesisVideoSignalingChannels } from "aws-sdk";
import { Uri } from "aws-sdk/clients/kinesisvideosignalingchannels";
import { ResourceEndpoint } from "./useSignalingEndpoints";
import { CredentialsType } from "../use-get-aws-credentials";


export interface useIceServersProps {
    credentials?: CredentialsType;
    channelARN: string | null;
    endpoint: ResourceEndpoint | null;
}

export const useIceServers = ({ credentials, channelARN, endpoint }: useIceServersProps) => {
    const [iceServers, setIceServers] = useState<Array<{ urls: Uri; username?: string; credential?: string }>>([
       
    ]);

    useEffect(() => {
        if (!credentials || !channelARN || !endpoint) return;

        const client = new KinesisVideoSignalingChannels({
            region: import.meta.env.VITE_REGION,
            accessKeyId: credentials.accessKeyId,
            secretAccessKey: credentials.secretAccessKey,
            sessionToken: credentials.sessionToken,
            endpoint: endpoint.HTTPS,
            correctClockSkew: true,
        });

        const fetchIceServers = async () => {
            try {
                const response = await client.getIceServerConfig({ ChannelARN: channelARN }).promise();
                const servers = response.IceServerList?.map(server => ({
                    urls: server.Uris || '',
                    username: server.Username,
                    credential: server.Password,
                })) as Array<{ urls: Uri; username: string | undefined; credential: string | undefined }>;
                if (servers) setIceServers(prev => [...prev, ...servers]);
            } catch (error) {
                console.error("Error fetching ICE servers:", error);
            }
        };

        fetchIceServers();
    }, [credentials, channelARN, endpoint]);

    return iceServers;
};
