import { useEffect, useState } from "react";
import { KinesisVideo } from "aws-sdk";
import { ChannelProtocol, ChannelRole } from "aws-sdk/clients/kinesisvideo";

export type ResourceEndpoint = { HTTPS: string; WSS: string };

export const useSignalingEndpoints = (kinesisVideoClient: KinesisVideo | null, channelARN: string | null) => {
    const [endpoint, setEndpoint] = useState<ResourceEndpoint | null>(null);

    useEffect(() => {
        if (!kinesisVideoClient || !channelARN) return;

        const fetchEndpoints = async () => {
            try {
                const response = await kinesisVideoClient.getSignalingChannelEndpoint({
                    ChannelARN: channelARN,
                    SingleMasterChannelEndpointConfiguration: {
                        Protocols: ["WSS", "HTTPS"] as ChannelProtocol[],
                        Role: "VIEWER" as ChannelRole,
                    },
                }).promise();

                const endpoints = response.ResourceEndpointList?.reduce((acc: ResourceEndpoint, item) => {
                    if (item.Protocol && item.ResourceEndpoint) {
                        acc[item.Protocol as "WSS" | "HTTPS"] = item.ResourceEndpoint;
                    }
                    return acc as ResourceEndpoint;
                }, { HTTPS: "", WSS: "" } as ResourceEndpoint);

                if (endpoints) 
                setEndpoint(endpoints);
            } catch (error) {
                console.error("Error fetching signaling endpoints:", error);
            }
        };

        fetchEndpoints();
    }, [kinesisVideoClient, channelARN]);

    return endpoint;
};
