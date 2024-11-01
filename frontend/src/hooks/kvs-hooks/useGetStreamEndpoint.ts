import { KinesisVideo, KinesisVideoArchivedMedia } from "aws-sdk";
import { useEffect, useState } from "react";
import { CredentialsType } from "../use-get-aws-credentials";

export interface useGetStreamEndpointsProps {
    kinesisVideoClient: KinesisVideo | null
    credentials?: CredentialsType
}

export const useGetStreamEndpoints = ({ kinesisVideoClient, credentials }: useGetStreamEndpointsProps) => {

    const [endpoint, setEndpoint] = useState<string>();

    useEffect(() => {
        if (!kinesisVideoClient || !credentials) return;

        const fetchEndpoints = async () => {
            try {
                const res = await kinesisVideoClient.getDataEndpoint({
                    APIName: "GET_HLS_STREAMING_SESSION_URL",
                    StreamName: "rover_iot_thing",
                }).promise()

                console.log('Streaming endpoint found', res);

                const archivedMediaClient = new KinesisVideoArchivedMedia({
                    region: import.meta.env.VITE_REGION,
                    accessKeyId: credentials.accessKeyId,
                    secretAccessKey: credentials.secretAccessKey,
                    sessionToken: credentials.sessionToken,
                    endpoint: res.DataEndpoint,
                })

                console.log('Streaming endpoint found ---------', archivedMediaClient);

                const getHLSStreamingSessionURLOptions = {
                    StreamName: 'rover_iot_thing',
                    PlaybackMode: 'LIVE'
                };
                const response = await archivedMediaClient
                    .getHLSStreamingSessionURL(getHLSStreamingSessionURLOptions)
                    .promise()

                console.log('Streaming endpoint found', response);

                const hlsUrl = response.HLSStreamingSessionURL;
                console.log('Streaming endpoint found', hlsUrl);
                setEndpoint(hlsUrl)

            } catch (error) {
                console.log('Streaming endpoint not found', error);
            }
        };

        fetchEndpoints();
    }, [kinesisVideoClient, credentials]);

    return endpoint
}