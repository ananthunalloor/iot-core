import { KinesisVideo, KinesisVideoArchivedMedia } from "aws-sdk";
import { GetDataEndpointOutput } from "aws-sdk/clients/kinesisvideo";
import { useEffect, useState } from "react";

export const useGetStreamEndpoints = (kinesisVideoClient: KinesisVideo | null) => {

    const [endpoint, setEndpoint] = useState<GetDataEndpointOutput | null>(null);

    useEffect(() => {
        if (!kinesisVideoClient) return;

        const fetchEndpoints = async () => {
            try {
                const res = await kinesisVideoClient.getDataEndpoint({
                    APIName: "GET_HLS_STREAMING_SESSION_URL",
                    StreamName: "rover_iot_thing"
                }).promise()

                const archive = new KinesisVideoArchivedMedia({

                })

                const response = await archive.getHLSStreamingSessionURL({
                    StreamName: 'rover_iot_thing',

                }).promise()

                const tt = await kinesisVideoClient.describeStream({ StreamName: 'rover_iot_thing' }).promise()



                console.log('response ', response.HLSStreamingSessionURL, tt.StreamInfo)
                setEndpoint(res)

            } catch (error) {
                console.error(error);
            }

        };

        fetchEndpoints();
    }, [kinesisVideoClient]);

    return endpoint?.DataEndpoint
}