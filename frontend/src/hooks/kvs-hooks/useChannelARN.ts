import { useEffect, useState } from "react";
import { KinesisVideo } from "aws-sdk";


export interface useChannelARNProps {
    kinesisVideoClient: KinesisVideo | null;
}

export const useChannelARN = ({ kinesisVideoClient }: useChannelARNProps) => {
    const [channelARN, setChannelARN] = useState<string | null>(null);

    useEffect(() => {
        if (!kinesisVideoClient) return;

        const fetchChannelARN = async () => {
            try {
                const response = await kinesisVideoClient.describeSignalingChannel({ ChannelName: "test_ros2_thing" }).promise();
                const arn = response.ChannelInfo?.ChannelARN;
                if (arn) setChannelARN(arn);
            } catch (error) {
                console.error("Error fetching channel ARN:", error);
            }
        };

        fetchChannelARN();
    }, [kinesisVideoClient]);

    return channelARN;
};