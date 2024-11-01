import React, { useEffect, useState } from "react";

import {
  CredentialsType,
  useAwsCredentials,
} from "../hooks/use-get-aws-credentials";
import { KinesisVideo } from "aws-sdk";;

export const AWSWebRTCConfig = () => {
  // const cred = useAwsCredentials();

  const [credentials, setCredentials] = useState<CredentialsType | null>(null);
  const [kinesisVideoClient, setKinesisVideoClient] =
    useState<KinesisVideo | null>(null);
  const [channelARN, setChannelARN] = useState<string>();

  // Set credentials when available
  // useEffect(() => {
  //   if (cred) {
  //     setCredentials(cred);
  //   }
  // }, [cred]);

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

  // // Fetch channel ARN once Kinesis Video client is available
  useEffect(() => {
    if (!kinesisVideoClient) return;

    const fetchChannelARN = async () => {
      try {
        const describeSignalingChannelResponse = await kinesisVideoClient
          .describeSignalingChannel({
            ChannelName: "rover_iot_thing",
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


  const config = React.useMemo(() => {
    if (!credentials) return null;

    if (!channelARN) return null;
    return {
      credentials: {
        accessKeyId: credentials.accessKeyId,
        secretAccessKey: credentials.secretAccessKey,
        sessionToken: credentials.sessionToken,
      },
      channelARN,
      region: import.meta.env.VITE_REGION as string,
      media: {
        audio: true,
        video: true,
      },
    };
  }, [channelARN, credentials]);
  return config;
};
