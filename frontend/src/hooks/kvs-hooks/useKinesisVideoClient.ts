import { useEffect, useState } from "react";
import { KinesisVideo } from "aws-sdk";
import { CredentialsType } from "../use-get-aws-credentials";


export interface useKinesisVideoClientProps {
    credentials?: CredentialsType;
}

export const useKinesisVideoClient = ({ credentials }: useKinesisVideoClientProps) => {
    const [kinesisVideoClient, setKinesisVideoClient] = useState<KinesisVideo | null>(null);

    useEffect(() => {
        if (credentials) {
            const client = new KinesisVideo({
                region: import.meta.env.VITE_REGION,
                accessKeyId: credentials.accessKeyId,
                secretAccessKey: credentials.secretAccessKey,
                sessionToken: credentials.sessionToken,
                correctClockSkew: true,
            });
            setKinesisVideoClient(client);
        }
    }, [credentials]);

    return kinesisVideoClient;
};