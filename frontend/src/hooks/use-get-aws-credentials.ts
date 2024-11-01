import AWS from "aws-sdk";
import { useEffect, useState } from "react";

export type CredentialsType = {
  accessKeyId?: string;
  secretAccessKey?: string;
  sessionToken?: string;
}

export const useAwsCredentials = () => {
  const [credentials, setCredentials] = useState<CredentialsType>();

  useEffect(() => {
    if (credentials?.accessKeyId) return;
    const getTemporaryCredentials = async () => {
      const sts = new AWS.STS();
      const params = {
        RoleArn: import.meta.env.VITE_ROLE_ARN,
        RoleSessionName: 'mqtt-session',
      };

      try {
        const data = await sts.assumeRole(params).promise();

        if (!data?.Credentials) {
          console.error('Error obtaining AWS credentials');
          setCredentials(undefined);
          return;
        }

        setCredentials({
          accessKeyId: data?.Credentials?.AccessKeyId,
          secretAccessKey: data?.Credentials?.SecretAccessKey,
          sessionToken: data?.Credentials?.SessionToken,
        });
        console.log('AWS credentials obtained successfully');
      } catch (err) {
        console.error('Error assuming role:', err);
        setCredentials(undefined);
      }
    };

    AWS.config.update({
      region: import.meta.env.VITE_REGION,
      accessKeyId: import.meta.env.VITE_ACCESS_KEY_ID,
      secretAccessKey: import.meta.env.VITE_SECRET_ACCESS_KEY,
    });

    getTemporaryCredentials();
  }, [credentials]);

  return credentials;
};