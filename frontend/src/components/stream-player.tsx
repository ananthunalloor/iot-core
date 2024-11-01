import ReactPlayer from 'react-player'
import { useAwsCredentials } from '../hooks/use-get-aws-credentials';
import { useKinesisVideoClient } from '../hooks/kvs-hooks/useKinesisVideoClient';
import { useGetStreamEndpoints } from '../hooks/kvs-hooks/useGetStreamEndpoint';
import { useCallback, useEffect, useState } from 'react';
export const StreamPlayer = () => {

    const [streamEndpoint, setStreamEndpoint] = useState<string | null>(null);

    const credentials = useAwsCredentials();
    const kinesisVideoClient = useKinesisVideoClient({ credentials })
    const endpoint = useGetStreamEndpoints(kinesisVideoClient);


    const connectToStream = useCallback(() => {
        console.log("connectToStream");
        if (!endpoint) return;

        setStreamEndpoint(`${endpoint}/hls/v1/getHLSMasterPlaylist.m3u8?SessionToken=${credentials?.sessionToken}`);
        console.log(credentials?.sessionToken);
    }, [credentials?.sessionToken, endpoint])


    const disconnectFromStream = useCallback(() => {
        setStreamEndpoint(null)
    }, [])



    return <div className="flex flex-col items-center justify-center w-full h-full gap-4 bg-gray-100 rounded-lg ">
        <ReactPlayer controls url={streamEndpoint || ''} width="100%" height="100%" muted />
        <div className="flex flex-row gap-4">
            <button
                className="px-6 py-2 text-lg text-white bg-blue-500 border-0 rounded"
                onClick={connectToStream}
            >
                Connect to Stream
            </button>
            <button
                className="px-6 py-2 text-lg text-white bg-red-500 border-0 rounded"
                onClick={disconnectFromStream}
            >
                Disconnect from Stream
            </button>
        </div>
    </div>;
}