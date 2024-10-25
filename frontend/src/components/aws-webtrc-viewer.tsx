import { useEffect, useRef } from "react";
import { useViewer } from "react-kinesis-webrtc";

export interface AWSWebRTCViewerProps {
    config: {
        credentials: {
            accessKeyId: string | undefined;
            secretAccessKey: string | undefined;
            sessionToken: string | undefined;
        };
        channelARN: string;
        region: string;
        media: {
            audio: boolean;
            video: boolean;
        };
    }
}

export const AWSWebRTCViewer = (config: AWSWebRTCViewerProps) => {

    console.log(config);

    const videoRef = useRef<HTMLVideoElement>(null);

    const { error, peer: { media, } = {} } = useViewer(config.config);


    useEffect(() => {
        if (videoRef.current) {
            videoRef.current.srcObject = media;
        }
    }, [media]);
    useEffect(() => {
        console.log('media', media, error);
    }, [media, error]);

    if (error) {
        return <p>An error occurred: {error.message}</p>;
    }


    return <video height='100%' width='100%' autoPlay ref={videoRef} />;
};
