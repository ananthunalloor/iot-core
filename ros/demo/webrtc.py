import boto3
import subprocess
import os

# Define your folder location and thing name
cert_folder_location = os.getenv("CERT_FOLDER_LOCATION", "/path/to/cert/folder/")
thing_name = os.getenv("THING_NAME", "your-thing-name")
iot_kvs_role_alias = os.getenv("IOT_KVS_ROLE_ALIAS", "your-role-alias")
aws_region = os.getenv("AWS_REGION", "your-region")

# Read the endpoint from iot-credential-provider.txt file
with open(f"{cert_folder_location}iot-credential-provider.txt", "r") as file:
    endpoint = file.read().strip()

# Construct file paths for certificates and keys
cert_file = f"{cert_folder_location}{thing_name}.cert.pem"
private_key = f"{cert_folder_location}{thing_name}.private.key"
root_cert_file = f"{cert_folder_location}kvs.cert.pem"

test_gst_command = [
    "gst-launch-1.0",
    "videotestsrc","pattern=ball","is-live=TRUE",
    "!",
    "queue",
    "!",
    "videorate",
    "!",
    "videoscale",
    "!",
    "videoconvert",
    "!",
    "video/x-raw",
    "!",
    "clockoverlay","halignment=right","valignment=top",
    "!",
    "x264enc","name=sampleVideoEncoder","bframes=0","speed-preset=veryfast","bitrate=512","byte-stream=TRUE","tune=zerolatency",
    "!",
    "video/x-h264",
    "!",
    "kvssink",
    f"stream-name={thing_name}",
    f"aws-region={aws_region}",
    f"iot-certificate=iot-certificate,endpoint={endpoint},cert-path={cert_file},key-path={private_key},ca-path={root_cert_file},role-aliases={iot_kvs_role_alias}"
]


def get_signaling_channel_endpoint(channel_name, region_name):
    # Initialize AWS SDK client for Kinesis Video Streams
    kvs_client = boto3.client("kinesisvideo", region_name=region_name)
    
    # Get the signaling endpoint for the specified channel
    response = kvs_client.get_signaling_channel_endpoint(
        ChannelName=channel_name,
        SingleMasterChannelEndpointConfiguration={
            'Protocols': ['WSS', 'HTTPS'],
            'Role': 'MASTER'
        }
    )
    # Return the WSS endpoint for WebRTC signaling
    return next(endpoint['ResourceEndpoint'] for endpoint in response['ResourceEndpointList'] if endpoint['Protocol'] == 'WSS')

def start_webrtc_stream(channel_name, region_name):
    # Get the WebRTC signaling endpoint
    signaling_endpoint = get_signaling_channel_endpoint(channel_name, region_name)
    
    # Configure GStreamer to use the KVS WebRTC plugin
    # gst_command = [
    # "gst-launch-1.0",
    # "v4l2src", "do-timestamp=TRUE", "device=/dev/video0",
    # "!", "jpegdec",
    # "!", "x264enc",
    # "!", "h264parse",
    # "!", "video/x-h264",
    # "!", "kvssink",
    # f"stream-name={thing_name}",
    # f"aws-region={aws_region}",
    #     f"wss-endpoint={signaling_endpoint}",  # WebSocket signaling endpoint for WebRTC
    #     "role=master"  # Set role as MASTER
    #  f"iot-certificate=iot-certificate,cert-path={cert_file},key-path={private_key},ca-path={root_cert_file},role-aliases={iot_kvs_role_alias}"
    # ]
    
    # Start the GStreamer pipeline
    try:
        subprocess.run(test_gst_command, check=True)
    except subprocess.CalledProcessError as e:
        print("Error in GStreamer pipeline:", e)

# Example usage
if __name__ == "__main__":
    # Replace with your KVS channel name and AWS region
    channel_name = "your-channel-name"
    region_name = "us-west-2"
    start_webrtc_stream(channel_name, region_name)
