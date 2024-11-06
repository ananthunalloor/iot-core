import os
import cv2
import boto3
import subprocess
from botocore.config import Config


cert_folder_location = os.getenv("CERT_FOLDER_LOCATION", "../cert/")
thing_name = os.getenv("THING_NAME", "your-thing-name")
iot_kvs_role_alias = os.getenv("IOT_KVS_ROLE_ALIAS", "your-role-alias")
aws_region = os.getenv("AWS_REGION", "your-region")

# Read the endpoint from iot-credential-provider.txt file
print(f"cert_folder_location: {cert_folder_location}")
with open(f"{cert_folder_location}iot-credential-provider.txt", "r") as file:
    endpoint = file.read().strip()

# Construct file paths for certificates and keys
cert_file = f"{cert_folder_location}{thing_name}.cert.pem"
private_key = f"{cert_folder_location}{thing_name}.private.key"
root_cert_file = f"{cert_folder_location}kvs.cert.pem"


my_config = Config(
    region_name = aws_region
)

kvs_client = boto3.client('kinesisvideo', config=my_config)

# Create the signaling channel (WebRTC)
kvs_signaling = kvs_client.create_signaling_channel(
    ChannelName=thing_name,
    Type='SINGLE_MASTER')

channel_arn = kvs_signaling['ChannelARN']

channel_info = kvs_client.get_signaling_channel_endpoint(
    ChannelName=thing_name,
    SingleMasterChannelEndpointConfiguration={
        'Protocols': ['WSS', 'HTTPS'],
        'Role': 'MASTER'
    }
)

# Initialize webcam
cap = cv2.VideoCapture(0)

gst_str = f"appsrc ! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency ! \
    rtph264pay config-interval=1 pt=96 ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! \
    webrtcbin name=sendrecv"

pipeline = subprocess.Popen(gst_str, shell=True)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Stream to KVS using WebRTC
    process_frame(frame)

cap.release()
pipeline.terminate()
