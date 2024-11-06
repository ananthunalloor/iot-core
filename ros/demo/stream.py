import subprocess
import sys
import os

# Define your folder location and thing name
cert_folder_location = os.getenv("CERT_FOLDER_LOCATION", "../certs")
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



def start_video_stream():

    gst_command = [
    "gst-launch-1.0",
    "v4l2src", "do-timestamp=TRUE", "device=/dev/video0",
    "!", "jpegdec",
    "!", "queue",
    "!", "videorate",
    "!", "videoscale",
    "!", "videoconvert",
    "!", "video/x-raw,format=I420,width=640,height=480,framerate=25/1",
    "!", "x264enc","name=sampleVideoEncoder","bframes=0","speed-preset=veryfast","bitrate=512","byte-stream=TRUE","tune=zerolatency",
    "!", "h264parse",
    "!", "video/x-h264",
    "!", "kvssink",
    f"stream-name={thing_name}",
    f"aws-region={aws_region}",
    f"iot-certificate=iot-certificate,endpoint={endpoint},cert-path={cert_file},key-path={private_key},ca-path={root_cert_file},role-aliases={iot_kvs_role_alias}"
]
    
    # Execute the GStreamer pipeline
    try:
        subprocess.run(gst_command, check=True)
    except subprocess.CalledProcessError as e:
        print("Error in GStreamer pipeline:", e, file=sys.stderr)

# Example usage
if __name__ == "__main__":
    start_video_stream()


# senderPipeline = gst_parse_launch(
#     "v4l2src do-timestamp=TRUE device=/dev/video0 ! jpegdec ! queue ! "
#     "videorate ! videoscale ! videoconvert ! video/x-raw,format=I420,width=640,height=480,framerate=25/1 ! "
#     "x264enc name=sampleVideoEncoder bframes=0 speed-preset=veryfast bitrate=512 byte-stream=TRUE tune=zerolatency ! h264parse ! "
#     "video/x-h264,stream-format=byte-stream,alignment=au,profile=baseline ! appsink sync=TRUE emit-signals=TRUE "
#     "name=appsink-video autoaudiosrc ! "
#     "queue leaky=2 max-size-buffers=400 ! audioconvert ! audioresample ! opusenc name=sampleAudioEncoder ! "
#     "audio/x-opus,rate=48000,channels=2 ! appsink sync=TRUE emit-signals=TRUE name=appsink-audio",
#     &error);