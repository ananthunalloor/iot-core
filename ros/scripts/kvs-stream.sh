#!/bin/bash

export ENDPOINT=$(cat ${CERT_FOLDER_LOCATION}iot-credential-provider.txt)

CERT_FILE="${CERT_FOLDER_LOCATION}${THING_NAME}.cert.pem"
PRIV="${CERT_FOLDER_LOCATION}${THING_NAME}.private.key"

ROOT_CERT_FILE="${CERT_FOLDER_LOCATION}kvs.cert.pem"


export AWS_IOT_CORE_THING_NAME="${THING_NAME}"
export AWS_REGION="${AWS_REGION}"
export AWS_IOT_CORE_CREDENTIAL_ENDPOINT="${ENDPOINT}"
export AWS_IOT_CORE_CERT="${CERT_FILE}"
export AWS_IOT_CORE_PRIVATE_KEY="${PRIV}"
export AWS_IOT_CORE_ROLE_ALIAS="$IOT_KVS_ROLE_ALIAS"
export AWS_IOT_CORE_CA_PATH="${ROOT_CERT_FILE}"

echo "Starting KVS stream"
# orginal
# gst-launch-1.0 nvarguscamerasrc do-timestamp=TRUE ! \
#     nvv4l2h264enc ! \
#     h264parse ! \
#     video/x-h264,stream-format=avc,alignment=au,profile=baseline,width=1280,height=720,framerate=25/1 ! \
#     kvssink \
#         stream-name="$AWS_IOT_CORE_THING_NAME" \
#         aws-region="$AWS_REGION" \
#         iot-certificate=" \
#             iot-certificate, \
#             endpoint=$AWS_IOT_CORE_CREDENTIAL_ENDPOINT, \
#             cert-path=$AWS_IOT_CORE_CERT, \
#             key-path=$AWS_IOT_CORE_PRIVATE_KEY, \
#             ca-path=$AWS_IOT_CORE_CA_PATH, \
#             role-aliases=$AWS_IOT_CORE_ROLE_ALIAS"

# working
# gst-launch-1.0 -v videotestsrc do-timestamp=TRUE ! video/x-raw,width=640,height=480 ! x264enc ! h264parse ! kvssink \
#     stream-name="$THING_NAME" \
#     aws-region="$AWS_REGION" \
#     iot-certificate=" \
#             iot-certificate, \
#             endpoint=$AWS_IOT_CORE_CREDENTIAL_ENDPOINT, \
#             cert-path=$AWS_IOT_CORE_CERT, \
#             key-path=$AWS_IOT_CORE_PRIVATE_KEY, \
#             ca-path=$AWS_IOT_CORE_CA_PATH, \
#             role-aliases=$AWS_IOT_CORE_ROLE_ALIAS"
# working
# gst-launch-1.0 -v videotestsrc do-timestamp=TRUE ! video/x-raw,width=640,height=480 ! x264enc ! h264parse ! kvssink \
#     stream-name="$THING_NAME" \
#     aws-region="$AWS_REGION" \
#     iot-certificate=" \
#             iot-certificate, \
#             endpoint=$AWS_IOT_CORE_CREDENTIAL_ENDPOINT, \
#             cert-path=$AWS_IOT_CORE_CERT, \
#             key-path=$AWS_IOT_CORE_PRIVATE_KEY, \
#             ca-path=$AWS_IOT_CORE_CA_PATH, \
#             role-aliases=$AWS_IOT_CORE_ROLE_ALIAS"

gst-launch-1.0 -v rtspsrc location="rtsp://192.168.100.39:8554/test" short-header=TRUE ! rtph264depay ! h264parse ! kvssink \
    stream-name="$THING_NAME" \
    aws-region="$AWS_REGION" \
    iot-certificate=" \
            iot-certificate, \
            endpoint=$AWS_IOT_CORE_CREDENTIAL_ENDPOINT, \
            cert-path=$AWS_IOT_CORE_CERT, \
            key-path=$AWS_IOT_CORE_PRIVATE_KEY, \
            ca-path=$AWS_IOT_CORE_CA_PATH, \
            role-aliases=$AWS_IOT_CORE_ROLE_ALIAS"

# gst-launch-1.0 v4l2src do-timestamp=TRUE device=/dev/video0 ! videoconvert ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! x264enc  bframes=0 key-int-max=45 bitrate=500 ! video/x-h264,stream-format=avc,alignment=au,profile=baseline ! kvssink stream-name="YourStreamName" storage-size=512 access-key="YourAccessKey" secret-key="YourSecretKey" aws-region="YourAWSRegion"

# gst-launch-1.0 -v autovideosrc ! queue ! videoconvert ! video/x-raw,width=640,height=480 ! x264enc bitrate=368640 tune=zerolatency ! h264parse ! kvssink \
#     stream-name="$AWS_IOT_CORE_THING_NAME" \
#     aws-region="$AWS_REGION" \
#     iot-certificate=" \
#             iot-certificate, \
#             endpoint=$AWS_IOT_CORE_CREDENTIAL_ENDPOINT, \
#             cert-path=$AWS_IOT_CORE_CERT, \
#             key-path=$AWS_IOT_CORE_PRIVATE_KEY, \
#             ca-path=$AWS_IOT_CORE_CA_PATH, \
#             role-aliases=$AWS_IOT_CORE_ROLE_ALIAS"