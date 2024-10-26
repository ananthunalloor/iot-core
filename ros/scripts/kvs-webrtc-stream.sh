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

/home/ros/kvs/amazon-kinesis-video-streams-webrtc-sdk-c/build/samples/kvsWebrtcClientMasterGstSample $AWS_IOT_CORE_THING_NAME video-only-storage rtsp://192.168.0.162:8554