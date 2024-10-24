#!/bin/bash

ENDPOINT=$(aws iot describe-endpoint --endpoint-type iot:Data-ATS --query endpointAddress --output text)
echo "ENDPOINT: $ENDPOINT"

ROOT_KVS_CERT="${CERT_FOLDER_LOCATION}rootCA.pem"

# check if .pem file exists
if [ -f "$ROOT_KVS_CERT" ]; then
    echo "$ROOT_KVS_CERT exists."
else
    # copy root CA cert and create a new file with .pem
    cp "$ROOT_CERT_FILE" "${CERT_FOLDER_LOCATION}rootCA.pem"
    echo "$ROOT_KVS_CERT created."
fi



ROOT_CERT_FILE="${CERT_FOLDER_LOCATION}rootCA.crt"
CERT_FILE="${CERT_FOLDER_LOCATION}${THING_NAME}.cert.pem"
PRIV="${CERT_FOLDER_LOCATION}${THING_NAME}.private.key"


export AWS_IOT_CORE_CREDENTIAL_ENDPOINT=ENDPOINT
export AWS_IOT_CORE_CERT=CERT
export AWS_IOT_CORE_PRIVATE_KEY=PRIV
export AWS_IOT_CORE_ROLE_ALIAS="$IOT_KVS_ROLE_ALIAS"
export AWS_IOT_CORE_THING_NAME="$THING_NAME"
./build/samples/kvsWebrtcClientMasterGstSample $AWS_IOT_CORE_THING_NAME video-only-storage nvargussrc