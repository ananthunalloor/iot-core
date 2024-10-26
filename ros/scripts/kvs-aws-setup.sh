#!/bin/bash

echo "Setting up AWS KVS"

#check if iam role already exists
if aws iam get-role --role-name $IAM_KVS_ROLE_NAME > /dev/null 2>&1; then
    echo "IAM role $IAM_KVS_ROLE_NAME already exists"
    aws iam get-role --role-name $IAM_KVS_ROLE_NAME > "$IAM_KVS_CONFIG_FILE"

else
    echo "Creating IAM role $IAM_KVS_ROLE_NAME"
    aws iam create-role --role-name $IAM_KVS_ROLE_NAME --assume-role-policy-document file://"$IAM_KVS_ROLE_POLICY_TEMPLATE" > "$IAM_KVS_CONFIG_FILE"
fi

aws iam put-role-policy --role-name $IAM_KVS_ROLE_NAME --policy-name $IAM_KVS_POLICY_NAME --policy-document file://"$IAM_KVS_POLICY_TEMPLATE"
aws iam put-role-policy --role-name $IAM_KVS_ROLE_NAME --policy-name $IAM_KVS_WEBRTC_POLICY_NAME --policy-document file://"$IAM_KVS_WEBRTC_POLICY_TEMPLATE"

IOT_KVS_ROLE_ARN=$(grep arn "$IAM_KVS_CONFIG_FILE" | cut -d '"' -f 4)

#check if role alias exists in aws
if aws iot describe-role-alias --role-alias $IOT_KVS_ROLE_ALIAS > /dev/null 2>&1; then
    echo "Role alias $IOT_KVS_ROLE_ALIAS already exists"
    aws iot describe-role-alias --role-alias $IOT_KVS_ROLE_ALIAS > "$IOT_KVS_ROLE_ALIAS_CONFIG_FILE"
else
    echo "Creating role alias $IOT_KVS_ROLE_ALIAS"
    aws iot create-role-alias --role-alias $IOT_KVS_ROLE_ALIAS --role-arn $IOT_KVS_ROLE_ARN --credential-duration-seconds 3600 > "$IOT_KVS_ROLE_ALIAS_CONFIG_FILE"
fi

IOT_KVS_ROLE_ALIAS_ARN=$(grep arn "$IOT_KVS_ROLE_ALIAS_CONFIG_FILE" | cut -d '"' -f 4)

truncate -s 0 "$IOT_KVS_POLICY_FILE"
cat "$IOT_KVS_POLICY_TEMPLATE" >> "$IOT_KVS_POLICY_FILE"
sed -i "s#RESOURCE#${IOT_KVS_ROLE_ALIAS_ARN}#g" "$IOT_KVS_POLICY_FILE"

#check if policy exists in aws
POLICY_EXISTS=$(aws iot list-policies | grep -c "\"policyName\": \"$IOT_KVS_POLICY_NAME\"")
if [ "$POLICY_EXISTS" -eq 0 ]; then
    echo "Policy does not exist. Creating it..."
    aws iot create-policy --policy-name "$IOT_KVS_POLICY_NAME" --policy-document file://"$IOT_KVS_POLICY_FILE"
    # aws iot create-policy --policy-name "$IOT_KVS_POLICY_NAME" --policy-document file://"$IOT_KVS_POLICY_FILE"
fi

CERT_ARN=$(aws iot list-certificates --query "certificates[?status=='ACTIVE'].certificateArn" --output text)

aws iot attach-policy --policy-name "$IOT_KVS_POLICY_NAME" --target "$CERT_ARN"

aws iot describe-endpoint --endpoint-type iot:CredentialProvider --output text > ${CERT_FOLDER_LOCATION}iot-credential-provider.txt

# check if kvs stream exists in aws
STREAM_EXISTS=$(aws kinesisvideo list-streams --query "StreamInfoList[?StreamName=='$THING_NAME'].StreamName" --output text)
if [ -z "$STREAM_EXISTS" ]; then
    echo "KVS stream does not exist. Creating it..."
    aws kinesisvideo create-stream --stream-name "$THING_NAME"
fi

# check if kvs signaling channel exists in aws
CHANNEL_EXISTS=$(aws kinesisvideo list-signaling-channels --query "ChannelInfoList[?ChannelName=='$THING_NAME'].ChannelName" --output text)
if [ -z "$CHANNEL_EXISTS" ]; then
    echo "KVS signaling channel does not exist. Creating it..."
    aws kinesisvideo create-signaling-channel --channel-name "$THING_NAME"
fi

curl --silent 'https://www.amazontrust.com/repository/SFSRootCAG2.pem' --output ${CERT_FOLDER_LOCATION}kvs.cert.pem