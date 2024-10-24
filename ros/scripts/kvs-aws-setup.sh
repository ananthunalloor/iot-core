#!/bin/bash

echo "Setting up AWS KVS"

aws iam create-role --role-name $IAM_KVS_ROLE_NAME --assume-role-policy-document file://"$IAM_KVS_ROLE_POLICY_TEMPLATE" > "$IAM_KVS_CONFIG_FILE"

aws iam put-role-policy --role-name $IAM_KVS_ROLE_NAME --policy-name $IAM_KVS_POLICY_NAME --policy-document file://"$IAM_KVS_POLICY_TEMPLATE"
aws iam put-role-policy --role-name $IAM_KVS_ROLE_NAME --policy-name $IAM_KVS_WEBRTC_POLICY_NAME --policy-document file://"$IAM_KVS_WEBRTC_POLICY_TEMPLATE"


IOT_KVS_ROLE_ARN=$(grep arn "$IAM_KVS_CONFIG_FILE" | cut -d '"' -f 4)

aws iot create-role-alias --role-alias $IOT_KVS_ROLE_ALIAS --role-arn $IOT_KVS_ROLE_ARN --credential-duration-seconds 3600 > "$IOT_KVS_ROLE_ALIAS_CONFIG_FILE"

IOT_KVS_ROLE_ALIAS_ARN=$(grep arn "$IOT_KVS_ROLE_ALIAS_CONFIG_FILE" | cut -d '"' -f 4)

truncate -s 0 "$IOT_KVS_POLICY_FILE"
cat "$IOT_KVS_POLICY_TEMPLATE" >> "$IOT_KVS_POLICY_FILE"
sed -i "s#RESOURCE#${IOT_KVS_ROLE_ALIAS_ARN}#g" "$IOT_KVS_POLICY_FILE"

aws iot create-policy --policy-name $IOT_KVS_POLICY_NAME --policy-document file://"$IOT_KVS_POLICY_FILE"

CERT_ARN=$(aws iot list-certificates --query "certificates[?status=='ACTIVE'].certificateArn" --output text)

aws iot attach-policy --policy-name "$IOT_KVS_POLICY_NAME" --target "$CERT_ARN"


#in aws create a kvs stream
aws kinesis-video-archived-media create-stream --stream-name "$THING_NAME"

# in aws create a kvs signaling channel
aws kinesis-video-archived-media create-signaling-channel --channel-name "$THING_NAME"