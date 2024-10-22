#!/bin/bash

echo "Setting up AWS IoT"

CONFIG_FILE="$IOT_CONFIG_FILE"
POLICY_FILE="$IOT_POLICY_FILE"

CERT_FILE="${CERT_FOLDER_LOCATION}${THING_NAME}.cert.pem"
ROOT_CERT_FILE="${CERT_FOLDER_LOCATION}rootCA.crt"

POLICY_EXISTS=$(aws iot list-policies | grep -c "\"policyName\": \"$IOT_POLICY_NAME\"")
CERT_ARN=$(aws iot list-certificates --query "certificates[?status=='ACTIVE'].certificateArn" --output text)

echo "THING_NAME: $THING_NAME"
echo "IOT_CONFIG_FILE: $IOT_CONFIG_FILE"
echo "IOT_POLICY_FILE: $IOT_POLICY_FILE"

echo "CERT_FOLDER_LOCATION: $CERT_FOLDER_LOCATION"
echo "POLICY_EXISTS: $POLICY_EXISTS"
echo "CERT_EXISTS: $CERT_EXISTS"


# Create the thing and get its ARN
THING_ARN=$(aws iot create-thing --thing-name "$THING_NAME" --query thingArn --output text)
export THING_ARN

echo "Creating cert and attaching to thing"
CERT_ARN=$(aws iot create-keys-and-certificate --set-as-active \
    --certificate-pem-outfile "${CERT_FOLDER_LOCATION}${THING_NAME}.cert.pem"  \
    --public-key-outfile "${CERT_FOLDER_LOCATION}${THING_NAME}.public.key" \
    --private-key-outfile "${CERT_FOLDER_LOCATION}${THING_NAME}.private.key" \
    --query certificateArn --output text)
export CERT_ARN

curl https://www.amazontrust.com/repository/AmazonRootCA1.pem -o "$ROOT_CERT_FILE"
CERT_ID=${CERT_ARN#*cert/}
aws iot attach-thing-principal --principal "$CERT_ARN" --thing-name "$THING_NAME"
echo "Cert created and attached to thing"

truncate -s 0 "$IOT_CONFIG_FILE"
ACCOUNT_ID=$(aws sts get-caller-identity --query Account --output text)
ENDPOINT_ADDRESS=$(aws iot describe-endpoint --endpoint-type iot:Data-ATS --query endpointAddress --output text)
PRIV_KEY_LOCATION="${CERT_FOLDER_LOCATION}${THING_NAME}.private.key"
CERT_FILE="${CERT_FOLDER_LOCATION}${THING_NAME}.cert.pem"

cat "$IOT_CONFIG_TEMPLATE" >> "$IOT_CONFIG_FILE"
sed -i -e "s/ENDPOINT/$ENDPOINT_ADDRESS/g" "$IOT_CONFIG_FILE"
sed -i -e "s/ROOTCA/$(echo "$ROOT_CERT_FILE" | sed 's_/_\\/_g')/g" "$IOT_CONFIG_FILE"
sed -i -e "s/PRIVATEKEY/$(echo "$PRIV_KEY_LOCATION" | sed 's_/_\\/_g')/g" "$IOT_CONFIG_FILE"
sed -i -e "s/CERTPATH/$(echo "$CERT_FILE" | sed 's_/_\\/_g')/g" "$IOT_CONFIG_FILE"
sed -i -e "s/CLIENT/$THING_NAME/g" "$IOT_CONFIG_FILE"
sed -i -e "s/PORT/$PORT/g" "$IOT_CONFIG_FILE"
sed -i -e "s/REGION/$AWS_REGION/g" "$IOT_CONFIG_FILE"

cat "$IOT_CONFIG_FILE"
echo "Config file created"

truncate -s 0 "$IOT_POLICY_FILE"
cat "$IOT_POLICY_TEMPLATE" >> "$IOT_POLICY_FILE"
sed -i -e "s/ACCOUNT_ID/$ACCOUNT_ID/g" "$IOT_POLICY_FILE"
sed -i -e "s/CLIENT/$THING_NAME/g" "$IOT_POLICY_FILE"
sed -i -e "s/REGION/$AWS_REGION/g" "$IOT_POLICY_FILE"

cat "$IOT_POLICY_FILE"
echo "Policy file created"
aws iot create-policy --policy-name "$IOT_POLICY_NAME" --policy-document file://"$IOT_POLICY_FILE"
aws iot attach-policy --policy-name "$IOT_POLICY_NAME" --target "$CERT_ARN"

