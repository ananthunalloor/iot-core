# pull the latest ros2 images
docker pull osrf/ros:jazzy-desktop
# run and start session
docker run -it osrf/ros:jazzy-desktop


# insted use docker compose file

# build docker compose file
docker-compose build

# start docker compose in background
docker-compose up -d

# to start a shell in ros2 container
docker-compose exec ros2 bash

# to test ros2 shell
ros2 --help

# to exit ros2 shell
exit

# stop docker compose 
docker-compose down

# if there is permissions issue in writing to file, grant permissions
sudo chown -R $USER:$USER /ros2_ws


# create an AWS IoT thing
export THING_ARN=$(aws iot create-thing --thing-name $THING_NAME --query thingArn --output text)
export CERT_ARN=$(aws iot create-keys-and-certificate --set-as-active \
--certificate-pem-outfile ${CERT_FOLDER_LOCATION}${THING_NAME}.cert.pem  \
--public-key-outfile ${CERT_FOLDER_LOCATION}${THING_NAME}.public.key \
--private-key-outfile ${CERT_FOLDER_LOCATION}${THING_NAME}.private.key \
--query certificateArn --output text)

ROOT_CERT_FILE=$CERT_FOLDER_LOCATION"rootCA".crt
curl https://www.amazontrust.com/repository/AmazonRootCA1.pem > $ROOT_CERT_FILE
export CERT_ID=${CERT_ARN#*cert/}
aws iot attach-thing-principal --principal $CERT_ARN --thing-name $THING_NAME


export ACCOUNT_ID=$(aws sts get-caller-identity --query Account --output text)
export ENDPOINT_ADDRESS=$(aws iot describe-endpoint --endpoint-type iot:Data-ATS --query endpointAddress  --output text)
export PORT=8883
cat $IOT_CONFIG_TEMPLATE >> $IOT_CONFIG_FILE
export PRIV_KEY_LOCATION=$CERT_FOLDER_LOCATION$THING_NAME.private.key
export CERT_FILE=$CERT_FOLDER_LOCATION$THING_NAME.cert.pem
sed -i -e "s/ENDPOINT/$ENDPOINT_ADDRESS/g" $IOT_CONFIG_FILE
sed -i -e "s/ROOTCA/$(echo $ROOT_CERT_FILE | sed 's_/_\\/_g')/g" $IOT_CONFIG_FILE
sed -i -e "s/PRIVATEKEY/$(echo $PRIV_KEY_LOCATION | sed 's_/_\\/_g')/g" $IOT_CONFIG_FILE
sed -i -e "s/CERTPATH/$(echo $CERT_FILE | sed 's_/_\\/_g')/g" $IOT_CONFIG_FILE
sed -i -e "s/CLIENT/$THING_NAME/g" $IOT_CONFIG_FILE
sed -i -e "s/PORT/$PORT/g" $IOT_CONFIG_FILE
sed -i -e "s/REGION/$AWS_REGION/g" $IOT_CONFIG_FILE
cat $IOT_CONFIG_FILE


export IOT_POLICY_FILE=/root/aws-iot-robot-connectivity-samples-ros2/iot_certs_and_config/iot_policy.json
cat $IOT_POLICY_TEMPLATE >> $IOT_POLICY_FILE
sed -i -e "s/ACCOUNT_ID/$ACCOUNT_ID/g" $IOT_POLICY_FILE
sed -i -e "s/CLIENT/$THING_NAME/g" $IOT_POLICY_FILE
sed -i -e "s/REGION/$AWS_REGION/g" $IOT_POLICY_FILE
cat $IOT_POLICY_FILE
aws iot create-policy --policy-name $IOT_POLICY_NAME --policy-document file://$IOT_POLICY_FILE
aws iot attach-policy --policy-name $IOT_POLICY_NAME --target $CERT_ARN

ros2 run telemetry_mqtt mqtt_telemetry_pub --ros-args --param path_for_config:=$IOT_CONFIG_FILE

/home/scripts# ./thing_setup.sh

cd /home/ros2_ws/sub_demo
./build.sh

ros2 run sub_demo subscriber_demo


truncate -s 0 iot_config.json
truncate -s 0 iot_policy.json


source /root/ros2_ws/aws-iot-robot-connectivity-samples-ros2/workspace/install/setup.bash

ros2 run telemetry_mqtt mock_telemetry_pub
