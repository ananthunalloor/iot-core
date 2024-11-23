import os
from botocore.config import Config
import asyncio
import boto3
from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription

cert_folder_location = os.getenv("CERT_FOLDER_LOCATION", "../cert/")
thing_name = os.getenv("THING_NAME", "your-thing-name")
iot_kvs_role_alias = os.getenv("IOT_KVS_ROLE_ALIAS", "your-role-alias")
aws_region = os.getenv("AWS_REGION", "your-region")

with open(f"{cert_folder_location}iot-credential-provider.txt", "r") as file:
    endpoint = file.read().strip()

cert_file = f"{cert_folder_location}{thing_name}.cert.pem"
private_key = f"{cert_folder_location}{thing_name}.private.key"
root_cert_file = f"{cert_folder_location}kvs.cert.pem"


my_config = Config(
    region_name = aws_region,
    signature_version = 'v4',
    retries = {
        'max_attempts': 10,
        'mode': 'standard'
    }
)

client_id = "master"

def get_signaling_client():
    session = boto3.Session()
    kinesis_client = session.client(
        "kinesisvideo",
        region_name=aws_region,
        config=my_config,
    )

    channel_arn = kinesis_client.describe_signaling_channel(
        ChannelName=thing_name
    )["ChannelInfo"]["ChannelARN"]

    response = kinesis_client.get_signaling_channel_endpoint(
        ChannelARN=channel_arn,
        SingleMasterChannelEndpointConfiguration={
            "Protocols": ["WSS", "HTTPS"],
            "Role": "MASTER",
        },
    )

    endpoint_url = response["ResourceEndpointList"][0]["ResourceEndpoint"]

    client = session.client('kinesis-video-signaling', endpoint_url=endpoint_url)
    ice_servers_list = client.get_ice_server_config(
        ChannelARN = channel_arn)['IceServerList']
    return ice_servers_list, channel_arn, endpoint_url



async def run_master():
     ice_servers_list , channel_arn, endpoint_url = get_signaling_client()
     ice_servers = [RTCIceServer(urls=f'stun:stun.kinesisvideo.{aws_region}.amazonaws.com:443')]
     
     for ice_server in ice_servers_list:
        ice_servers.append(RTCIceServer(
                urls=ice_server['Uris'],
                username=ice_server['Username'],
                credential=ice_server['Password']
            ))
        configuration = RTCConfiguration(iceServers=ice_servers)

        PCMap = {}
        pc = RTCPeerConnection(configuration=configuration)
        PCMap[client_id] = pc

        print("Creating offer", PCMap)



if __name__ == "__main__":
    asyncio.run(run_master())