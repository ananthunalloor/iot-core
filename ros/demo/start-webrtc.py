import rclpy
import json
import os
from rclpy.node import Node
from awscrt import mqtt
from awsiot import mqtt_connection_builder
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRelay
from botocore.config import Config
import boto3
import asyncio
from base64 import b64decode, b64encode
from botocore.auth import SigV4QueryAuth
from botocore.awsrequest import AWSRequest
from botocore.credentials import Credentials
from botocore.session import Session
import websockets
from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription
from aiortc.sdp import candidate_from_sdp



TOPIC = "rover_iot_thing/publish_topic"

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

class MediaTrackManager:
    def __init__(self):
        self.audio_track = None
        self.video_track = None
        self.relay = MediaRelay()

    def create_media_track(self):
        options = {'framerate': '30'}

        # use a test pattern as mp4 video source instead of a real camera
        media = MediaPlayer('/home/ros/test-media/BigBuckBunny.mp4', format='mp4', options=options)

        audio_track = self.relay.subscribe(media.audio) if media.audio else None
        video_track = self.relay.subscribe(media.video) if media.video else None

        if audio_track is None and video_track is None:
            raise ValueError("Neither audio nor video track could be created from the source.")

        return audio_track, video_track

class WebRTCStreamController(Node):

    def __init__(self):
        super().__init__('turtle_sim_controller')
        self.mqtt_connection = self.configure_mqtt_client()
        self.init_pubs()
    
    def init_pubs(self):
        logger = self.get_logger()
        self.mqtt_connection = self.configure_mqtt_client()
        self.mqtt_connection.connect()
        logger.info("Connected to AWS IoT")
        subscribe_future, _ = self.mqtt_connection.subscribe(
            TOPIC,
            mqtt.QoS.AT_LEAST_ONCE,
            self.on_message_received
        )
        subscribe_future.result()
        logger.info("Subscribed successfully")

    def configure_mqtt_client(self):
        with open('/home/ros/cert/iot_config.json') as f:
            cert_data = json.load(f)
        
        mqtt_connection = mqtt_connection_builder.mtls_from_path(
            endpoint=cert_data["endpoint"],
            port=8883,
            cert_filepath=cert_data["certificatePath"],
            pri_key_filepath=cert_data["privateKeyPath"],
            ca_filepath=cert_data["rootCAPath"],
            client_id=cert_data.get("clientID"),
    )
        return mqtt_connection

    def on_message_received(self, topic, payload, **kwargs):
        logger = self.get_logger()
        kvs = KinesisVideoClient()

        message = json.loads(payload)

        logger.info(f"Received message from topic '{topic}': {message}")

class KinesisVideoClient(Node):
    def __init__(self):
        super().__init__('webrtc_stream')
        logger = self.get_logger()
        self.session = boto3.Session()
        self.media_manager = MediaTrackManager()

        self.endpoints = None
        self.endpoint_https = None
        self.endpoint_wss = None
        self.ice_servers = None 
        self.PCMap = {}
        self.DCMap = {}

        logger.info("Connecting to AWS Kinesis Video")

        self.kinesis_client = self.session.client(
        "kinesisvideo",
        region_name=aws_region,
        config=my_config,
    )
        self.channel_arn = self.kinesis_client.describe_signaling_channel(
        ChannelName=thing_name
    )["ChannelInfo"]["ChannelARN"]
        logger.info("Connected to AWS Kinesis Video")
        
    def get_signaling_channel_endpoint(self):
        logger = self.get_logger()
        if self.endpoints is None:  # Check if endpoints are already fetched
            endpoints = self.kinesis_client.get_signaling_channel_endpoint(
                ChannelARN=self.channel_arn,
                SingleMasterChannelEndpointConfiguration={'Protocols': ['HTTPS', 'WSS'], 'Role': 'MASTER'}
            )
            self.endpoints = {
                'HTTPS': next(o['ResourceEndpoint'] for o in endpoints['ResourceEndpointList'] if o['Protocol'] == 'HTTPS'),
                'WSS': next(o['ResourceEndpoint'] for o in endpoints['ResourceEndpointList'] if o['Protocol'] == 'WSS')
            }
            self.endpoint_https = self.endpoints['HTTPS']
            self.endpoint_wss = self.endpoints['WSS']
        logger.info("Signaling Channel Endpoint fetched")
        return self.endpoints
    
    def prepare_ice_servers(self):
        logger = self.get_logger()
        client = self.session.client('kinesis-video-signaling', endpoint_url=self.endpoint_https, region_name=aws_region)
        ice_servers_list = client.get_ice_server_config(
        ChannelARN = self.channel_arn)['IceServerList']

        iceServers = [RTCIceServer(urls=f'stun:stun.kinesisvideo.{aws_region}.amazonaws.com:443')]
        for iceServer in ice_servers_list:
            iceServers.append(RTCIceServer(
                urls=iceServer['Uris'],
                username=iceServer['Username'],
                credential=iceServer['Password']
            ))
        self.ice_servers = iceServers
        logger.info("ICE Servers prepared")
        return self.ice_servers

    def create_wss_url(self):
        logger = self.get_logger()
        session = Session()
        auth_credentials = session.get_credentials()

        SigV4 = SigV4QueryAuth(auth_credentials, 'kinesisvideo', aws_region, 299)
        aws_request = AWSRequest(
            method='GET',
            url=self.endpoint_wss,
            params={'X-Amz-ChannelARN': self.channel_arn, 'X-Amz-ClientId': client_id}
        )
        SigV4.add_auth(aws_request)
        PreparedRequest = aws_request.prepare()
        logger.info("WSS URL created and authenticated")
        return PreparedRequest.url
    
    def decode_msg(self, msg):
        try:
            data = json.loads(msg)
            payload = json.loads(b64decode(data['messagePayload'].encode('ascii')).decode('ascii'))
            return data['messageType'], payload, data.get('senderClientId')
        except json.decoder.JSONDecodeError:
            return '', {}, ''

    def encode_msg(self, action, payload, client_id):
        return json.dumps({
            'action': action,
            'messagePayload': b64encode(json.dumps(payload.__dict__).encode('ascii')).decode('ascii'),
            'recipientClientId': client_id,
        })
    
    async def handle_sdp_offer(self, payload, client_id, audio_track, video_track, websocket):
        logger = self.get_logger()
        iceServers = self.prepare_ice_servers()
        configuration = RTCConfiguration(iceServers=iceServers)
        pc = RTCPeerConnection(configuration=configuration)
        self.DCMap[client_id] = pc.createDataChannel('teleop')
        self.PCMap[client_id] = pc
        @pc.on('connectionstatechange')
        async def on_connectionstatechange():
            logger.info(f'[{client_id}] connectionState --1: {pc.connectionState}')
            if client_id in self.PCMap:
                logger.info(f'[{client_id}] connectionState  11: {self.PCMap[client_id].connectionState}')    

        @pc.on('iceconnectionstatechange')
        async def on_iceconnectionstatechange():
            logger.info(f'[{client_id}] iceConnectionState -ice-2: {pc.iceConnectionState}')
            if client_id in self.PCMap:
                logger.info(f'[{client_id}] iceConnectionState: 22{self.PCMap[client_id].iceConnectionState}')

        @pc.on('icegatheringstatechange')
        async def on_icegatheringstatechange():
            logger.info(f'[{client_id}] iceGatheringState 3: {pc.iceGatheringState}')
            if client_id in self.PCMap:
                logger.info(f'[{client_id}] iceGatheringState: 33{self.PCMap[client_id].iceGatheringState}')

        @pc.on('signalingstatechange')
        async def on_signalingstatechange():
            signaling_state = pc.signalingState
            logger.info(f'[{client_id}] signalingState: 4{pc.signalingState}')
            if client_id in self.PCMap:
                logger.info(f'[{client_id}] signalingState: 44{self.PCMap[client_id].signalingState}')
                # if signaling_state == 'closed':
                #     await self.handle_disconnect(client_id)

        @pc.on('track')
        def on_track(track):
            logger.info(f'[{client_id}] Track added: {track.kind}')
            MediaBlackhole().addTrack(track)

        @pc.on('datachannel')
        async def on_datachannel(channel):
            logger.info(f'[{client_id}] Data channel opened: {channel.label}')
            @channel.on('message')
            def on_message(dc_message):
                logger.info(f'[{channel.label}] datachannel_message: {dc_message}')
                for i in self.PCMap:
                    if self.DCMap[i].readyState == 'open':
                        try:
                            self.DCMap[i].send(f'broadcast: {dc_message}')
                        except Exception as e:
                            logger.error(f"Error sending datachannel message: {e}")
                    else:
                        logger.warning(f"Data channel not open for {i}")

        if audio_track:
            self.PCMap[client_id].addTrack(audio_track)
        if video_track:
            self.PCMap[client_id].addTrack(video_track)

        await self.PCMap[client_id].setRemoteDescription(RTCSessionDescription(
            sdp=payload['sdp'],
            type=payload['type']
        ))
        await self.PCMap[client_id].setLocalDescription(await self.PCMap[client_id].createAnswer())
        await websocket.send(self.encode_msg('SDP_ANSWER', self.PCMap[client_id].localDescription, client_id))

    async def handle_ice_candidate(self, payload, client_id):
        if client_id in self.PCMap:
            candidate = candidate_from_sdp(payload['candidate'])
            candidate.sdpMid = payload['sdpMid']
            candidate.sdpMLineIndex = payload['sdpMLineIndex']
            await self.PCMap[client_id].addIceCandidate(candidate)
    
    async def handle_disconnect(self, client_id):
        logger = self.get_logger()
        if client_id in self.PCMap:
            await self.PCMap[client_id].close()
            del self.PCMap[client_id]
            logger.info(f"Peer connection closed and removed for {client_id}")
        if client_id in self.DCMap:
            await self.DCMap[client_id].close()
            del self.DCMap[client_id]
            logger.info(f"Data channel closed and removed for {client_id}")

    async def reconnect(self, client_id):
        logger = self.get_logger()
        await self.handle_disconnect(client_id)
        logger.info(f"Reconnecting to signaling channel")
        # wait for handle_disconnect to complete
        await asyncio.sleep(5)
        logger.info(f"Reconnected to signaling channel")
        # await self.handle_sdp_offer(payload, client_id, audio_track, video_track, websocket)


    async def signaling_client(self):
        logger = self.get_logger()
        audio_track, video_track = self.media_manager.create_media_track()
        self.get_signaling_channel_endpoint() 
        wss_url = self.create_wss_url()

        while True:
            try:
                async with websockets.connect(wss_url) as websocket:
                    logger.info("Connected to signaling channel")
                    async for message in websocket:
                        msg_type, payload, client_id = self.decode_msg(message)
                        logger.info(f"Received message: {msg_type} from {client_id}")
                        if msg_type == 'SDP_OFFER':
                            logger.info(f"Received SDP offer from {client_id}")
                            if client_id not in self.PCMap:
                                await self.handle_sdp_offer(payload, client_id, audio_track, video_track, websocket)
                            else:
                                logger.warning(f"Peer connection already exists for {client_id}")
                        
                        elif msg_type == 'ICE_CANDIDATE':
                            logger.info(f"Received ICE candidate from {client_id}")
                            
                            await self.handle_ice_candidate(payload, client_id)

            except websockets.ConnectionClosed:
                logger.info("Connection closed. Reconnecting...")
                await asyncio.sleep(5)
                wss_url = self.create_wss_url()
                continue

async def run_client(client):
    await client.signaling_client()
    
async def main(args=None):
    rclpy.init(args=args)

    node = WebRTCStreamController()
    kvs = KinesisVideoClient()
    await run_client(kvs)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())