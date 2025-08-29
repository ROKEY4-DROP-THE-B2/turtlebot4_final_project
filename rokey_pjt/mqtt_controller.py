import random
import time
from paho.mqtt import client as mqtt_client

broker = 'r1782871.ala.us-east-1.emqxsl.com'
port = 8883
username = 'sg'
password = '1234'
client_id = f'python-mqtt-{random.randint(0, 100)}' #세션ID가 자동으로 랜덤 생성되어 관리되므로 그대로 사용해도 됨.

SUCCEEDED = 0
FAILED = 1

def singleton(cls):
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]
    
    return get_instance

@singleton
class MqttController:
    def __init__(self, on_message_callback):
        self.client = self.connect_mqtt()
        self.on_message = on_message_callback

    def connect_mqtt(self) -> mqtt_client.Client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(client_id=client_id, protocol=mqtt_client.MQTTv311)
        client.tls_set()
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect(broker, port)
        return client

    def publish(self, topic, msg):
        self.client.loop_start()
        result = self.client.publish(topic, msg)
        status = result[0]
        time.sleep(1)
        self.client.loop_stop()
        return SUCCEEDED if status == 0 else FAILED

