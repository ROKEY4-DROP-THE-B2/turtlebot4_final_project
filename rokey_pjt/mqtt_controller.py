import random
from paho.mqtt import client as mqtt_client

broker = 'r1782871.ala.us-east-1.emqxsl.com'
port = 8883
username = 'sg'
password = '1234'
client_id = f'python-mqtt-{random.randint(0, 100)}' #세션ID가 자동으로 랜덤 생성되어 관리되므로 그대로 사용해도 됨.

SUCCEEDED = 0
FAILED = 1

class MqttController:
    def __init__(self, topics, on_message_callback):
        self.client = None
        if isinstance(topics, str):
            self.topics = [(topics, 2)]
        else:
            self.topics = map(topics, lambda x: (x, 2))
        self._on_message = on_message_callback

    def start_mqtt(self):
        self.client = mqtt_client.Client(client_id=client_id, protocol=mqtt_client.MQTTv311)
        self.client.tls_set()
        self.client.username_pw_set(username, password)
        self.client.on_message = self._on_message
        self.client.connect(broker, port)
        self.client.subscribe(self.topics)
        self.client.loop_forever()

    def publish(self, topic, msg):
        result = self.client.publish(topic, msg)
        status = result[0]
        return SUCCEEDED if status == 0 else FAILED
    
    def stop_mqtt(self):
        self.client.loop_stop()
