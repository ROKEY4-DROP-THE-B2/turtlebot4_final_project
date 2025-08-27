import random
import time
from paho.mqtt import client as mqtt_client

broker = 'r1782871.ala.us-east-1.emqxsl.com'
port = 8883
username = 'sg'
password = '1234'

topic = "python/mqtt"  #토픽이름은 자유롭게 정하면 됨
client_id = f'python-mqtt-{random.randint(0, 100)}' #세션ID가 자동으로 랜덤 생성되어 관리되므로 그대로 사용해도 됨.

def connect_mqtt() -> mqtt_client.Client:
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


def publish(client):
    msg_count = 0
    while True:
        msg = f"Message number {msg_count}"  # 토픽으로 보낼 메시지 형식
        result = client.publish(topic, msg)
        status = result[0]
        if status == 0:
            print(f"Sent `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")
        msg_count += 1
        time.sleep(1)


def run():
    client = connect_mqtt()
    client.loop_start()
    try:
        publish(client)
    except KeyboardInterrupt:
        print("Publishing stopped by user.")
        client.loop_stop()


if __name__ == '__main__':
    run()
