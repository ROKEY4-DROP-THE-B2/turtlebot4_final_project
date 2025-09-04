# app.py
from flask import Flask, request
from paho.mqtt import client as mqtt_client
from flask_cors import CORS

# --- MQTT 접속 정보 ---
BROKER = 'r1782871.ala.us-east-1.emqxsl.com'
PORT = 8883
USERNAME = 'sg'
PASSWORD = '1234'
TOPIC = "/robot2/move"
CLIENT_ID = "robot1-status-publisher"
# --------------------------

app = Flask(__name__)
CORS(app)  # 모든 출처에 대해 CORS 허용

@app.route('/move/<action>')
def start(action):
    mqtt_client = connect_mqtt()
    mqtt_client.loop_start()
    result = mqtt_client.publish(TOPIC, action)
    mqtt_client.loop_stop()
    if result[0] == 0:
        return ["요청이 성공적으로 처리되었습니다.", 200]
    else:
        return ["요청이 처리되지 않았습니다.", 400]

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("MQTT 브로커에 성공적으로 연결되었습니다!")
        else:
            print(f"MQTT 연결 실패, 코드: {rc}")

    client = mqtt_client.Client(client_id=CLIENT_ID, protocol=mqtt_client.MQTTv311)
    client.tls_set()
    client.username_pw_set(USERNAME, PASSWORD)
    client.on_connect = on_connect
    client.connect(BROKER, PORT)
    return client

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')