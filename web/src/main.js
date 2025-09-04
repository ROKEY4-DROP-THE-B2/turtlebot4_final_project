const COMMAND = {
    1: '언도킹',
    2: 'UNKNOWN',
    31: '보급품 수령',
    32: '보급품 수령',
    33: '보급품 수령',
    312: '보급품 수령', 
    313: '보급품 수령',
    323: '보급품 수령',
    3123: '보급품 수령',
    4: '보급품 운반',
    5: '도킹',
}

const main_text = document.getElementById('main-text')
main_text.textContent = ''

const number_btns = document.querySelectorAll('.child-div');
number_btns.forEach((button, i) => {
    button.addEventListener('click', e => {
        main_text.textContent += String(i + 1)
    });
});

const number_btns2 = document.querySelectorAll('.child-div2');
number_btns2.forEach((button, i) => {
        button.addEventListener('click', e => {
            main_text.textContent += String(i + 4)
        });
});

document.getElementById('confirm-btn').addEventListener('click', e => {
    const sendData = async () => {
        const queryParam = main_text.textContent
        if (main_text.textContent === '') {
            return
        }
        const url = 'http://192.168.0.27:5000/move/' + queryParam;

        try {
            const response = await fetch(url);
            
            // HTTP 상태 코드가 200번대(성공)가 아니면 에러를 발생시킵니다.
            if (!response.ok) {
                throw new Error(`HTTP 오류! 상태: ${response.status}`);
            }
            
            const data = await response.json(); // 응답을 JSON 형식으로 변환합니다.
            if (data[1] === 200) {
                main_text.textContent = COMMAND[Number(queryParam)] + '을 요청하였습니다.'
                setTimeout(() => {
                    main_text.textContent = ''
                }, 2000)
            }
            // console.log('성공적으로 데이터를 가져왔습니다:', data);
        } catch (error) {
            console.error('데이터를 가져오는 중 오류가 발생했습니다:', error);
        } 
    }

    // 함수 호출
    sendData();
})

// 1. MQTT 클라이언트 인스턴스 생성
// 호스트, 포트, 클라이언트 ID를 지정합니다.
// 보통 웹 소켓을 사용하므로 포트는 80, 443, 또는 8083, 9001 등을 사용합니다.
// const client = new Paho.MQTT.Client(
//     'r1782871.ala.us-east-1.emqxsl.com',
//     8084,
//     "clientId-" + new Date().getTime()
// );
// // 2. 콜백 핸들러 설정
// // 연결 끊김 시 호출될 함수
// client.onConnectionLost = onConnectionLost;
// // 메시지 도착 시 호출될 함수
// client.onMessageArrived = onMessageArrived;
// client.onConnectionLost = onConnectionLost;

// // 3. 브로커에 연결
// client.connect({
//     onSuccess: onConnect,
//     userName: 'sg',  // <--- 여기에 사용자 이름 설정
//     password: '1234',   // <--- 여기에 비밀번호 설정
//     useSSL: true
// });

// // 4. 연결 성공 시 호출될 함수
// function onConnect() {
//     console.log("MQTT 브로커에 연결되었습니다.");
//     client.subscribe("mission_completed");
// }

// // 5. 연결 끊김 시 호출될 함수
// function onConnectionLost(responseObject) {
//     if (responseObject.errorCode !== 0) {
//         console.log("연결 끊김: " + responseObject.errorMessage);
//     }
// }

// // 6. 메시지 도착 시 호출될 함수 (가장 중요)
// // 메시지를 수신하면 이 함수가 호출됩니다.
// function onMessageArrived(message) {
//     console.log(message)
// }