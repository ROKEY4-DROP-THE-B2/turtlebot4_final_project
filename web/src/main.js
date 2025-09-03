// import ROSLIB from 'roslib'

// var ros = new ROSLIB.Ros({
//     url : 'ws://192.168.0.27:9090'
// });

// ros.on('connection', function() {
//     console.log('Connected to rosbridge_server.');
// });
const main_text = document.getElementById('main-text')
main_text.textContent = 11


// document.getElementById('send_action').addEventListener('click', e => {

    // async function getData() {
    //     const url = 'http://192.168.0.27:5000/action/1';

    //     try {
    //         const response = await fetch(url);
            
    //         // HTTP 상태 코드가 200번대(성공)가 아니면 에러를 발생시킵니다.
    //         if (!response.ok) {
    //             throw new Error(`HTTP 오류! 상태: ${response.status}`);
    //         }
            
    //         const data = await response.json(); // 응답을 JSON 형식으로 변환합니다.
    //         console.log('성공적으로 데이터를 가져왔습니다:', data);
    //     } catch (error) {
    //         console.error('데이터를 가져오는 중 오류가 발생했습니다:', error);
    //     } 
    // }

    // // 함수 호출
    // getData();
// })