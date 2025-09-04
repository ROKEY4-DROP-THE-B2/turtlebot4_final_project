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
                main_text.textContent = COMMAND[Number(queryParam)] + '요청을 전송하였습니다.'
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