사용자의 돌발상황으로 인한 로봇팔 충돌 감지 시뮬레이션

1. 로봇팔이 메인 태스크 진행중에 사람에 의해 충돌 감지
2. user_inteface에서 
<div class="modal-overlay" id="errorModal">
        <div class="modal-content error">
            <h2>⚠️ 앗, 오류 발생!</h2>
            <p>기계 요정에게 문제가 생겼어요.</p>
            <p class="status-msg" id="errorStatusText">상태: 알 수 없는 오류</p>
            <p>관리자에게 문의해주세요.</p>
            <button class="close-btn" onclick="closeError()">닫기</button>
        </div>
    </div>

-> 팝업이 생기면서 작동 중지(로봇팔 노란불)

3. 닫기를 누르고 수동으로 톱나바퀴 아이콘(개발자 인터페이스)으로 들어감.
4. 개발자 인터페이스에서

<!-- 충돌 팝업 (모달) -->
<div id="collision-modal" class="modal">
    <div class="modal-content">
        <h2 class="red">충돌 감지</h2>
        <p>로봇에 충돌이 발생하여 동작이 중지되었습니다.</p>
        <button class="modal-btn red-btn" id="collision-resume-button">충돌 해제 및 재개</button>
    </div>
</div>

--> 충돌 팝업이 생기고, 충돌 해제 및 재개 버튼을 누르면 일시중지 됬던(노란불) 로봇팔이 원래 했던 동작 그대로 이어나감. (재개와 동일함)