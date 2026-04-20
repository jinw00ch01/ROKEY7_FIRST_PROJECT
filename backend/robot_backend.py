import sys
import os
import time
import threading
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

# ===== Firebase 설정 =====
SERVICE_ACCOUNT_KEY_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "rokey-fe6a9-firebase-adminsdk-fbsvc-4856a2fb9f.json"
)
DATABASE_URL = "https://rokey-fe6a9-default-rtdb.asia-southeast1.firebasedatabase.app"

# ===== 상태 관리 =====
is_running = False
status_ref = None
command_queue_ref = None


def init_firebase():
    """Firebase 초기화"""
    global status_ref, command_queue_ref
    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {
            'databaseURL': DATABASE_URL
        })
    except ValueError:
        print("Firebase 앱이 이미 초기화되었습니다.")

    status_ref = db.reference("/robot_status")
    command_queue_ref = db.reference("/robot_commands/start_requests")


def update_status(running, status_text, sauce="선택없음", powder="선택없음"):
    """Firebase에 로봇 상태 업데이트"""
    status_ref.update({
        "is_running": running,
        "status_text": status_text,
        "selected_sauce": sauce,
        "selected_powder": powder,
        "last_update_timestamp": time.time()
    })


def run_robot_task(request_id, sauce, powder):
    """main.py의 작업 함수를 직접 호출"""
    global is_running

    # cobot1.main에서 작업 함수 import (ROS2 환경이 소싱된 상태여야 함)
    from cobot1.main import (
        perform_task_dough_grip,
        perform_task_press,
        perform_task_plate_setting,
        perform_task_spatula,
    )

    print(f"\n{'='*50}")
    print(f"[로봇 실행] 요청: {request_id}")
    print(f"  소스: {sauce}, 가루: {powder}")
    print(f"{'='*50}")

    update_status(True, "작동 중", sauce, powder)
    status_ref.update({"last_processed_request_id": request_id})

    try:
        # ===== 전체 작업 순서대로 수행 =====
        update_status(True, "작업 1: 반죽 집기", sauce, powder)
        perform_task_dough_grip()

        update_status(True, "작업 2: 프레스 누르기", sauce, powder)
        perform_task_press()

        update_status(True, "작업 3: 접시 세팅", sauce, powder)
        perform_task_plate_setting()

        update_status(True, "작업 4: 뒤집개", sauce, powder)
        perform_task_spatula()

        print(f"[완료] 요청 {request_id} 전체 작업 완료!")
        update_status(False, "완료 - 대기 중")

    except Exception as e:
        print(f"[오류] 작업 중 예외 발생: {e}")
        update_status(False, f"오류: {e}")
    finally:
        is_running = False


def main():
    global is_running

    # ===== 1. Firebase 초기화 =====
    init_firebase()

    # ===== 2. ROS2 초기화 + 로봇 셋업 =====
    import rclpy
    import DR_init

    rclpy.init()

    from cobot1.main import ROBOT_ID, initialize_robot, setup_io_clients

    node = rclpy.create_node("robot_backend", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    initialize_robot()
    setup_io_clients(node)

    # ===== 3. 대기 루프 =====
    print("\n" + "=" * 50)
    print("로봇 백엔드 서버 시작")
    print("웹 UI에서 시작 버튼을 누르면 로봇이 동작합니다.")
    print("Ctrl+C로 종료")
    print("=" * 50)

    update_status(False, "대기 중")

    try:
        while True:
            pending_requests = command_queue_ref.get() or {}

            if pending_requests and not is_running:
                for request_id, request_data in pending_requests.items():
                    request_data = request_data or {}
                    sauce = request_data.get("sauce", "선택없음")
                    powder = request_data.get("powder", "선택없음")

                    command_queue_ref.child(request_id).delete()

                    if not is_running:
                        is_running = True
                        robot_thread = threading.Thread(
                            target=run_robot_task,
                            args=(request_id, sauce, powder),
                            daemon=True
                        )
                        robot_thread.start()
                    else:
                        print(f"[무시] 요청 {request_id}: 이미 작동 중")

            elif pending_requests and is_running:
                for request_id in pending_requests:
                    print(f"[무시] 요청 {request_id}: 이미 작동 중")
                    command_queue_ref.child(request_id).delete()

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[종료] 백엔드 서버를 종료합니다.")
        update_status(False, "서버 종료됨")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
