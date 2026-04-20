import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

# Firebase Admin SDK 초기화
SERVICE_ACCOUNT_KEY_PATH = "./rokeycakey-e4bbe-firebase-adminsdk-fbsvc-a1e9895d4d.json"
DATABASE_URL = "https://rokeycakey-e4bbe-default-rtdb.asia-southeast1.firebasedatabase.app"

try:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {
        "databaseURL": DATABASE_URL
    })
except ValueError:
    print("Firebase 앱이 이미 초기화되었습니다.")

status_ref = db.reference("/robot_status")
command_queue_ref = db.reference("/robot_commands/start_requests")

print("웹 시작 요청을 기다립니다. Python이 5초 동안 작동 상태를 관리합니다. (Ctrl+C로 종료)")

status_ref.update({
    "is_running": False,
    "status_text": "대기 중",
    "remaining_seconds": 0,
    "selected_sauce": "선택없음",
    "selected_powder": "선택없음",
    "last_update_timestamp": time.time()
})

running_until = None

try:
    while True:
        current_time = time.time()

        if running_until is not None and current_time >= running_until:
            status_ref.update({
                "is_running": False,
                "status_text": "대기 중",
                "remaining_seconds": 0,
                "last_update_timestamp": current_time
            })
            running_until = None
            print("작동 종료: 상태를 대기 중으로 변경했습니다.")
        elif running_until is not None:
            remaining_seconds = max(0, int(running_until - current_time + 0.999))
            status_ref.update({
                "remaining_seconds": remaining_seconds,
                "last_update_timestamp": current_time
            })

        pending_requests = command_queue_ref.get() or {}
        is_running = bool((status_ref.get() or {}).get("is_running"))

        if pending_requests:
            for request_id, request_data in pending_requests.items():
                request_data = request_data or {}
                selected_sauce = request_data.get("sauce", "선택없음")
                selected_powder = request_data.get("powder", "선택없음")

                if not is_running and running_until is None:
                    running_until = current_time + 5
                    status_ref.update({
                        "is_running": True,
                        "status_text": "작동 중",
                        "remaining_seconds": 5,
                        "selected_sauce": selected_sauce,
                        "selected_powder": selected_powder,
                        "last_update_timestamp": current_time,
                        "last_processed_request_id": request_id
                    })
                    is_running = True
                    print(
                        f"작동 시작: {request_id}, "
                        f"소스={selected_sauce}, 가루={selected_powder}, 5초 동안 작동 중입니다."
                    )

                else:
                    print(f"무시된 요청: {request_id}, 이미 작동 중입니다.")

                command_queue_ref.child(request_id).delete()

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")
