import os
import time
import threading
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

# ROS 2 서비스 호출을 위한 import 추가
try:
    from dsr_msgs2.srv import MovePause, MoveResume
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
except ImportError:
    pass

# ===== 시뮬레이션(가짜) 모드 설정 =====
SIMULATION_MODE = True  # True로 설정하면 로봇 없이 UI 테스트 가능

# ===== Firebase 설정 =====
SERVICE_ACCOUNT_KEY_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "rokey-fe6a9-firebase-adminsdk-fbsvc-4856a2fb9f.json"
)
DATABASE_URL = "https://rokey-fe6a9-default-rtdb.asia-southeast1.firebasedatabase.app"

# ===== 상태 관리 =====
is_running = False
is_paused = False
is_collided = False
status_ref = None
command_queue_ref = None
control_queue_ref = None

# ===== ROS2 제어용 전역 변수 =====
control_executor = None  # 제어 전용 executor (메인 executor와 분리)
control_node = None


def init_firebase():
    """Firebase 초기화"""
    global status_ref, command_queue_ref, control_queue_ref
    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {
            'databaseURL': DATABASE_URL
        })
    except ValueError:
        print("Firebase 앱이 이미 초기화되었습니다.")

    status_ref = db.reference("/robot_status")
    command_queue_ref = db.reference("/robot_commands/start_requests")
    control_queue_ref = db.reference("/robot_commands/control_requests")


def update_status(running, status_text, sauce="선택없음", powder="선택없음"):
    """Firebase에 로봇 상태 업데이트"""
    global is_paused, is_collided
    status_ref.update({
        "is_running": running,
        "is_paused": is_paused,
        "is_collided": is_collided,
        "status_text": status_text,
        "selected_sauce": sauce,
        "selected_powder": powder,
        "last_update_timestamp": time.time()
    })


def call_ros2_service_safe(client, request, service_name, timeout=3.0):
    """
    ROS2 서비스를 스레드 안전하게 호출하는 함수.
    별도의 executor를 사용하여 메인 executor와의 충돌(generator already executing)을 방지.
    """
    global control_executor, control_node
    try:
        if not client.wait_for_service(timeout_sec=1.0):
            print(f"[경고] {service_name} 서비스를 찾을 수 없습니다.")
            return False

        future = client.call_async(request)

        # 별도 executor로 spin하여 메인 스레드의 executor와 충돌 방지
        control_executor.spin_until_future_complete(future, timeout_sec=timeout)

        if future.done():
            result = future.result()
            print(f"[제어] {service_name} 서비스 호출 성공")
            return True
        else:
            print(f"[경고] {service_name} 서비스 호출 타임아웃")
            return False

    except Exception as e:
        print(f"[오류] {service_name} 서비스 호출 중 예외: {e}")
        return False


def simulated_sleep(seconds):
    """시뮬레이션 모드에서 로봇 동작을 흉내내는 함수 (일시 정지/충돌 시 진행 멈춤)"""
    global is_paused, is_collided
    elapsed = 0
    while elapsed < seconds:
        if is_paused or is_collided:
            time.sleep(0.5)
            continue
        time.sleep(0.5)
        elapsed += 0.5


def run_robot_task(request_id, sauce, powder):
    """main.py의 작업 함수를 직접 호출"""
    global is_running

    if not SIMULATION_MODE:
        from cobot1.main import (
            perform_task_dough_grip,
            perform_task_press,
            perform_task_plate_setting,
            perform_task_spatula,
            perform_task_source,
            perform_task_powder_snap,
        )
        from cobot1 import press_test
        from cobot1 import source_test
        from cobot1 import powder_test

    print(f"\n{'='*50}")
    print(f"[로봇 실행] 요청: {request_id}")
    print(f"  소스: {sauce}, 가루: {powder}")
    print(f"{'='*50}")

    update_status(True, "작동 중", sauce, powder)
    status_ref.update({"last_processed_request_id": request_id})

    try:
        # ===== 전체 작업 순서대로 수행 =====
        update_status(True, "작업 1: 반죽 집기", sauce, powder)
        if SIMULATION_MODE: simulated_sleep(3)
        else: perform_task_dough_grip()

        update_status(True, "작업 2: 프레스 누르기", sauce, powder)
        if SIMULATION_MODE: simulated_sleep(3)
        else: perform_task_press()

        update_status(True, "작업 3: 접시 세팅", sauce, powder)
        if SIMULATION_MODE: simulated_sleep(3)
        else: perform_task_plate_setting()

        update_status(True, "작업 4: 뒤집개", sauce, powder)
        if SIMULATION_MODE: simulated_sleep(3)
        else: perform_task_spatula()

        if sauce != "선택없음":
            update_status(True, "작업 5: 소스 뿌리기", sauce, powder)
            if SIMULATION_MODE: simulated_sleep(3)
            else: perform_task_source()

        if powder != "선택없음":
            update_status(True, "작업 6: 가루 뿌리기", sauce, powder)
            if SIMULATION_MODE: simulated_sleep(3)
            else: perform_task_powder_snap()

        print(f"[완료] 요청 {request_id} 전체 작업 완료!")
        update_status(False, "완료 - 대기 중")

    except Exception as e:
        print(f"[오류] 작업 중 예외 발생: {e}")
        update_status(False, f"오류: {e}")
    finally:
        is_running = False


def main():
    global is_running, is_paused, is_collided
    global control_executor, control_node

    # ===== 1. Firebase 초기화 =====
    init_firebase()

    # ===== 2. ROS2 초기화 + 로봇 셋업 =====
    pause_cli = None
    resume_cli = None

    if SIMULATION_MODE:
        print("\n[INFO] 시뮬레이션(가짜) 모드로 실행 중입니다. (로봇 미연결)")
    else:
        import rclpy
        import DR_init
        from rclpy.executors import SingleThreadedExecutor

        rclpy.init()

        from cobot1.main import ROBOT_ID, initialize_robot, setup_io_clients

        node = rclpy.create_node("robot_backend", namespace=ROBOT_ID)
        DR_init.__dsr__node = node

        initialize_robot()
        setup_io_clients(node)
        press_test.setup_io_clients(node)
        source_test.setup_io_clients(node)
        powder_test.setup_io_clients(node)

        # 제어용 별도 ROS2 노드 + 전용 executor (spin 충돌 방지)
        control_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
        control_executor = SingleThreadedExecutor()
        control_executor.add_node(control_node)

        pause_cli = control_node.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
        resume_cli = control_node.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')

    # ===== 3. 시작 전 이전 세션의 잔여 명령 정리 =====
    try:
        command_queue_ref.delete()
        control_queue_ref.delete()
        print("[초기화] 이전 세션의 잔여 명령 큐를 정리했습니다.")
    except Exception as e:
        print(f"[경고] 명령 큐 초기화 중 오류 (무시 가능): {e}")

    # ===== 4. 대기 루프 =====
    print("\n" + "=" * 50)
    print("로봇 백엔드 서버 시작")
    print("웹 UI에서 시작 버튼을 누르면 로봇이 동작합니다.")
    print("Ctrl+C로 종료")
    print("=" * 50)

    update_status(False, "대기 중")

    rclpy_initialized = not SIMULATION_MODE

    try:
        while True:
            # --- 1. 제어 명령(일시정지/충돌/재개) 처리 ---
            try:
                control_requests = control_queue_ref.get() or {}
            except Exception as e:
                print(f"[경고] Firebase 제어 큐 읽기 실패: {e}")
                control_requests = {}

            for req_id, req_data in control_requests.items():
                req_data = req_data or {}
                command = req_data.get("command")
                
                if command == "pause":
                    is_paused = True
                    print("\n[제어] 일시 정지 명령 수신")
                    if not SIMULATION_MODE and pause_cli:
                        call_ros2_service_safe(
                            pause_cli, MovePause.Request(), "move_pause"
                        )
                    update_status(is_running, "일시 정지됨")
                
                elif command == "simulate_collision":
                    is_collided = True
                    is_paused = True
                    print("\n[제어] 충돌 시뮬레이션 수신")
                    if not SIMULATION_MODE and pause_cli:
                        call_ros2_service_safe(
                            pause_cli, MovePause.Request(), "move_pause"
                        )
                    update_status(is_running, "충돌 감지 (시뮬레이션)")
                
                elif command == "resume":
                    is_paused = False
                    is_collided = False
                    print("\n[제어] 작동 재개 명령 수신")
                    if not SIMULATION_MODE and resume_cli:
                        call_ros2_service_safe(
                            resume_cli, MoveResume.Request(), "move_resume"
                        )
                    update_status(is_running, "작동 재개 중...")
                
                elif command == "resume_collision":
                    is_collided = False
                    is_paused = False
                    print("\n[제어] 충돌 해제 및 재개 명령 수신")
                    if not SIMULATION_MODE and resume_cli:
                        call_ros2_service_safe(
                            resume_cli, MoveResume.Request(), "move_resume"
                        )
                    update_status(is_running, "충돌 해제 및 재개 중...")
                
                # 처리한 명령 삭제
                try:
                    control_queue_ref.child(req_id).delete()
                except Exception as e:
                    print(f"[경고] 제어 명령 삭제 실패: {e}")

            # --- 2. 기존 시작 요청 처리 ---
            try:
                pending_requests = command_queue_ref.get() or {}
            except Exception as e:
                print(f"[경고] Firebase 시작 큐 읽기 실패: {e}")
                pending_requests = {}

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
        # 종료 시 Firebase 명령 큐 정리 (다음 실행 시 오래된 명령 방지)
        try:
            command_queue_ref.delete()
            control_queue_ref.delete()
            print("[종료] 명령 큐를 정리했습니다.")
        except Exception:
            pass
        try:
            update_status(False, "서버 종료됨")
        except Exception:
            pass
        if rclpy_initialized:
            try:
                if control_executor:
                    control_executor.shutdown()
                rclpy.shutdown()
            except Exception as e:
                print(f"[종료] ROS2 종료 중 경고 (무시 가능): {e}")


if __name__ == "__main__":
    main()