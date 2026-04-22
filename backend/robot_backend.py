import os
import time
import threading
import traceback
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
SIMULATION_MODE = False  # True로 설정하면 로봇 없이 UI 테스트 가능

# ===== Firebase 설정 =====
SERVICE_ACCOUNT_KEY_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "rokey-fe6a9-firebase-adminsdk-fbsvc-4856a2fb9f.json"
)
DATABASE_URL = "https://rokey-fe6a9-default-rtdb.asia-southeast1.firebasedatabase.app"

# ===== 상태 관리 (threading.Event로 스레드 안전하게) =====
is_running = False
pause_event = threading.Event()   # set = 일시정지 상태
collide_event = threading.Event() # set = 충돌 상태
status_ref = None
command_queue_ref = None
control_queue_ref = None

# ===== ROS2 제어용 전역 변수 =====
control_executor = None
control_node = None

# 편의 함수: 이전 코드와 호환
def _is_paused():
    return pause_event.is_set()

def _is_collided():
    return collide_event.is_set()


def log(tag, msg):
    """타임스탬프 포함 통일된 로그 출력"""
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}][{tag}] {msg}", flush=True)


def init_firebase():
    """Firebase 초기화"""
    global status_ref, command_queue_ref, control_queue_ref
    log("FIREBASE", "초기화 시작...")
    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {
            'databaseURL': DATABASE_URL
        })
        log("FIREBASE", "앱 초기화 완료")
    except ValueError:
        log("FIREBASE", "앱이 이미 초기화되었습니다. 재사용합니다.")

    status_ref = db.reference("/robot_status")
    command_queue_ref = db.reference("/robot_commands/start_requests")
    control_queue_ref = db.reference("/robot_commands/control_requests")
    log("FIREBASE", "DB 참조 설정 완료")


def update_status(running, status_text, sauce="선택없음", powder="선택없음"):
    """Firebase에 로봇 상태 업데이트"""
    paused = _is_paused()
    collided = _is_collided()
    log("STATUS", f"업데이트 → running={running}, paused={paused}, collided={collided}, text='{status_text}'")
    try:
        status_ref.update({
            "is_running": running,
            "is_paused": paused,
            "is_collided": collided,
            "status_text": status_text,
            "selected_sauce": sauce,
            "selected_powder": powder,
            "last_update_timestamp": time.time()
        })
    except Exception as e:
        log("STATUS", f"[오류] Firebase 상태 업데이트 실패: {e}")


def wait_if_paused():
    """
    일시정지 또는 충돌 상태이면 해제될 때까지 블로킹 대기.
    실제 로봇 작업 함수 호출 직전에 삽입하여 일시정지를 구현.
    """
    if _is_paused() or _is_collided():
        log("WAIT", "⏸  일시정지/충돌 상태 감지 → 재개 대기 중...")
        while _is_paused() or _is_collided():
            time.sleep(0.3)
        log("WAIT", "▶  재개됨 → 작업 계속")


def call_ros2_service_safe(client, request, service_name, timeout=3.0):
    """
    ROS2 서비스를 스레드 안전하게 호출.
    별도 executor를 사용해 메인 executor와의 충돌을 방지.
    """
    global control_executor
    log("ROS2", f"서비스 호출 시도: {service_name}")
    try:
        log("ROS2", f"  서비스 대기 중... (timeout=1.0s)")
        if not client.wait_for_service(timeout_sec=1.0):
            log("ROS2", f"  [경고] {service_name} 서비스를 찾을 수 없음!")
            return False

        log("ROS2", f"  서비스 발견. 비동기 호출 실행...")
        future = client.call_async(request)

        control_executor.spin_until_future_complete(future, timeout_sec=timeout)

        if future.done():
            result = future.result()
            log("ROS2", f"  ✅ {service_name} 서비스 호출 성공: {result}")
            return True
        else:
            log("ROS2", f"  [경고] {service_name} 서비스 호출 타임아웃 ({timeout}s)")
            return False

    except Exception as e:
        log("ROS2", f"  [오류] {service_name} 호출 중 예외:\n{traceback.format_exc()}")
        return False


def simulated_sleep(seconds, label=""):
    """시뮬레이션 모드용: 일시정지/충돌 시 진행 멈춤"""
    log("SIM", f"  작업 '{label}' 시뮬레이션 시작 ({seconds}s)")
    elapsed = 0
    while elapsed < seconds:
        if _is_paused() or _is_collided():
            log("SIM", f"  ⏸  일시정지/충돌 대기 중... (elapsed={elapsed:.1f}s)")
            time.sleep(0.3)
            continue
        time.sleep(0.5)
        elapsed += 0.5
    log("SIM", f"  작업 '{label}' 시뮬레이션 완료")


def run_robot_task(request_id, sauce, powder):
    """로봇 작업 스레드: 전체 공정 순서대로 실행"""
    global is_running

    log("TASK", f"{'='*50}")
    log("TASK", f"작업 시작 | 요청ID={request_id} | 소스={sauce} | 가루={powder}")
    log("TASK", f"SIMULATION_MODE={SIMULATION_MODE}")
    log("TASK", f"{'='*50}")

    if not SIMULATION_MODE:
        log("TASK", "cobot1 모듈 import 중...")
        try:
            from cobot1.main import (
                perform_task_dough_grip,
                perform_task_press,
                perform_task_plate_setting,
                perform_task_spatula,
                perform_task_source,
                perform_task_powder_snap,
            )
            log("TASK", "cobot1 모듈 import 완료")
        except Exception as e:
            log("TASK", f"[치명적 오류] cobot1 import 실패:\n{traceback.format_exc()}")
            is_running = False
            return

    update_status(True, "작동 중", sauce, powder)
    try:
        status_ref.update({"last_processed_request_id": request_id})
    except Exception as e:
        log("TASK", f"[경고] request_id 업데이트 실패: {e}")

    try:
        # ===== 작업 1: 반죽 집기 =====
        log("TASK", "--- 작업 1: 반죽 집기 시작 ---")
        wait_if_paused()
        update_status(True, "작업 1: 반죽 집기", sauce, powder)
        if SIMULATION_MODE:
            simulated_sleep(3, "반죽 집기")
        else:
            log("TASK", "  perform_task_dough_grip() 호출")
            perform_task_dough_grip()
            log("TASK", "  perform_task_dough_grip() 완료")

        # ===== 작업 2: 프레스 누르기 =====
        log("TASK", "--- 작업 2: 프레스 누르기 시작 ---")
        wait_if_paused()
        update_status(True, "작업 2: 프레스 누르기", sauce, powder)
        if SIMULATION_MODE:
            simulated_sleep(3, "프레스 누르기")
        else:
            log("TASK", "  perform_task_press() 호출")
            perform_task_press()
            log("TASK", "  perform_task_press() 완료")

        # ===== 작업 3: 접시 세팅 =====
        log("TASK", "--- 작업 3: 접시 세팅 시작 ---")
        wait_if_paused()
        update_status(True, "작업 3: 접시 세팅", sauce, powder)
        if SIMULATION_MODE:
            simulated_sleep(3, "접시 세팅")
        else:
            log("TASK", "  perform_task_plate_setting() 호출")
            perform_task_plate_setting()
            log("TASK", "  perform_task_plate_setting() 완료")

        # ===== 작업 4: 뒤집개 =====
        log("TASK", "--- 작업 4: 뒤집개 시작 ---")
        wait_if_paused()
        update_status(True, "작업 4: 뒤집개", sauce, powder)
        if SIMULATION_MODE:
            simulated_sleep(3, "뒤집개")
        else:
            log("TASK", "  perform_task_spatula() 호출")
            perform_task_spatula()
            log("TASK", "  perform_task_spatula() 완료")

        # ===== 작업 5: 소스 뿌리기 =====
        if sauce != "선택없음":
            log("TASK", "--- 작업 5: 소스 뿌리기 시작 ---")
            wait_if_paused()
            update_status(True, "작업 5: 소스 뿌리기", sauce, powder)
            if SIMULATION_MODE:
                simulated_sleep(3, "소스 뿌리기")
            else:
                log("TASK", "  perform_task_source() 호출")
                perform_task_source()
                log("TASK", "  perform_task_source() 완료")
        else:
            log("TASK", "소스 선택없음 → 작업 5 스킵")

        # ===== 작업 6: 가루 뿌리기 =====
        if powder != "선택없음":
            log("TASK", "--- 작업 6: 가루 뿌리기 시작 ---")
            wait_if_paused()
            update_status(True, "작업 6: 가루 뿌리기", sauce, powder)
            if SIMULATION_MODE:
                simulated_sleep(3, "가루 뿌리기")
            else:
                log("TASK", "  perform_task_powder_snap() 호출")
                perform_task_powder_snap()
                log("TASK", "  perform_task_powder_snap() 완료")
        else:
            log("TASK", "가루 선택없음 → 작업 6 스킵")

        log("TASK", f"✅ 요청 {request_id} 전체 작업 완료!")
        update_status(False, "완료 - 대기 중")

    except Exception as e:
        log("TASK", f"[오류] 작업 중 예외 발생:\n{traceback.format_exc()}")
        update_status(False, f"오류: {e}")
    finally:
        is_running = False
        log("TASK", "is_running = False 설정 완료")


def main():
    global is_running, control_executor, control_node

    # ===== 1. Firebase 초기화 =====
    init_firebase()

    # ===== 2. ROS2 초기화 + 로봇 셋업 =====
    pause_cli = None
    resume_cli = None

    if SIMULATION_MODE:
        log("MAIN", "시뮬레이션(가짜) 모드로 실행 중입니다. (로봇 미연결)")
    else:
        log("MAIN", "실제 로봇 모드 - ROS2 초기화 시작...")
        try:
            import rclpy
            import DR_init
            from rclpy.executors import SingleThreadedExecutor

            rclpy.init()
            log("MAIN", "rclpy.init() 완료")

            from cobot1.main import ROBOT_ID, initialize_robot, setup_io_clients
            from cobot1 import press_test, source_test, powder_test

            node = rclpy.create_node("robot_backend", namespace=ROBOT_ID)
            DR_init.__dsr__node = node
            log("MAIN", f"robot_backend 노드 생성 완료 (namespace={ROBOT_ID})")

            initialize_robot()
            log("MAIN", "initialize_robot() 완료")

            setup_io_clients(node)
            press_test.setup_io_clients(node)
            source_test.setup_io_clients(node)
            powder_test.setup_io_clients(node)
            log("MAIN", "IO 클라이언트 설정 완료")

            # 제어용 별도 ROS2 노드 + 전용 executor (spin 충돌 방지)
            control_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
            control_executor = SingleThreadedExecutor()
            control_executor.add_node(control_node)
            log("MAIN", "control_node + executor 생성 완료")

            pause_cli = control_node.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
            resume_cli = control_node.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
            log("MAIN", f"pause/resume 클라이언트 생성 완료")
            log("MAIN", f"  pause_cli 서비스명: /{ROBOT_ID}/motion/move_pause")
            log("MAIN", f"  resume_cli 서비스명: /{ROBOT_ID}/motion/move_resume")

        except Exception as e:
            log("MAIN", f"[치명적 오류] ROS2 초기화 실패:\n{traceback.format_exc()}")
            return

    # ===== 3. 시작 전 이전 세션의 잔여 명령 정리 =====
    log("MAIN", "이전 세션 잔여 명령 큐 정리 중...")
    try:
        command_queue_ref.delete()
        control_queue_ref.delete()
        log("MAIN", "✅ 잔여 명령 큐 정리 완료")
    except Exception as e:
        log("MAIN", f"[경고] 큐 초기화 중 오류 (무시 가능): {e}")

    # ===== 4. 상태 초기화 =====
    pause_event.clear()
    collide_event.clear()

    # ===== 5. 대기 루프 =====
    print("\n" + "=" * 50, flush=True)
    print("로봇 백엔드 서버 시작", flush=True)
    print("웹 UI에서 시작 버튼을 누르면 로봇이 동작합니다.", flush=True)
    print("Ctrl+C로 종료", flush=True)
    print("=" * 50, flush=True)

    update_status(False, "대기 중")
    rclpy_initialized = not SIMULATION_MODE
    loop_count = 0

    try:
        while True:
            loop_count += 1

            # --- 제어 명령(일시정지/충돌/재개) 처리 ---
            try:
                control_requests = control_queue_ref.get() or {}
            except Exception as e:
                log("LOOP", f"[경고] Firebase 제어 큐 읽기 실패: {e}")
                control_requests = {}

            if control_requests:
                log("LOOP", f"제어 명령 {len(control_requests)}개 수신!")

            for req_id, req_data in control_requests.items():
                req_data = req_data or {}
                command = req_data.get("command", "")
                log("CTRL", f"명령 처리: id={req_id}, command='{command}'")
                log("CTRL", f"  현재 상태: is_running={is_running}, is_paused={_is_paused()}, is_collided={_is_collided()}")

                if command == "pause":
                    log("CTRL", "⏸  일시 정지 처리")
                    pause_event.set()
                    if not SIMULATION_MODE and pause_cli:
                        log("CTRL", "  ROS2 move_pause 서비스 호출 중...")
                        result = call_ros2_service_safe(pause_cli, MovePause.Request(), "move_pause")
                        log("CTRL", f"  move_pause 결과: {result}")
                    update_status(is_running, "일시 정지됨")

                elif command == "simulate_collision":
                    log("CTRL", "🔴 충돌 시뮬레이션 처리")
                    collide_event.set()
                    pause_event.set()
                    if not SIMULATION_MODE and pause_cli:
                        log("CTRL", "  ROS2 move_pause 서비스 호출 중...")
                        result = call_ros2_service_safe(pause_cli, MovePause.Request(), "move_pause")
                        log("CTRL", f"  move_pause 결과: {result}")
                    update_status(is_running, "충돌 감지 (시뮬레이션)")

                elif command == "resume":
                    log("CTRL", "▶  작동 재개 처리")
                    pause_event.clear()
                    collide_event.clear()
                    if not SIMULATION_MODE and resume_cli:
                        log("CTRL", "  ROS2 move_resume 서비스 호출 중...")
                        result = call_ros2_service_safe(resume_cli, MoveResume.Request(), "move_resume")
                        log("CTRL", f"  move_resume 결과: {result}")
                    update_status(is_running, "작동 재개 중...")

                elif command == "resume_collision":
                    log("CTRL", "▶  충돌 해제 및 재개 처리")
                    collide_event.clear()
                    pause_event.clear()
                    if not SIMULATION_MODE and resume_cli:
                        log("CTRL", "  ROS2 move_resume 서비스 호출 중...")
                        result = call_ros2_service_safe(resume_cli, MoveResume.Request(), "move_resume")
                        log("CTRL", f"  move_resume 결과: {result}")
                    update_status(is_running, "충돌 해제 및 재개 중...")

                else:
                    log("CTRL", f"[경고] 알 수 없는 명령: '{command}'")

                # 처리한 명령 삭제
                try:
                    control_queue_ref.child(req_id).delete()
                    log("CTRL", f"  명령 삭제 완료: {req_id}")
                except Exception as e:
                    log("CTRL", f"  [경고] 명령 삭제 실패: {e}")

            # --- 시작 요청 처리 ---
            try:
                pending_requests = command_queue_ref.get() or {}
            except Exception as e:
                log("LOOP", f"[경고] Firebase 시작 큐 읽기 실패: {e}")
                pending_requests = {}

            if pending_requests:
                log("LOOP", f"시작 요청 {len(pending_requests)}개 감지 | is_running={is_running}")

            if pending_requests and not is_running:
                for request_id, request_data in pending_requests.items():
                    request_data = request_data or {}
                    sauce = request_data.get("sauce", "선택없음")
                    powder = request_data.get("powder", "선택없음")
                    log("LOOP", f"시작 요청 처리: id={request_id}, sauce={sauce}, powder={powder}")

                    try:
                        command_queue_ref.child(request_id).delete()
                    except Exception as e:
                        log("LOOP", f"[경고] 시작 요청 삭제 실패: {e}")

                    if not is_running:
                        is_running = True
                        log("LOOP", f"✅ 로봇 작업 스레드 시작")
                        # 시작 시 일시정지/충돌 플래그 초기화
                        pause_event.clear()
                        collide_event.clear()
                        robot_thread = threading.Thread(
                            target=run_robot_task,
                            args=(request_id, sauce, powder),
                            daemon=True
                        )
                        robot_thread.start()
                    else:
                        log("LOOP", f"[무시] 요청 {request_id}: 이미 작동 중")

            elif pending_requests and is_running:
                for request_id in pending_requests:
                    log("LOOP", f"[무시] 요청 {request_id}: 이미 작동 중")
                    try:
                        command_queue_ref.child(request_id).delete()
                    except Exception:
                        pass

            # 10루프마다 현재 상태 출력 (5초마다)
            if loop_count % 10 == 0:
                log("HEARTBEAT", f"running={is_running} | paused={_is_paused()} | collided={_is_collided()}")

            time.sleep(0.5)

    except KeyboardInterrupt:
        log("MAIN", "종료 신호(Ctrl+C) 수신 → 서버 종료 중...")
        try:
            command_queue_ref.delete()
            control_queue_ref.delete()
            log("MAIN", "명령 큐 정리 완료")
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
                log("MAIN", "ROS2 종료 완료")
            except Exception as e:
                log("MAIN", f"ROS2 종료 중 경고 (무시 가능): {e}")
        log("MAIN", "백엔드 서버 종료됨")


if __name__ == "__main__":
    main()