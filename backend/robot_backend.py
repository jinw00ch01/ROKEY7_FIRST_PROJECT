import os
import time
import threading
import traceback
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

# ROS 2 서비스 관련
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from dsr_msgs2.srv import MovePause, MoveResume, MoveStop, SetRobotMode
    from dsr_msgs2.msg import RobotState
except ImportError:
    pass

# ===== 설정 =====
SIMULATION_MODE = False 
SERVICE_ACCOUNT_KEY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rokey-fe6a9-firebase-adminsdk-fbsvc-4856a2fb9f.json")
DATABASE_URL = "https://rokey-fe6a9-default-rtdb.asia-southeast1.firebasedatabase.app"

# ===== 상태 관리 =====
is_running = False
pause_event = threading.Event()
collide_event = threading.Event()
safety_stop_event = threading.Event()      # 안전정지 (노란불)
emergency_stop_event = threading.Event()   # 비상정지 (빨간불)
status_ref = None
command_queue_ref = None
control_queue_ref = None

def log(tag, msg):
    print(f"[{time.strftime('%H:%M:%S')}][{tag}] {msg}", flush=True)

# 제어 노드 전역 참조 (서비스 호출 시 spin용)
_control_node = None

def call_control_service(client, request, name, timeout=3.0):
    """제어 노드에서 서비스를 동기적으로 호출 (executor 충돌 방지)"""
    global _control_node
    if client is None or _control_node is None:
        return False
    try:
        if not client.wait_for_service(timeout_sec=1.0):
            log("CTRL", f"  [경고] {name} 서비스 없음")
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(_control_node, future, timeout_sec=timeout)
        if future.done() and future.result() is not None:
            log("CTRL", f"  {name} 호출 성공")
            return True
        else:
            log("CTRL", f"  {name} 호출 타임아웃")
            return False
    except Exception as e:
        log("CTRL", f"  {name} 호출 오류: {e}")
        return False

def init_firebase():
    global status_ref, command_queue_ref, control_queue_ref
    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
    except ValueError: pass
    status_ref = db.reference("/robot_status")
    command_queue_ref = db.reference("/robot_commands/start_requests")
    control_queue_ref = db.reference("/robot_commands/control_requests")

def update_status(running, status_text, sauce="선택없음", powder="선택없음"):
    try:
        status_ref.update({
            "is_running": running,
            "is_paused": pause_event.is_set(),
            "is_collided": collide_event.is_set(),
            "is_safety_stopped": safety_stop_event.is_set(),
            "is_emergency_stopped": emergency_stop_event.is_set(),
            "status_text": status_text,
            "selected_sauce": sauce,
            "selected_powder": powder,
            "last_update_timestamp": time.time()
        })
    except Exception as e: log("ERROR", f"Firebase Update Fail: {e}")

# ===== 로봇 상태 모니터링 (RobotState 토픽 콜백) =====
def robot_state_callback(msg):
    """RobotState 토픽을 구독하여 안전정지/비상정지를 자동 감지"""
    try:
        state = msg.robot_state
        # 5: SAFE_STOP (보호정지/안전정지 - 노란불)
        if state == 5 and not safety_stop_event.is_set():
            safety_stop_event.set()
            pause_event.set()
            log("SAFETY", "안전정지(Protective Stop) 감지 - 노란불")
            update_status(is_running, "안전정지 발생 (노란불)")
        # 6: EMERGENCY_STOP (비상정지 - 빨간불)
        elif state == 6 and not emergency_stop_event.is_set():
            emergency_stop_event.set()
            pause_event.set()
            log("SAFETY", "비상정지(Emergency Stop) 감지 - 빨간불")
            update_status(is_running, "비상정지 발생 (빨간불)")
    except Exception as e:
        log("SAFETY", f"RobotState 콜백 오류: {e}")

# ===== 핵심: 안전한 이동 래퍼 함수 (Tasks에서 호출해야 함) =====
def safe_motion_wrapper(motion_func, *args, **kwargs):
    """비동기 모션을 실행하고, 폴링을 통해 일시정지/충돌을 감시합니다."""
    if SIMULATION_MODE:
        time.sleep(2)
        return

    # 모션 시작 (예: amovel)
    motion_func(*args, **kwargs)
    
    from DR_init import check_motion, stop
    # 모션이 진행 중인 동안(2: BUSY) 루프를 돌며 상태 감시
    while check_motion() == 2:
        if collide_event.is_set():
            stop(1) # DR_HOLD: 즉시 강제 정지
            log("SAFETY", "Collision Detected! Immediate Stop.")
            while collide_event.is_set(): time.sleep(0.1)
            break # 정지 후 루프 탈출 (재개 로직은 상위에서 처리)
        
        if pause_event.is_set():
            stop(0) # DR_SSTOP: 부드러운 정지
            log("SAFETY", "Pause Requested. Soft Stop.")
            while pause_event.is_set(): time.sleep(0.1)
            break
        time.sleep(0.05)

def run_robot_task(request_id, sauce, powder):
    global is_running
    is_running = True
    update_status(True, "작동 중", sauce, powder)
    
    try:
        # 실제 로봇 모듈 import
        from cobot1.main import (
            perform_task_dough_grip, perform_task_press,
            perform_task_plate_setting, perform_task_spatula,
            perform_task_source, perform_task_powder_snap
        )
        
        # 각 작업 단계 (내부에서 safe_motion_wrapper를 사용하도록 수정 권장)
        update_status(True, "작업 1: 반죽 집기", sauce, powder)
        perform_task_dough_grip()
        
        update_status(True, "작업 2: 프레스 누르기", sauce, powder)
        perform_task_press()
        
        update_status(True, "작업 3: 접시 세팅", sauce, powder)
        perform_task_plate_setting()
        
        update_status(True, "작업 4: 뒤집개 작업", sauce, powder)
        perform_task_spatula()

        if sauce != "선택없음":
            update_status(True, "작업 5: 소스 뿌리기", sauce, powder)
            perform_task_source()

        if powder != "선택없음":
            update_status(True, "작업 6: 가루 뿌리기", sauce, powder)
            perform_task_powder_snap()

        update_status(False, "완료 - 대기 중")
    except Exception as e:
        log("TASK_ERR", traceback.format_exc())
        update_status(False, f"오류 발생: {e}")
    finally:
        is_running = False

def main():
    global is_running, _control_node
    init_firebase()

    # 이전 세션 잔여 명령 큐 정리
    try:
        command_queue_ref.delete()
        control_queue_ref.delete()
        log("MAIN", "잔여 명령 큐 정리 완료")
    except Exception as e:
        log("MAIN", f"큐 초기화 중 오류 (무시 가능): {e}")

    pause_event.clear()
    collide_event.clear()
    safety_stop_event.clear()
    emergency_stop_event.clear()

    if SIMULATION_MODE:
        log("MAIN", "시뮬레이션 모드로 실행 중 (로봇 미연결)")
        node = None
        control_node = None
        pause_cli = resume_cli = stop_cli = set_mode_cli = None
    else:
        # ROS 2 초기화
        log("MAIN", "ROS2 초기화 중...")
        import DR_init
        rclpy.init()

        from cobot1.main import ROBOT_ID, initialize_robot, setup_io_clients
        from cobot1 import press_test, source_test, powder_test

        # 태스크용 노드 (DSR_ROBOT2가 spin_until_future_complete로 점유)
        node = rclpy.create_node("robot_backend", namespace=ROBOT_ID)
        DR_init.__dsr__node = node
        log("MAIN", "robot_backend 태스크 노드 생성 완료")

        initialize_robot()
        setup_io_clients(node)
        press_test.setup_io_clients(node)
        source_test.setup_io_clients(node)
        powder_test.setup_io_clients(node)
        log("MAIN", "로봇 초기화 및 IO 클라이언트 설정 완료")

        # 제어용 별도 노드 (태스크 노드와 executor 충돌 방지)
        control_node = rclpy.create_node("robot_control", namespace=ROBOT_ID)
        pause_cli = control_node.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
        resume_cli = control_node.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
        stop_cli = control_node.create_client(MoveStop, f'/{ROBOT_ID}/motion/move_stop')
        set_mode_cli = control_node.create_client(SetRobotMode, f'/{ROBOT_ID}/system/set_robot_mode')
        log("MAIN", "제어용 별도 노드 + 서비스 클라이언트 생성 완료")
        log("MAIN", f"  pause    : /{ROBOT_ID}/motion/move_pause")
        log("MAIN", f"  resume   : /{ROBOT_ID}/motion/move_resume")
        log("MAIN", f"  stop     : /{ROBOT_ID}/motion/move_stop")
        log("MAIN", f"  set_mode : /{ROBOT_ID}/system/set_robot_mode")

        # RobotState 토픽 구독 (제어 노드에서 구독 - 독립 spin 가능)
        state_sub = control_node.create_subscription(
            RobotState, f'/{ROBOT_ID}/state', robot_state_callback, 10)
        log("MAIN", f"RobotState 구독 시작: /{ROBOT_ID}/state")

        _control_node = control_node  # 전역 참조 설정

    update_status(False, "대기 중")
    log("MAIN", "서버 대기 중...")

    try:
        while True:
            # 1. 제어 명령 처리 (일시정지/재개/충돌)
            try:
                control_reqs = control_queue_ref.get() or {}
            except Exception as e:
                log("LOOP", f"제어 큐 읽기 실패: {e}")
                control_reqs = {}

            for req_id, data in control_reqs.items():
                data = data or {}
                cmd = data.get("command", "")
                log("CTRL", f"명령 수신: '{cmd}' | running={is_running} | paused={pause_event.is_set()} | collided={collide_event.is_set()}")

                if cmd == "pause":
                    pause_event.set()
                    log("CTRL", "⏸  pause_event 설정")
                    if not SIMULATION_MODE:
                        call_control_service(pause_cli, MovePause.Request(), "move_pause")
                    update_status(is_running, "일시 정지됨")

                elif cmd == "simulate_collision":
                    collide_event.set()
                    pause_event.set()
                    log("CTRL", "충돌 시뮬레이션 - collide/pause_event 설정")
                    if not SIMULATION_MODE:
                        req = MoveStop.Request()
                        req.stop_mode = 1  # DR_HOLD: 즉시 강제 정지
                        call_control_service(stop_cli, req, "move_stop(DR_HOLD)")
                    update_status(is_running, "충돌 감지됨!")

                elif cmd in ["resume", "resume_collision"]:
                    pause_event.clear()
                    collide_event.clear()
                    log("CTRL", "▶  pause/collide_event 해제")
                    if not SIMULATION_MODE:
                        call_control_service(resume_cli, MoveResume.Request(), "move_resume")
                    update_status(is_running, "작동 재개 중...")

                elif cmd == "release_safety_stop":
                    log("CTRL", "안전정지(노란불) 해제 시도...")
                    safety_stop_event.clear()
                    pause_event.clear()
                    if not SIMULATION_MODE:
                        mode_req = SetRobotMode.Request()
                        mode_req.robot_mode = 0  # ROBOT_MODE_MANUAL
                        call_control_service(set_mode_cli, mode_req, "set_mode(MANUAL)")
                        time.sleep(0.5)
                        mode_req.robot_mode = 1  # ROBOT_MODE_AUTONOMOUS
                        call_control_service(set_mode_cli, mode_req, "set_mode(AUTONOMOUS)")
                        time.sleep(0.5)
                        call_control_service(resume_cli, MoveResume.Request(), "move_resume")
                    update_status(is_running, "안전정지 해제 - 작동 재개")

                elif cmd == "release_emergency_stop":
                    log("CTRL", "비상정지(빨간불) 해제 시도...")
                    emergency_stop_event.clear()
                    pause_event.clear()
                    if not SIMULATION_MODE:
                        mode_req = SetRobotMode.Request()
                        mode_req.robot_mode = 0  # ROBOT_MODE_MANUAL
                        call_control_service(set_mode_cli, mode_req, "set_mode(MANUAL)")
                        time.sleep(1.0)
                        mode_req.robot_mode = 1  # ROBOT_MODE_AUTONOMOUS
                        call_control_service(set_mode_cli, mode_req, "set_mode(AUTONOMOUS)")
                        time.sleep(1.0)
                        call_control_service(resume_cli, MoveResume.Request(), "move_resume")
                    update_status(is_running, "비상정지 해제 - 작동 재개")

                else:
                    log("CTRL", f"[경고] 알 수 없는 명령: '{cmd}'")

                try:
                    control_queue_ref.child(req_id).delete()
                except Exception as e:
                    log("CTRL", f"명령 삭제 실패: {e}")

            # 2. 시작 요청 처리
            if not is_running:
                try:
                    start_reqs = command_queue_ref.get() or {}
                except Exception as e:
                    log("LOOP", f"시작 큐 읽기 실패: {e}")
                    start_reqs = {}

                if start_reqs:
                    req_id = list(start_reqs.keys())[0]
                    data = start_reqs[req_id] or {}
                    sauce = data.get("sauce", "선택없음") or "선택없음"
                    powder = data.get("powder", "선택없음") or "선택없음"
                    log("LOOP", f"시작 요청: id={req_id}, sauce={sauce}, powder={powder}")
                    try:
                        command_queue_ref.child(req_id).delete()
                    except Exception as e:
                        log("LOOP", f"시작 요청 삭제 실패: {e}")
                    pause_event.clear()
                    collide_event.clear()
                    threading.Thread(
                        target=run_robot_task,
                        args=(req_id, sauce, powder),
                        daemon=True
                    ).start()
                    log("LOOP", "✅ 로봇 작업 스레드 시작")

            # 제어 노드 1회 spin (RobotState 콜백 등 이벤트 처리)
            if not SIMULATION_MODE and control_node:
                rclpy.spin_once(control_node, timeout_sec=0.05)

            time.sleep(0.1)

    except KeyboardInterrupt:
        log("MAIN", "종료 신호 수신 → 서버 종료 중...")
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
    finally:
        if not SIMULATION_MODE:
            try:
                if control_node:
                    control_node.destroy_node()
                if node:
                    node.destroy_node()
                rclpy.shutdown()
                log("MAIN", "ROS2 종료 완료")
            except Exception as e:
                log("MAIN", f"ROS2 종료 중 경고: {e}")
        log("MAIN", "백엔드 서버 종료됨")

if __name__ == "__main__":
    main()