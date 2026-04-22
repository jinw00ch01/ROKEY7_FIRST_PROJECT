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
    from dsr_msgs2.srv import MovePause, MoveResume, MoveStop
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
status_ref = None
command_queue_ref = None
control_queue_ref = None

def log(tag, msg):
    print(f"[{time.strftime('%H:%M:%S')}][{tag}] {msg}", flush=True)

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
            "status_text": status_text,
            "selected_sauce": sauce,
            "selected_powder": powder,
            "last_update_timestamp": time.time()
        })
    except Exception as e: log("ERROR", f"Firebase Update Fail: {e}")

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
    global is_running
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

    if SIMULATION_MODE:
        log("MAIN", "시뮬레이션 모드로 실행 중 (로봇 미연결)")
        node = None
        pause_cli = resume_cli = stop_cli = None
    else:
        # ROS 2 초기화
        log("MAIN", "ROS2 초기화 중...")
        rclpy.init()
        node = rclpy.create_node("robot_backend_control")
        log("MAIN", "robot_backend_control 노드 생성 완료")

        # 서비스 클라이언트 설정
        pause_cli = node.create_client(MovePause, '/dsr01/motion/move_pause')
        resume_cli = node.create_client(MoveResume, '/dsr01/motion/move_resume')
        stop_cli = node.create_client(MoveStop, '/dsr01/motion/move_stop')
        log("MAIN", f"서비스 클라이언트 생성 완료")
        log("MAIN", f"  pause : /dsr01/motion/move_pause")
        log("MAIN", f"  resume: /dsr01/motion/move_resume")
        log("MAIN", f"  stop  : /dsr01/motion/move_stop")

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
                    if not SIMULATION_MODE and pause_cli:
                        if pause_cli.wait_for_service(timeout_sec=1.0):
                            pause_cli.call_async(MovePause.Request())
                            log("CTRL", "  move_pause 비동기 호출 완료")
                        else:
                            log("CTRL", "  [경고] move_pause 서비스 없음")
                    update_status(is_running, "일시 정지됨")

                elif cmd == "simulate_collision":
                    collide_event.set()
                    pause_event.set()
                    log("CTRL", "🔴 충돌 시뮬레이션 - collide/pause_event 설정")
                    if not SIMULATION_MODE and stop_cli:
                        if stop_cli.wait_for_service(timeout_sec=1.0):
                            req = MoveStop.Request()
                            req.stop_mode = 1  # DR_HOLD: 즉시 강제 정지
                            stop_cli.call_async(req)
                            log("CTRL", "  move_stop(DR_HOLD) 비동기 호출 완료")
                        else:
                            log("CTRL", "  [경고] move_stop 서비스 없음")
                    update_status(is_running, "충돌 감지됨!")

                elif cmd in ["resume", "resume_collision"]:
                    pause_event.clear()
                    collide_event.clear()
                    log("CTRL", "▶  pause/collide_event 해제")
                    if not SIMULATION_MODE and resume_cli:
                        if resume_cli.wait_for_service(timeout_sec=1.0):
                            resume_cli.call_async(MoveResume.Request())
                            log("CTRL", "  move_resume 비동기 호출 완료")
                        else:
                            log("CTRL", "  [경고] move_resume 서비스 없음")
                    update_status(is_running, "작동 재개 중...")

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

            # ROS 노드 1회 spin (이벤트 처리)
            if not SIMULATION_MODE and node:
                rclpy.spin_once(node, timeout_sec=0.05)

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
        if not SIMULATION_MODE and node:
            try:
                node.destroy_node()
                rclpy.shutdown()
                log("MAIN", "ROS2 종료 완료")
            except Exception as e:
                log("MAIN", f"ROS2 종료 중 경고: {e}")
        log("MAIN", "백엔드 서버 종료됨")

if __name__ == "__main__":
    main()