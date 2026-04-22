import os
import time
import threading
import traceback
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

# ===== 로봇 설정 (PDF 반영) =====
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL  = "Tool Weight"
ROBOT_TCP   = "GripperDA_v1"
VELOCITY    = 40
ACC         = 60

# ROS 2 관련
SIMULATION_MODE = False
SERVICE_ACCOUNT_KEY_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "rokey-fe6a9-firebase-adminsdk-fbsvc-4856a2fb9f.json"
)
DATABASE_URL = "https://rokey-fe6a9-default-rtdb.asia-southeast1.firebasedatabase.app"

# 제어 상수 (PDF 반영)
CONTROL_RESET_SAFE_STOP = 2   # 보호 정지 해제
CONTROL_RESET_SAFE_OFF  = 3   # 서보 켜기 (Safe Off -> Standby)

# 로봇 상태 코드 매핑 (PDF 반영)
ROBOT_STATE_MAP = {
    0:  "STATE_INITIALIZING (초기화 중)",
    1:  "STATE_STANDBY (대기 중 - 정상)",
    2:  "STATE_MOVING (이동 중)",
    3:  "STATE_SAFE_OFF (서보 꺼짐)",
    4:  "STATE_TEACHING (티칭 모드)",
    5:  "STATE_SAFE_STOP (안전 정지 - 외부 충격 등)",
    6:  "STATE_EMERGENCY_STOP (비상 정지)",
    7:  "STATE_HOMMING (호밍 중)",
    8:  "STATE_RECOVERY (복구 모드)",
    9:  "STATE_SAFE_STOP2 (안전 정지 2)",
    10: "STATE_SAFE_OFF2 (서보 꺼짐 2)",
    15: "STATE_NOT_READY (준비 안 됨)"
}

# ===== 전역 상태 =====
is_running     = False
pause_event    = threading.Event()
collide_event  = threading.Event()
current_robot_state = 1
status_ref     = None
command_queue_ref = None
control_queue_ref = None

# ROS 전역
_node          = None
_pause_cli     = None
_resume_cli    = None
_stop_cli      = None
_control_cli   = None

def log(tag, msg):
    print(f"[{time.strftime('%H:%M:%S')}][{tag}] {msg}", flush=True)

# -------------------------------------------------------
# Firebase 초기화
# -------------------------------------------------------
def init_firebase():
    global status_ref, command_queue_ref, control_queue_ref
    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
    except ValueError:
        pass
    status_ref        = db.reference("/robot_status")
    command_queue_ref = db.reference("/robot_commands/start_requests")
    control_queue_ref = db.reference("/robot_commands/control_requests")

def update_status(running, status_text, sauce="선택없음", powder="선택없음"):
    try:
        status_ref.update({
            "is_running":  running,
            "is_paused":   pause_event.is_set(),
            "is_collided": collide_event.is_set(),
            "status_text": status_text,
            "selected_sauce":  sauce,
            "selected_powder": powder,
            "robot_state_code": current_robot_state,
            "robot_state_desc": ROBOT_STATE_MAP.get(current_robot_state, "UNKNOWN"),
            "last_update_timestamp": time.time(),
        })
    except Exception as e:
        log("ERROR", f"Firebase Update Fail: {e}")

# -------------------------------------------------------
# ROS 2 서비스 호출 (PDF의 spin_once 패턴 적용)
# -------------------------------------------------------
def _call_service_sync(cli, request, tag="SRV", timeout_sec=10.0):
    if cli is None or _node is None:
        log(tag, "[Err] 클라이언트가 초기화되지 않았습니다.")
        return False
    
    if not cli.wait_for_service(timeout_sec=2.0):
        log(tag, f"[Err] {cli.srv_name} 서비스를 찾을 수 없습니다.")
        return False

    import rclpy
    future = cli.call_async(request)
    
    start_wait = time.time()
    while not future.done():
        rclpy.spin_once(_node, timeout_sec=0.01)
        if time.time() - start_wait > timeout_sec:
            log(tag, "[Err] 서비스 호출 시간 초과")
            return False
    
    try:
        res = future.result()
        log(tag, f"서비스 호출 결과: {res}")
        # dsr_msgs2의 많은 서비스는 success 필드를 가짐
        return getattr(res, 'success', True)
    except Exception as e:
        log(tag, f"[Err] 서비스 호출 실패: {e}")
        return False

def call_pause():
    if SIMULATION_MODE: return True
    from dsr_msgs2.srv import MovePause
    return _call_service_sync(_pause_cli, MovePause.Request(), "PAUSE")

def call_resume():
    if SIMULATION_MODE: return True
    from dsr_msgs2.srv import MoveResume
    return _call_service_sync(_resume_cli, MoveResume.Request(), "RESUME")

def call_move_stop(stop_mode=1):
    if SIMULATION_MODE: return True
    from dsr_msgs2.srv import MoveStop
    req = MoveStop.Request()
    req.stop_mode = stop_mode
    return _call_service_sync(_stop_cli, req, "STOP")

def call_set_robot_control(control_value):
    if SIMULATION_MODE: return True
    from dsr_msgs2.srv import SetRobotControl
    req = SetRobotControl.Request()
    req.robot_control = control_value
    return _call_service_sync(_control_cli, req, "SET_CONTROL")

# -------------------------------------------------------
# 로봇 초기화 (PDF 반영)
# -------------------------------------------------------
def initialize_robot():
    if SIMULATION_MODE: return
    from DSR_ROBOT2 import (
        set_tool, set_tcp, get_tool, get_tcp,
        get_robot_mode, set_robot_mode,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        set_safe_stop_reset_type
    )
    log("INIT", "로봇 설정 초기화 중...")
    
    # [PDF 권장] 안전 정지 리셋 타입 설정 (0: Program Stop)
    try:
        set_safe_stop_reset_type(0)
    except:
        pass

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)
    
    log("INIT", "Settings Loaded.")
    log("INIT", f"TCP: {get_tcp()}, TOOL: {get_tool()}, MODE: {get_robot_mode()}")

# -------------------------------------------------------
# 로봇 상태 모니터링 스레드 (PDF의 상태 확인 로직 반영)
# -------------------------------------------------------
def robot_state_monitor():
    global current_robot_state
    log("MONITOR", "로봇 상태 모니터링 시작")
    
    while True:
        if SIMULATION_MODE:
            time.sleep(1)
            continue
            
        try:
            from DSR_ROBOT2 import get_robot_state
            current_robot_state = get_robot_state()
            
            # 충돌/안전정지 상태 감지 (5, 6, 9)
            if current_robot_state in [5, 6, 9]:
                if not collide_event.is_set():
                    log("MONITOR", f"!!! 충돌/정지 감지: {current_robot_state} !!!")
                    collide_event.set()
                    pause_event.set()
            # 서보 꺼짐 상태 감지 (3, 10)
            elif current_robot_state in [3, 10]:
                if not collide_event.is_set():
                    log("MONITOR", f"*** 서보 꺼짐 감지: {current_robot_state} ***")
                    collide_event.set()
            # 정상 상태 (1)
            elif current_robot_state == 1:
                if collide_event.is_set():
                    log("MONITOR", "정상 상태 복귀 확인")
                    collide_event.clear()
            
        except Exception as e:
            pass
            
        time.sleep(0.5)

# -------------------------------------------------------
# 충돌 복구 시퀀스 (PDF 반영)
# -------------------------------------------------------
def try_collision_recovery():
    if SIMULATION_MODE:
        collide_event.clear()
        pause_event.clear()
        return True

    from DSR_ROBOT2 import get_robot_state, drl_script_stop, DR_QSTOP_STO
    
    state = get_robot_state()
    log("RECOVERY", f"복구 시도 시작 (현재 상태: {state})")

    # 1. 스크립트 정지
    try:
        drl_script_stop(DR_QSTOP_STO)
    except:
        pass
    time.sleep(1.0)

    # 2. 상태별 리셋 명령
    success = False
    if state in [5, 9]: # Safe Stop
        log("RECOVERY", "보호 정지 리셋 시도 (SetRobotControl 2)")
        success = call_set_robot_control(CONTROL_RESET_SAFE_STOP)
    elif state in [3, 10]: # Safe Off
        log("RECOVERY", "서보 ON 시도 (SetRobotControl 3)")
        success = call_set_robot_control(CONTROL_RESET_SAFE_OFF)
    else:
        log("RECOVERY", "리셋이 필요 없는 상태입니다.")
        success = True

    if success:
        log("RECOVERY", "명령 전송 성공. 안정화 대기...")
        time.sleep(3.0)
        if get_robot_state() == 1:
            log("RECOVERY", "✅ 복구 완료")
            initialize_robot()
            return True
    
    log("RECOVERY", "❌ 복구 실패")
    return False

# -------------------------------------------------------
# 작업 실행 스레드
# -------------------------------------------------------
def run_robot_task(request_id, sauce, powder):
    global is_running
    is_running = True
    update_status(True, "작동 준비 중", sauce, powder)

    try:
        from cobot1.main import (
            perform_task_dough_grip, perform_task_press,
            perform_task_plate_setting, perform_task_spatula,
            perform_task_source, perform_task_powder_snap
        )

        steps = [
            ("작업 1: 반죽 집기", perform_task_dough_grip),
            ("작업 2: 프레스 누르기", perform_task_press),
            ("작업 3: 접시 세팅", perform_task_plate_setting),
            ("작업 4: 뒤집개 작업", perform_task_spatula),
        ]
        if sauce != "선택없음":
            steps.append(("작업 5: 소스 뿌리기", perform_task_source))
        if powder != "선택없음":
            steps.append(("작업 6: 가루 뿌리기", perform_task_powder_snap))

        for step_name, step_fn in steps:
            # 일시정지 또는 충돌 상태면 대기
            while pause_event.is_set() or collide_event.is_set():
                if collide_event.is_set():
                    update_status(True, "충돌/오류 발생 - 조치 필요", sauce, powder)
                else:
                    update_status(True, "일시 정지됨", sauce, powder)
                time.sleep(0.5)

            update_status(True, step_name, sauce, powder)
            log("TASK", f"시작: {step_name}")
            step_fn()
            log("TASK", f"완료: {step_name}")

        update_status(False, "모든 작업 완료")
    except Exception as e:
        log("TASK_ERR", traceback.format_exc())
        update_status(False, f"작업 중단 오류: {e}")
    finally:
        is_running = False

# -------------------------------------------------------
# 메인 루프
# -------------------------------------------------------
def main():
    global _node, _pause_cli, _resume_cli, _stop_cli, _control_cli
    
    init_firebase()
    
    # 큐 정리
    try:
        command_queue_ref.delete()
        control_queue_ref.delete()
    except: pass

    if not SIMULATION_MODE:
        import rclpy
        import DR_init
        from dsr_msgs2.srv import MovePause, MoveResume, MoveStop, SetRobotControl
        
        rclpy.init()
        # PDF와 동일하게 namespace 설정
        _node = rclpy.create_node("move_pause_resume", namespace=ROBOT_ID)
        
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = _node
        
        # 서비스 클라이언트 초기화 (절대 경로)
        _pause_cli = _node.create_client(MovePause, f"/{ROBOT_ID}/motion/move_pause")
        _resume_cli = _node.create_client(MoveResume, f"/{ROBOT_ID}/motion/move_resume")
        _stop_cli = _node.create_client(MoveStop, f"/{ROBOT_ID}/motion/move_stop")
        _control_cli = _node.create_client(SetRobotControl, f"/{ROBOT_ID}/system/set_robot_control")
        
        initialize_robot()
        
        # 상태 모니터링 스레드 시작
        threading.Thread(target=robot_state_monitor, daemon=True).start()

    log("MAIN", "서버 대기 중...")
    update_status(False, "대기 중")

    try:
        while True:
            # 1. 제어 명령 처리
            try:
                control_reqs = control_queue_ref.get() or {}
            except: control_reqs = {}

            for req_id, data in control_reqs.items():
                cmd = data.get("command", "")
                log("CTRL", f"명령 수신: {cmd}")

                if cmd == "pause":
                    pause_event.set()
                    call_pause()
                    update_status(is_running, "일시 정지됨")
                
                elif cmd == "resume":
                    pause_event.clear()
                    call_resume()
                    update_status(is_running, "작동 재개")

                elif cmd == "simulate_collision":
                    collide_event.set()
                    pause_event.set()
                    call_move_stop(1) # DR_HOLD
                    update_status(is_running, "충돌 감지됨!")

                elif cmd == "resume_collision":
                    log("CTRL", "충돌 복구 시퀀스 시작")
                    if try_collision_recovery():
                        collide_event.clear()
                        pause_event.clear()
                        call_resume()
                        update_status(is_running, "복구 완료 - 재개")
                    else:
                        update_status(is_running, "복구 실패 - 수동 조치 필요")

                control_queue_ref.child(req_id).delete()

            # 2. 시작 요청 처리
            if not is_running:
                try:
                    start_reqs = command_queue_ref.get() or {}
                except: start_reqs = {}

                if start_reqs:
                    req_id = list(start_reqs.keys())[0]
                    data = start_reqs[req_id]
                    sauce = data.get("sauce", "선택없음")
                    powder = data.get("powder", "선택없음")
                    
                    command_queue_ref.child(req_id).delete()
                    pause_event.clear()
                    collide_event.clear()
                    
                    threading.Thread(
                        target=run_robot_task,
                        args=(req_id, sauce, powder),
                        daemon=True
                    ).start()

            # ROS 1회 spin
            if _node:
                import rclpy
                rclpy.spin_once(_node, timeout_sec=0.05)
            
            # 실시간 상태 동기화 (모니터링 결과 반영)
            if not is_running:
                update_status(False, ROBOT_STATE_MAP.get(current_robot_state, "대기 중"))

            time.sleep(0.1)

    except KeyboardInterrupt:
        log("MAIN", "종료 중...")
    finally:
        if _node:
            import rclpy
            _node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()