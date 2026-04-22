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
safety_stop_event = threading.Event()
emergency_stop_event = threading.Event()
status_ref = None
command_queue_ref = None
control_queue_ref = None

def log(tag, msg):
    print(f"[{time.strftime('%H:%M:%S')}][{tag}] {msg}", flush=True)

_control_node = None
_task_node = None
_io_manager = None

def call_control_service(client, request, name, timeout=3.0):
    global _control_node
    if client is None or _control_node is None: return False
    try:
        if not client.wait_for_service(timeout_sec=1.0):
            log("CTRL", f"  [경고] {name} 서비스 없음")
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(_control_node, future, timeout_sec=timeout)
        return True if future.done() and future.result() else False
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

def robot_state_callback(msg):
    try:
        state = msg.robot_state
        if state == 5 and not safety_stop_event.is_set():
            safety_stop_event.set()
            collide_event.set()
            pause_event.set()
            log("SAFETY", "안전정지(Protective Stop) 감지 - 노란불")
            update_status(is_running, "안전정지 발생 (노란불)")
        elif state == 6 and not emergency_stop_event.is_set():
            emergency_stop_event.set()
            pause_event.set()
            log("SAFETY", "비상정지(Emergency Stop) 감지 - 빨간불")
            update_status(is_running, "비상정지 발생 (빨간불)")
    except Exception as e: log("SAFETY", f"RobotState 오류: {e}")

def run_robot_task(request_id, sauce, powder):
    global is_running, _task_node, _io_manager
    is_running = True
    update_status(True, "작동 준비 중", sauce, powder)

    try:
        from cobot1.managers.tool_manager import ToolManager
        from cobot1.managers.object_manager import ObjectManager
        from cobot1.tasks.dough_task import DoughTask
        from cobot1.tasks.press_task import PressTask
        from cobot1.tasks.flip_task import FlipTask
        from cobot1.tasks.sauce_task import SauceTask
        from cobot1.tasks.powder_task import PowderTask

        node = _task_node
        io = _io_manager
        tool_mgr = ToolManager(node, io=io)
        obj_mgr = ObjectManager(node, io=io)
        dough = DoughTask(node, io=io)
        press = PressTask(node, io=io)
        flip_ = FlipTask(node, io=io)
        sauce_task = SauceTask(node, io=io)
        powder_task = PowderTask(node, io=io)

        steps = [
            ("작업 1/6: 반죽 집기", lambda: (tool_mgr.pick_tool('tongs'), dough.place_dough_with_tongs(), tool_mgr.return_tool('tongs'))),
            ("작업 2/6: 프레스 누르기", lambda: (tool_mgr.pick_tool('presser'), press.press_dough(), tool_mgr.return_tool('presser'))),
            ("작업 3/6: 접시 배치", lambda: (obj_mgr.pick_and_place_plate())),
            ("작업 4/6: 뒤집개 작업", lambda: (tool_mgr.pick_tool('spatula'), flip_.flip_item_with_spatula(), tool_mgr.return_tool('spatula'))),
        ]
        if sauce != "선택없음":
            steps.append(("작업 5/6: 소스 뿌리기", lambda: (tool_mgr.pick_tool('sauce_bottle'), sauce_task.dispense_sauce(), tool_mgr.return_tool('sauce_bottle'))))
        if powder != "선택없음":
            steps.append(("작업 6/6: 가루 뿌리기", lambda: (tool_mgr.pick_tool('powder_bottle'), powder_task.sprinkle_powder(), tool_mgr.return_tool('powder_bottle'))))

        for step_name, step_fn in steps:
            while pause_event.is_set() or collide_event.is_set():
                update_status(True, "중단됨 - 조치 필요" if collide_event.is_set() else "일시 정지됨", sauce, powder)
                time.sleep(0.5)

            update_status(True, step_name, sauce, powder)
            log("TASK", f"시작: {step_name}")
            try:
                step_fn()
                log("TASK", f"완료: {step_name}")
            except Exception as e:
                log("TASK_ERR", f"작업 중단됨: {e}")
                update_status(False, f"작업 중단: {step_name}")
                return

        update_status(False, "완료 - 대기 중")
    except Exception as e:
        log("TASK_ERR", traceback.format_exc())
        update_status(False, f"시스템 오류: {e}")
    finally:
        is_running = False

def main():
    global is_running, _control_node, _task_node, _io_manager
    init_firebase()
    try:
        command_queue_ref.delete()
        control_queue_ref.delete()
    except Exception: pass

    pause_event.clear(); collide_event.clear(); safety_stop_event.clear(); emergency_stop_event.clear()

    from cobot1.helpers.io_manager import IOManager

    if SIMULATION_MODE:
        log("MAIN", "시뮬레이션 모드 활성화")
        _io_manager = IOManager(node=None)
    else:
        log("MAIN", "ROS2 초기화 중...")
        import rclpy
        import DR_init
        import DSR_ROBOT2
        rclpy.init()

        from cobot1.helpers.pose_manager import ROBOT_CONFIG
        ROBOT_ID = ROBOT_CONFIG['robot_id']
        ROBOT_MODEL = ROBOT_CONFIG['robot_model']

        node = rclpy.create_node("robot_backend", namespace=ROBOT_ID)
        DR_init.__dsr__node = node
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DSR_ROBOT2.__dsr__node = node
        DSR_ROBOT2.__dsr__id = ROBOT_ID
        DSR_ROBOT2.__dsr__model = ROBOT_MODEL
        _task_node = node

        def _pause_check():
            if pause_event.is_set() or collide_event.is_set() or \
               safety_stop_event.is_set() or emergency_stop_event.is_set():
                from DR_init import stop
                if collide_event.is_set() or emergency_stop_event.is_set(): stop(1)
                log("SAFETY", "Pause/Safety detected. Waiting for resume...")
                while pause_event.is_set() or collide_event.is_set() or \
                      safety_stop_event.is_set() or emergency_stop_event.is_set():
                    time.sleep(0.1)

        _io_manager = IOManager(node, pause_check=_pause_check)
        _io_manager.set_tool_tcp()
        _io_manager.set_robot_mode_autonomous()

        control_node = rclpy.create_node("robot_control", namespace=ROBOT_ID)
        _control_node = control_node
        pause_cli = control_node.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
        resume_cli = control_node.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
        stop_cli = control_node.create_client(MoveStop, f'/{ROBOT_ID}/motion/move_stop')
        set_mode_cli = control_node.create_client(SetRobotMode, f'/{ROBOT_ID}/system/set_robot_mode')
        state_sub = control_node.create_subscription(RobotState, f'/{ROBOT_ID}/state', robot_state_callback, 10)

    update_status(False, "대기 중")
    log("MAIN", "서버 대기 중...")

    try:
        while True:
            control_reqs = control_queue_ref.get() or {}
            for req_id, data in control_reqs.items():
                cmd = (data or {}).get("command", "")
                log("CTRL", f"명령 수신: {cmd}")

                if cmd == "pause":
                    pause_event.set()
                    if not SIMULATION_MODE: call_control_service(pause_cli, MovePause.Request(), "pause")
                    update_status(is_running, "일시 정지됨")

                elif cmd == "simulate_collision":
                    collide_event.set(); pause_event.set(); safety_stop_event.set()
                    if not SIMULATION_MODE:
                        req = MoveStop.Request(); req.stop_mode = 1
                        call_control_service(stop_cli, req, "stop(DR_HOLD)")
                    update_status(is_running, "충돌 감지 (시뮬레이션)")

                elif cmd in ["resume", "resume_collision"]:
                    if collide_event.is_set() or safety_stop_event.is_set():
                        log("CTRL", "안전 상태 복구 중...")
                        if not SIMULATION_MODE:
                            req = SetRobotMode.Request()
                            req.robot_mode = 0; call_control_service(set_mode_cli, req, "MANUAL"); time.sleep(0.8)
                            req.robot_mode = 1; call_control_service(set_mode_cli, req, "AUTONOMOUS"); time.sleep(0.8)
                    
                    pause_event.clear(); collide_event.clear(); safety_stop_event.clear()
                    if not SIMULATION_MODE: call_control_service(resume_cli, MoveResume.Request(), "resume")
                    update_status(is_running, "작동 재개")

                elif cmd in ["release_safety_stop", "release_emergency_stop"]:
                    log("CTRL", f"{cmd} 수행 중...")
                    safety_stop_event.clear(); emergency_stop_event.clear(); pause_event.clear(); collide_event.clear()
                    if not SIMULATION_MODE:
                        req = SetRobotMode.Request()
                        req.robot_mode = 0; call_control_service(set_mode_cli, req, "MANUAL"); time.sleep(1.0)
                        req.robot_mode = 1; call_control_service(set_mode_cli, req, "AUTONOMOUS"); time.sleep(1.0)
                        call_control_service(resume_cli, MoveResume.Request(), "resume")
                    update_status(is_running, "안전 정지 해제 완료")

                control_queue_ref.child(req_id).delete()

            if not is_running:
                start_reqs = command_queue_ref.get() or {}
                if start_reqs:
                    req_id = list(start_reqs.keys())[0]
                    data = start_reqs[req_id] or {}
                    command_queue_ref.child(req_id).delete()
                    pause_event.clear(); collide_event.clear()
                    threading.Thread(target=run_robot_task, args=(req_id, data.get("sauce", "선택없음"), data.get("powder", "선택없음")), daemon=True).start()

            if not SIMULATION_MODE and _control_node: rclpy.spin_once(_control_node, timeout_sec=0.05)
            time.sleep(0.1)
    except KeyboardInterrupt: pass
    finally:
        if not SIMULATION_MODE: rclpy.shutdown()

if __name__ == "__main__": main()