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
    from dsr_msgs2.srv import (
        MovePause, MoveResume, MoveStop, SetRobotMode,
        SetRobotControl, GetRobotState,
        SetCtrlBoxDigitalOutput,
        DrlStop,
    )
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
_task_node = None
_io_manager = None
_digital_out_cli = None  # 제어 노드용 디지털 출력 클라이언트 (그리퍼 복원용)
_drl_stop_cli = None     # DrlStop 서비스 클라이언트 (블로킹 해제용)

def call_control_service(client, request, name, timeout=3.0):
    """제어 노드에서 서비스를 동기적으로 호출 (executor 충돌 방지).

    control_node는 전용 스레드에서 spin 중이므로,
    spin_until_future_complete 대신 future 완료를 폴링으로 대기한다.
    """
    if client is None:
        return False
    try:
        if not client.wait_for_service(timeout_sec=1.0):
            log("CTRL", f"  [경고] {name} 서비스 없음")
            return False
        future = client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)
        if future.done() and future.result() is not None:
            log("CTRL", f"  {name} 호출 성공")
            return True
        else:
            log("CTRL", f"  {name} 호출 타임아웃")
            return False
    except Exception as e:
        log("CTRL", f"  {name} 호출 오류: {e}")
        return False

def _restore_gripper():
    """마지막 그리퍼 프리셋을 제어 노드 경유로 재적용 (executor 충돌 방지).

    IOManager.restore_gripper()는 태스크 노드의 spin_until_future_complete를 사용하므로
    폴링/제어 스레드에서 호출하면 'generator already executing' 에러 발생.
    대신 제어 노드의 디지털 출력 클라이언트 + call_control_service(폴링 대기)를 사용한다.
    """
    global _digital_out_cli
    if _digital_out_cli is None or SIMULATION_MODE:
        return
    try:
        from cobot1.helpers.io_manager import IOManager, GRIPPER_PRESETS
        preset = IOManager._last_gripper_preset
        if preset is None:
            return
        config = GRIPPER_PRESETS[preset]
        log("CTRL", f"  그리퍼 복원: {preset} (제어 노드 경유)")
        for index in [3, 2, 1]:
            req = SetCtrlBoxDigitalOutput.Request()
            req.index = index
            req.value = config[index]
            call_control_service(_digital_out_cli, req,
                                 f"digital_out({index}={config[index]})", timeout=2.0)
    except Exception as e:
        log("CTRL", f"  그리퍼 복원 오류: {e}")

_set_mode_cli = None  # SetRobotMode 서비스 클라이언트 (제어 노드)

def _reinitialize_robot():
    """안전정지/비상정지 해제 후 로봇 재초기화.

    주의: 이 함수는 메인 스레드(제어 명령 루프)에서 호출된다.
    태스크 노드의 spin_until_future_complete와 충돌하지 않도록
    제어 노드의 서비스 클라이언트만 사용한다.
    """
    global _set_mode_cli
    try:
        from cobot1.helpers.io_manager import IOManager
        # Tool/TCP 검증 플래그만 리셋 → 다음 모션 시 태스크 스레드에서 자동 재설정
        IOManager.invalidate_tool_tcp()

        # AUTONOMOUS 모드 전환 (제어 노드 경유)
        if _set_mode_cli is not None:
            # MANUAL(0) → AUTONOMOUS(1) 순서로 설정
            mode_req = SetRobotMode.Request()
            mode_req.robot_mode = 0  # MANUAL
            call_control_service(_set_mode_cli, mode_req,
                                 "set_robot_mode(MANUAL)", timeout=5.0)
            time.sleep(0.5)
            mode_req.robot_mode = 1  # AUTONOMOUS
            call_control_service(_set_mode_cli, mode_req,
                                 "set_robot_mode(AUTONOMOUS)", timeout=5.0)
            time.sleep(2.0)

        # 그리퍼 상태 복원 (제어 노드 경유)
        _restore_gripper()
        log("CTRL", "  로봇 재초기화 완료 (mode AUTONOMOUS + gripper restore)")
    except Exception as e:
        log("CTRL", f"  로봇 재초기화 오류: {e}")

def init_firebase():
    global status_ref, command_queue_ref, control_queue_ref
    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
    except ValueError: pass
    status_ref = db.reference("/robot_status")
    command_queue_ref = db.reference("/robot_commands/start_requests")
    control_queue_ref = db.reference("/robot_commands/control_requests")

def update_status(running, status_text, source="선택없음", powder="선택없음"):
    try:
        status_ref.update({
            "is_running": running,
            "is_paused": pause_event.is_set(),
            "is_collided": collide_event.is_set(),
            "is_safety_stopped": safety_stop_event.is_set(),
            "is_emergency_stopped": emergency_stop_event.is_set(),
            "status_text": status_text,
            "selected_source": source,
            "selected_powder": powder,
            "last_update_timestamp": time.time()
        })
    except Exception as e: log("ERROR", f"Firebase Update Fail: {e}")

# ===== 로봇 상태 모니터링 (GetRobotState 서비스 폴링) =====
_get_state_cli = None  # GetRobotState 서비스 클라이언트

def _process_robot_state(state):
    """로봇 상태 값을 처리하여 안전정지/비상정지/충돌 이벤트를 설정"""
    # 5: SAFE_STOP, 10: SAFE_STOP2 (보호정지/안전정지 - 노란불)
    # 충돌 감지로 간주: collide_event도 설정하여 UI에 충돌 모달 표시
    if state in (5, 10) and not safety_stop_event.is_set():
        safety_stop_event.set()
        collide_event.set()
        pause_event.set()
        log("SAFETY", f"충돌/안전정지(Protective Stop) 감지 - 노란불 (state={state})")
        # 즉시 그리퍼 복원 (정지 시 디지털 출력 리셋 방지)
        _restore_gripper()
        update_status(is_running, "충돌 감지됨!")
    # 6, 7: EMERGENCY_STOP (비상정지 - 빨간불)
    elif state in (6, 7) and not emergency_stop_event.is_set():
        emergency_stop_event.set()
        pause_event.set()
        log("SAFETY", "비상정지(Emergency Stop) 감지 - 빨간불")
        # 즉시 그리퍼 복원 (정지 시 디지털 출력 리셋 방지)
        _restore_gripper()
        update_status(is_running, "비상정지 발생 (빨간불)")
    # 3: SAFE_OFF, 11: SAFE_OFF2 (서보 OFF 상태)
    elif state in (3, 11) and not emergency_stop_event.is_set() and not safety_stop_event.is_set():
        safety_stop_event.set()
        pause_event.set()
        log("SAFETY", f"서보 OFF 감지 (state={state})")
        # 즉시 그리퍼 복원 (정지 시 디지털 출력 리셋 방지)
        _restore_gripper()
        update_status(is_running, "서보 OFF - 복구 필요")
    # 1: STANDBY - 정상 복귀 시 이벤트 자동 클리어
    elif state == 1:
        if safety_stop_event.is_set() or emergency_stop_event.is_set() or collide_event.is_set():
            safety_stop_event.clear()
            emergency_stop_event.clear()
            collide_event.clear()
            pause_event.clear()
            log("SAFETY", "로봇 STATE_STANDBY 복귀 - 이벤트 자동 클리어")
            if is_running:
                update_status(True, "작동 재개 중...")
            else:
                update_status(False, "대기 중")

def _poll_robot_state_loop():
    """GetRobotState 서비스를 주기적으로 호출하여 로봇 상태를 감시"""
    global _get_state_cli
    while rclpy.ok():
        try:
            if _get_state_cli is None:
                time.sleep(1.0)
                continue
            if not _get_state_cli.wait_for_service(timeout_sec=0.5):
                time.sleep(1.0)
                continue
            future = _get_state_cli.call_async(GetRobotState.Request())
            start = time.time()
            while not future.done() and (time.time() - start) < 2.0:
                time.sleep(0.05)
            if future.done() and future.result() is not None:
                _process_robot_state(future.result().robot_state)
        except Exception as e:
            log("STATE_POLL", f"상태 폴링 오류: {e}")
        time.sleep(0.1)

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

def run_robot_task(request_id, source, powder):
    """새 노드 아키텍처(ToolManager + tasks + IOManager) 기반 작업 실행"""
    global is_running
    is_running = True
    update_status(True, "작동 준비 중", source, powder)

    try:
        from cobot1.managers.tool_manager import ToolManager
        from cobot1.managers.object_manager import ObjectManager
        from cobot1.tasks.dough_task import DoughTask
        from cobot1.tasks.press_task import PressTask
        from cobot1.tasks.flip_task import FlipTask
        from cobot1.tasks.source_task import SourceTask
        from cobot1.tasks.powder_task import PowderTask

        node = _task_node
        io = _io_manager
        tool_mgr = ToolManager(node, io=io)
        obj_mgr = ObjectManager(node, io=io)
        dough = DoughTask(node, io=io)
        press = PressTask(node, io=io)
        flip_ = FlipTask(node, io=io)
        source_task = SourceTask(node, io=io)
        powder_task = PowderTask(node, io=io)

        steps = [
            ("작업 1/6: 반죽 집기", lambda: (
                tool_mgr.pick_tool('tongs'),
                dough.place_dough_with_tongs(),
                tool_mgr.return_tool('tongs'),
            )),
            ("작업 2/6: 프레스 누르기", lambda: (
                tool_mgr.pick_tool('presser'),
                press.press_dough(),
                tool_mgr.return_tool('presser'),
            )),
            ("작업 3/6: 접시 배치", lambda: (
                obj_mgr.pick_and_place_plate(),
            )),
            ("작업 4/6: 뒤집개 작업", lambda: (
                tool_mgr.pick_tool('spatula'),
                flip_.flip_item_with_spatula(),
                tool_mgr.return_tool('spatula'),
            )),
        ]
        if source != "선택없음":
            steps.append(("작업 5/6: 소스 뿌리기", lambda: (
                tool_mgr.pick_tool('source_bottle'),
                source_task.dispense_source(),
                tool_mgr.return_tool('source_bottle'),
            )))
        if powder != "선택없음":
            steps.append(("작업 6/6: 가루 뿌리기", lambda: (
                tool_mgr.pick_tool('powder_bottle'),
                powder_task.sprinkle_powder(),
                tool_mgr.return_tool('powder_bottle'),
            )))

        for step_name, step_fn in steps:
            while pause_event.is_set() or collide_event.is_set():
                if collide_event.is_set():
                    update_status(True, "충돌/오류 발생 - 조치 필요", source, powder)
                else:
                    update_status(True, "일시 정지됨", source, powder)
                time.sleep(0.5)

            update_status(True, step_name, source, powder)
            log("TASK", f"시작: {step_name}")
            step_fn()
            log("TASK", f"완료: {step_name}")

        update_status(False, "완료 - 대기 중")
    except Exception as e:
        log("TASK_ERR", traceback.format_exc())
        update_status(False, f"오류 발생: {e}")
    finally:
        is_running = False

def main():
    global is_running, _control_node, _task_node, _io_manager, _get_state_cli
    global _digital_out_cli, _set_mode_cli, _drl_stop_cli
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
        robot_ctrl_cli = None
    else:
        # ROS 2 초기화
        log("MAIN", "ROS2 초기화 중...")
        import DR_init
        rclpy.init()

        from cobot1.helpers.pose_manager import ROBOT_CONFIG
        from cobot1.helpers.io_manager import IOManager
        ROBOT_ID = ROBOT_CONFIG['robot_id']
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_CONFIG['robot_model']

        # 태스크용 노드
        node = rclpy.create_node("robot_backend", namespace=ROBOT_ID)
        DR_init.__dsr__node = node
        _task_node = node
        log("MAIN", "robot_backend 태스크 노드 생성 완료")

        # IOManager로 로봇 초기화 (한 번만 생성)
        def _pause_check():
            """일시정지/충돌/안전정지/비상정지 중이면 해제될 때까지 블로킹."""
            while pause_event.is_set() or collide_event.is_set() or \
                  safety_stop_event.is_set() or emergency_stop_event.is_set():
                time.sleep(0.1)

        def _interrupt_check():
            """모션 완료 후 정지 이벤트가 발생했는지 확인. True면 모션이 중단된 것."""
            return (pause_event.is_set() or collide_event.is_set() or
                    safety_stop_event.is_set() or emergency_stop_event.is_set())

        _io_manager = IOManager(node, pause_check=_pause_check,
                                interrupt_check=_interrupt_check)
        _io_manager.set_robot_mode_autonomous()  # MANUAL → set_tool/tcp → AUTONOMOUS (강의안 순서)
        log("MAIN", "로봇 초기화 완료 (IOManager + pause_check 연동)")

        # 제어용 별도 노드 (태스크 노드와 executor 충돌 방지)
        control_node = rclpy.create_node("robot_control", namespace=ROBOT_ID)
        pause_cli = control_node.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
        resume_cli = control_node.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
        stop_cli = control_node.create_client(MoveStop, f'/{ROBOT_ID}/motion/move_stop')
        set_mode_cli = control_node.create_client(SetRobotMode, f'/{ROBOT_ID}/system/set_robot_mode')
        robot_ctrl_cli = control_node.create_client(SetRobotControl, f'/{ROBOT_ID}/system/set_robot_control')

        # 그리퍼 복원용 디지털 출력 클라이언트 (제어 노드 경유 → executor 충돌 방지)
        _digital_out_cli = control_node.create_client(
            SetCtrlBoxDigitalOutput, f'/{ROBOT_ID}/io/set_ctrl_box_digital_output')
        _set_mode_cli = set_mode_cli  # _reinitialize_robot에서 사용

        # DrlStop 클라이언트 — 안전정지 시 블로킹된 드라이버 해제용 (강의안 drl_script_stop 대응)
        _drl_stop_cli = control_node.create_client(
            DrlStop, f'/{ROBOT_ID}/drl/drl_stop')

        log("MAIN", "제어용 별도 노드 + 서비스 클라이언트 생성 완료")
        log("MAIN", f"  pause       : /{ROBOT_ID}/motion/move_pause")
        log("MAIN", f"  resume      : /{ROBOT_ID}/motion/move_resume")
        log("MAIN", f"  stop        : /{ROBOT_ID}/motion/move_stop")
        log("MAIN", f"  set_mode    : /{ROBOT_ID}/system/set_robot_mode")
        log("MAIN", f"  robot_ctrl  : /{ROBOT_ID}/system/set_robot_control")
        log("MAIN", f"  digital_out : /{ROBOT_ID}/io/set_ctrl_box_digital_output")
        log("MAIN", f"  drl_stop    : /{ROBOT_ID}/drl/drl_stop")

        # GetRobotState 서비스 폴링 (토픽 발행이 없으므로 서비스로 대체)
        _get_state_cli = control_node.create_client(
            GetRobotState, f'/{ROBOT_ID}/system/get_robot_state')
        log("MAIN", f"GetRobotState 서비스 클라이언트 생성: /{ROBOT_ID}/system/get_robot_state")

        _control_node = control_node  # 전역 참조 설정

        # control_node를 전용 스레드에서 spin (태스크 스레드와 executor 충돌 방지)
        _control_executor = SingleThreadedExecutor()
        _control_executor.add_node(control_node)
        _control_spin_thread = threading.Thread(
            target=_control_executor.spin, daemon=True)
        _control_spin_thread.start()
        log("MAIN", "제어 노드 전용 spin 스레드 시작")

        # 로봇 상태 폴링 스레드 시작
        _state_poll_thread = threading.Thread(
            target=_poll_robot_state_loop, daemon=True)
        _state_poll_thread.start()
        log("MAIN", "로봇 상태 폴링 스레드 시작 (0.1초 주기)")

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

                elif cmd == "resume":
                    pause_event.clear()
                    collide_event.clear()
                    log("CTRL", "▶  pause/collide_event 해제")
                    if not SIMULATION_MODE:
                        call_control_service(resume_cli, MoveResume.Request(), "move_resume")
                    update_status(is_running, "작동 재개 중...")

                elif cmd == "resume_collision":
                    log("CTRL", "충돌 해제 및 재개 시도...")
                    if not SIMULATION_MODE and safety_stop_event.is_set():
                        # 실제 충돌(state 5/9) → 안전정지 복구 절차 (강의안 기준)
                        # 1단계: DrlStop — 드라이버의 블로킹된 모션 호출 해제
                        log("CTRL", "  [1/4] DrlStop (드라이버 블로킹 해제)")
                        drl_req = DrlStop.Request()
                        drl_req.stop_mode = 0  # DR_QSTOP_STO
                        call_control_service(_drl_stop_cli, drl_req,
                                             "drl_stop(QSTOP_STO)", timeout=5.0)
                        time.sleep(3.0)  # 드라이버 정리 + 외력 제거 대기

                        # 2단계: SetRobotControl(2) - 보호정지 리셋 (State 5 → State 1)
                        log("CTRL", "  [2/4] 보호정지 리셋 (SetRobotControl(2))")
                        ctrl_req = SetRobotControl.Request()
                        ctrl_req.robot_control = 2  # CONTROL_RESET_SAFE_STOP
                        call_control_service(robot_ctrl_cli, ctrl_req,
                                             "set_robot_control(RESET_SAFE_STOP)", timeout=5.0)
                        time.sleep(3.0)  # 상태 전환 대기

                        # 3단계: 로봇 재초기화 (AUTONOMOUS + gripper restore)
                        log("CTRL", "  [3/4] 로봇 재초기화")
                        _reinitialize_robot()

                        # 4단계: 모션 재개
                        log("CTRL", "  [4/4] 모션 재개 (MoveResume)")
                        call_control_service(resume_cli, MoveResume.Request(),
                                             "move_resume", timeout=5.0)
                    elif not SIMULATION_MODE:
                        # 시뮬레이션 충돌 → move_resume만
                        call_control_service(resume_cli, MoveResume.Request(), "move_resume")
                    safety_stop_event.clear()
                    collide_event.clear()
                    pause_event.clear()
                    update_status(is_running, "충돌 해제 - 작동 재개 중...")

                elif cmd == "release_safety_stop":
                    log("CTRL", "안전정지(노란불) 해제 시도...")
                    if not SIMULATION_MODE:
                        # 1. DrlStop — 드라이버 블로킹 해제
                        log("CTRL", "  [1/4] DrlStop (드라이버 블로킹 해제)")
                        drl_req = DrlStop.Request()
                        drl_req.stop_mode = 0
                        call_control_service(_drl_stop_cli, drl_req,
                                             "drl_stop(QSTOP_STO)", timeout=5.0)
                        time.sleep(3.0)

                        # 2. SetRobotControl(2)로 SAFE_STOP → STANDBY 리셋
                        log("CTRL", "  [2/4] 보호정지 리셋 (SetRobotControl(2))")
                        ctrl_req = SetRobotControl.Request()
                        ctrl_req.robot_control = 2
                        call_control_service(robot_ctrl_cli, ctrl_req,
                                             "set_robot_control(RESET_SAFE_STOP)", timeout=5.0)
                        time.sleep(3.0)

                        # 3. 로봇 재초기화 (AUTONOMOUS + gripper restore)
                        log("CTRL", "  [3/4] 로봇 재초기화")
                        _reinitialize_robot()

                        # 4. 모션 재개
                        log("CTRL", "  [4/4] 모션 재개")
                        call_control_service(resume_cli, MoveResume.Request(),
                                             "move_resume", timeout=5.0)
                    safety_stop_event.clear()
                    pause_event.clear()
                    update_status(is_running, "안전정지 해제 - 작동 재개")

                elif cmd == "release_emergency_stop":
                    log("CTRL", "비상정지(빨간불) 해제 시도...")
                    log("CTRL", "  ※ 물리적 비상정지 버튼이 해제되어 있어야 합니다.")
                    log("CTRL", "  ※ 버튼 해제 후 상태: 6(E-STOP) → 3(SAFE_OFF)")
                    if not SIMULATION_MODE:
                        # 1. DrlStop — 드라이버 블로킹 해제
                        log("CTRL", "  [1/4] DrlStop (드라이버 블로킹 해제)")
                        drl_req = DrlStop.Request()
                        drl_req.stop_mode = 0
                        call_control_service(_drl_stop_cli, drl_req,
                                             "drl_stop(QSTOP_STO)", timeout=5.0)
                        time.sleep(1.0)

                        # 2. SetRobotControl(3)으로 서보 ON (SAFE_OFF → STANDBY)
                        log("CTRL", "  [2/4] 서보 ON (SetRobotControl(3))")
                        ctrl_req = SetRobotControl.Request()
                        ctrl_req.robot_control = 3
                        call_control_service(robot_ctrl_cli, ctrl_req,
                                             "set_robot_control(SERVO_ON)", timeout=5.0)
                        time.sleep(3.0)  # 브레이크 해제 대기

                        # 3. 로봇 재초기화 (AUTONOMOUS + gripper restore)
                        log("CTRL", "  [3/4] 로봇 재초기화")
                        _reinitialize_robot()

                        # 4. 모션 재개
                        log("CTRL", "  [4/4] 모션 재개")
                        call_control_service(resume_cli, MoveResume.Request(),
                                             "move_resume", timeout=5.0)
                    emergency_stop_event.clear()
                    pause_event.clear()
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
                    source = data.get("source", "선택없음") or "선택없음"
                    powder = data.get("powder", "선택없음") or "선택없음"
                    log("LOOP", f"시작 요청: id={req_id}, source={source}, powder={powder}")
                    try:
                        command_queue_ref.child(req_id).delete()
                    except Exception as e:
                        log("LOOP", f"시작 요청 삭제 실패: {e}")
                    pause_event.clear()
                    collide_event.clear()
                    threading.Thread(
                        target=run_robot_task,
                        args=(req_id, source, powder),
                        daemon=True
                    ).start()
                    log("LOOP", "✅ 로봇 작업 스레드 시작")

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