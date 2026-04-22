import rclpy
import time
from rclpy.node import Node
try:
    from dsr_msgs2.srv import (
        SetCtrlBoxDigitalOutput,
        GetCtrlBoxDigitalInput,
        CheckForceCondition,
    )
except ImportError:
    pass

# ===== 로봇 설정 상수 =====
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"
DEFAULT_VELOCITY = 150
DEFAULT_ACC = 100
ON, OFF = 1, 0

# ===== 그리퍼 프리셋 정의 (매뉴얼 기준 6종) =====
GRIPPER_PRESETS = {
    'grip_20mm':    {3: OFF, 2: OFF, 1: ON},  # 완전 그립
    'grip_12mm':    {3: OFF, 2: ON,  1: ON},  # 약한 그립
    'grip_40mm':    {3: ON,  2: OFF, 1: ON},  # 중간 그립 (소스병)
    'grip_50mm':    {3: ON,  2: ON,  1: OFF}, # 넓은 그립 (반죽)
    'release_65mm': {3: OFF, 2: ON,  1: OFF}, # 중간 열림
    'release_90mm': {3: ON,  2: OFF, 1: OFF}, # 완전 열림
}

class IOManager:
    """로봇 I/O 및 모션을 중앙 관리하는 래퍼 클래스."""

    def __init__(self, node=None, pause_check=None):
        self._node = node
        self._pause_check = pause_check
        self._robot_id = "dsr01"
        
        if node:
            self._logger = node.get_logger()
            # 네임스페이스 자동 인식
            ns = node.get_namespace().strip('/')
            if ns:
                self._robot_id = ns
            
            # 서비스 클라이언트
            self._cli_set_digital_output = node.create_client(SetCtrlBoxDigitalOutput, f'/{self._robot_id}/io/set_ctrl_box_digital_output')
            self._cli_get_digital_input = node.create_client(GetCtrlBoxDigitalInput, f'/{self._robot_id}/io/get_digital_input')
            self._cli_check_force_condition = node.create_client(CheckForceCondition, f'/{self._robot_id}/force/check_force_condition')
        else:
            # 시뮬레이션 모드 대응
            print(f"[IOManager] Simulation Mode Enabled (Robot ID: {self._robot_id})")

    def _wait_if_paused(self):
        if self._pause_check:
            self._pause_check()

    def _safe_motion_loop(self, amotion_func, *args, **kwargs):
        """비동기 모션 실행 및 실시간 상태 감시 루프"""
        if not self._node:
            print(f"[SIM] Motion: {amotion_func.__name__} {args}")
            time.sleep(1.0)
            return

        from DR_init import check_motion
        self._wait_if_paused()
        res = amotion_func(*args, **kwargs)
        if res == -1:
            raise RuntimeError("Motion command rejected by controller.")

        # 시작 대기
        sw = time.time()
        while time.time() - sw < 1.0:
            if check_motion() == 2: break
            time.sleep(0.01)
        
        # BUSY 루프
        while True:
            state = check_motion()
            if state == 1: break
            self._wait_if_paused()
            if state == 0:
                time.sleep(0.1)
                if check_motion() == 0:
                    raise RuntimeError("Robot motion was interrupted.")
            time.sleep(0.05)

    def set_digital_output(self, index, value):
        if not self._node: return
        self._wait_if_paused()
        req = SetCtrlBoxDigitalOutput.Request()
        req.index = index
        req.value = value
        future = self._cli_set_digital_output.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)

    def get_digital_input(self, index):
        if not self._node: return 0
        req = GetCtrlBoxDigitalInput.Request()
        req.index = index
        future = self._cli_get_digital_input.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
        return future.result().value if future.result() else 0

    def wait_input_ok(self, signal_index, poll_interval=0.5):
        if not self._node: return
        from DSR_ROBOT2 import wait
        while not self.get_digital_input(signal_index):
            self._wait_if_paused()
            wait(poll_interval)

    def check_force_condition(self, axis, min_val=0.0, max_val=0.0, ref=0):
        if not self._node: return False
        req = CheckForceCondition.Request()
        req.axis = axis
        req.min = float(min_val)
        req.max = float(max_val)
        req.ref = ref
        future = self._cli_check_force_condition.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
        return future.result().success if future.result() else False

    def gripper_open(self, size='90mm'):
        self._apply_gripper_preset(f'release_{size}')

    def gripper_close(self, size='20mm'):
        self._apply_gripper_preset(f'grip_{size}')

    def _apply_gripper_preset(self, preset_name):
        if preset_name in GRIPPER_PRESETS:
            self._wait_if_paused()
            config = GRIPPER_PRESETS[preset_name]
            for index in [3, 2, 1]:
                self.set_digital_output(index, config[index])

    def move_joint_safe(self, pos, vel=DEFAULT_VELOCITY, acc=DEFAULT_ACC, time_val=0, radius=0, mod=0, ra=0):
        if not self._node:
            self._safe_motion_loop(None, pos)
            return
        from DR_init import amovej
        self._safe_motion_loop(amovej, pos, vel=vel, acc=acc, time=time_val, radius=radius, mod=mod, ra=ra)

    def move_line_safe(self, pos, vel=DEFAULT_VELOCITY, acc=DEFAULT_ACC, time_val=0, radius=0, mod=0, ra=0, ref=None):
        if not self._node:
            self._safe_motion_loop(None, pos)
            return
        from DR_init import amovel
        if ref is not None:
            self._safe_motion_loop(amovel, pos, vel=vel, acc=acc, time=time_val, radius=radius, mod=mod, ra=ra, ref=ref)
        else:
            self._safe_motion_loop(amovel, pos, vel=vel, acc=acc, time=time_val, radius=radius, mod=mod, ra=ra)

    def set_tool_tcp(self, tool_name=ROBOT_TOOL, tcp_name=ROBOT_TCP):
        if not self._node: return
        from DSR_ROBOT2 import set_tool, set_tcp
        self._wait_if_paused()
        set_tool(tool_name)
        set_tcp(tcp_name)

    def set_robot_mode_autonomous(self):
        if not self._node: return
        from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        self._wait_if_paused()
        set_robot_mode(ROBOT_MODE_MANUAL)
        time.sleep(0.5)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(1.0)
