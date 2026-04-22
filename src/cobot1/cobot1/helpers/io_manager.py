"""
io_manager.py
로봇 모션 및 I/O 호출을 중앙 집중화하는 래퍼 모듈.

- ROS2 서비스 상호작용을 이 모듈 안에 격리
- 프로젝트 전체에서 안전하고 단순한 함수를 제공
- 기존 main.py, press_test, source_test, powder_test에 중복된 IO 코드를 일원화

사용법:
    from cobot1.helpers.io_manager import IOManager
    io = IOManager(node)
    io.gripper_open('90mm')
    io.move_joint_safe([0, 0, 90, 0, 90, 0], vel=150, acc=100)
"""
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import (
    SetCtrlBoxDigitalOutput,
    GetCtrlBoxDigitalInput,
    CheckForceCondition,
)

# ===== 로봇 설정 상수 =====
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"
DEFAULT_VELOCITY = 150
DEFAULT_ACC = 100
ON, OFF = 1, 0

# ===== 그리퍼 프리셋 정의 =====
# 디지털 출력 (index 3, 2, 1)의 ON/OFF 조합
GRIPPER_PRESETS = {
    'grip_20mm':    {3: OFF, 2: OFF, 1: ON},
    'grip_12mm':    {3: OFF, 2: ON,  1: ON},
    'grip_40mm':    {3: ON,  2: OFF, 1: ON},
    'grip_50mm':    {3: ON,  2: ON,  1: OFF},
    'release_65mm': {3: OFF, 2: ON,  1: OFF},
    'release_90mm': {3: ON,  2: OFF, 1: OFF},
}


class IOManager:
    """로봇 I/O 및 모션을 중앙 관리하는 래퍼 클래스.

    ROS2 서비스 클라이언트를 생성하고, 그리퍼/힘 센서/모션 명령에 대한
    안전하고 단순한 인터페이스를 제공한다.
    """

    def __init__(self, node: Node):
        """IOManager 초기화.

        Args:
            node: ROS2 노드. 서비스 클라이언트 생성 및 spin에 사용.
        """
        self._node = node
        self._logger = node.get_logger()

        # ROS2 서비스 클라이언트 생성
        self._cli_set_digital_output = node.create_client(
            SetCtrlBoxDigitalOutput,
            f'/{ROBOT_ID}/io/set_ctrl_box_digital_output')
        self._cli_get_digital_input = node.create_client(
            GetCtrlBoxDigitalInput,
            f'/{ROBOT_ID}/io/get_ctrl_box_digital_input')
        self._cli_check_force_condition = node.create_client(
            CheckForceCondition,
            f'/{ROBOT_ID}/force/check_force_condition')

        # 서비스 대기
        self._cli_set_digital_output.wait_for_service(timeout_sec=5.0)
        self._cli_get_digital_input.wait_for_service(timeout_sec=5.0)
        self._cli_check_force_condition.wait_for_service(timeout_sec=5.0)
        self._logger.info('[IOManager] IO/Force service clients ready.')

    # ===== 디지털 I/O =====

    def set_digital_output(self, index: int, value: int):
        """디지털 출력을 설정한다.

        Args:
            index: 출력 핀 번호 (1~16).
            value: ON(1) 또는 OFF(0).
        """
        req = SetCtrlBoxDigitalOutput.Request()
        req.index = index
        req.value = value
        future = self._cli_set_digital_output.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        result = future.result()
        if result is None or not result.success:
            self._logger.warn(f'set_digital_output({index}, {value}) failed')

    def get_digital_input(self, index: int) -> int:
        """디지털 입력 값을 읽는다.

        Args:
            index: 입력 핀 번호 (1~16).

        Returns:
            핀의 현재 값 (0 또는 1). 실패 시 0.
        """
        req = GetCtrlBoxDigitalInput.Request()
        req.index = index
        future = self._cli_get_digital_input.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        result = future.result()
        if result is None:
            return 0
        return result.value

    def wait_input_ok(self, signal_index: int, poll_interval: float = 0.5):
        """디지털 입력 신호가 ON이 될 때까지 대기한다.

        Args:
            signal_index: 감시할 입력 핀 번호.
            poll_interval: 폴링 간격 (초). 기본 0.5초.
        """
        from DSR_ROBOT2 import wait
        while not self.get_digital_input(signal_index):
            wait(poll_interval)

    # ===== 힘 센서 =====

    def check_force_condition(self, axis: int, min_val: float = 0.0,
                              max_val: float = 0.0, ref: int = 0) -> bool:
        """힘/토크 조건을 확인한다.

        Args:
            axis: 확인할 축 (0=Fx, 1=Fy, 2=Fz, 3=Mx, 4=My, 5=Mz).
            min_val: 최소 임계값.
            max_val: 최대 임계값.
            ref: 참조 좌표계 (0=BASE, 1=TOOL).

        Returns:
            조건 충족 시 True, 아니면 False.
        """
        req = CheckForceCondition.Request()
        req.axis = axis
        req.min = float(min_val)
        req.max = float(max_val)
        req.ref = ref
        future = self._cli_check_force_condition.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        result = future.result()
        if result is None:
            return False
        return result.success

    # ===== 그리퍼 제어 =====

    def gripper_open(self, size: str = '90mm'):
        """그리퍼를 연다.

        Args:
            size: 열림 크기. '65mm' 또는 '90mm'. 기본 '90mm'.
        """
        preset_name = f'release_{size}'
        self._apply_gripper_preset(preset_name)

    def gripper_close(self, size: str = '20mm'):
        """그리퍼를 닫는다 (그립).

        Args:
            size: 그립 크기. '20mm', '12mm', '40mm', '50mm'. 기본 '20mm'.
        """
        preset_name = f'grip_{size}'
        self._apply_gripper_preset(preset_name)

    def _apply_gripper_preset(self, preset_name: str):
        """그리퍼 프리셋을 적용한다.

        Args:
            preset_name: GRIPPER_PRESETS의 키 이름.
        """
        if preset_name not in GRIPPER_PRESETS:
            self._logger.error(f'Unknown gripper preset: {preset_name}')
            return

        config = GRIPPER_PRESETS[preset_name]
        self._logger.info(f'[IOManager] gripper: {preset_name}')
        for index in [3, 2, 1]:
            self.set_digital_output(index, config[index])

    # ===== 로봇 모션 =====

    def move_joint_safe(self, pos, vel: int = DEFAULT_VELOCITY,
                        acc: int = DEFAULT_ACC, time_val: float = 0,
                        radius: float = 0, mod: int = 0, ra: int = 0):
        """관절 공간에서 안전하게 이동한다.

        대규모 이동, 장애물 회피에 적합.

        Args:
            pos: 목표 관절 위치 [J1, J2, J3, J4, J5, J6] (도).
            vel: 속도 (기본 150).
            acc: 가속도 (기본 100).
            time_val: 이동 시간 (0=속도 기반).
            radius: 블렌딩 반경.
            mod: 이동 모드 (0=절대, 1=상대).
            ra: 특이점 처리.
        """
        from DSR_ROBOT2 import movej
        self._logger.debug(f'move_joint_safe: pos={pos}, vel={vel}, acc={acc}')
        movej(pos, vel=vel, acc=acc, time=time_val, radius=radius, mod=mod, ra=ra)

    def move_line_safe(self, pos, vel: int = DEFAULT_VELOCITY,
                       acc: int = DEFAULT_ACC, time_val: float = 0,
                       radius: float = 0, mod: int = 0, ra: int = 0,
                       ref: int = None):
        """직선(카테시안)으로 안전하게 이동한다.

        정밀한 접근/삽입/후퇴에 적합.

        Args:
            pos: 목표 카테시안 위치 [X, Y, Z, Rx, Ry, Rz].
            vel: 속도 (기본 150).
            acc: 가속도 (기본 100).
            time_val: 이동 시간 (0=속도 기반).
            radius: 블렌딩 반경.
            mod: 이동 모드 (0=절대, 1=상대).
            ra: 특이점 처리.
            ref: 참조 좌표계 (None=기본, DR_BASE, DR_TOOL 등).
        """
        from DSR_ROBOT2 import movel
        self._logger.debug(f'move_line_safe: pos={pos}, vel={vel}, acc={acc}')
        if ref is not None:
            movel(pos, vel=vel, acc=acc, time=time_val, radius=radius,
                  mod=mod, ra=ra, ref=ref)
        else:
            movel(pos, vel=vel, acc=acc, time=time_val, radius=radius,
                  mod=mod, ra=ra)

    # ===== 로봇 초기화 =====

    def set_tool_tcp(self, tool_name: str = ROBOT_TOOL,
                     tcp_name: str = ROBOT_TCP):
        """로봇의 Tool과 TCP를 설정한다.

        Args:
            tool_name: 도구 이름 (기본 'Tool Weight').
            tcp_name: TCP 이름 (기본 'GripperDA_v1').
        """
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool(tool_name)
        set_tcp(tcp_name)
        self._logger.info(f'[IOManager] set_tool_tcp: tool={tool_name}, tcp={tcp_name}')

    def set_robot_mode_autonomous(self):
        """로봇을 자동(Autonomous) 모드로 전환한다."""
        from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        import time
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(2)
        self._logger.info('[IOManager] Robot mode set to AUTONOMOUS.')
