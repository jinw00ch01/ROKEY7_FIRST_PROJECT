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

    # 클래스 레벨: 마지막 그리퍼 프리셋 추적 (안전정지/비상정지 시 복원용)
    _last_gripper_preset = None

    def __init__(self, node: Node, pause_check=None, interrupt_check=None):
        """IOManager 초기화.

        Args:
            node: ROS2 노드. 서비스 클라이언트 생성 및 spin에 사용.
            pause_check: 일시정지 확인 콜백 (선택).
                         설정 시 매 모션 전에 호출되어 일시정지 상태면 블로킹.
                         robot_backend에서 threading.Event 기반 대기에 사용.
            interrupt_check: 중단 확인 콜백 (선택).
                         모션 완료 후 호출하여 정지 이벤트 발생 여부를 확인.
                         True 반환 시 모션이 중단된 것으로 판단하여 재실행.
        """
        self._node = node
        self._logger = node.get_logger()
        self._pause_check = pause_check
        self._interrupt_check = interrupt_check

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
        # 마지막 프리셋 기록 (안전정지/비상정지 복구 시 복원용)
        IOManager._last_gripper_preset = preset_name

    def restore_gripper(self):
        """마지막으로 적용했던 그리퍼 프리셋을 다시 적용한다.

        안전정지/비상정지로 인해 디지털 출력이 리셋되었을 때 호출.
        """
        preset = IOManager._last_gripper_preset
        if preset is None:
            self._logger.info('[IOManager] restore_gripper: 복원할 프리셋 없음 (건너뜀)')
            return
        self._logger.info(f'[IOManager] restore_gripper: {preset} 재적용')
        config = GRIPPER_PRESETS[preset]
        for index in [3, 2, 1]:
            self.set_digital_output(index, config[index])

    # ===== 일시정지 지원 =====

    def _wait_if_paused(self):
        """pause_check 콜백이 설정되어 있으면 호출하여 일시정지 중 블로킹."""
        if self._pause_check:
            self._pause_check()

    def _was_interrupted(self) -> bool:
        """모션 후 안전정지/충돌로 중단되었는지 확인.

        mwait() 리턴 후 상태 폴링(0.1초 주기)이 아직 감지 못했을 수 있으므로
        즉시 확인 후 0.15초 대기 + 재확인으로 레이스 컨디션을 방지한다.
        중단된 경우 _wait_if_paused()로 해제 대기 후 True 반환.
        호출자는 모션을 재실행해야 한다.
        """
        if self._interrupt_check is None:
            return False
        # 즉시 확인
        if self._interrupt_check():
            self._logger.info('[IOManager] 모션 중단 감지 - 재개 대기 중...')
            self._wait_if_paused()
            return True
        # 폴링 주기(0.1초) 대기 후 재확인
        import time
        time.sleep(0.15)
        if self._interrupt_check():
            self._logger.info('[IOManager] 모션 중단 감지(지연) - 재개 대기 중...')
            self._wait_if_paused()
            return True
        return False

    # ===== Tool/TCP 검증 =====

    _tool_tcp_ok = False  # 클래스 레벨 플래그 (첫 모션 전 1회 검증)

    def _ensure_tool_tcp(self):
        """현재 Tool/TCP 설정이 올바른지 확인.

        매 모션 전에 호출되지만 실제 검증은 세션당 1회만 수행.
        - 빈 문자열 → 모드 전환 등으로 리셋됨 → 자동 재설정 시도
        - 비어있지 않은데 불일치 → 컨트롤러 설정 오류 → RuntimeError
        """
        if IOManager._tool_tcp_ok:
            return
        try:
            from DSR_ROBOT2 import get_tool, get_tcp, set_tool, set_tcp
            cur_tool = get_tool()
            cur_tcp = get_tcp()

            # 빈 문자열 = 모드 전환으로 인한 리셋 → 자동 재설정
            if not cur_tool or not cur_tcp:
                self._logger.warning(
                    f'[IOManager] Tool/TCP 빈 값 감지 (tool="{cur_tool}", tcp="{cur_tcp}") → 재설정')
                set_tool(ROBOT_TOOL)
                set_tcp(ROBOT_TCP)
                IOManager._tool_tcp_ok = True
                self._logger.info(
                    f'[IOManager] Tool/TCP 재설정 완료: tool={ROBOT_TOOL}, tcp={ROBOT_TCP}')
                return

            # 비어있지 않은데 불일치 → 컨트롤러 설정 오류
            errors = []
            if cur_tool != ROBOT_TOOL:
                errors.append(
                    f'Tool 불일치: 현재="{cur_tool}", 필요="{ROBOT_TOOL}"')
            if cur_tcp != ROBOT_TCP:
                errors.append(
                    f'TCP 불일치: 현재="{cur_tcp}", 필요="{ROBOT_TCP}"')
            if errors:
                msg = ' | '.join(errors)
                self._logger.error(
                    f'[IOManager] {msg} → 로봇 컨트롤러에서 설정을 확인하세요.')
                raise RuntimeError(
                    f'Tool/TCP 설정 불일치 - 동작 거부. {msg}. '
                    f'DART-Platform에서 Tool="{ROBOT_TOOL}", TCP="{ROBOT_TCP}"로 설정 후 재시작하세요.')

            IOManager._tool_tcp_ok = True
            self._logger.info(
                f'[IOManager] Tool/TCP 검증 통과: tool="{cur_tool}", tcp="{cur_tcp}"')
        except RuntimeError:
            raise
        except Exception as e:
            self._logger.error(f'[IOManager] Tool/TCP 검증 오류: {e}')

    @classmethod
    def invalidate_tool_tcp(cls):
        """Tool/TCP 검증 플래그를 리셋. 안전정지/비상정지 복구 후 호출."""
        cls._tool_tcp_ok = False

    # ===== 로봇 모션 =====

    def move_joint_safe(self, pos, vel: int = DEFAULT_VELOCITY,
                        acc: int = DEFAULT_ACC, time_val: float = 0,
                        radius: float = 0, mod: int = 0, ra: int = 0):
        """관절 공간에서 안전하게 이동한다.

        대규모 이동, 장애물 회피에 적합.
        안전정지/충돌로 모션이 중단되면 대기 후 자동 재실행.

        Args:
            pos: 목표 관절 위치 [J1, J2, J3, J4, J5, J6] (도).
            vel: 속도 (기본 150).
            acc: 가속도 (기본 100).
            time_val: 이동 시간 (0=속도 기반).
            radius: 블렌딩 반경.
            mod: 이동 모드 (0=절대, 1=상대).
            ra: 특이점 처리.
        """
        from DSR_ROBOT2 import movej, mwait
        while True:
            self._wait_if_paused()
            self._ensure_tool_tcp()
            self._logger.debug(f'move_joint_safe: pos={pos}, vel={vel}, acc={acc}')
            movej(pos, vel=vel, acc=acc, time=time_val, radius=radius, mod=mod, ra=ra)
            mwait()
            if not self._was_interrupted():
                break
            self._logger.info('[IOManager] move_joint_safe 중단됨 - 재실행 대기')

    def move_line_safe(self, pos, vel: int = DEFAULT_VELOCITY,
                       acc: int = DEFAULT_ACC, time_val: float = 0,
                       radius: float = 0, mod: int = 0, ra: int = 0,
                       ref: int = None):
        """직선(카테시안)으로 안전하게 이동한다.

        정밀한 접근/삽입/후퇴에 적합.
        안전정지/충돌로 모션이 중단되면 대기 후 자동 재실행.

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
        from DSR_ROBOT2 import movel, mwait
        while True:
            self._wait_if_paused()
            self._ensure_tool_tcp()
            self._logger.debug(f'move_line_safe: pos={pos}, vel={vel}, acc={acc}')
            if ref is not None:
                movel(pos, vel=vel, acc=acc, time=time_val, radius=radius,
                      mod=mod, ra=ra, ref=ref)
            else:
                movel(pos, vel=vel, acc=acc, time=time_val, radius=radius,
                      mod=mod, ra=ra)
            mwait()
            if not self._was_interrupted():
                break
            self._logger.info('[IOManager] move_line_safe 중단됨 - 재실행 대기')

    def move_circle_safe(self, pos_via, pos_to, vel: int = DEFAULT_VELOCITY,
                         acc: int = DEFAULT_ACC, time_val: float = 0,
                         radius: float = 0, mod: int = 0, ra: int = 0,
                         ref: int = None):
        """원호(호) 경로로 안전하게 이동한다.

        접시 pick처럼 곡선 접근이 필요한 경우에 사용.
        안전정지/충돌로 모션이 중단되면 대기 후 자동 재실행.

        Args:
            pos_via: 경유 카테시안 위치 [X, Y, Z, Rx, Ry, Rz].
            pos_to: 목표 카테시안 위치 [X, Y, Z, Rx, Ry, Rz].
            vel: 속도 (기본 150).
            acc: 가속도 (기본 100).
            time_val: 이동 시간 (0=속도 기반).
            radius: 블렌딩 반경.
            mod: 이동 모드 (0=절대, 1=상대).
            ra: 특이점 처리.
            ref: 참조 좌표계 (None=기본).
        """
        from DSR_ROBOT2 import movec, mwait
        while True:
            self._wait_if_paused()
            self._ensure_tool_tcp()
            self._logger.debug(f'move_circle_safe: via={pos_via}, to={pos_to}, vel={vel}, acc={acc}')
            if ref is not None:
                movec(pos_via, pos_to, vel=vel, acc=acc, time=time_val,
                      radius=radius, mod=mod, ra=ra, ref=ref)
            else:
                movec(pos_via, pos_to, vel=vel, acc=acc, time=time_val,
                      radius=radius, mod=mod, ra=ra)
            mwait()
            if not self._was_interrupted():
                break
            self._logger.info('[IOManager] move_circle_safe 중단됨 - 재실행 대기')

    def move_spline_safe(self, pos_list, vel=None, acc=None,
                         time_val: float = 0, ref: int = None,
                         mod: int = 0, vel_opt: int = None):
        """스플라인 곡선 경로로 여러 웨이포인트를 연속 이동한다 (movesx).

        여러 movel을 하나의 부드러운 곡선으로 연결하여 효율적으로 이동.
        안전정지/충돌로 모션이 중단되면 대기 후 자동 재실행.

        Args:
            pos_list: posx 좌표 리스트 [posx1, posx2, ...].
            vel: 속도. float 또는 [선속도, 각속도]. None이면 글로벌 설정 사용.
            acc: 가속도. float 또는 [선가속도, 각가속도]. None이면 글로벌 설정 사용.
            time_val: 이동 시간 (0=속도 기반).
            ref: 참조 좌표계 (None=기본, DR_BASE 등).
            mod: 이동 모드 (0=절대, 1=상대).
            vel_opt: 속도 옵션 (DR_MVS_VEL_NONE, DR_MVS_VEL_CONST).
        """
        from DSR_ROBOT2 import movesx, mwait
        while True:
            self._wait_if_paused()
            self._ensure_tool_tcp()
            self._logger.debug(
                f'move_spline_safe: {len(pos_list)} waypoints, vel={vel}, acc={acc}')
            kwargs = {}
            if vel is not None:
                kwargs['vel'] = vel
            if acc is not None:
                kwargs['acc'] = acc
            if time_val:
                kwargs['time'] = time_val
            if ref is not None:
                kwargs['ref'] = ref
            if mod:
                kwargs['mod'] = mod
            if vel_opt is not None:
                kwargs['vel_opt'] = vel_opt
            movesx(pos_list, **kwargs)
            mwait()
            if not self._was_interrupted():
                break
            self._logger.info('[IOManager] move_spline_safe 중단됨 - 재실행 대기')

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
        """로봇을 자동(Autonomous) 모드로 전환한다.

        강의안 기준 초기화 순서:
        MANUAL → set_tool/tcp → AUTONOMOUS → 안정화 대기
        MANUAL 전환 시 Tool/TCP가 리셋되므로 반드시 MANUAL 상태에서 재설정해야 한다.
        """
        from DSR_ROBOT2 import set_robot_mode, set_tool, set_tcp
        from DSR_ROBOT2 import ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        import time
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        time.sleep(2)
        self._logger.info(
            f'[IOManager] Robot mode AUTONOMOUS (tool={ROBOT_TOOL}, tcp={ROBOT_TCP})')
