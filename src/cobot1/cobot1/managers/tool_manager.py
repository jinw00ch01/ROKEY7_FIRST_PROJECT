"""
tool_manager.py
도구별 pick/return 전략을 실제 모션으로 구현하는 매니저 모듈.

각 도구의 pick/return 시퀀스를 io_manager + pose_manager 기반으로 수행.
"""
from rclpy.node import Node
from cobot1.helpers.io_manager import IOManager, DEFAULT_VELOCITY, DEFAULT_ACC
from cobot1.helpers.pose_manager import get_poses, JOINT_POSES


class ToolManager:
    """도구 pick/return을 관리하는 헬퍼 클래스"""

    def __init__(self, node: Node, io: IOManager = None):
        self._node = node
        self._io = io or IOManager(node)
        self._logger = node.get_logger()
        self._current_tool = None
        self._poses = None

    def _get_poses(self):
        if self._poses is None:
            self._poses = get_poses()
        return self._poses

    @property
    def current_tool(self) -> str:
        return self._current_tool

    def pick_tool(self, tool_name: str) -> bool:
        """도구를 집는다."""
        if self._current_tool is not None:
            self._logger.warn(
                f'Tool conflict: {tool_name} requested but {self._current_tool} held.')
            return False

        dispatch = {
            'tongs': self._pick_tongs,
            'presser': self._pick_presser,
            'spatula': self._pick_spatula,
            'sauce_bottle': self._pick_sauce_bottle,
            'powder_bottle': self._pick_powder_bottle,
        }
        fn = dispatch.get(tool_name)
        if fn is None:
            self._logger.error(f'Unknown tool: {tool_name}')
            return False

        self._logger.info(f'[ToolManager] Picking up: {tool_name}')
        fn()
        self._current_tool = tool_name
        self._logger.info(f'[ToolManager] {tool_name} picked up.')
        return True

    def return_tool(self, tool_name: str) -> bool:
        """도구를 반납한다."""
        if self._current_tool != tool_name:
            self._logger.warn(
                f'Tool mismatch: returning {tool_name} but holding {self._current_tool}')
            return False

        dispatch = {
            'tongs': self._return_tongs,
            'presser': self._return_presser,
            'spatula': self._return_spatula,
            'sauce_bottle': self._return_sauce_bottle,
            'powder_bottle': self._return_powder_bottle,
        }
        fn = dispatch.get(tool_name)
        if fn is None:
            self._logger.error(f'Unknown tool: {tool_name}')
            return False

        self._logger.info(f'[ToolManager] Returning: {tool_name}')
        fn()
        self._current_tool = None
        self._logger.info(f'[ToolManager] {tool_name} returned.')
        return True

    # ===== 집게 (Tongs) =====

    def _pick_tongs(self):
        io = self._io
        p = self._get_poses()['dough']
        io.gripper_open('90mm')
        io.move_joint_safe(JOINT_POSES['ready'])
        io.move_line_safe(p['pick_start_1'], vel=200, acc=DEFAULT_ACC)
        io.gripper_open('65mm')
        from DSR_ROBOT2 import wait
        wait(0.5)
        io.move_line_safe(p['pick_start_2'], vel=200, acc=150)

    def _return_tongs(self):
        io = self._io
        p = self._get_poses()['dough']
        io.move_line_safe(p['pick_start_2'], vel=DEFAULT_VELOCITY, acc=DEFAULT_ACC)
        io.move_line_safe(p['pick_start_1'], vel=200, acc=DEFAULT_ACC)
        io.gripper_open('90mm')
        io.move_line_safe(p['pick_start_2'], vel=DEFAULT_VELOCITY, acc=DEFAULT_ACC)

    # ===== 프레스 (Presser) =====

    def _pick_presser(self):
        io = self._io
        p = self._get_poses()['press']
        io.gripper_open('65mm')
        io.move_joint_safe(JOINT_POSES['ready'])
        io.move_line_safe(p['tool_pickup_1'])
        io.move_line_safe(p['tool_pickup_2'])
        io.gripper_close('20mm')
        from DSR_ROBOT2 import wait
        wait(0.5)
        io.move_line_safe(p['tool_pickup_1'])

    def _return_presser(self):
        io = self._io
        p = self._get_poses()['press']
        io.move_line_safe(p['tool_backup_1'])
        io.move_line_safe(p['tool_backup_2'])
        io.gripper_open('65mm')
        io.move_line_safe(p['tool_backup_1'])

    # ===== 뒤집개 (Spatula) =====

    def _pick_spatula(self):
        io = self._io
        p = self._get_poses()['spatula']
        io.gripper_open('65mm')
        io.move_joint_safe(JOINT_POSES['ready'], vel=150, acc=100)
        io.move_line_safe(p['anchor_pos_0'], vel=150, acc=100)
        io.move_line_safe(p['anchor_pos_1'], vel=150, acc=100)
        io.gripper_close('12mm')
        from DSR_ROBOT2 import wait
        wait(0.5)
        io.move_line_safe(p['anchor_pos_0'], vel=150, acc=100)

    def _return_spatula(self):
        """뒤집개 반납 — 힘 감지 반복 알고리즘으로 안전하게 놓기"""
        from DSR_ROBOT2 import posx, amovel, wait, check_motion, get_tool_force, DR_BASE
        io = self._io
        FORCE_THRESHOLD = 7
        x_adjust = 0
        max_attempts = 10

        for attempt in range(max_attempts):
            cur_back_0 = posx([371 + x_adjust, -340, 311, 61, -173, -176])
            cur_back_1 = posx([371 + x_adjust, -340, 136, 61, -173, -176])

            io.move_line_safe(cur_back_0, vel=60, acc=60)

            base_force = get_tool_force(DR_BASE)
            base_fz = abs(base_force[2])
            self._logger.info(
                f'[Spatula return] Attempt {attempt+1}, base Fz={base_fz:.1f}N, x_adj={x_adjust}')

            amovel(cur_back_1, vel=60, acc=60)

            force_detected = False
            while check_motion() != 0:
                cur_force = get_tool_force(DR_BASE)
                cur_fz = abs(cur_force[2])
                if cur_fz - base_fz > FORCE_THRESHOLD:
                    force_detected = True
                    break
                wait(0.1)

            if not force_detected:
                io.gripper_open('65mm')
                io.move_line_safe(cur_back_0, vel=60, acc=60)
                self._logger.info(f'Spatula placed at x_adjust={x_adjust}')
                return

            io.move_line_safe(cur_back_0, vel=60, acc=60)
            x_adjust -= 4

    # ===== 소스병 (Sauce Bottle) =====

    def _pick_sauce_bottle(self):
        io = self._io
        p = self._get_poses()['sauce']
        io.gripper_open('90mm')
        io.move_joint_safe(JOINT_POSES['ready'])
        io.move_joint_safe(JOINT_POSES['sauce_bottle1'])
        io.move_line_safe(p['bottle2'])
        io.gripper_close('40mm')
        io.move_line_safe(p['bottle1'])

    def _return_sauce_bottle(self):
        io = self._io
        p = self._get_poses()['sauce']
        io.move_line_safe(p['plate1'])
        io.move_line_safe(p['bottle1'])
        io.move_line_safe(p['bottle2'])
        io.gripper_open('90mm')
        io.move_line_safe(p['bottle1'])
        io.move_joint_safe(JOINT_POSES['ready'])

    # ===== 가루통 (Powder Bottle) =====

    def _pick_powder_bottle(self):
        from DSR_ROBOT2 import wait
        io = self._io
        p = self._get_poses()['powder']
        io.gripper_open('90mm')
        wait(0.5)
        io.move_joint_safe(JOINT_POSES['ready'], vel=100, acc=100)
        io.move_joint_safe(JOINT_POSES['powder_bottle_lift'], vel=100, acc=100)
        io.move_line_safe(p['bottle_pick'], vel=80, acc=100)
        wait(0.5)
        io.gripper_open('65mm')
        wait(0.5)
        io.move_line_safe(p['bottle_lift'], vel=80, acc=100)

    def _return_powder_bottle(self):
        io = self._io
        p = self._get_poses()['powder']
        io.move_line_safe(p['bottle_lift'], vel=100, acc=100)
        io.move_line_safe(p['bottle_pick'], vel=80, acc=100)
        io.gripper_open('90mm')
        io.move_line_safe(p['bottle_lift'], vel=100, acc=100)
        io.move_joint_safe(JOINT_POSES['ready'], vel=100, acc=100)
