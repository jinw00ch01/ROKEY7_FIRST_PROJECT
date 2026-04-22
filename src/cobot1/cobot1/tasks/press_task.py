"""
press_task.py
프레스 도구로 반죽을 누르는 코어 로직.
도구 pick/return은 ToolManager가 처리.
"""
from rclpy.node import Node
from cobot1.helpers.io_manager import IOManager
from cobot1.helpers.pose_manager import get_poses

PRESS_FORCE = 150  # 반죽 누르기 힘 (N)


class PressTask:
    def __init__(self, node: Node, io: IOManager = None):
        self._node = node
        self._io = io
        self._logger = node.get_logger()

    def _get_io(self):
        if self._io is None:
            self._io = IOManager(self._node)
        return self._io

    def press_dough(self) -> bool:
        """프레스로 반죽을 눌러 납작하게 (도구는 이미 들고 있는 상태)"""
        from DSR_ROBOT2 import (
            task_compliance_ctrl, release_compliance_ctrl,
            set_desired_force, release_force,
            move_periodic, DR_FC_MOD_REL, DR_TOOL
        )
        self._logger.info('[PressTask] press_dough()')
        io = self._get_io()
        p = get_poses()['press']

        # 반죽 위로 이동
        io.move_line_safe(p['above_dough'])
        self._logger.info('[PressTask] Above dough position.')

        # 컴플라이언스 모드 + 힘 제어로 누르기
        task_compliance_ctrl(stx=[3000, 3000, 3000, 200, 200, 200])
        set_desired_force(
            fd=[0, 0, -PRESS_FORCE, 0, 0, 0],
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_REL)

        io.move_line_safe(p['press_down'], vel=80, acc=60)

        release_force()
        release_compliance_ctrl()

        # 들어올리기
        io.move_line_safe(p['lift_up'])
        self._logger.info('[PressTask] Press done, lifting up.')

        # 도구 털기
        move_periodic(
            amp=[0, 0, 30, 0, 0, 30],
            period=[0, 0, 1, 0, 0, 1],
            atime=0.5, repeat=5, ref=DR_TOOL)
        self._logger.info('[PressTask] Tool shaking done.')

        return True
