"""
flip_task.py
뒤집개로 음식을 뒤집는 코어 로직.
도구 pick/return은 ToolManager가 처리.
"""
from rclpy.node import Node
from cobot1.helpers.io_manager import IOManager
from cobot1.helpers.pose_manager import get_poses


class FlipTask:
    def __init__(self, node: Node, io: IOManager = None):
        self._node = node
        self._io = io
        self._logger = node.get_logger()

    def _get_io(self):
        if self._io is None:
            self._io = IOManager(self._node)
        return self._io

    def flip_item_with_spatula(self) -> bool:
        """뒤집개로 팬케이크 뒤집기 (도구는 이미 들고 있는 상태)"""
        from DSR_ROBOT2 import (
            task_compliance_ctrl, release_compliance_ctrl,
            set_desired_force, release_force,
            DR_FC_MOD_ABS
        )
        self._logger.info('[FlipTask] flip_item_with_spatula()')
        io = self._get_io()
        p = get_poses()['spatula']

        # 컴플라이언스 모드
        task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
        set_desired_force(
            fd=[0, 0, 20, 0, 0, 0],
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_ABS)

        # 하강 시퀀스
        io.move_line_safe(p['down_1'], vel=150, acc=100)
        io.move_line_safe(p['down_2'], vel=150, acc=100)
        io.move_line_safe(p['down_3'], vel=150, acc=100)

        # 그릴 위 이동
        io.move_line_safe(p['grill_1'], vel=100, acc=100)

        # 스윙 동작 (뒤집기)
        io.move_line_safe(p['swing_1'], vel=100, acc=150)
        io.move_line_safe(p['swing_2'], vel=100, acc=150)
        from DSR_ROBOT2 import movel
        movel(p['swing_3'], time=0.6)

        # 스윕 (쓸어내기) - sweep_1은 movel, sweep_2~4는 movesx 스플라인
        io.move_line_safe(p['sweep_1'], vel=100, acc=60)
        sweep_list = [p['sweep_2'], p['sweep_3'], p['sweep_4']]
        io.move_spline_safe(sweep_list, vel=100, acc=100)

        release_force()
        release_compliance_ctrl()

        # 들어올리기 → 접시로 이동
        io.move_line_safe(p['lift_up'], vel=100, acc=100)
        io.move_line_safe(p['plate_up'], vel=100, acc=100)
        movel(p['plate_down'], time=0.5)

        # 복귀
        io.move_line_safe(p['back_home'], vel=150, acc=150)

        self._logger.info('[FlipTask] Flip complete.')
        return True
