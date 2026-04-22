"""
dough_task.py
집게로 반죽을 잡아 그릴에 배치하는 코어 로직.
도구 pick/return은 ToolManager가 처리.
"""
from rclpy.node import Node
from cobot1.helpers.io_manager import IOManager
from cobot1.helpers.pose_manager import get_poses, JOINT_POSES


class DoughTask:
    def __init__(self, node: Node, io: IOManager = None):
        self._node = node
        self._io = io
        self._logger = node.get_logger()

    def _get_io(self):
        if self._io is None:
            self._io = IOManager(self._node)
        return self._io

    def place_dough_with_tongs(self) -> bool:
        """집게로 반죽을 잡아 그릴 위에 배치 (도구는 이미 들고 있는 상태)"""
        from DSR_ROBOT2 import wait
        self._logger.info('[DoughTask] place_dough_with_tongs()')
        io = self._get_io()
        p = get_poses()['dough']

        # 반죽 위치로 이동
        io.move_line_safe(p['dough_start_1'], vel=200, acc=150)
        io.move_line_safe(p['dough_start_2'], vel=200, acc=150)

        # 반죽 그립
        io.gripper_close('50mm')
        wait(0.5)

        # 반죽 들어올리기
        io.move_line_safe(p['dough_start_1'], vel=150, acc=100)

        # 그릴 위에 배치
        io.move_line_safe(p['dough_end_1'], vel=150, acc=100)

        # 릴리스
        io.gripper_open('65mm')
        wait(0.5)

        io.move_line_safe(p['dough_end_2'], vel=150, acc=100)

        self._logger.info('[DoughTask] Dough placed on grill.')
        return True
