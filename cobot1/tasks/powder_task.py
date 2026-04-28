"""
powder_task.py
가루를 뿌리는 코어 로직.
도구 pick/return은 ToolManager가 처리.
"""
from rclpy.node import Node
from cobot1.helpers.io_manager import IOManager
from cobot1.helpers.pose_manager import get_poses


class PowderTask:
    def __init__(self, node: Node, io: IOManager = None):
        self._node = node
        self._io = io
        self._logger = node.get_logger()

    def _get_io(self):
        if self._io is None:
            self._io = IOManager(self._node)
        return self._io

    def sprinkle_powder(self) -> bool:
        """가루통으로 스냅 동작 가루 뿌리기 (도구는 이미 들고 있는 상태)"""
        from DSR_ROBOT2 import movel
        self._logger.info('[PowderTask] sprinkle_powder()')
        io = self._get_io()
        p = get_poses()['powder']

        # 접시 위로 이동
        io.move_line_safe(p['plate_above'], vel=100, acc=100)

        # 가루통 뒤집기 (B축 회전)
        io.move_line_safe(p['plate_flipped'], vel=100, acc=100)
        self._logger.info('[PowderTask] Bottle flipped.')

        # 스냅 동작으로 가루 뿌리기 (move_periodic 방식) - 축 하나 더 amp 가속
        from DSR_ROBOT2 import move_periodic, DR_BASE
        move_periodic(
            amp=[0, -15, 30, 30, 0, 0],
            period=[0, 1, 1, 1, 0, 0],
            atime=0.5, repeat=5, ref=DR_BASE)

        # 스냅 동작으로 가루 뿌리기 (movel 급가속 방식)
        # SNAP_REPEAT = 5
        # for i in range(SNAP_REPEAT):
        #     movel(p['plate_above'], time=0.3)
        #     movel(p['plate_flipped'], time=0.3)
        #     self._logger.info(f'[PowderTask] Snap {i+1}/{SNAP_REPEAT}')
        # self._logger.info('[PowderTask] Powder sprinkled.')

        # plate_flipped와 plate_above 변수대신 다른 새로운 변수 생성시킬것, 

        # 가루통 원위치 (출구가 위로)
        io.move_line_safe(p['plate_above'], vel=100, acc=100)

        return True
