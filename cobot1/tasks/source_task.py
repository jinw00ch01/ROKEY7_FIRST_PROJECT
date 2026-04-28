"""
source_task.py
소스를 뿌리는 코어 로직.
src/cobot1/cobot1/source_test.py의 perform_task_source()와 동일한 동작.
도구 pick/return은 ToolManager가 처리.
"""
from rclpy.node import Node
from cobot1.helpers.io_manager import IOManager
from cobot1.helpers.pose_manager import get_poses


class SourceTask:
    def __init__(self, node: Node, io: IOManager = None):
        self._node = node
        self._io = io
        self._logger = node.get_logger()

    def _get_io(self):
        if self._io is None:
            self._io = IOManager(self._node)
        return self._io

    def dispense_source(self) -> bool:
        """소스병으로 나선 패턴 소스 뿌리기 (도구는 이미 들고 있는 상태)"""
        from DSR_ROBOT2 import move_spiral, mwait, DR_AXIS_Z, DR_BASE
        self._logger.info('[SourceTask] dispense_source()')
        io = self._get_io()
        p = get_poses()['source']

        # 접시 위로 이동 + 뿌리기 자세
        io.move_line_safe(p['plate1'])
        io.move_line_safe(p['plate2'])

        # 그리퍼 조임 (소스 분출) - grip_20mm
        io.gripper_close('20mm')

        # 나선 패턴 뿌리기
        move_spiral(rev=5, rmax=30.0, lmax=-20, v=80, a=80,
                    axis=DR_AXIS_Z, ref=DR_BASE)
        mwait(0.5)

        self._logger.info('[SourceTask] Source dispensed.')
        return True
