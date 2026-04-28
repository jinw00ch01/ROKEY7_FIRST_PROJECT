"""
object_manager.py
물체(접시 등)의 pick & place를 io_manager + pose_manager 기반으로 수행.
"""
from rclpy.node import Node
from cobot1.helpers.io_manager import IOManager
from cobot1.helpers.pose_manager import get_poses, JOINT_POSES


class ObjectManager:
    """물체 pick & place를 관리하는 헬퍼 클래스"""

    def __init__(self, node: Node, io: IOManager = None):
        self._node = node
        self._io = io
        self._logger = node.get_logger()
        self._plate_placed = False

    def _get_io(self):
        if self._io is None:
            self._io = IOManager(self._node)
        return self._io

    def pick_and_place_plate(self) -> bool:
        """접시를 스택에서 집어 서빙 위치에 배치"""
        from DSR_ROBOT2 import movec, wait
        self._logger.info('[ObjectManager] pick_and_place_plate()')

        if self._plate_placed:
            self._logger.warn('Plate already placed. Skipping.')
            return True

        io = self._get_io()
        p = get_poses()['plate']

        # 그리퍼 열기 + 홈 이동
        io.gripper_open('65mm')
        io.move_joint_safe(JOINT_POSES['ready'], vel=150, acc=150)

        # 접시 위치로 원호 이동
        movec(p['start0'], p['start1'], vel=200, acc=150)

        # 접시 그립 (약하게)
        io.gripper_close('12mm')
        wait(1.0)

        # 중간 경유
        io.move_line_safe(p['start2'], vel=150, acc=150)

        # 서빙 위치로 이동 + 내려놓기
        io.move_line_safe(p['end1'], vel=200, acc=150)
        io.move_line_safe(p['end2'], vel=150, acc=150)

        # 릴리스
        io.gripper_open('65mm')

        io.move_line_safe(p['end3'], vel=150, acc=150)

        self._plate_placed = True
        self._logger.info('[ObjectManager] Plate placed.')
        return True

    def reset(self):
        """상태 초기화 (다음 워크플로우용)"""
        self._plate_placed = False
