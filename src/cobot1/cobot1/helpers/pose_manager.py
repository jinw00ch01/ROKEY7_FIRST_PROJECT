"""
pose_manager.py
모든 좌표/위치 상수를 도구별·작업별로 중앙 관리하는 모듈.

- 각 태스크 파일에 흩어진 posx() 좌표를 한 곳에 통합
- 로봇 설정 상수도 여기서 관리
- posx()는 DSR_ROBOT2에서 import해야 하므로, 함수로 감싸서 lazy 로딩

사용법:
    from cobot1.helpers.pose_manager import get_poses, ROBOT_CONFIG, JOINT_POSES
    poses = get_poses()
    dough_start = poses['dough']['dough_start_2']
"""


# ===== 로봇 설정 상수 =====
ROBOT_CONFIG = {
    'robot_id': 'dsr01',
    'robot_model': 'm0609',
    'robot_tool': 'Tool Weight',
    'robot_tcp': 'GripperDA_v1',
}

# ===== 관절 좌표 (Joint Positions) =====
JOINT_POSES = {
    'ready': [0, 0, 90, 0, 90, 0],

    # source_test 전용
    'sauce_bottle1': [-15, 41, 59, 86, -70, 9],
    'sauce_bottle2': [-15, 46, 68, 81, -71, 24],

    # powder_test 전용
    'powder_bottle_lift': [-17, 25, 79, -95, 66, 12],
    'powder_bottle_pick': [-17, 33, 90, -103, 69, 33],
}


def get_poses() -> dict:
    """모든 카테시안 좌표를 도구/작업별 딕셔너리로 반환한다.

    posx()는 DSR_ROBOT2에서 import해야 하므로 함수 호출 시점에 로딩.

    Returns:
        도구/작업별 좌표 딕셔너리.
    """
    from DSR_ROBOT2 import posx

    return {
        # ===== 반죽 집기 (dough_grip_test) =====
        'dough': {
            'pick_start_1':  posx([230, 202, -3, 36, -179, 36]),
            'pick_start_2':  posx([230, 202, 150, 36, -179, 36]),
            'dough_start_1': posx([461, 241, 150, 36, -178, 36]),
            'dough_start_2': posx([461, 241, 9, 36, -178, 36]),
            'dough_end_1':   posx([323, -10, 38, 17, -177, 17]),
            'dough_end_2':   posx([323, -10, 150, 17, -177, 17]),
        },

        # ===== 프레스 (press_test) =====
        'press': {
            'tool_pickup_1':  posx([610, 8, 150, 32, -179, 33]),
            'tool_pickup_2':  posx([610, 8, 100, 32, -179, 33]),
            'above_dough':    posx([328, -107, 213, 174, 179, 175]),
            'press_down':     posx([328, -107, 120, 174, 179, 175]),
            'lift_up':        posx([328, -107, 195, 174, 179, 175]),
            'tool_backup_1':  posx([610, 8, 150, 32, -179, 33]),
            'tool_backup_2':  posx([610, 8, 100, 32, -179, 33]),
        },

        # ===== 접시 세팅 (plate_setting_test) =====
        'plate': {
            'start0': posx([622, 219, 244, 5, 173, -171]),
            'start1': posx([623, 220, 210, 5, 173, -171]),
            'start2': posx([623, 50, 275, 5, 173, -171]),
            'end1':   posx([710, -183, 254, 170, -106, -1]),
            'end2':   posx([710, -183, 95, 170, -106, -1]),
            'end3':   posx([606, -183, 95, 170, -106, -1]),
        },

        # ===== 뒤집개 (spatula_test) =====
        'spatula': {
            'anchor_pos_0':  posx([372, -356, 210, 32, -172, 154]),
            'anchor_pos_1':  posx([372, -356, 99, 32, -172, 154]),
            'down_1':        posx([315, 140, 223, 89, -143, -178]),
            'down_2':        posx([335, 144, 159, 86, -129, -175]),
            'down_3':        posx([339, 129, 110, 87, -127, -178]),
            'grill_1':       posx([310, 12, 50, 87, -107, 179]),
            'swing_1':       posx([326, 25, 50, 90, -92, 179]),
            'swing_2':       posx([326, 25, 40, 90, -92, 140]),
            'swing_3':       posx([463, 25, 65, 90, -93, 32]),
            'sweep_1':       posx([221, 20, 30, 85, -108, 178]),
            'sweep_2':       posx([313, 47, 30, 101, -110, -179]),
            'sweep_3':       posx([457, 105, 30, 88, -113, -177]),
            'sweep_4':       posx([451, 10, 30, 87, -111, 179]),
            'lift_up':       posx([451, 10, 297, 87, -111, 179]),
            'plate_up':      posx([690, -98, 252, 119, -109, -175]),
            'plate_down':    posx([690, -98, 252, 119, -109, 111]),
            'back_home':     posx([342, -141, 355, 94, -122, 85]),
            'anchor_back_0': posx([374, -348, 311, 61, -173, -176]),
            'anchor_back_1': posx([374, -348, 136, 61, -173, -176]),
        },

        # ===== 소스 (source_test) =====
        'sauce': {
            'bottle1': posx([635, -513, 374, 94, -90, -91]),
            'bottle2': posx([635, -513, 258, 94, -90, -91]),
            'plate1':  posx([783, -151, 280, 139, -93, -89]),
            'plate2':  posx([783, -151, 280, 139, -93, 90]),
        },

        # ===== 가루 (powder_test) =====
        'powder': {
            'bottle_lift':  posx([543, -506, 408, 96, -90, 86]),
            'bottle_pick':  posx([543, -506, 264, 96, -90, 86]),
            'plate_above':  posx([826, -169, 219, 4, 99, -140]),
            'plate_flipped': posx([826, -217, 219, 4, 99, 141]),
        },
    }
