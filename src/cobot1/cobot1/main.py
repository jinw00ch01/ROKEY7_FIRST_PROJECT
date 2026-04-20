import rclpy
import DR_init
import time
from dsr_msgs2.srv import (
    SetCtrlBoxDigitalOutput, GetCtrlBoxDigitalInput,
    CheckForceCondition
)

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 150
ACC = 100

# 디지털 출력 상태
ON, OFF = 1, 0

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 글로벌 노드 및 서비스 클라이언트
g_node = None
cli_set_digital_output = None
cli_get_digital_input = None
cli_check_force_condition = None


def setup_io_clients(node):
    """DSR_ROBOT2의 IO/Force 서비스 버그를 우회하여 직접 서비스 클라이언트 생성"""
    global g_node, cli_set_digital_output, cli_get_digital_input, cli_check_force_condition
    g_node = node
    cli_set_digital_output = node.create_client(
        SetCtrlBoxDigitalOutput, "/" + ROBOT_ID + "/io/set_ctrl_box_digital_output")
    cli_get_digital_input = node.create_client(
        GetCtrlBoxDigitalInput, "/" + ROBOT_ID + "/io/get_ctrl_box_digital_input")
    cli_check_force_condition = node.create_client(
        CheckForceCondition, "/" + ROBOT_ID + "/force/check_force_condition")
    cli_set_digital_output.wait_for_service(timeout_sec=5.0)
    cli_get_digital_input.wait_for_service(timeout_sec=5.0)
    cli_check_force_condition.wait_for_service(timeout_sec=5.0)
    print("IO/Force service clients ready.")


def set_digital_output(index, value):
    req = SetCtrlBoxDigitalOutput.Request()
    req.index = index
    req.value = value
    future = cli_set_digital_output.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    result = future.result()
    if result is None or not result.success:
        g_node.get_logger().warn(f"set_digital_output({index}, {value}) failed")


def get_digital_input(index):
    req = GetCtrlBoxDigitalInput.Request()
    req.index = index
    future = cli_get_digital_input.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    result = future.result()
    if result is None:
        return 0
    return result.value


def check_force_condition(axis, min=0, max=0, ref=0):
    """조건 충족 시 True, 미충족 시 False 반환"""
    req = CheckForceCondition.Request()
    req.axis = axis
    req.min = float(min)
    req.max = float(max)
    req.ref = ref
    future = cli_check_force_condition.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    result = future.result()
    if result is None:
        return False
    return result.success


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}")
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


# ===== 그리퍼 공통 함수 =====

def wait_digital_input(sig_num):
    from DSR_ROBOT2 import wait
    while not get_digital_input(sig_num):
        wait(0.5)

def release_65mm():
    print("65mm_Releasing...")
    set_digital_output(3, OFF)
    set_digital_output(2, ON)
    set_digital_output(1, OFF)

def release_90mm():
    print("90mm_Releasing...")
    set_digital_output(3, ON)
    set_digital_output(2, OFF)
    set_digital_output(1, OFF)

def grip_20mm():
    print("Gripping...")
    set_digital_output(3, OFF)
    set_digital_output(2, OFF)
    set_digital_output(1, ON)

def grip_12mm():
    print("Half Gripping...")
    set_digital_output(3, OFF)
    set_digital_output(2, ON)
    set_digital_output(1, ON)


# ===== 작업 1: 반죽 집기 =====

def perform_task_dough_grip():
    """반죽 집기 작업 수행"""
    print("Performing dough grip task...")
    from DSR_ROBOT2 import (
        posx, movej, movel, movec, wait, DR_MV_MOD_REL,
    )

    # 위치 정의
    JReady = [0, 0, 90, 0, 90, 0]
    pick_start_1 = posx([230, 212, 12, 36, -179, 36])
    pick_start_2 = posx([230, 212, 150, 36, -179, 36])
    dough_start_1 = posx([461, 211, 150, 36, -178, 36])
    dough_start_2 = posx([461, 211, 17, 36, -178, 36])
    dough_end_1 = posx([303, -39, 45, 17, -177, 17])
    dough_end_2 = posx([303, -39, 150, 17, -177, 17])

    # 1. 그리퍼 릴리스 초기화
    print("Step 1: 그리퍼 release_90mm 초기화")
    release_90mm()

    # 초기 위치로 이동
    print("Moving to ready position...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 2. 집게 위치로 이동
    print("Step 2: 집게 위치로 이동")
    movel(pick_start_1, vel=200, acc=ACC)

    # 3. 집게 그립
    print("Step 3: 집게 그립")
    release_65mm()
    wait(0.5)

    print("Step 3: pick_start_2로 이동")
    movel(pick_start_2, vel=VELOCITY, acc=ACC)

    print("Step 5: dough_start_2로 이동")
    movel(dough_start_1, vel=150, acc=ACC)

    # 4. 도우 목적지가 있는 위치로 이동
    print("Step 4: dough_start_1로 이동")
    movel(dough_start_2, vel=150, acc=ACC)

    # 5. 반죽 그립
    print("Step 5: 반죽 그립")
    grip_20mm()
    wait(0.5)

    movel(dough_start_1, vel=150, acc=ACC)

    # 6. 반죽 놓기 위치로 이동
    print("Step 6: dough_end로 이동")
    movel(dough_end_1, vel=VELOCITY, acc=ACC)

    # 7. 그리퍼 릴리스
    print("Step 6: 그리퍼 릴리스 65mm")
    release_65mm()
    wait(0.5)

    movel(dough_end_2, vel=VELOCITY, acc=ACC)
    movel(pick_start_2, vel=VELOCITY, acc=ACC)

    print("Step 2: 집게 위치로 이동")
    movel(pick_start_1, vel=200, acc=ACC)

    release_90mm()


# ===== 작업 2: 프레스 누르기 =====

def perform_task_press():
    """누르기 작업 수행"""
    print("Performing press task...")
    from DSR_ROBOT2 import (
        posx, movej, movel, amovel, wait,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force,
        check_position_condition,
        get_current_posx, amove_periodic,
        DR_AXIS_Z, DR_BASE, DR_FC_MOD_ABS, DR_TOOL, DR_FC_MOD_REL,
        move_periodic,
    )

    # 위치 정의
    JReady = [0, 0, 90, 0, 90, 0]
    pos_tool_pickup_1 = posx([563, -5, 153, 7, -179, 8])
    pos_tool_pickup_2 = posx([563, -5, 64, 7, -179, 8])

    pos_above_dough = posx([316, -85, 153, 166, 179, 167])
    pos_press_down = posx([316, -85, 60, 166, 179, 167])
    pos_lift_up = posx([316, -85, 120, 166, 179, 167])

    pos_shake_up = posx([316, -85, 140, 166, 179, 167])
    pos_shake_down = posx([316, -85, 160, 166, 179, 167])

    PRESS_FORCE = 200

    # ===== 1단계: 누르기 도구 위치로 이동 후 그리핑 =====
    release_65mm()

    movej(JReady, vel=VELOCITY, acc=ACC)

    movel(pos_tool_pickup_1, vel=VELOCITY, acc=ACC)
    movel(pos_tool_pickup_2, vel=VELOCITY, acc=ACC)
    grip_20mm()
    wait(0.5)
    print("[Step 1] 누르기 도구 위치로 이동 후 그리핑 성공")

    movel(pos_tool_pickup_1, vel=VELOCITY, acc=ACC)

    # ===== 2단계: 반죽 위로 이동 =====
    movel(pos_above_dough, vel=VELOCITY, acc=ACC)
    print("[Step 2] 누르기 도구를 반죽 위로 이동")

    # ===== 3단계: 컴플라이언스 모드 - 반죽 누르기 =====
    print("[Step 3] 컴플라이언스 모드 - 반죽 누르기")
    task_compliance_ctrl(stx=[3000, 3000, 3000, 200, 200, 200])
    set_desired_force(fd=[0, 0, -PRESS_FORCE, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    # Z축 하강
    print("[Step 4] Z축 하강 시작")
    movel(pos_press_down, vel=80, acc=60)

    # 누르기 완료 → 힘 제어 해제
    release_force()
    release_compliance_ctrl()

    # ===== 5단계: 도구 들어올리기 =====
    print("[Step 5] 도구 들어올리기")
    movel(pos_lift_up, vel=VELOCITY, acc=ACC)

    # ===== 6단계: 도구 털기 =====
    print("[Step 6] 도구 털기 시작")
    movel(pos_shake_up, vel=200, acc=ACC)
    move_periodic(amp=[0, 0, 30, 0, 0, 30], period=[0, 0, 1, 0, 0, 1], atime=0.5, repeat=5, ref=DR_TOOL)
    movel(pos_shake_down, vel=200, acc=ACC)
    print("  -> 털기 완료!")

    movel(pos_tool_pickup_1, vel=VELOCITY, acc=ACC)

    # 프레스기 원위치
    movel(pos_tool_pickup_2, vel=VELOCITY, acc=ACC)
    release_65mm()


# ===== 작업 3: 접시 세팅 =====

def perform_task_plate_setting():
    """접시 세팅 작업 수행"""
    print("Performing plate setting task...")
    from DSR_ROBOT2 import (
        posx, movej, movel, movec, wait, DR_MV_MOD_REL,
    )

    # 위치 정의
    JReady = [0, 0, 90, 0, 90, 0]
    plate_start0 = posx([622, 219, 244, 5, 173, -171])
    plate_start1 = posx([623, 220, 210, 5, 173, -171])
    plate_start2 = posx([623, 50, 275, 5, 173, -171])
    plate_end1 = [-31, 25, 111, -235, 37, 36]
    plate_end2 = posx([639, -215, 90, 0, 110, 179])

    # 1. 그리퍼 릴리스 초기화
    print("Step 1: 그리퍼 릴리스 초기화")
    release_65mm()

    # 초기 위치로 이동
    print("Moving to ready position...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 2. 접시 위치로 이동
    print("Step 2: 접시 위치로 이동")
    movec(plate_start0, plate_start1, vel=200, acc=ACC)

    # 3. 접시 그립
    print("Step 3: 접시 그립")
    grip_12mm()
    wait(1.0)

    print("Step 3: plate_start2로 이동")
    movel(plate_start2, vel=VELOCITY, acc=ACC)

    # 4. 세팅 목적지로 이동
    print("Step 4: plate_end1로 이동")
    movej(plate_end1, vel=200, acc=ACC)

    # 5. 그리퍼 릴리스
    print("Step 5: 그리퍼 릴리스")
    release_65mm()
    wait(0.5)

    # 6. 접시 내려놓기 위치로 이동
    print("Step 6: plate_end2로 이동")
    movel(plate_end2, vel=VELOCITY, acc=ACC)

    # 초기 위치로 이동
    print("Moving to ready position...")
    movej(JReady, vel=VELOCITY, acc=ACC)
    print("Plate setting task completed!")


# ===== 작업 4: 뒤집개 =====

def perform_task_spatula():
    """뒤집개 작업 수행"""
    print("Performing spatula task...")
    from DSR_ROBOT2 import (
        posx, movej, movel, movec, wait, amovel,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force,
        DR_AXIS_Z, DR_BASE, DR_FC_MOD_ABS, DR_TOOL,
    )

    SPATULA_VEL = 60
    SPATULA_ACC = 60

    # 위치 정의
    JReady = [0, 0, 90, 0, 90, 0]

    anchor_pos_0 = posx([316, -110, 350, 94, -162, -177])
    anchor_pos_1 = posx([316, -145, 255, 94, -162, -177])
    anchor_pos_2 = posx([312, -114, 317, 94, -163, -179])

    pos1 = posx([321, 107, 271, 94, -163, -179])
    pos1_2 = posx([316, 163, 188, 87, -134, 177])
    pos2 = posx([321, 169, 119, 91, -132, 177])
    pos3 = posx([321, 64, 119, 91, -132, 177])
    pos4 = posx([321, 64, 209, 91, -132, 177])
    pos5 = posx([317, 131, 135, 90, -108, 175])
    pos6 = posx([286, 111, 117, 84, -101, 124])
    pos7 = posx([330, 113, 158, 92, -95, 131])

    release_65mm()

    print("Moving to ready position...")
    movej(JReady, vel=SPATULA_VEL, acc=SPATULA_ACC)

    movel(anchor_pos_0, vel=100, acc=100)
    movel(anchor_pos_1, vel=100, acc=100)

    grip_12mm()
    wait(0.5)

    movel(anchor_pos_2, vel=100, acc=100)

    task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
    fd = [0, 0, 20, 0, 0, 0]
    fctrl_dir = [0, 0, 1, 0, 0, 0]
    set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_ABS)

    movel(pos1, vel=SPATULA_VEL, acc=SPATULA_ACC)
    movel(pos1_2, vel=SPATULA_VEL, acc=SPATULA_ACC)
    movel(pos2, vel=SPATULA_VEL, acc=SPATULA_ACC)
    movel(pos3, vel=100, acc=SPATULA_ACC)
    movel(pos4, vel=100, acc=SPATULA_ACC)
    movel(pos5, vel=100, acc=SPATULA_ACC)
    movel(pos6, vel=100, acc=SPATULA_ACC)
    movel(pos7, vel=100, acc=SPATULA_ACC)

    release_force()
    release_compliance_ctrl()


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 전체 작업 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("main_task", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        setup_io_clients(node)

        # 전체 작업 순서대로 수행
        perform_task_dough_grip()
        perform_task_press()
        perform_task_plate_setting()
        perform_task_spatula()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
