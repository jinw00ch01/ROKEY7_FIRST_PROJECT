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


def perform_task_press():
    """누르기 작업 수행"""
    print("Performing press task...")
    from DSR_ROBOT2 import (
        posx, movej, movel, amovel, wait,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force,
        check_position_condition,
        get_current_posx,amove_periodic,
        DR_AXIS_Z, DR_BASE, DR_FC_MOD_ABS, DR_TOOL,DR_FC_MOD_REL
    )

    # 디지털 입력 신호 대기 함수
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)

    # Release 동작
    def release_65mm():
        print("65mm_Releasing...")
        set_digital_output(3, OFF)
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        # wait_digital_input(2)

    def release_90mm():
        print("90mm_Releasing...")
        set_digital_output(3, ON)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)

    # Grip 동작
    def grip_20mm():
        print("Gripping...")
        # release()
        set_digital_output(3, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        # wait_digital_input(1)

    def grip_12mm():
        print("Half Gripping...")
        set_digital_output(3, OFF)
        set_digital_output(2, ON)
        set_digital_output(1, ON)
        

    # ===== 위치 정의 (실제 환경에 맞게 수정 필요) =====
    JReady = [0, 0, 90, 0, 90, 0]                          # 초기 자세
    pos_tool_pickup_1 = posx([563, -5, 153, 7, -179, 8])  # 누르기 도구 위치
    pos_tool_pickup_2 = posx([563, -5, 64, 7, -179, 8])

    pos_above_dough = posx([316, -85, 153, 166, 179, 167])     # 반죽 위 위치    
    pos_press_down = posx([316, -85, 70, 166, 179, 167])       # Z축 하강 목표 (충분히 낮게 설정)
    pos_lift_up = posx([316, -85, 120, 166, 179, 167])         # 들어올리기 위치




    pos_shake_up = posx([316, -85, 140, 166, 179, 167])           # 도구 털기 위로 위치
    pos_shake_down = posx([316, -85, 160, 166, 179, 167])           # 도구 털기 위로 위치

    CONTACT_FORCE = 10.0    # 반죽 접촉 감지 힘 임계값 (N)
    PRESS_FORCE = 150      # 반죽 누르기 힘 (N)
    TARGET_HEIGHT = 10   # 누르기 목표 높이 (mm, Z축 절대 위치)

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

    # ===== 5단계: 외력으로 반죽 누르기 =====
    print("[Step 5] 컴플라이언스 모드 - 반죽 누르기")
    task_compliance_ctrl(stx=[3000, 3000, 3000, 200, 200, 200])
    set_desired_force(fd=[0, 0, -PRESS_FORCE, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    # ===== 3단계: Z축 하강 (비동기 이동) =====
    print("[Step 3] Z축 하강 시작")
    movel(pos_press_down, vel=80, acc=60)

    release_force()
    release_compliance_ctrl()

    

    # ===== 7단계: 도구 들어올리기 =====
    print("[Step 7] 도구 들어올리기")
    movel(pos_lift_up, vel=VELOCITY, acc=ACC)

    # ===== 8단계: 도구 털기 (move_periodic으로 Z축 상하 + X축/Rx축 흔들기 동시 수행) =====
    print("[Step 8] 도구 털기 시작")
    movel(pos_shake_up, vel=200, acc=ACC)
    # amp: [X, Y, Z, Rx, Ry, Rz] - Z축 상하 20mm + X축 10mm + Rx 0.5도 동시 진동
    # period: 각 축별 주기(초) - Z축은 느리게, X/Rx는 빠르게
    from DSR_ROBOT2 import move_periodic
    move_periodic(amp=[0, 0, 30, 0, 0, 30], period=[0, 0, 1, 0, 0, 1], atime=0.5, repeat=5, ref=DR_TOOL)
    movel(pos_shake_down, vel=200, acc=ACC)
    print("  -> 털기 완료!")

    movel(pos_tool_pickup_1, vel=VELOCITY, acc=ACC)

    # == 프레스기 원위치 == 
    movel(pos_tool_pickup_2, vel=VELOCITY, acc=ACC)
    release_65mm()
    # print("요구사항 3번까지 완료! (릴리스 → 도구 위치 이동 → 그립)")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("press_test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        setup_io_clients(node)
        perform_task_press()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
