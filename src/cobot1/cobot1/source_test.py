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


def perform_task_source():
    """소스 뿌리기 작업 수행"""
    print("Performing powder/sauce task...")
    from DSR_ROBOT2 import (
        posx, movej, movel, move_spiral, amove_spiral, wait, mwait,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force,
        DR_AXIS_Z, DR_BASE, DR_FC_MOD_ABS, DR_TOOL
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

    def release_90mm():
        print("90mm_Releasing...")
        set_digital_output(3, ON)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)

    # Grip 동작
    def grip_20mm():
        print("grip_20mm Gripping...")
        set_digital_output(3, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)

    def grip_40mm():
        print("grip_40mm Gripping...")
        set_digital_output(3, ON)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)

    def grip_12mm():
        print("grip_12mm Gripping...")
        set_digital_output(3, OFF)
        set_digital_output(2, ON)
        set_digital_output(1, ON)

    # ===== 위치 정의 (실제 환경에 맞게 수정 필요) =====
    JReady = [0, 0, 90, 0, 90, 0]
    # posx([367, 4, 194, 27, -179, 28])

    # 소스통 위치 Z (소스통이 놓여있는 곳 - Z축)
    bottle1_l = posx([635, -513, 374, 94, -90, -91])
    bottle1_j = [-15, 41, 59, 86, -70, 9]

    # 소스통 위치 (소스통이 놓여있는 곳) 
    bottle2_l = posx([635, -513, 258, 94, -90, -91])
    bottle2_j = [-15, 46, 68, 81, -71, 24]

    # 접시 위치
    plate1 = posx([823, -201, 280, 139, -93, -89])
    plate2 = posx([823, -201, 280, 139, -93, 90])

    # ===== 1단계: 소스통 잡고 들어올리기 =====
    #print("[Step 1] 소스통 위치로 이동 및 그리핑")
    #release_65mm()

    release_90mm()

    movej(JReady, vel=VELOCITY, acc=ACC)

    movej(bottle1_j, vel=VELOCITY, acc=ACC)  
    movel(bottle2_l, vel=VELOCITY, acc=ACC)
    grip_40mm()
    movel(bottle1_l, vel=VELOCITY, acc=ACC)


    movel(plate1, vel=VELOCITY, acc=ACC)
    movel(plate2, vel=VELOCITY, acc=ACC)

    # 소스 뿌리기 구현
    grip_20mm()
    move_spiral(rev=3,rmax=10.0,lmax=-10,v=40,a=40,axis=DR_AXIS_Z, ref=DR_BASE)
    wait(0.5)
    move_spiral(rev=3,rmax=10.0,lmax=-10,v=40,a=40,axis=DR_AXIS_Z, ref=DR_BASE)
    mwait(0.5)

    # 소스통 귀환
    movel(plate1, vel=VELOCITY, acc=ACC)

    movel(bottle1_l, vel=VELOCITY, acc=ACC)
    movel(bottle2_l, vel=VELOCITY, acc=ACC)

    release_90mm()
    movel(bottle1_l, vel=VELOCITY, acc=ACC)

    movej(JReady, vel=VELOCITY, acc=ACC)
    

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("powder_test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        setup_io_clients(node)
        perform_task_source()
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
