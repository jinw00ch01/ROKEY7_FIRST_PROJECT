import rclpy
import DR_init
import time
from dsr_msgs2.srv import (
    SetCtrlBoxDigitalOutput, GetCtrlBoxDigitalInput,
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


def setup_io_clients(node):
    """DSR_ROBOT2의 IO 서비스 버그를 우회하여 직접 서비스 클라이언트 생성"""
    global g_node, cli_set_digital_output, cli_get_digital_input
    g_node = node
    cli_set_digital_output = node.create_client(
        SetCtrlBoxDigitalOutput, "/" + ROBOT_ID + "/io/set_ctrl_box_digital_output")
    cli_get_digital_input = node.create_client(
        GetCtrlBoxDigitalInput, "/" + ROBOT_ID + "/io/get_ctrl_box_digital_input")
    cli_set_digital_output.wait_for_service(timeout_sec=5.0)
    cli_get_digital_input.wait_for_service(timeout_sec=5.0)
    print("IO service clients ready.")


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


def perform_task_powder_snap():
    """가루 뿌리기 작업 수행 (snap 방식)"""
    print("Performing powder snap task...")
    from DSR_ROBOT2 import (
        posx, movej, movel, wait,
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

    def grip_12mm():
        print("grip_12mm Gripping...")
        set_digital_output(3, OFF)
        set_digital_output(2, ON)
        set_digital_output(1, ON)

    # ===== 위치 정의 (실제 환경에 맞게 수정 필요) =====
    JReady = [0, 0, 90, 0, 90, 0]

    # 가루통 위치 (가루통이 놓여있는 곳)
    pos_bottle_above = posx([400, 200, 200, 0, -180, 0])     # 가루통 위 접근 위치
    pos_bottle_pick = posx([400, 200, 100, 0, -180, 0])      # 가루통 그립 위치

    # 서빙 접시 위 위치 (출구가 위를 향한 상태)
    pos_plate_above = posx([300, -80, 250, 0, -180, 0])

    # 가루통 뒤집은 상태 (출구가 아래로, B축 180도 회전)
    pos_plate_flipped = posx([300, -80, 250, 0, 0, 0])

    # ===== 1단계: 가루통 잡고 들어올리고 서빙할 접시 위로 이동 =====
    print("[Step 1] 가루통 잡고 들어올려서 접시 위로 이동")
    release_90mm()
    wait(0.3)

    movej(JReady, vel=VELOCITY, acc=ACC)

    movel(pos_bottle_above, vel=VELOCITY, acc=ACC)
    movel(pos_bottle_pick, vel=80, acc=ACC)

    grip_20mm()
    wait(0.5)
    print("  -> 가루통 그립 완료")

    # 가루통 들어올리기
    movel(pos_bottle_above, vel=VELOCITY, acc=ACC)

    # 서빙 접시 위로 이동
    movel(pos_plate_above, vel=VELOCITY, acc=ACC)

    # ===== 2단계: 가루통 뒤집기 (출구가 아래로, B축 0도 = 180도 회전) =====
    print("[Step 2] 가루통 뒤집기 (출구 아래로, B축 0도)")
    movel(pos_plate_flipped, vel=80, acc=ACC)
    wait(0.5)

    # ===== 3단계: Snap으로 가루 뿌리기 =====
    print("[Step 3] 그리퍼 snap으로 가루 뿌리기")
    
    print("  -> 가루 뿌리기 완료")

    # ===== 4단계: 가루통 원위치로 회전 (출구가 위로) =====
    print("[Step 4] 가루통 원위치 회전 (출구 위로, B축 -180도)")
    movel(pos_plate_above, vel=80, acc=ACC)
    wait(0.3)

    # ===== 5단계: 가루통을 원래 위치에 되돌려놓기 =====
    print("[Step 5] 가루통 원래 위치로 복귀")
    movel(pos_bottle_above, vel=VELOCITY, acc=ACC)
    movel(pos_bottle_pick, vel=80, acc=ACC)

    release_65mm()
    wait(0.5)
    print("  -> 가루통 릴리스 완료")

    movel(pos_bottle_above, vel=VELOCITY, acc=ACC)

    # 초기 위치로 복귀
    movej(JReady, vel=VELOCITY, acc=ACC)
    print("Powder snap task completed!")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("source_test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        setup_io_clients(node)
        perform_task_powder_snap()
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
