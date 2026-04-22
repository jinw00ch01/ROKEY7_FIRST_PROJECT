import rclpy
import DR_init
import time
from dsr_msgs2.srv import SetCtrlBoxDigitalOutput, GetCtrlBoxDigitalInput

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
    """DSR_ROBOT2의 IO/Force 서비스 버그를 우회하여 직접 서비스 클라이언트 생성"""
    global g_node, cli_set_digital_output, cli_get_digital_input
    g_node = node
    cli_set_digital_output = node.create_client(
        SetCtrlBoxDigitalOutput, "/" + ROBOT_ID + "/io/set_ctrl_box_digital_output")
    cli_get_digital_input = node.create_client(
        GetCtrlBoxDigitalInput, "/" + ROBOT_ID + "/io/get_ctrl_box_digital_input")
    cli_set_digital_output.wait_for_service(timeout_sec=5.0)
    cli_get_digital_input.wait_for_service(timeout_sec=5.0)
    print("IO clients ready.")

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


def perform_task_dough_to_plate():
    """반죽을 접시로 옮기는 작업 수행"""
    print("Performing dough to plate task...")
    from DSR_ROBOT2 import posx, movel, movej, get_current_posj, wait

    # Release 동작
    def release_65mm():
        print("65mm_Releasing...")
        set_digital_output(3, OFF)
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    # ===== 위치 정의 =====
    pos1 = posx([321, 64, 119, 91, -132, 177])
    pos2 = posx([464, 172, 133, 86, -134, -178])
    pos3 = posx([446, 78, 128, 88, -126, -179])
    pos4 = posx([446, 78, 259, 88, -126, -179])
    pos5 = posx([722, -32, 262, 124, -118, -154])
    pos7 = posx([312, -114, 317, 94, -163, -179])
    pos8 = posx([316, -145, 255, 94, -162, -177])

    # 1.
    print("[Step 1] 이동")
    movel(pos1, vel=VELOCITY, acc=ACC)
    
    # 2.
    print("[Step 2] 이동")
    movel(pos2, vel=VELOCITY, acc=ACC)

    # 3.
    print("[Step 3] 이동")
    movel(pos3, vel=VELOCITY, acc=ACC)

    # 4.
    print("[Step 4] 이동")
    movel(pos4, vel=VELOCITY, acc=ACC)

    # 5.
    print("[Step 5] 이동")
    movel(pos5, vel=VELOCITY, acc=ACC)

    # 6. 5번 값에서 j6값만 +145도
    print("[Step 6] 5번 위치에서 j6값만 +145도 회전")
    current_j = get_current_posj()
    current_j[5] += 145.0
    movej(current_j, vel=VELOCITY, acc=ACC)
    
    # 7.
    print("[Step 7] 이동")
    movel(pos7, vel=VELOCITY, acc=ACC)

    # 8.
    print("[Step 8] 이동")
    movel(pos8, vel=VELOCITY, acc=ACC)

    # 9. 그립퍼 릴리즈
    print("[Step 9] 그립퍼 릴리즈")
    release_65mm()
    wait(1.0)
    
    print("반죽을 접시로 옮기는 작업 완료!")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("dough_to_plate_node", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        setup_io_clients(node)
        perform_task_dough_to_plate()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()