import rclpy
import DR_init
import time
from dsr_msgs2.srv import (
    SetCtrlBoxDigitalOutput, GetCtrlBoxDigitalInput,
    CheckForceCondition
)

# 각 작업 파일에서 perform_task 함수 import
from cobot1.dough_grip_test import perform_task_dough_grip
from cobot1.press_test import perform_task_press
from cobot1.plate_setting_test import perform_task_plate_setting
from cobot1.spatula_test import perform_task_spatula
from cobot1.dough_to_plate import perform_task_dough_to_plate

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


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 전체 작업 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("main_task", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        setup_io_clients(node)

        # 각 파일에서 import한 작업 함수를 순서대로 수행
        #perform_task_dough_grip()
        #perform_task_press()
        #perform_task_plate_setting()
        perform_task_spatula()
        perform_task_dough_to_plate()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
