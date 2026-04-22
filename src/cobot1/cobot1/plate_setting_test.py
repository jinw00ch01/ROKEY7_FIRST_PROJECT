import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 150
ACC = 150

# 디지털 출력 상태
ON, OFF = 1, 0

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    # 설정된 상수 출력
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


def perform_task_plate_setting():
    """접시 세팅 작업 수행"""
    print("Performing plate setting task...")
    from DSR_ROBOT2 import (
        posx,
        movej,
        movel,
        movec,
        set_digital_output,
        get_digital_input,
        wait,
        DR_MV_MOD_REL,
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
        print("grip_20mm Gripping...")
        # release()
        set_digital_output(3, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        # wait_digital_input(1)

    def grip_12mm():
        print("grip_12mm Gripping...")
        set_digital_output(3, OFF)
        set_digital_output(2, ON)
        set_digital_output(1, ON)


    # 위치 정의 (실제 환경에 맞게 수정 필요)
    JReady = [0, 0, 90, 0, 90, 0]                          # 초기 위치
    #plate_pick_pos = posx([19, 42, 27, -181, -103, 21])    # 접시가 있는 위치
    #plate_place_pos = posx([]) # 접시 세팅 목적지
    #place_down_offset = posx([0, 0, -100, 0, 0, 0])         # 접시를 내려놓기 위한 Z축 하강량
    plate_start0 = posx([622, 219, 244, 5, 173, -171])
    plate_start1 = posx([623, 220, 210, 5, 173, -171]) #[19,42,27,-181,-103,21]
    plate_start2 = posx([623,50,275,5,173,-171])

    # 접시 놓기 Z축 위 이동
    plate_end1 = posx([710, -183, 254, 170, -106, -1])

    plate_end2 = posx([710, -183, 95, 170, -106, -1])

    plate_end3 = posx([606, -183, 95, 170, -106, -1])


    # 1. 그리퍼를 릴리스한다
    print("Step 1: 그리퍼 릴리스 초기화")
    release_65mm()

    # 초기 위치로 이동
    print("Moving to ready position...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 2. 접시가 있는 지정된 위치로 이동한다
    print("Step 2: 접시 위치로 이동")
    movec(plate_start0, plate_start1, vel=200, acc=ACC)

    # 3. 접시가 있는 위치에서 그리퍼를 그립한다
    print("Step 3: 접시 그립")
    grip_12mm()
    wait(1.0)

    # 세팅 목적지 전에 필요한 위치 함수 추가
    print("Step 3: plate_start2로 이동")
    movel(plate_start2, vel=VELOCITY, acc=ACC)

    # 4. 세팅 목적지가 있는 위치로 이동한다
    print("Step 4: plate_end1로 이동")
    movel(plate_end1, vel=200, acc=ACC)
    movel(plate_end2, vel=VELOCITY, acc=ACC)

    release_65mm()

    movel(plate_end3, vel=VELOCITY, acc=ACC)


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("plate_setting_test", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    # 초기화는 한 번만 수행
    initialize_robot()

    perform_task_plate_setting()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
