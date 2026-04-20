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
ACC = 100

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


def perform_task():
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
        


    # 위치 정의 (실제 환경에 맞게 수정 필요)
    JReady = [0, 0, 90, 0, 90, 0]                          # 초기 위치
    pick_start_1 = posx([230, 212, 12, 36,-179,36]) #초기 집개 위치
    pick_start_2 = posx([230, 212, 150, 36, -179,36]) #초기 집개 위치2 (z축 상승)
    dough_start_1 =  posx([461, 211, 150, 36, -178, 36]) # 반죽 위치
    dough_start_2 =  posx([461, 211, 17, 36, -178, 36]) # 반죽 위치
    dough_end_1 = posx([303, -39, 45, 17, -177, 17]) # 반죽 놓기 위치
    dough_end_2 = posx([303, -39, 150, 17, -177, 17]) # 반죽 놓기 위치 (Z축 상승)

    # 1. 그리퍼를 릴리스한다
    print("Step 1: 그리퍼 release_90mm 초기화")
    release_90mm()

    # 초기 위치로 이동
    print("Moving to ready position...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 2. 
    print("Step 2: 집게 위치로 이동")
    movel(pick_start_1, vel=200, acc=ACC)

    # 3. 
    print("Step 3: 집게 그립")
    release_65mm()
    wait(0.5)

    # 세팅 목적지 전에 필요한 위치 함수 추가
    print("Step 3: pick_start_2로 이동")
    movel(pick_start_2, vel=VELOCITY, acc=ACC)

    print("Step 5: dough_start_2로 이동")
    movel(dough_start_1, vel=150, acc=ACC)

    # 4. 도우 목적지가 있는 위치로 이동한다
    print("Step 4: dough_start_1로 이동")
    movel(dough_start_2, vel=150, acc=ACC)

    # 5. 반죽 그립한다.
    print("Step 5: 반죽 그립")
    grip_20mm()
    wait(0.5)

    movel(dough_start_1, vel=150, acc=ACC)

    # 6. 접시를 반듯하게 내려놓기 위해 관절을 조절한다
    print("Step 6: dough_end로 이동")
    movel(dough_end_1, vel=VELOCITY, acc=ACC)

    # 7. 그리퍼를 릴리스한다
    print("Step 6: 그리퍼 릴리스 65mm")
    release_65mm()
    wait(0.5)

    movel(dough_end_2, vel=VELOCITY, acc=ACC)    

    movel(pick_start_2, vel=VELOCITY, acc=ACC)

    print("Step 2: 집게 위치로 이동")
    movel(pick_start_1, vel=200, acc=ACC)

    release_90mm()

    #print("Step 5: dough_start_2로 이동")
    #movel(dough_start_2, vel=150, acc=ACC)

    

    
    #print("Step 6: pick_start_1 처음 있던 집게 위치로 이동")
    #movel(pick_start_1, vel=VELOCITY, acc=ACC)

    #print("Step 6: 그리퍼 릴리스 90mm")
    #release_90mm()
    #wait(0.5)

    # 초기 위치로 이동
    #print("Moving to ready position...")
    #movej(JReady, vel=VELOCITY, acc=ACC)
    #print("Plate setting task completed!")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("plate_setting_test", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    # 초기화는 한 번만 수행
    initialize_robot()

    perform_task()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
