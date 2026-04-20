import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 60
ACC = 60

# 디지털 출력 상태
ON, OFF = 1, 0

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp,get_tool,get_tcp,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트
    from DSR_ROBOT2 import get_robot_mode,set_robot_mode

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



def perform_task_spatula():
    """로봇이 수행할 작업"""
    print("Performing grip task...")
    from DSR_ROBOT2 import (
        set_digital_output,
        get_digital_input,
        movej,wait, posx,
        movel,
        movec,
        amovel,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force,
        DR_AXIS_Z, DR_BASE, DR_FC_MOD_ABS, DR_TOOL
    )

    # 디지털 입력 신호 대기 함수
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            # print("Waiting for digital input...")


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

    # 초기 위치로 이동
    JReady = [0, 0, 90, 0, 90, 0]
    print("Moving to ready position...")

    release_65mm()

    anchor_pos_0 = posx([316, -110, 350, 94, -162, -177])
    anchor_pos_1 = posx([316, -145, 255, 94, -162, -177])
    anchor_pos_2 = posx([312, -114, 317, 94, -163, -179])
    
    
    
    # 바닥까지 하강 전의 뒤집개
    pos1 = posx([321, 107, 271, 94, -163, -179])


    pos1_2 = posx([316, 163, 188, 87, -134, 177])

    # 바닥까지 하강하는 뒤집개 119
    pos2 = posx([321, 169, 119, 91, -132, 177])

    # 바닥에서 Y축으로 이동 y=149
    pos3 = posx([321, 64, 119, 91, -132, 177])

    # 반죽을 집고 Z축으로 이동
    pos4 = posx([321, 64, 209, 91, -132, 177])







    # x축으로 이동하기 전 - 회전
    pos5 = posx([317, 131, 135, 90, -108, 175])

    # x축으로 이동
    pos6 = posx([286, 111, 117, 84, -101, 124])

    pos7 = posx([330, 113, 158, 92, -95, 131])


    #pos2 = posx([338, 95, 100, 89, -129, 175])
    # X=338.54, Y=173.24, Z=129.04, A=89.42, B=-129.60, C=175.40

    # pos3 = posx([324, 90, 144, 83, -100, 63])
    #[50, 30, 91, -39, 120, 44]

    # pos4 = posx([374, 90, 144, 83, -100, 63])

    # pos5 = posx([350, 69, 190, 85, -107, 79])

    movej(JReady, vel=VELOCITY, acc=ACC)
    
    movel(anchor_pos_0, vel=100, acc=100)
    movel(anchor_pos_1, vel=100, acc=100)

    grip_12mm()
    wait(0.5)
  

    movel(anchor_pos_2, vel=100, acc=100)

    task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
    fd = [0, 0, 20, 0, 0, 0] #x,y,z,rx,ry,rz
    fctrl_dir= [0, 0, 1, 0, 0, 0] #z축 기준
    set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_ABS)  

    movel(pos1, vel=VELOCITY, acc=ACC)

    movel(pos1_2, vel=VELOCITY, acc=ACC)

    movel(pos2, vel=VELOCITY, acc=ACC)

    movel(pos3, vel=100, acc=ACC)

    movel(pos4, vel=100, acc=ACC) 
    
    movel(pos5, vel=100, acc=ACC)
    
    movel(pos6, vel=100, acc=ACC)

    movel(pos7, vel=100, acc=ACC)


    release_force()
    release_compliance_ctrl()

    # movel(pos5, vel=190, acc=ACC)


    # Grip 및 Release 반복
    #while rclpy.ok():
    #    grip_20mm()
    #    wait(1.5)
    #    grip_12mm()
    #    wait(1.5)
    #    release_65mm()
    #    wait(1.5)
    #    release_90mm()
    #    wait(1.5)

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("grip_simple", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    # 초기화는 한 번만 수행
    initialize_robot()

    perform_task_spatula()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()