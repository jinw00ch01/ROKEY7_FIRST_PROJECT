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
        check_motion, get_tool_force,
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

    # 뒤집개 집기 전 z축 이동방향
    anchor_pos_0 = posx([372, -356, 210, 32, -172, 154])
    
    # 뒤집개 집기 위치
    anchor_pos_1 = posx([372, -356, 99, 32, -172, 154])
    
    
    # 바닥까지 하강 전의 뒤집개
    down_1 = posx([315, 140, 223, 89, -143, -178])
    # 바닥까지 하강 전 미들 포인트
    down_2 = posx([335, 144, 159, 86, -129, -175])
    # 바닥까지 하강하는 뒤집개 
    down_3 = posx([339, 129, 110, 87, -127, -178])



    # 바닥에서 Y축으로 이동(변경 전) 
    #grill_1 = posx([339, 9, 110, 87, -127, -178])

    # 바닥에서 y축으로 이동 처음 뒤집기 z축 상승전(변경 후)
    grill_1 = posx([310, 12, 50, 87, -107, 179])

    # 반죽을 집고 Z축으로 이동


    # 스윙 전
    swing_1 = posx([326, 25, 50, 90, -92, 179])

    # 스윙 중간
    swing_2 = posx([326, 25, 40, 90, -92, 140])

    # 스윙 후
    swing_3 = posx([463, 25, 65, 90, -93, 32])



    # 스윕 준비 z축 이동
    #sweep_1 = posx([234, 34, 154, 88, -113, -177])

    # 스윕 준비 - 바닥
    #sweep_2 = posx([234, 34, 50, 88, -113, -177])

    # 스윕1 - 중간점
    #sweep_3 = posx([337, 70, 50, 88, -113, -177])
    # 스윕2
    #sweep_4 = posx([440, 137, 50, 88, -113, -177])
    # 스윕3
    #sweep_5 = posx([440, 16, 50, 88, -113, -177])

    # 접시로 이동 바닥에서 쓸어가기1 (변경 후) 
    sweep_1 = posx([221, 20, 30, 85, -108, 178])
    # 접시로 바닥에서 쓸어가기2
    sweep_2 = posx([313, 47, 30, 101, -110, -179])
    # 접시로 이동 바닥에서 쓸어가기3
    sweep_3 = posx([457, 105, 30, 88, -113, -177])
    # 접시로 이동 바닥에서 쓸어가기4 z축 들어올리기전
    sweep_4 = posx([451, 10, 30, 87, -111, 179])
    # 접시 이동 z축 상승
    lift_up = posx([451, 10, 297, 87, -111, 179])

    

    #lift_up = posx([440, 16, 297, 88, -113, -177])

    plate_up = posx([690, -98, 252, 119, -109, -175])

    plate_down = posx([690, -98, 252, 119, -109, 111])

    back_home = posx([342, -141, 355, 94, -122, 85])


    anchor_back_0 = posx([374, -348, 311, 61, -173, -176])
    anchor_back_1 = posx([374, -348, 136, 61, -173, -176])



    movej(JReady, vel=150, acc=100)
    
    movel(anchor_pos_0, vel=150, acc=100)
    movel(anchor_pos_1, vel=150, acc=100)

    grip_12mm()
    wait(0.5)
  

    movel(anchor_pos_0, vel=150, acc=100)

    task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
    fd = [0, 0, 20, 0, 0, 0] #x,y,z,rx,ry,rz
    fctrl_dir= [0, 0, 1, 0, 0, 0] #z축 기준
    set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_ABS)  

    movel(down_1, vel=150, acc=ACC)
    movel(down_2, vel=VELOCITY, acc=ACC)
    movel(down_3, vel=VELOCITY, acc=ACC)

    movel(grill_1, vel=VELOCITY, acc=ACC)

    movel(swing_1, vel=100, acc=150)
    movel(swing_2, vel=100, acc=150)
    movel(swing_3, time=0.6)
    
    movel(sweep_1, vel=VELOCITY, acc=ACC)
    movel(sweep_2, vel=VELOCITY, acc=ACC)
    movel(sweep_3, vel=VELOCITY, acc=ACC)
    movel(sweep_4, vel=VELOCITY, acc=ACC)
    #movel(lift_up, vel=VELOCITY, acc=ACC)

    release_force()
    release_compliance_ctrl()

    movel(lift_up, vel=VELOCITY, acc=ACC)
    movel(plate_up, vel=VELOCITY, acc=ACC)
    movel(plate_down, time=0.4)

    movel(back_home, vel=VELOCITY, acc=ACC)

    # 뒤집개 내려놓기 - 힘 감지 반복 알고리즘
    FORCE_THRESHOLD = 7  # 기준 대비 힘 증가량(N) - 환경에 맞게 조정
    x_adjust = 0
    max_attempts = 10

    for attempt in range(max_attempts):
        cur_back_0 = posx([371 + x_adjust, -340, 311, 61, -173, -176])
        cur_back_1 = posx([371 + x_adjust, -340, 136, 61, -173, -176])

        movel(cur_back_0, vel=VELOCITY, acc=ACC)

        # 하강 전 기준 힘 측정
        base_force = get_tool_force(DR_BASE)
        base_fz = abs(base_force[2])
        print(f"[Attempt {attempt + 1}] Base Z force: {base_fz:.1f}N, x_adjust: {x_adjust}")

        # 비동기로 하강 시작
        amovel(cur_back_1, vel=VELOCITY, acc=ACC)

        force_detected = False
        while check_motion() != 0:
            cur_force = get_tool_force(DR_BASE)
            cur_fz = abs(cur_force[2])
            if cur_fz - base_fz > FORCE_THRESHOLD:
                force_detected = True
                print(f"  Force spike: {cur_fz:.1f}N (base: {base_fz:.1f}N, diff: {cur_fz - base_fz:.1f}N)")
                break
            wait(0.1)

        if not force_detected:
            # 힘 없이 도착 -> 릴리스 후 복귀
            release_65mm()
            movel(cur_back_0, vel=VELOCITY, acc=ACC)
            print(f"Spatula placed successfully (x_adjust={x_adjust})")
            break

        # 힘 감지 -> 위로 복귀 후 x를 -3 조정
        movel(cur_back_0, vel=VELOCITY, acc=ACC)
        x_adjust -= 4

    







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