'''
1. anchor 되어 있는 뒤집개
2. 뒤집개를 잡는 동작
3. 뒤집개를 들어올리는 동작
4. 이동 (뒤집기 시퀜스 입장 전까지)
'''

import rclpy
import DR_init
import time

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 40
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    # 로봇 초기화에 필요한 함수들을 DSR_ROBOT2에서 임포트
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode

    # Tool과 TCP 설정을 안전하게 적용하기 위해 로봇을 수동(Manual) 모드로 변경
    set_robot_mode(ROBOT_MODE_MANUAL)
    # 정의한 툴 설정 적용
    set_tool(ROBOT_TOOL)
    # 정의한 TCP 설정 적용
    set_tcp(ROBOT_TCP)
    
    # 설정을 마친 후 다시 로봇을 자동(Autonomous) 모드로 복귀
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    # 모드 변경 및 설정이 로봇에 완전히 반영되도록 2초간 대기
    time.sleep(2) 
    
    # 설정이 정상적으로 완료되었는지 확인하기 위해 터미널에 상태 출력
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

def pick_and_lift_flipper(anchor_pos, ending_pos,lift_height=200):
    
    """
    뒤집개를 잡고 설정된 높이만큼 들어올리는 동작
    :param anchor_pos: 뒤집개 위치 (posx 타입)
    :param lift_height: 들어올릴 높이 (mm)
    """
    print("Picking and lifting the flipper...")
    from DSR_ROBOT2 import posx, movel,set_tool_digital_output,wait,DR_TOOL,DR_MV_MOD_REL  # 필요한 기능만 임포트

    # 초기 위치 및 목표 위치 설정
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([500, 80, 200, 150, 179, 150])
    
    # 1. 뒤집개 위치로 이동
    movel(anchor_pos, vel=100, acc=100)
    
    # 2. 그리퍼 잡기 (Tool I/O 1번 채널 ON 예시)
    # 실제 환경에 맞게 index를 수정하세요 
    set_tool_digital_output(index=1, val=1)
    
    # 그리퍼가 완전히 잡을 때까지 잠시 대기 (필요 시)
    wait(0.5) 
    
    # 3. 뒤집개 들어올리기 (상대 이동 사용)
    # Z축으로 지정된 높이만큼 상대 이동 
    movel([0, 0, lift_height, 0, 0, 0], vel=30, acc=30, ref=DR_TOOL, mod=DR_MV_MOD_REL)

    # 4. 다음 시퀀스 진입을 위한 대기 위치로 이동
    movel(ending_pos, vel=100, acc=100)

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    # ROS2 파이썬 라이브러리 초기화
    rclpy.init(args=args)
    # 로봇 제어를 담당할 'spatula_task' 노드 생성 (네임스페이스는 로봇 ID 사용)
    node = rclpy.create_node("spatula_task", namespace=ROBOT_ID)

    # 생성된 노드 객체를 Doosan 로봇 초기화 모듈에 연결
    DR_init.__dsr__node = node

    try:
        # 로봇 툴, TCP, 모드 등을 설정하는 초기화 함수 호출 (1회 실행)
        initialize_robot()

        # 정의된 순서대로 작업(플로우)을 수행하는 함수 호출 (1회 실행)
        pick_and_lift_flipper()
        
    # 터미널에서 사용자가 강제로 종료(Ctrl+C)를 시도할 경우 프로그램 안전 종료 처리
    except KeyboardInterrupt:
        pass
    # 동작 중 예상치 못한 에러가 발생할 경우 그 원인을 출력
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    # 작업이 성공적으로 끝났거나 강제 종료되었을 때 반드시 실행되는 구문
    finally:
        # ROS2 통신을 종료하고 메모리 및 자원 반환
        rclpy.shutdown()

# 파일이 직접 실행되었을 때(모듈로 임포트되지 않았을 때) main() 함수를 실행하도록 하는 진입점
if __name__ == "__main__":
    main()

# --- 함수 사용 예시 ---
flipper_pos = posx(600, 43, 500, 0, 180, 0) # 뒤집개 위치 정의 [cite: 3]
ending_pos = posx(600, 600, 600, 0, 175, 0) # 다음 시퀀스 진입을 위한 대기 위치
pick_and_lift_flipper(flipper_pos, ending_pos, 200)
