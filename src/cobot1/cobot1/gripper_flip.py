import time
import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01" # 로봇 ID 설정
ROBOT_MODEL = "m0609" # 로봇 모델명 설정
ROBOT_TOOL = "Tool Weight" # 사용 중인 툴 이름 설정
ROBOT_TCP = "GripperDA_v1" # 사용 중인 TCP 이름 설정

# 기본 이동 속도 및 가속도 설정
VELOCITY = 60 # 관절 이동 목표 속도
ACC = 60 # 관절 이동 목표 가속도

# DR_init에 로봇 ID와 모델 정보 등록
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


def perform_task():
    """로봇이 수행할 작업 (뒤집개 조작 플로우)"""
    # 작업 시작을 알리는 메시지 출력
    print("시작")
    
    # 작업 수행에 필요한 모션 및 제어 함수 임포트
    from DSR_ROBOT2 import (
        movel, movej, 
        task_compliance_ctrl, release_compliance_ctrl,
        DR_TOOL, DR_MV_MOD_REL
    )
    # 좌표 설정을 위한 posx(직교 좌표), posj(관절 좌표) 임포트
    from DR_common2 import posx, posj

    # 1. 그릴 바깥 지점에서 뒤집개를 잡고 있는 상태 (명령어 생략 및 쥐고 있음 유지)
    print("1. 그립퍼가 뒤집개를 잡고 있는 상태 (유지)")

    # 2. 뒤집개를 잡은 채로 이동 (그릴 중앙 및 도우 아래로 밀어 넣기)
    print("2. 도우 아래로 뒤집개 밀어 넣기...")
    # 직접 교시로 얻은 j1~j6 관절 각도 설정
    p2_posj = posj(51.98, 31.76, 79.11, -29.97, 109.99, 137.89)
    # 설정한 관절 각도(p2_posj)를 향해 관절 보간 이동(movej) 수행
    movej(p2_posj, vel=VELOCITY, acc=ACC)

    # 3. Y축 음의 방향으로 밀기 (외력이 느껴질 때까지 도달한 좌표)
    print("3. Y축 음의 방향으로 밀어넣기...")
    # 벽에 부딪혔을 때 측정한 j1~j6 관절 각도 설정
    p3_posj = posj(48.72, 28.23, 89.60, -31.19, 102.52, 138.79)
    # 정밀한 힘 제어 전 접근이므로 기본 속도 그대로 관절 이동(movej) 수행
    movej(p3_posj, vel=VELOCITY, acc=ACC)

    # 4. 순응 제어 상태에서 그립퍼 90도 제자리 회전
    print("4. 순응 제어 활성화 및 90도 회전...")
    # 바닥에 부딪혀도 로봇이 유연하게 헛돌거나 순응할 수 있도록 강성(Stiffness) 부여
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    # 툴의 끝단을 기준으로 '제자리 회전'을 해야 하므로, 이 부분만 직교 상대 이동(movel + DR_MV_MOD_REL)을 사용
    movel(posx(0, 0, 0, 0, 90, 0), vel=30, acc=30, ref=DR_TOOL, mod=DR_MV_MOD_REL)

    # 5. 회전했던 반대 방향으로 다시 순응 제어 상태에서 90도 원복 회전
    print("5. 반대 방향으로 90도 원복 회전...")
    # 동일하게 툴 좌표계 기준으로 -90도 제자리 회전하여 원 위치로 복구
    movel(posx(0, 0, 0, 0, -90, 0), vel=30, acc=30, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    
    # 바닥과 관련된 순응 제어가 끝났으므로 순응 제어를 끄고 위치 제어 모드로 복귀
    release_compliance_ctrl()

    # 6. X축 양의 방향으로 밀기 (벽에 부딪혀서 외력이 느껴질 때까지의 좌표)
    print("6. X축 양의 방향으로 밀기...")
    # 직접 교시로 얻은 미는 동작 완료 시점의 j1~j6 관절 각도 설정
    p6_posj = posj(40.23, 31.73, 81.06, -40.95, 96.55, 138.38)
    # 해당 관절 자세를 향해 관절 보간 이동(movej) 수행
    movej(p6_posj, vel=VELOCITY, acc=ACC)

    # 7. Z축 양방향으로 위로 올리기 (뒤집개 들어 올린 좌표)
    print("7. Z축 양방향으로 뒤집개 들어올리기...")
    # 위로 들어 올려진 자세의 j1~j6 관절 각도 설정
    p7_posj = posj(38.79, 30.62, 68.40, -49.92, 114.59, 117.70)
    # 들어 올리는 관절 자세를 향해 관절 이동(movej) 수행
    movej(p7_posj, vel=VELOCITY, acc=ACC)

    # 8. 접시 위로 조인트 이동 (높이를 유지한 채 그릇 위로)
    print("8. 접시 위로 조인트 이동...")
    # 최종 목적지인 접시 위의 j1~j6 관절 각도 설정
    p8_posj = posj(29.12, 27.14, 77.01, -70.81, 76.14, 103.69)
    # 해당 목적지 자세로 관절 이동(movej) 수행
    movej(p8_posj, vel=VELOCITY, acc=ACC)

    # 모든 공정이 끝난 후 그립퍼 release 명령을 내리지 않아 뒤집개를 쥐고 있는 상태 유지
    print("종료")


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
        perform_task()
        
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