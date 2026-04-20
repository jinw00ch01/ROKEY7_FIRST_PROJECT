import rclpy
import DR_init
import time

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def servo_on():
    """로봇 서보온 수행"""
    from DSR_ROBOT2 import (
        set_robot_mode, get_robot_mode, set_tool, set_tcp,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    )

    print("서보온 시작...")

    # 매뉴얼 모드로 전환 후 Tool/TCP 설정
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    # 자율 모드(서보온)로 전환
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)

    mode = get_robot_mode()
    print("#" * 50)
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_MODE (0:수동, 1:자동): {mode}")
    if mode == 1:
        print("서보온 완료!")
    else:
        print("서보온 실패 - 모드를 확인하세요.")
    print("#" * 50)


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 서보온 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("servo_on", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        servo_on()
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
