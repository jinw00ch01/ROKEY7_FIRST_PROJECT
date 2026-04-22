# Doosan M0609 팬케이크 공정 로봇 - 프로젝트 실행 매뉴얼

## 1. 프로젝트 구조

```
cobot_ws/
├── src/cobot1/                           # ROS2 Python 패키지
│   ├── setup.py                          # 패키지 설정
│   └── cobot1/
│       ├── main.py                       # 기존 순차 실행 (레거시)
│       │
│       ├── helpers/                      # 공통 헬퍼 모듈
│       │   ├── io_manager.py             # 로봇 모션 + 그리퍼 IO 통합 래퍼
│       │   └── pose_manager.py           # 좌표 상수 중앙 관리
│       │
│       ├── managers/                     # 매니저 노드
│       │   ├── task_manager_node.py      # 워크플로우 총괄 (ROS2 노드)
│       │   ├── tool_manager.py           # 도구 pick/return 전략
│       │   └── object_manager.py         # 물체(접시) pick & place
│       │
│       ├── tasks/                        # 태스크 코어 로직
│       │   ├── dough_task.py             # 반죽 배치
│       │   ├── press_task.py             # 프레스 누르기
│       │   ├── flip_task.py              # 뒤집개 뒤집기
│       │   ├── sauce_task.py             # 소스 뿌리기
│       │   └── powder_task.py            # 가루 뿌리기
│       │
│       └── *_test.py                     # 기존 개별 테스트 (하위 호환용)
│
└── backend/                              # 웹 UI + Firebase 백엔드
    ├── robot_backend.py                  # Firebase 연동 서버
    ├── index.html                        # 관리자 대시보드
    └── user_interface.html               # 고객 키오스크 UI
```

## 2. 사전 준비

### 2.1 환경 요구사항
- Ubuntu 22.04
- ROS2 Humble
- Doosan Robot Controller (dsr_common2, dsr_msgs2)
- Python 3.10
- Firebase Admin SDK (`pip install firebase-admin`)

### 2.2 빌드
```bash
cd ~/cobot_ws
colcon build --packages-select cobot1
source install/setup.bash
```

## 3. 실행 방법

### 3.1 방법 A: 신규 아키텍처 (task_manager_node)

**소스/가루 없이 실행:**
```bash
ros2 run cobot1 task_manager
```

**소스/가루 선택하여 실행:**
```bash
ros2 run cobot1 task_manager --ros-args -p sauce:=sauce1 -p powder:=powder1
```

**ROS2 서비스로 외부에서 트리거:**
```bash
# 터미널 1: 노드 실행 (대기 모드로 전환 필요 시 main() 수정)
ros2 run cobot1 task_manager

# 터미널 2: 서비스 호출
ros2 service call /dsr01/start_workflow std_srvs/srv/Trigger
```

**워크플로우 상태 모니터링:**
```bash
ros2 topic echo /dsr01/workflow_status
```

### 3.2 방법 B: 기존 방식 (main.py)
```bash
ros2 run cobot1 main
```

### 3.3 방법 C: 웹 UI + Firebase 백엔드
```bash
# 터미널 1: 백엔드 서버 실행
cd ~/cobot_ws/backend
python3 robot_backend.py

# 브라우저에서 UI 열기
# 관리자: backend/index.html
# 고객:   backend/user_interface.html
```

### 3.4 개별 태스크 테스트
```bash
ros2 run cobot1 dough_grip_test      # 반죽 집기만
ros2 run cobot1 press_test           # 프레스만
ros2 run cobot1 plate_setting_test   # 접시 세팅만
ros2 run cobot1 spatula_test         # 뒤집개만
ros2 run cobot1 source_test          # 소스만
ros2 run cobot1 powder_test          # 가루만
```

## 4. 워크플로우 순서

```
1. [tongs]         집게 pick → 반죽 잡기 → 그릴에 배치 → 집게 return
2. [presser]       프레스 pick → 컴플라이언스+힘 제어 누르기 → 도구 털기 → 프레스 return
3. [plate]         접시 pick → 서빙 위치에 place (movec 원호 이동)
4. [spatula]       뒤집개 pick → 컴플라이언스 모드 뒤집기 → 스윕 → 뒤집개 return (힘 감지)
5. [sauce_bottle]  소스병 pick → 나선 패턴 뿌리기 (move_spiral) → 소스병 return
6. [powder_bottle] 가루통 pick → 뒤집기 → 스냅 동작 뿌리기 (move_periodic) → 가루통 return
```

## 5. 관리자 UI (index.html) 기능

| 버튼 | 기능 | Firebase 명령 |
|------|------|--------------|
| 시작 | 워크플로우 시작 (소스/가루 선택 가능) | start_requests |
| 일시 정지 | 로봇 일시 정지 (MovePause) | pause |
| 재개 | 일시 정지 해제 (MoveResume) | resume |
| 충돌 시뮬레이션 | 강제 정지 테스트 (MoveStop DR_HOLD) | simulate_collision |
| 안전정지(노란불) 해제 | 보호정지 해제 (SetRobotMode) | release_safety_stop |
| 비상정지(빨간불) 해제 | 비상정지 해제 (SetRobotMode) | release_emergency_stop |

## 6. 아키텍처 데이터 흐름

```
task_manager_node
  │
  ├── tool_manager.pick_tool("tongs")
  │     └── io_manager.gripper_open("90mm")
  │     └── io_manager.move_joint_safe(JOINT_POSES['ready'])
  │     └── io_manager.move_line_safe(poses['dough']['pick_start_1'])
  │     └── io_manager.gripper_open("65mm")
  │
  ├── dough_task.place_dough_with_tongs()
  │     └── io_manager.move_line_safe(...)
  │     └── io_manager.gripper_close("50mm")
  │     └── io_manager.gripper_open("65mm")
  │
  └── tool_manager.return_tool("tongs")
        └── io_manager.move_line_safe(...)
        └── io_manager.gripper_open("90mm")
```

## 7. 주요 파일별 역할

| 파일 | 역할 |
|------|------|
| `io_manager.py` | 모든 로봇 IO를 한 곳에 통합. movej/movel 래퍼, 그리퍼 6종 프리셋, 힘 센서 조회 |
| `pose_manager.py` | 6개 태스크의 모든 좌표를 딕셔너리로 중앙 관리. 좌표 변경 시 이 파일만 수정 |
| `tool_manager.py` | 5가지 도구의 pick/return 실제 모션 시퀀스 구현 |
| `object_manager.py` | 접시 pick & place (원호 이동 movec 포함) |
| `task_manager_node.py` | 6단계 워크플로우 오케스트레이션 + ROS2 서비스 인터페이스 |

## 8. 좌표 수정 방법

모든 좌표는 `src/cobot1/cobot1/helpers/pose_manager.py`에서 관리합니다.

```python
# pose_manager.py에서 좌표 수정 예시
'dough': {
    'dough_start_2': posx([461, 241, 9, 36, -178, 36]),  # ← 이 값만 변경
}
```

수정 후 재빌드:
```bash
colcon build --packages-select cobot1
source install/setup.bash
```

## 9. 그리퍼 프리셋 참조

| 이름 | 크기 | 용도 |
|------|------|------|
| `grip_20mm` | 20mm | 완전 그립 (프레스, 뒤집개, 소스 분출) |
| `grip_12mm` | 12mm | 약한 그립 (접시, 뒤집개) |
| `grip_40mm` | 40mm | 중간 그립 (소스병) |
| `grip_50mm` | 50mm | 넓은 그립 (반죽) |
| `release_65mm` | 65mm | 중간 열림 (집게, 도구 반납) |
| `release_90mm` | 90mm | 완전 열림 (초기화, 대형 도구) |

## 10. 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `AttributeError: 'NoneType' ... 'create_client'` | DR_init 노드 미설정 | `DR_init.__dsr__node = node` 확인 |
| `NameError: press_test is not defined` | import 누락 | `from cobot1 import press_test` 추가 |
| 일시정지/재개가 안 됨 | executor 충돌 | 제어용 별도 노드 사용 (robot_backend.py 참조) |
| setup.py entry point 오류 | 콤마 누락 | 각 entry 줄 끝에 `,` 확인 |
| 서비스 타임아웃 | Doosan 드라이버 미실행 | `ros2 launch dsr_bringup2 ...` 확인 |
