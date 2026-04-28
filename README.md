# Doosan M0609 팬케이크 공정 자동화 로봇 시스템

## 프로젝트 개요

Doosan M0609 6축 협동 로봇을 활용한 **팬케이크 조리 공정 자동화 시스템**입니다.
반죽 배치부터 프레스, 뒤집기, 접시 세팅, 소스/가루 토핑까지 6단계 공정을 자동 수행하며,
Firebase 기반 웹 UI를 통해 원격 제어 및 실시간 모니터링이 가능합니다.

### 핵심 기능

- **6단계 자동 공정**: 반죽 → 프레스 → 접시 세팅 → 뒤집기 → 소스 → 가루
- **5종 도구 자동 교환**: 집게, 프레스, 뒤집개, 소스병, 가루통의 자동 pick/return
- **힘 제어 및 컴플라이언스**: 반죽 누르기(힘 제어), 뒤집기(컴플라이언스 모드), 뒤집개 반납(힘 감지)
- **스플라인 경로**: 뒤집기 후 스윕 동작에 movesx 스플라인 곡선 적용 (부드러운 연속 이동)
- **웹 기반 원격 제어**: Firebase Realtime DB를 통한 실시간 명령/상태 동기화
- **4단계 안전 복구 시스템**: DrlStop → SetRobotControl → 재초기화 → MoveResume
- **그리퍼 자동 복원**: 안전정지/비상정지 시 디지털 출력 리셋 방지를 위한 즉시 복원
- **Tool/TCP 자동 검증**: 매 모션 전 1회 검증, 모드 전환 시 자동 재설정
- **이중 UI**: 관리자 대시보드 + 고객용 키오스크 UI (팬케이크 요정 테마, TTS 음성 안내)

---

## 시스템 환경

| 항목 | 사양 |
|------|------|
| OS | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Python | 3.10 |
| 로봇 | Doosan M0609 |
| 그리퍼 | GripperDA_v1 (디지털 출력 3핀 제어, 6종 프리셋) |
| 드라이버 | doosan-robot2 (dsr_bringup2, dsr_msgs2) |
| 백엔드 | Firebase Admin SDK + Realtime Database |
| 프론트엔드 | HTML/CSS/JS + Firebase JS SDK + Web Speech API (TTS) |

---

## 사용 장비 목록

### 로봇 / 제어 하드웨어

| 분류 | 품목 | 비고 |
|------|------|------|
| 협동 로봇 | Doosan M0609 (6축) | 가반하중 6 kg, 작업반경 900 mm |
| 로봇 컨트롤러 | Doosan 컨트롤박스 | 디지털 I/O, 비상정지 입력 포함 |
| 그리퍼 | GripperDA_v1 | 컨트롤박스 디지털 출력 3핀(1·2·3)으로 6종 프리셋 제어 |
| 비상정지 버튼 | 외부 E-STOP | 컨트롤박스 안전 입력 연결 |
| 티칭 펜던트 | Doosan TP | 수동 모드 / 좌표 티칭 용도 |

### 운영 PC

| 분류 | 용도 | 비고 |
|------|------|------|
| 제어 PC (노트북 #1) | ROS2 드라이버 + `robot_backend.py` 실행 | Ubuntu 22.04 / ROS2 Humble |
| UI PC (노트북 #2) | 관리자 대시보드 / 키오스크 UI 표시 | 브라우저만 있으면 가능 (Firebase 경유 동기화) |

> 1대로도 운용 가능하지만, 키오스크 화면을 손님에게 분리 노출하는 경우 2대 운용을 권장합니다 (자세한 절차는 `PROJECT_MANUAL.md` 참고).

### 작업 환경 / 소모품

| 분류 | 품목 | 용도 |
|------|------|------|
| 도구 | 집게 (tongs) | 반죽 pick & place |
| 도구 | 프레스 (presser) | 반죽 누르기 |
| 도구 | 뒤집개 (spatula) | 팬케이크 뒤집기 |
| 도구 | 소스병 (source_bottle) | 소스 분출 |
| 도구 | 가루통 (powder_bottle) | 가루 토핑 |
| 거치대 | 도구 거치대 5종 | 각 도구 pick/return 위치 |
| 조리 | 전기 그릴 / 핫플레이트 | 팬케이크 가열 면 |
| 조리 | 접시 스택 | 접시 pick 위치 (원호 이동) |
| 조리 | 반죽 디스펜서 | 반죽 pick 위치 |
| 소모품 | 팬케이크 반죽, 블루베리맛 소스, 팝핑 캔디 가루 | 메뉴 재료 |

---

## 의존성

### Python 패키지 — `requirements.txt`

```bash
pip install -r requirements.txt
```

| 패키지 | 용도 |
|--------|------|
| `firebase-admin` (>=6.0.0) | `robot_backend.py` Firebase Realtime DB 연동 |

### ROS2 / 로봇 드라이버 (apt 또는 소스 빌드)

| 패키지 | 비고 |
|--------|------|
| `ros-humble-desktop` | ROS2 Humble 기본 패키지 (rclpy, std_msgs, std_srvs 포함) |
| `doosan-robot2` | `dsr_bringup2`, `dsr_msgs2`, `DR_init` 모듈 제공 (소스 빌드) |

### 프론트엔드 (별도 설치 불필요)

| 라이브러리 | 로드 방식 |
|-----------|----------|
| Firebase JS SDK | HTML 내 CDN |
| Web Speech API | 브라우저 내장 (Chrome 권장) |

### Firebase 서비스 계정 키

`backend/rokey-*.json` 파일을 사용합니다 (저장소에는 포함되지 않음 — 별도 발급 필요).

---

## 프로젝트 구조

```
cobot_ws/
├── src/cobot1/                              # ROS2 Python 패키지
│   ├── setup.py                             # 패키지 설정 (entry_points)
│   ├── package.xml                          # ROS2 패키지 메타데이터
│   └── cobot1/
│       ├── helpers/                         # 공통 헬퍼 모듈
│       │   ├── io_manager.py               # 로봇 모션 + 그리퍼 IO 통합 래퍼
│       │   └── pose_manager.py             # 좌표 상수 중앙 관리
│       │
│       ├── managers/                        # 매니저 노드
│       │   ├── task_manager_node.py         # 워크플로우 총괄 ROS2 노드
│       │   ├── run_single_task.py           # 개별 태스크 실행기
│       │   ├── tool_manager.py              # 도구 pick/return 전략
│       │   └── object_manager.py            # 물체(접시) pick & place
│       │
│       ├── tasks/                           # 태스크 코어 로직
│       │   ├── dough_task.py                # 반죽 배치
│       │   ├── press_task.py                # 프레스 누르기
│       │   ├── flip_task.py                 # 뒤집개 뒤집기
│       │   ├── plate_setting_task.py        # 접시 세팅
│       │   ├── source_task.py               # 소스 뿌리기
│       │   └── powder_task.py               # 가루 뿌리기
│       │
│       ├── requirement/                     # 설계 요구사항 문서
│       └── *_test.py                        # 레거시 개별 테스트 (하위 호환)
│
├── backend/                                 # 웹 UI + Firebase 백엔드
│   ├── robot_backend.py                     # Firebase 연동 + ROS2 제어 서버
│   ├── index.html                           # 관리자 대시보드
│   ├── index.css                            # 관리자 대시보드 스타일
│   ├── user_interface.html                  # 고객 키오스크 UI
│   ├── user_interface.css                   # 키오스크 UI 스타일
│   └── rokey-*.json                         # Firebase 서비스 계정 키
│
├── build/                                   # colcon 빌드 산출물
├── install/                                 # colcon 설치 산출물
└── PROJECT_MANUAL.md                        # 프로젝트 실행 매뉴얼
```

---

## 아키텍처

### 노드 아키텍처 설계 근거: 단일 노드 방식을 선택한 이유

ROS2 노드 아키텍처는 크게 두 가지 방식이 있습니다:

| | 단일 노드 (본 프로젝트) | 멀티 노드 |
|---|---|---|
| 구조 | 노드 1개, 기능은 Python 클래스/메서드로 구현 | 기능마다 독립 노드, 노드 간 ROS2 서비스/토픽 통신 |
| 통신 | 같은 프로세스 내 메서드 호출 (오버헤드 없음) | ROS2 서비스 요청/응답 (직렬화/역직렬화 오버헤드) |
| 장점 | 단순, 디버깅 쉬움, 상태 공유 용이 | 병렬 처리, 독립 실행/재시작, 분산 배치 가능 |

**본 프로젝트에서 단일 노드 방식을 선택한 이유:**

1. **순차 공정 특성**: 팬케이크 공정은 반드시 순서대로 실행해야 합니다 (반죽 → 프레스 → 접시 → 뒤집기 → 소스 → 가루). 병렬 실행이 불가능하므로 멀티 노드의 장점(병렬 처리)을 활용할 수 없습니다.
2. **도구 상태 공유**: `ToolManager`가 현재 들고 있는 도구를 추적하며 충돌을 방지합니다. 멀티 노드에서는 이 상태를 노드 간에 동기화해야 하는 복잡성이 추가됩니다.
3. **IOManager 공유**: 모든 태스크가 동일한 `IOManager` 인스턴스를 공유하여 ROS2 서비스 클라이언트를 재사용합니다.
4. **단순성**: 디버깅이 쉽고, 상태 추적이 명확하며, 노드 간 통신 오류 가능성이 없습니다.

```
본 프로젝트 구조 (단일 노드):

TaskManagerNode (ROS2 노드 1개)
  ├── ToolManager.pick_tool('tongs')              # 메서드 호출
  ├── DoughTask.place_dough_with_tongs()          # 메서드 호출
  ├── ToolManager.return_tool('tongs')            # 메서드 호출
  ├── ToolManager.pick_tool('presser')            # 메서드 호출
  ├── PressTask.press_dough()                     # 메서드 호출
  ├── ...                                         # (순차 실행)
  └── PowderTask.sprinkle_powder()                # 메서드 호출
```

> `robot_backend.py`에서만 **태스크 노드 + 제어 노드 = 2개**를 사용합니다.
> 이는 태스크 기능 분리가 아니라 `spin_until_future_complete` 중첩 호출 시 발생하는 executor 충돌을 방지하기 위한 구조입니다.

### 시스템 전체 구조

```
┌────────────────────────────────────────────────────────────────┐
│  웹 브라우저                                                    │
│  ┌──────────────────┐  ┌────────────────────────┐              │
│  │ index.html       │  │ user_interface.html     │              │
│  │ (관리자 대시보드) │  │ (고객 키오스크 UI)      │              │
│  │ + index.css      │  │ + user_interface.css    │              │
│  └────────┬─────────┘  └──────────┬─────────────┘              │
│           │  Firebase Realtime DB  │                            │
│           └──────────┬─────────────┘                            │
│                      ▼                                          │
│  ┌─────────────────────────────────────────┐                   │
│  │ robot_backend.py                        │                   │
│  │ ├── 태스크 노드 (robot_backend)         │                   │
│  │ │   ├── IOManager (모션+그리퍼)         │                   │
│  │ │   ├── ToolManager (도구 관리)         │                   │
│  │ │   └── *Task (6종 태스크)              │                   │
│  │ └── 제어 노드 (robot_control)           │                   │
│  │     ├── MovePause / MoveResume          │                   │
│  │     ├── MoveStop / DrlStop              │                   │
│  │     ├── SetRobotMode / SetRobotControl  │                   │
│  │     ├── SetCtrlBoxDigitalOutput (복원)  │                   │
│  │     └── GetRobotState 폴링 (0.5초)     │                   │
│  └────────────────┬────────────────────────┘                   │
│                   │ ROS2 서비스/토픽                             │
│                   ▼                                             │
│  ┌─────────────────────────────────────────┐                   │
│  │ Doosan 드라이버 (dsr_bringup2)          │                   │
│  │ └── 로봇 컨트롤러 (M0609)              │                   │
│  └─────────────────────────────────────────┘                   │
└────────────────────────────────────────────────────────────────┘
```

### 모듈 의존 관계

```
task_manager_node / run_single_task / robot_backend
  │
  ├── ToolManager ──────────┐
  ├── ObjectManager         │
  ├── DoughTask             ├──→ IOManager ──→ ROS2 서비스 (DSR_ROBOT2)
  ├── PressTask             │       ├──→ 그리퍼 프리셋 (디지털 출력)
  ├── FlipTask              │       ├──→ Tool/TCP 자동 검증
  ├── SourceTask            │       └──→ 그리퍼 복원 (_last_gripper_preset)
  └── PowderTask ───────────┘
                                └──→ pose_manager (좌표 딕셔너리)
```

---

## 핵심 모듈 상세 설명

### helpers/io_manager.py — 로봇 I/O 통합 래퍼

모든 로봇 제어를 단일 인터페이스로 추상화합니다.

```python
class IOManager:
    # ROS2 서비스 클라이언트
    # - SetCtrlBoxDigitalOutput: 그리퍼 핀 제어
    # - GetCtrlBoxDigitalInput: 입력 신호 읽기
    # - CheckForceCondition: 힘/토크 센서 조회

    # 그리퍼 제어 (6종 프리셋)
    def gripper_open(size='90mm')     # release_65mm / release_90mm
    def gripper_close(size='20mm')    # grip_20mm / grip_12mm / grip_40mm / grip_50mm
    def restore_gripper()             # 안전정지/비상정지 후 마지막 프리셋 재적용

    # 로봇 모션
    def move_joint_safe(pos, vel, acc, ...)    # 관절 공간 이동 (movej 래퍼)
    def move_line_safe(pos, vel, acc, ...)     # 카테시안 직선 이동 (movel 래퍼)
    def move_circle_safe(via, to, vel, ...)    # 원호 이동 (movec 래퍼)
    def move_spline_safe(pos_list, vel, ...)   # 스플라인 곡선 이동 (movesx 래퍼)

    # Tool/TCP 검증 (세션당 1회, 모드 전환 시 자동 재설정)
    def _ensure_tool_tcp()             # 매 모션 전 자동 호출
    def invalidate_tool_tcp()          # 안전정지/비상정지 복구 후 리셋

    # 초기화
    def set_tool_tcp()                 # Tool Weight + GripperDA_v1 설정
    def set_robot_mode_autonomous()    # MANUAL → set_tool/tcp → AUTONOMOUS 전환
```

**그리퍼 프리셋** — 디지털 출력 핀 3, 2, 1의 ON/OFF 조합:

| 프리셋 | 크기 | 핀 (3,2,1) | 용도 |
|--------|------|-----------|------|
| `grip_20mm` | 20mm | OFF,OFF,ON | 프레스 pick, 소스 분출 |
| `grip_12mm` | 12mm | OFF,ON,ON | 접시 pick, 뒤집개 pick |
| `grip_40mm` | 40mm | ON,OFF,ON | 소스병 pick |
| `grip_50mm` | 50mm | ON,ON,OFF | 반죽 그립 |
| `release_65mm` | 65mm | OFF,ON,OFF | 도구 반납, 가루통 pick |
| `release_90mm` | 90mm | ON,OFF,OFF | 초기화, 소스병/가루통 반납 |

### helpers/pose_manager.py — 좌표 중앙 관리

모든 카테시안/관절 좌표를 도구별·작업별 딕셔너리로 통합 관리합니다.

```python
ROBOT_CONFIG = {
    'robot_id': 'dsr01', 'robot_model': 'm0609',
    'robot_tool': 'Tool Weight', 'robot_tcp': 'GripperDA_v1'
}

JOINT_POSES = {
    'ready':             [0, 0, 90, 0, 90, 0],        # 홈 포지션
    'source_bottle1':    [-15, 41, 59, 86, -70, 9],    # 소스병 접근
    'source_bottle2':    [-15, 46, 68, 81, -71, 24],    # 소스병 접근 2
    'powder_bottle_lift':[-19, 21, 85, -96, 64, 18],   # 가루통 들기
    'powder_bottle_pick':[-19, 29, 96, -104, 68, 38],  # 가루통 집기
}

def get_poses() -> dict:
    """6개 카테고리의 posx() 좌표 반환 (dough, press, plate, spatula, source, powder)"""
```

| 카테고리 | 좌표 수 | 용도 |
|---------|--------|------|
| `dough` | 6개 | 반죽 pick/place, 집게 접근 |
| `press` | 7개 | 프레스 도구 접근, 반죽 위/아래, 도구 반납 |
| `plate` | 6개 | 접시 스택 접근(원호), 서빙 위치 |
| `spatula` | 17개 | 뒤집개 접근, 하강 3단계, 그릴, 스윙 3단계, 스윕 4단계, 접시 이동, 반납 경로 |
| `source` | 4개 | 소스병 위치, 접시 위 분출 자세 |
| `powder` | 4개 | 가루통 들기, 접시 위 뒤집기 자세 |

---

### managers/tool_manager.py — 도구 자동 교환

5가지 도구의 pick/return 모션 시퀀스를 구현합니다.

```python
class ToolManager:
    def pick_tool(tool_name: str) -> bool    # 도구 집기 (충돌 방지: 이미 들고 있으면 거부)
    def return_tool(tool_name: str) -> bool  # 도구 반납 (불일치 감지)
```

| 도구 | pick 전략 | return 전략 |
|------|----------|------------|
| `tongs` (집게) | 90mm open → ready → 접근 → 65mm open | 역순 이동 → 90mm open |
| `presser` (프레스) | 65mm open → 접근 → 20mm grip | 접근 → 65mm open |
| `spatula` (뒤집개) | 65mm open → 접근 → 12mm grip | **힘 감지 반복 알고리즘** (7N 임계값, x축 -4mm씩 조정, 최대 10회) |
| `source_bottle` (소스병) | 90mm open → 관절 이동 → 40mm grip | plate → bottle → 90mm open → ready |
| `powder_bottle` (가루통) | 90mm open → 관절 이동 → 65mm open (거치대에서 들기) | bottle 위치 → 90mm open → ready |

**뒤집개 반납 — 힘 감지 반복 알고리즘:**
```
for attempt in range(10):
    1. 현재 x_adjust 적용된 위치로 이동
    2. 기준 힘(base_fz) 측정
    3. 비동기 하강(amovel) 시작
    4. 하강 중 힘 모니터링:
       - 힘 증가량 > 7N → 장애물 감지 → 위로 복귀 → x_adjust -= 4mm
       - 바닥 도달(힘 증가 없음) → 릴리스 → 성공
```

---

### managers/task_manager_node.py — 워크플로우 오케스트레이션

6단계 공정을 순차 실행하는 ROS2 노드입니다.

```python
class TaskManagerNode(Node):
    # ROS2 인터페이스
    # - 토픽 퍼블리셔: /dsr01/workflow_status (String)
    # - 서비스 서버: /dsr01/start_workflow (std_srvs/Trigger)

    # 파라미터
    # - source: 'none' | 'source1' | 'source2'
    # - powder: 'none' | 'powder1' | 'powder2'

    # 상태 머신: IDLE → RUNNING → COMPLETED | ERROR
```

### managers/run_single_task.py — 개별 태스크 실행기

```python
TASK_MAP = {
    'dough':   ('tongs',         '집게로 반죽 배치'),
    'press':   ('presser',       '프레스로 반죽 누르기'),
    'plate':   (None,            '접시 배치'),           # 도구 불필요
    'spatula': ('spatula',       '뒤집개로 뒤집기'),
    'source':  ('source_bottle', '소스 뿌리기'),
    'powder':  ('powder_bottle', '가루 뿌리기'),
}
# 실행: ros2 run cobot1 run_task --ros-args -p task:=press
```

---

### tasks/ — 6종 태스크 코어 로직

모든 태스크는 동일한 패턴을 따릅니다:
```python
class XxxTask:
    def __init__(self, node: Node, io: IOManager = None)
    def _get_io(self) -> IOManager     # lazy 초기화
    def 핵심_메서드(self) -> bool       # 태스크 수행, 성공 시 True
```

| 태스크 | 메서드 | 핵심 동작 | 사용 API |
|--------|--------|----------|---------|
| `DoughTask` | `place_dough_with_tongs()` | 반죽 위치 이동 → grip_50mm → 그릴 배치 → release_65mm | `movel` |
| `PressTask` | `press_dough()` | 컴플라이언스 모드 → -150N 힘 제어 하강 → 도구 털기 | `task_compliance_ctrl`, `set_desired_force`, `move_periodic` |
| `FlipTask` | `flip_item_with_spatula()` | 컴플라이언스+20N → 3단계 하강 → 스윙(time=0.6) → 스윕(movel+movesx 스플라인) → 접시 이동 | `task_compliance_ctrl`, `set_desired_force`, `movel(time=)`, `movesx` |
| `PlateSettingTask` | `pick_and_place_plate()` | 원호 이동으로 접시 pick → grip_12mm → 서빙 위치 place | `movec` |
| `SourceTask` | `dispense_source()` | 접시 위 이동 → grip_20mm(분출) → 나선 패턴 5회전 | `move_spiral` |
| `PowderTask` | `sprinkle_powder()` | 접시 위 이동 → B축 뒤집기 → move_periodic 진동 5회 (Y/Z/Rx축) | `move_periodic` |

---

### backend/robot_backend.py — Firebase + ROS2 통합 서버

#### 듀얼 노드 구조

```
┌──────────────────────────┐  ┌──────────────────────────────┐
│ 태스크 노드               │  │ 제어 노드                     │
│ (robot_backend)           │  │ (robot_control)               │
│                           │  │ [전용 spin 스레드]             │
│ - IOManager               │  │                               │
│ - ToolManager             │  │ - MovePause / MoveResume      │
│ - 6종 Task 모듈           │  │ - MoveStop / DrlStop          │
│ - 태스크 스레드 실행       │  │ - SetRobotMode                │
│                           │  │ - SetRobotControl             │
│                           │  │ - SetCtrlBoxDigitalOutput     │
│                           │  │ - GetRobotState (0.5초 폴링)  │
└──────────────────────────┘  └──────────────────────────────┘
```

> 두 노드를 분리한 이유: 같은 노드에서 `spin_until_future_complete`를 중첩 호출하면 executor 충돌이 발생하므로, 태스크 실행과 제어 명령을 독립 노드로 분리합니다.
> 제어 노드는 `SingleThreadedExecutor`로 전용 스레드에서 spin하며, 서비스 호출은 `call_async` + 폴링 대기 방식을 사용합니다.

#### 스레딩 모델

- **메인 스레드**: Firebase 명령 큐 폴링 (0.1초 간격) + 제어 명령 처리
- **태스크 스레드**: 시작 요청 시 데몬 스레드로 6단계 워크플로우 실행
- **제어 노드 spin 스레드**: `SingleThreadedExecutor`로 제어 노드 전용 spin
- **상태 폴링 스레드**: `GetRobotState` 서비스 0.5초 주기 호출 → 안전정지/비상정지 감지
- **일시정지 메커니즘**: `threading.Event` 기반 (`pause_event`, `collide_event`, `safety_stop_event`, `emergency_stop_event`)

#### Firebase 데이터 구조

```
Firebase Realtime DB
├── robot_status/
│   ├── is_running, is_paused, is_collided      (bool)
│   ├── is_safety_stopped, is_emergency_stopped  (bool)
│   ├── status_text         (string)  ← 현재 단계 표시
│   ├── selected_source     (string)
│   ├── selected_powder     (string)
│   └── last_update_timestamp(float)
│
└── robot_commands/
    ├── start_requests/     (push queue)
    │   └── {id}: { source, powder, requested_at }
    └── control_requests/   (push queue)
        └── {id}: { command, requested_at }
```

#### 안전 시스템

**GetRobotState 서비스 폴링 기반 자동 감지 (0.5초 주기):**

| 상태 코드 | 상수 | LED | 감지 동작 |
|----------|------|-----|----------|
| 5, 10 | SAFE_STOP / SAFE_STOP2 | 노란색 | `safety_stop_event` + `collide_event` 설정, 그리퍼 즉시 복원 |
| 6, 7 | EMERGENCY_STOP | 빨간색 | `emergency_stop_event` 설정, 그리퍼 즉시 복원 |
| 3, 11 | SAFE_OFF / SAFE_OFF2 | 빨간색 | `safety_stop_event` 설정, 그리퍼 즉시 복원 |
| 1 | STANDBY | 흰색 | 모든 이벤트 자동 클리어 |

**안전정지(노란불) 해제 — 4단계 절차:**
```
[1/4] DrlStop(QSTOP_STO)         # 드라이버 블로킹된 모션 호출 해제
  ↓ 3초 대기
[2/4] SetRobotControl(2)          # CONTROL_RESET_SAFE_STOP → STANDBY 전이
  ↓ 3초 대기
[3/4] _reinitialize_robot()       # MANUAL → set_tool/tcp → AUTONOMOUS + 그리퍼 복원
  ↓
[4/4] MoveResume()                # 모션 재개
```

**비상정지(빨간불) 해제 — 4단계 절차:**
```
[사람] 비상정지 버튼 시계 방향으로 돌려 물리 해제
  ↓ 상태: 6(E-STOP) → 3(SAFE_OFF)
[1/4] DrlStop(QSTOP_STO)         # 드라이버 블로킹 해제
  ↓ 1초 대기
[2/4] SetRobotControl(3)          # 서보 ON (SAFE_OFF → STANDBY)
  ↓ 3초 대기 (브레이크 해제)
[3/4] _reinitialize_robot()       # Tool/TCP 재설정 + AUTONOMOUS 모드 + 그리퍼 복원
  ↓
[4/4] MoveResume()                # 모션 재개
```

**충돌(보호정지) 해제 — resume_collision 명령:**
```
실제 충돌(state 5/10) 감지 시:
  [1/4] DrlStop → [2/4] SetRobotControl(2) → [3/4] 재초기화 → [4/4] MoveResume
시뮬레이션 충돌 시:
  MoveResume만 호출
```

**그리퍼 복원 메커니즘:**
- `IOManager._last_gripper_preset`: 마지막 적용된 그리퍼 프리셋을 클래스 레벨에서 추적
- 안전정지/비상정지 감지 즉시 `_restore_gripper()` 호출 (디지털 출력 리셋 방지)
- 복원은 제어 노드의 디지털 출력 클라이언트 경유 (executor 충돌 방지)

---

### backend/index.html — 관리자 대시보드

실시간 로봇 상태 모니터링 및 제어 인터페이스.

| 버튼 | Firebase 명령 | ROS2 동작 |
|------|-------------|------------|
| 시작 | `start_requests` push | 태스크 스레드 시작 |
| 일시 정지 | `pause` | `move_pause` |
| 재개 | `resume` | `move_resume` |
| 안전정지 해제 | `release_safety_stop` | 4단계 절차 (DrlStop → Reset → Reinit → Resume) |
| 비상정지 해제 | `release_emergency_stop` | 4단계 절차 (DrlStop → ServoOn → Reinit → Resume) |
| 충돌 해제 및 재개 | `resume_collision` | 충돌 유형별 복구 |

**UI 요소:**
- 소스 선택: 선택없음 / 블루베리맛 소스
- 가루 선택: 선택없음 / 팝핑 캔디 가루
- 안전정지(노란불) / 비상정지(빨간불) 인디케이터 라이트
- 충돌/안전정지/비상정지 팝업 모달

### backend/user_interface.html — 고객 키오스크 UI

팬케이크 요정 테마의 고객 대면 인터페이스.

- **홈 화면**: 팬케이크 요정 일러스트 + 주문 방식 선택 (먹고가기/포장하기)
- **토핑 선택**: 시럽(블루베리맛 소스) + 마법 가루(팝핑 캔디 가루) 라디오 버튼
- **진행 화면**: 실시간 공정 애니메이션 (반죽 → 프레스 → 뒤집기 → 접시 이동 → 소스/가루)
- **완성 모달**: 30초 카운트다운 후 자동 홈 복귀
- **TTS 음성 안내**: Web Speech API 기반, 단계별 한국어 안내 음성 재생
- **안전 이벤트 연동**: 충돌/안전정지/비상정지 감지 시 에러 모달 표시 + 애니메이션 정지
- Firebase `status_text` 동기화로 단계별 애니메이션 자동 전환
- 관리자 페이지 링크 버튼 (index.html)

---

## 워크플로우 상세

```
┌───────────────────────────────────────────────────────────────────┐
│ 1/6  반죽 배치                                                    │
│      ToolManager.pick_tool('tongs')                              │
│      → DoughTask.place_dough_with_tongs()                        │
│      → ToolManager.return_tool('tongs')                          │
│      사용 API: movej, movel, gripper (50mm grip, 65mm release)   │
├───────────────────────────────────────────────────────────────────┤
│ 2/6  프레스 누르기                                                │
│      ToolManager.pick_tool('presser')                            │
│      → PressTask.press_dough()                                   │
│      → ToolManager.return_tool('presser')                        │
│      사용 API: task_compliance_ctrl, set_desired_force(-150N),    │
│              move_periodic (도구 털기)                            │
├───────────────────────────────────────────────────────────────────┤
│ 3/6  접시 세팅                                                    │
│      ObjectManager.pick_and_place_plate()                        │
│      사용 API: movec (원호 이동), movel                           │
│      도구 불필요 — 그리퍼로 직접 접시 pick & place                 │
├───────────────────────────────────────────────────────────────────┤
│ 4/6  뒤집기                                                      │
│      ToolManager.pick_tool('spatula')                            │
│      → FlipTask.flip_item_with_spatula()                         │
│      → ToolManager.return_tool('spatula')                        │
│      사용 API: task_compliance_ctrl, set_desired_force(20N),     │
│              movel(time=0.6) 스윙, movesx 스플라인 스윕,          │
│              힘 감지 반납                                         │
├───────────────────────────────────────────────────────────────────┤
│ 5/6  소스 뿌리기 (선택)                                           │
│      ToolManager.pick_tool('source_bottle')                      │
│      → SourceTask.dispense_source()                              │
│      → ToolManager.return_tool('source_bottle')                  │
│      사용 API: move_spiral (5회전, 30mm 반경), grip_20mm 분출     │
├───────────────────────────────────────────────────────────────────┤
│ 6/6  가루 뿌리기 (선택)                                           │
│      ToolManager.pick_tool('powder_bottle')                      │
│      → PowderTask.sprinkle_powder()                              │
│      → ToolManager.return_tool('powder_bottle')                  │
│      사용 API: move_periodic (Y/Z/Rx축 진동), B축 회전 뒤집기    │
└───────────────────────────────────────────────────────────────────┘
```

---

## 빌드 및 실행

### 빌드

```bash
cd ~/cobot_ws
colcon build --packages-select cobot1
source install/setup.bash
```

### 로봇 드라이버 실행 (항상 먼저)

```bash
# 시뮬레이션 (가상 로봇 + RViz)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609

# 실제 로봇
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```

### 전체 공정 실행

```bash
# 방법 1: ROS2 노드 직접 실행
ros2 run cobot1 task_manager --ros-args -p source:=source1 -p powder:=powder1

# 방법 2: 웹 UI + Firebase 백엔드
cd ~/cobot_ws/backend
python3 robot_backend.py
# → 브라우저에서 index.html 또는 user_interface.html 열기
```

### 개별 태스크 테스트

```bash
ros2 run cobot1 run_task --ros-args -p task:=dough    # 반죽 집기
ros2 run cobot1 run_task --ros-args -p task:=press    # 프레스
ros2 run cobot1 run_task --ros-args -p task:=plate    # 접시 세팅
ros2 run cobot1 run_task --ros-args -p task:=spatula  # 뒤집개
ros2 run cobot1 run_task --ros-args -p task:=source   # 소스
ros2 run cobot1 run_task --ros-args -p task:=powder   # 가루
```

---

## 사용된 Doosan 로봇 API

| API | 용도 | 사용 태스크 |
|-----|------|-----------|
| `movej()` | 관절 공간 이동 | 홈 복귀, 도구 접근 |
| `movel()` | 카테시안 직선 이동 | 모든 태스크 |
| `movec()` | 원호(호) 이동 | 접시 세팅 |
| `movesx()` | 스플라인 곡선 이동 (다중 웨이포인트) | 뒤집기 스윕 |
| `amovel()` | 비동기 직선 이동 | 뒤집개 반납 (힘 감지 중) |
| `move_spiral()` | 나선 패턴 이동 | 소스 뿌리기 |
| `move_periodic()` | 주기적 진동 이동 | 프레스 도구 털기, 가루 스냅 |
| `task_compliance_ctrl()` | 컴플라이언스 모드 활성화 | 프레스, 뒤집기 |
| `set_desired_force()` | 힘 제어 목표 설정 | 프레스(-150N), 뒤집기(20N) |
| `release_force()` / `release_compliance_ctrl()` | 힘/컴플라이언스 해제 | 프레스, 뒤집기 |
| `get_tool_force()` | 현재 힘 센서 값 조회 | 뒤집개 반납 |
| `check_motion()` | 모션 진행 상태 확인 | 뒤집개 반납 |
| `get_tool()` / `get_tcp()` | 현재 Tool/TCP 설정 조회 | IOManager 검증 |
| `set_tool()` / `set_tcp()` | Tool/TCP 설정 | 초기화, 재설정 |
| `set_robot_mode()` | 로봇 모드 전환 | 초기화 (MANUAL→AUTONOMOUS) |
| `set_digital_output()` | 그리퍼 핀 제어 | 모든 그리퍼 동작 |

---

## 좌표 수정

모든 좌표는 `src/cobot1/cobot1/helpers/pose_manager.py` 한 곳에서 관리합니다.
좌표 변경 시 이 파일만 수정하면 모든 태스크/매니저에 자동 반영됩니다.

```python
# 예: 반죽 위치 변경
'dough': {
    'dough_start_2': posx([461, 241, 6, 36, -178, 36]),  # ← 이 값만 수정
}
```

수정 후:
```bash
colcon build --packages-select cobot1 && source install/setup.bash
```
