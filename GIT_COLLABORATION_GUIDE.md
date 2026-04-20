# ROKEY7_FIRST_PROJECT 협업 가이드

이 문서는 **처음 Git을 사용하는 개발자**를 위한 프로젝트 협업 가이드입니다.
아래 순서대로 따라하면 프로젝트를 내 PC에 가져와서 함께 개발할 수 있습니다.

---

## 0. 사전 준비

### 0-1. Git 설치 확인

터미널을 열고 아래 명령어를 입력합니다.

```bash
git --version
```

`git version 2.x.x` 같은 결과가 나오면 이미 설치된 것입니다.
설치가 안 되어 있다면:

```bash
sudo apt update
sudo apt install git -y
```

### 0-2. GitHub 계정 만들기

1. https://github.com 에 접속합니다.
2. `Sign up` 버튼을 눌러 계정을 만듭니다.
3. 이메일 인증을 완료합니다.

### 0-3. Git 사용자 정보 설정

**최초 1회만** 설정하면 됩니다. 본인의 이름과 이메일을 넣으세요.

```bash
git config --global user.name "홍길동"
git config --global user.email "honggildong@example.com"
```

설정 확인:

```bash
git config --global --list
```

---

## 1. 프로젝트를 내 PC로 가져오기

대부분의 개발자가 이미 `~/cobot_ws` 폴더에 `build/`, `install/`, `log/`, `src/doosan-robot2/` 등을 가지고 있을 것입니다.
이 경우 **기존 워크스페이스를 유지하면서** Git 저장소를 연동하는 방법을 사용합니다.

### 1-1. (이미 cobot_ws가 있는 경우) 기존 워크스페이스에 Git 연동

```bash
cd ~/cobot_ws

# 1) 기존 src/cobot1 백업 (혹시 모르니까)
cp -r src/cobot1 src/cobot1_backup  2>/dev/null

# 2) Git 초기화 및 원격 저장소 연결
git init
git remote add origin https://github.com/jinw00ch01/ROKEY7_FIRST_PROJECT.git

# 3) 원격 저장소의 코드 가져오기
git fetch origin

# 4) 원격의 main 브랜치를 로컬에 적용
git checkout -b main origin/main
```

> **걱정 마세요:** `build/`, `install/`, `log/` 폴더는 `.gitignore`에 등록되어 있어서 Git이 건드리지 않습니다. 기존 빌드 결과물은 그대로 유지됩니다.

만약 충돌 메시지가 나오면:

```bash
# 기존 파일과 충돌 시, 원격 저장소 코드를 우선 적용
git checkout -f main
```

### 1-2. (cobot_ws가 없는 경우) 새로 clone

```bash
cd ~
git clone https://github.com/jinw00ch01/ROKEY7_FIRST_PROJECT.git cobot_ws
cd ~/cobot_ws
```

### 1-3. 연동 확인

```bash
cd ~/cobot_ws
git status
ls
```

`src/`, `backend/`, `README.md` 등의 파일이 보이고, `git status`에서 `On branch main`이 표시되면 성공입니다.

---

## 2. 프로젝트 빌드 및 실행 환경 준비

### 2-1. ROS2 Humble 환경 소싱

```bash
source /opt/ros/humble/setup.bash
```

> **팁:** 매번 입력하기 귀찮다면 `~/.bashrc` 파일 맨 아래에 추가하세요.
> ```bash
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
> ```

### 2-2. 의존 패키지 설치 (doosan-robot2)

이 프로젝트는 두산 로봇 ROS2 패키지(`doosan-robot2`)가 필요합니다.
`src/doosan-robot2` 디렉토리가 이미 포함되어 있으므로, 전체 빌드를 진행합니다.

```bash
cd ~/cobot_ws
colcon build --symlink-install
```

빌드 완료 후 환경을 소싱합니다:

```bash
source install/setup.bash
```

### 2-3. Python 패키지 설치

백엔드(Firebase 연동)를 사용하려면 추가 패키지가 필요합니다.

```bash
pip install firebase-admin
```

### 2-4. Firebase 인증 키 설정

보안상 Firebase 인증 키 파일(`.json`)은 Git에 포함되어 있지 않습니다.
팀장에게 인증 키 파일을 받아서 `backend/` 폴더에 넣어야 합니다.

```bash
# 팀장에게 받은 파일을 backend/ 폴더에 복사
cp ~/Downloads/rokey-fe6a9-firebase-adminsdk-xxxxx.json ~/cobot_ws/backend/
```

> **주의:** 이 파일은 절대로 GitHub에 올리면 안 됩니다. `.gitignore`에 이미 등록되어 있으므로 정상적으로는 커밋되지 않습니다.

---

## 3. 코드 수정 및 커밋하기

### 3-1. 브랜치 만들기 (중요!)

**main 브랜치에 바로 작업하지 마세요!** 반드시 새 브랜치를 만들어서 작업합니다.

```bash
# 새 브랜치 만들고 이동
git checkout -b feature/내이름-작업내용
```

예시:

```bash
git checkout -b feature/gildong-press-fix
```

### 3-2. 코드 수정

에디터(VS Code 등)에서 파일을 수정합니다.

### 3-3. 변경 사항 확인

```bash
# 어떤 파일이 변경되었는지 확인
git status

# 변경 내용 자세히 보기
git diff
```

### 3-4. 변경 파일 스테이징 (add)

```bash
# 특정 파일만 추가
git add src/cobot1/cobot1/press_test.py

# 또는 변경된 파일 전부 추가 (주의: 불필요한 파일이 포함될 수 있음)
git add .
```

> **주의:** `git add .`을 할 때는 `git status`로 먼저 확인하세요. `.json` 인증 키 같은 민감한 파일이 포함되면 안 됩니다.

### 3-5. 커밋하기

```bash
git commit -m "press_test: 힘 제어 해제 순서 수정"
```

커밋 메시지는 **무엇을 왜 바꿨는지** 간단히 적어주세요.

---

## 4. GitHub에 올리기 (push)

### 4-1. 처음 push 할 때

```bash
git push -u origin feature/gildong-press-fix
```

> GitHub 로그인을 요청하면 GitHub 아이디와 **Personal Access Token**을 입력합니다.
> (비밀번호가 아닙니다! 아래 4-2 참고)

### 4-2. Personal Access Token 만들기

GitHub는 비밀번호 대신 토큰을 사용합니다.

1. GitHub 접속 → 오른쪽 위 프로필 클릭 → `Settings`
2. 왼쪽 메뉴 맨 아래 `Developer settings` 클릭
3. `Personal access tokens` → `Tokens (classic)` → `Generate new token (classic)`
4. Note: `cobot_ws` (아무 이름)
5. Expiration: 원하는 기간 선택
6. 체크 항목: `repo` (전체 체크)
7. `Generate token` 클릭
8. **표시되는 토큰을 반드시 복사해서 메모장에 저장!** (다시 볼 수 없음)

push 시 비밀번호 대신 이 토큰을 입력하면 됩니다.

### 4-3. 두 번째 push부터

```bash
git push
```

---

## 5. Pull Request (PR) 보내기

코드를 main에 합치려면 **Pull Request**를 만들어야 합니다.

1. https://github.com/jinw00ch01/ROKEY7_FIRST_PROJECT 접속
2. 상단에 `Compare & pull request` 버튼이 보이면 클릭
   - 안 보이면: `Pull requests` 탭 → `New pull request` 클릭
3. base: `main` ← compare: `feature/gildong-press-fix` 로 설정
4. 제목과 설명을 작성합니다
   - 제목 예: `press_test: 힘 제어 해제 순서 수정`
   - 설명 예: `movel 이후 release_force를 먼저 호출하도록 변경`
5. `Create pull request` 클릭
6. 팀장이 코드를 확인하고 `Merge` 합니다

---

## 6. 최신 코드 받기 (pull)

다른 사람이 main에 코드를 합친 후, 내 PC에도 최신 코드를 받아야 합니다.

```bash
# main 브랜치로 이동
git checkout main

# 최신 코드 받기
git pull origin main

# 내 작업 브랜치로 돌아가서 main 변경사항 반영
git checkout feature/gildong-press-fix
git merge main
```

---

## 7. 자주 발생하는 문제 해결

### Q1. `git push` 할 때 권한 오류가 나요

```
remote: Permission to jinw00ch01/ROKEY7_FIRST_PROJECT.git denied
```

→ 저장소 소유자(jinw00ch01)가 **Collaborator로 초대**해야 합니다.
- 소유자: GitHub 저장소 → `Settings` → `Collaborators` → `Add people` → 상대방 GitHub 아이디 입력

### Q2. `git pull` 할 때 충돌(conflict)이 발생해요

```
CONFLICT (content): Merge conflict in src/cobot1/cobot1/press_test.py
```

→ 해당 파일을 열면 아래처럼 표시됩니다:

```
<<<<<<< HEAD
내가 수정한 코드
=======
다른 사람이 수정한 코드
>>>>>>> main
```

→ 둘 중 하나를 선택하거나 합쳐서 수정한 뒤, `<<<<<<<`, `=======`, `>>>>>>>` 줄을 삭제합니다.
→ 수정 후:

```bash
git add 충돌파일명
git commit -m "충돌 해결: press_test.py"
```

### Q3. 실수로 main에 직접 커밋했어요

```bash
# 커밋을 취소하고 변경사항은 유지
git reset --soft HEAD~1

# 새 브랜치를 만들어서 이동
git checkout -b feature/내이름-작업내용

# 다시 커밋
git add .
git commit -m "커밋 메시지"
```

### Q4. `colcon build` 가 실패해요

```bash
# 빌드 캐시 삭제 후 재빌드
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

---

## 8. 프로젝트 실행 방법 요약

### 터미널 1: 로봇 드라이버 실행

```bash
# 가상 모드
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609

# 실제 로봇
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```

### 터미널 2: 백엔드 실행 (Firebase 연동)

```bash
cd ~/cobot_ws
colcon build --packages-select cobot1 --symlink-install
source install/setup.bash
cd backend
python3 robot_backend.py
```

### 웹 UI

`backend/index.html` 파일을 브라우저에서 더블클릭하여 열고, 시작 버튼을 클릭합니다.

### 개별 테스트 실행

```bash
cd ~/cobot_ws
source install/setup.bash
ros2 run cobot1 move_basic          # 기본 동작 테스트
ros2 run cobot1 press_test          # 프레스 테스트
ros2 run cobot1 plate_setting_test  # 접시 세팅 테스트
ros2 run cobot1 dough_grip_test     # 반죽 집기 테스트
ros2 run cobot1 spatula_test        # 뒤집개 테스트
ros2 run cobot1 main                # 전체 작업 실행
```

---

## 9. 프로젝트 구조

```
cobot_ws/
├── backend/                          # Firebase 백엔드
│   ├── robot_backend.py              # Firebase 감시 + 로봇 실행 서버
│   ├── index.html                    # 웹 모니터링 UI
│   ├── read_from_firebase.py         # Firebase 읽기 예제
│   ├── write_to_firebase.py          # Firebase 쓰기 예제
│   └── rokey-xxxxx.json              # Firebase 인증 키 (Git 미포함)
├── src/
│   ├── cobot1/                       # 로봇 동작 패키지
│   │   └── cobot1/
│   │       ├── main.py               # 전체 작업 통합 실행
│   │       ├── dough_grip_test.py    # 반죽 집기
│   │       ├── press_test.py         # 프레스 누르기
│   │       ├── plate_setting_test.py # 접시 세팅
│   │       ├── spatula_test.py       # 뒤집개
│   │       └── requirement/          # 요구사항 문서
│   └── doosan-robot2/                # 두산 로봇 ROS2 드라이버
├── .gitignore
└── README.md
```
