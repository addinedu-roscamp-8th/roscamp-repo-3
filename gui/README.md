## 🚀 실행 방법

### 1. 워크스페이스 빌드

```bash
cd /home/addinedu/roscamp-repo-3/gui

# 빌드
colcon build --packages-select lovo_gui lovo_interfaces lovo_main_server

# 빌드된 패키지 로드
source install/setup.bash
```

### 2. Domain Bridge 실행 (별도 터미널)

```bash
# Domain Bridge 노드 실행
ros2 run lovo_main_server domain_bridge_node
```

### 3. 로봇 컨트롤러 실행 (별도 터미널)

```bash
# 각 로봇의 ROS2 노드 실행
ROS_DOMAIN_ID=61 ros2 launch lovo_main_server robot_controller.launch.py
ROS_DOMAIN_ID=60 ros2 launch lovo_main_server robot_controller.launch.py
```

### 4. GUI 애플리케이션 실행

```bash
실행(간단)
-----------------
필수: ROS2 환경이 설정되고 빌드된 상태에서 아래 명령을 실행하세요.

1) 전체 빌드 및 환경 설정

```bash
cd /home/addinedu/roscamp-repo-3
colcon build --symlink-install
source install/setup.bash
```

2) GUI 실행

```bash
# ROS 환경이 설정된 상태에서
ros2 run lovo_gui lovo_gui
# 또는 (대체)
python3 -m lovo_gui.main_window
```

3) (옵션) AI 서버 실행

```bash
cd server/ai_server
python3 main.py
```

경로 변경: 정적 리소스 폴더 `systemData`는 `src/systemData/`로 이동되었습니다. 하드코딩된 경로를 사용 중이면 수정하세요.

문제가 발생하면 알려주시면 구체적으로 도와드리겠습니다.

