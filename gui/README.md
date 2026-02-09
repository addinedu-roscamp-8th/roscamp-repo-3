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
# 메인 애플리케이션 실행
ROS_DOMAIN_ID=70 python3 src/lovo_gui/lovo_gui/main.py

# 또는
cd src/lovo_gui && python3 -m lovo_gui.main_window
```

