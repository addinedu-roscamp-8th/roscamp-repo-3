# Lovo 로봇 제어 시스템

ROS2 기반 멀티 도메인 브리지를 활용한 로봇 제어 GUI 애플리케이션

## 📋 개요

이 프로젝트는 PyQt6 기반의 GUI를 통해 다수의 로봇팔(협작 로봇)과 자율주행 로봇(AMR)을 통합 관리하고 제어하는 시스템입니다.

**주요 로봇:**
- 상차 로봇팔 (robot1, domain: 61)
- 하차 로봇팔 (robot2, domain: 60)
- 운송 로봇 3대 (domain: 50, 51, 52)

## 🔧 요구사항

### 시스템 요구사항
- Ubuntu 20.04 이상 (ROS2 Foxy 이상 권장)
- Python 3.8+
- 4GB 이상 RAM

### 의존성 설치

```bash
# ROS2 설치 (https://docs.ros.org/en/humble/Installation.html 참고)
sudo apt-get install ros-humble-desktop

# Python 패키지 설치
sudo apt-get install python3-pip
pip3 install PyQt6

# ROS2 Python 패키지
pip3 install rclpy geometry-msgs std-msgs

# colcon 빌드 도구
sudo apt install python3-colcon-common-extensions
```

## 📁 폴더 구조

```
gui/
├── README.md                           # 이 파일
├── robotname.json                      # 로봇 설정 파일
├── src/
│   └── lovo_gui/
│       └── lovo_gui/
│           ├── config/                 # 설정 파일들
│           │   ├── robotname.json
│           │   ├── domain_bridge_config.yaml
│           │   └── pose_memory/        # 로봇팔 포즈 메모리
│           ├── controllers/            # ROS2 노드 (로봇 제어)
│           │   └── robot_controller.py
│           ├── tabs/                   # GUI 탭들
│           │   ├── main_tab.py
│           │   ├── manual_tab.py       # 매뉴얼 제어
│           │   ├── communication_tab.py
│           │   ├── monitoring_tab.py
│           │   ├── ros_monitor_tab.py  # ROS 토픽 모니터링
│           │   └── log_tab.py
│           ├── widgets/                # 커스텀 위젯들
│           │   ├── robot_dashboard.py  # 로봇팔 대시보드
│           │   ├── amr_dashboard.py    # AMR 대시보드
│           │   └── camera_dialog.py    # 카메라 화면
│           ├── utils/                  # 유틸리티
│           │   ├── config_manager.py   # 설정 관리
│           │   └── communication.py    # 통신 관리
│           ├── constants.py            # 상수 정의
│           └── main_window.py          # 메인 윈도우
├── build/                              # 빌드 결과물
└── install/                            # 설치된 패키지
```

## ⚙️ 설정

### 1. robotname.json 수정

```json
{
    "server_domain": 70,
    "robots": [
        {
            "name": "상차 로봇팔",
            "domain": 61,
            "id": "jecobot_126b",
            "ip": "192.168.0.61"
        },
        {
            "name": "하차 로봇팔",
            "domain": 60,
            "id": "jecobot_aab4",
            "ip": "192.168.0.60"
        }
        // ... 추가 로봇들
    ]
}
```

### 2. domain_bridge_config.yaml 설정

도메인 간 ROS 토픽 매핑을 정의합니다.

```yaml
ros__parameters:
  ros_discovery_server: "127.0.0.1:11811"
  
  # Domain 60 (하차 로봇팔) <-> Domain 70 (서버)
  domain_60_70:
    topic_mapping: [
      "/robot2/target_angles",
      "/robot2/servo_status",
      ...
    ]
```

더 자세한 설정은 [config/domain_bridge_config.yaml](src/lovo_gui/lovo_gui/config/domain_bridge_config.yaml) 참고

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

## 📊 GUI 탭 설명

### Main 탭
- 전체 시스템 상태 모니터링
- 로봇 상태 요약

### Manual 탭
- 각 로봇별 수동 제어
- **로봇팔 대시보드:**
  - Servo ON/OFF, HOME 이동
  - 좌표 제어 (Cartesian Controller)
  - 각도 제어 (Joint Control)
  - 포즈 메모리 저장/이동 (P1~P5)
  - **현재 엔코더 각도 표시** (J1~J6)

### Monitoring 탭
- 실시간 상태 모니터링
- 안전 점검 현황

### Communication 탭
- ROS2 노드 간 통신 상태
- 메시지 송수신 현황

### **ROS Monitor 탭** ⭐
- 모든 로봇의 ROS 토픽 실시간 모니터링
- 토픽별 메시지 값 표시
- 로봇 연결 상태 표시 (● 초록=연결, 빨강=끊김)

**표시 토픽 (로봇팔):**
- `/robot1/current_angles` - 현재 관절 각도
- `/robot1/current_coords` - 현재 좌표
- `/robot1/current_pose` - 현재 포즈
- `/robot1/goal_pose` - 목표 포즈
- `/robot1/gripper_control` - 그리퍼 제어
- `/robot1/servo_status` - 서보 상태
- `/robot1/target_angles` - 목표 각도
- `/robot1/target_coords` - 목표 좌표

(robot2도 동일)

### Log 탭
- 전체 시스템 로그
- 에러 및 경고 메시지

## 🌐 ROS 토픽 정보

### 토픽 이름 규칙
- 상차 로봇팔: `/robot1/*`
- 하차 로봇팔: `/robot2/*`

### 주요 토픽

| 토픽 | 메시지 타입 | 설명 |
|------|-----------|------|
| `/robotX/current_angles` | Float64MultiArray | 현재 관절 각도 (6개) |
| `/robotX/current_coords` | Float64MultiArray | 현재 좌표 |
| `/robotX/current_pose` | Pose | 현재 포즈 |
| `/robotX/target_angles` | Float64MultiArray | 목표 관절 각도 |
| `/robotX/target_coords` | Float64MultiArray | 목표 좌표 |
| `/robotX/goal_pose` | Pose | 목표 포즈 |
| `/robotX/servo_status` | Bool | 서보 ON/OFF 상태 |
| `/robotX/gripper_control` | Int32 | 그리퍼 제어값 |

### 토픽 확인

```bash
# ROS 토픽 목록 확인
ros2 topic list

# 특정 토픽 정보 확인
ros2 topic info /robot2/current_angles

# 토픽 데이터 실시간 모니터링
ros2 topic echo /robot2/current_angles
```

## 💾 데이터 저장

### 포즈 메모리
로봇팔의 각 포즈는 CSV 파일로 저장됩니다:
- 위치: `config/pose_memory/`
- 파일명: `{로봇명}_로봇팔_pose_memory.csv`

### 자동 저장
- GUI 종료 시 자동으로 현재 포즈 메모리 저장
- Manual 탭에서 "저장" 버튼으로 수동 저장 가능

## 🔍 트러블슈팅

### Domain Bridge 연결 안 됨
```bash
# 방화벽 확인
sudo ufw status

# DDS 설정 확인
export ROS_DOMAIN_ID=70
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
```

### 토픽 메시지가 없음
```bash
# 로봇 노드 실행 확인
ros2 node list

# 토픽 활성화 확인
ros2 topic list
```

### GUI 실행 안 됨
```bash
# PyQt6 설치 확인
python3 -c "from PyQt6.QtWidgets import QApplication; print('OK')"

# ROS2 환경 확인
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 📝 주요 기능

✅ **멀티 로봇 관리**
- 최대 5개 로봇 동시 제어

✅ **좌표/각도 제어**
- Cartesian 좌표 제어
- 관절 각도 제어
- MoveIt 통합 지원

✅ **포즈 메모리**
- 5개 슬롯 저장/이동
- CSV 파일로 자동 저장

✅ **실시간 모니터링**
- ROS 토픽 값 표시
- 로봇 연결 상태 표시
- 메시지 흐름 모니터링

✅ **카메라 스트리밍**
- UDP 기반 카메라 피드 수신

✅ **안전 기능**
- 서보 ON/OFF
- 비상 정지
- 상태 표시

## 📞 지원

문제 발생 시:
1. Log 탭에서 오류 메시지 확인
2. `ros2 topic list`로 토픽 연결 확인
3. `ros2 node list`로 노드 실행 확인

## 📄 라이선스

MIT License

## 👨‍💻 개발자

Roscamp 프로젝트

---

**마지막 업데이트:** 2026년 2월 5일
