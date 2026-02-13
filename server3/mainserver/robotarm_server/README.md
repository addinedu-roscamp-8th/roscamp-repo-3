# Robot Arm Server (로봇팔 제어)

메인 서버(Domain 59)가 두 대의 로봇팔(Picking: Domain 60, Packing: Domain 61)을 제어하기 위한 모듈입니다.

## 📂 파일 구조
- **[arm_handler.py](file:///home/addinedu/Desktop/roscamp-repo-3/server/mainserver/robotarm_server/arm_handler.py)**: 메인 서버에서 로봇팔과 통신하는 ROS 2 노드 클래스.
- **[config/bridge_robotarm.yaml](file:///home/addinedu/Desktop/roscamp-repo-3/server/mainserver/config/bridge_robotarm.yaml)**: 도메인 브릿지 설정 파일 (Domain 59 <-> 60, 61).

## 📡 통신 구조 (Domain Bridge)

### 1. 명령 (Command) - Downstream (59 -> 60/61)
메인 서버(59)에서 발행하는 명령 토픽들은 브릿지를 통해 각 로봇의 로컬 토픽으로 전달됩니다.

| Main Server Topic | Type | Robot Topic | Description |
| :--- | :--- | :--- | :--- |
| `/robotX/PTP_gripper_command` | `Int32` | `/PTP_gripper_command` | 그리퍼 제어 |
| `/robotX/PTP_servo_status` | `Bool` | `/PTP_servo_status` | 서보 On/Off |
| `/robotX/PTP_goal_pose` | `Float64MultiArray` | `/PTP_goal_pose` | 목표 좌표 이동 |
| `/robotX/order_command` | `Float64` | `/order_command` | 작업 명령 (Zone/Parts) |
| `/robotX/.../compressed` | `CaptureImage...` | `/PTP_capture...` | 이미지 캡처 명령 |

*(여기서 `robotX`는 `robot1`(Picking, Domain 60) 또는 `robot2`(Packing, Domain 61)입니다.)*

### 2. 상태 (State) - Upstream (60/61 -> 59)
각 로봇의 상태 토픽들은 브릿지를 통해 메인 서버의 구분된 토픽으로 전달됩니다.

| Robot Topic | Type | Main Server Topic | Description |
| :--- | :--- | :--- | :--- |
| `/joint_states` | `JointState` | `/robotX/joint_states` | 관절 각도 |
| `/PTP_tcp_pose` | `Float64MultiArray` | `/robotX/PTP_tcp_pose` | 현재 TCP 좌표 |
| `/robot_state` | `Int8` | `/robotX/robot_state` | 로봇 상태 (Idle, Busy 등) |

## 💻 사용법 (Main Server)

`app.py` 또는 다른 로직에서 `ArmHandler`를 import하여 사용합니다.

```python
from robotarm_server.arm_handler import ArmHandler
import rclpy

# ROS 2 초기화 (앱 시작 시 1회)
rclpy.init()

# 핸들러 생성
arm_node = ArmHandler()

# 예시: 로봇 1 (Picking) 제어
arm_node.r1_cmd_servo(True)           # 서보 켜기
arm_node.r1_cmd_gripper(1)            # 그리퍼 동작
arm_node.r1_cmd_order(1.4507)         # 작업 명령
arm_node.r1_cmd_goal([100.0, ...])    # 좌표 이동

# 예시: 로봇 2 (Packing) 제어
arm_node.r2_cmd_order(0.0000)         # 패킹 이동 명령
```

## 🧪 테스트 (Seed Data Script)
로봇팔 제어 기능을 검증하기 위한 테스트 스크립트가 포함되어 있습니다.

```bash
# 1. 로봇팔 서버 폴더로 이동
cd server/mainserver/robotarm_server

# 2. 테스트 스크립트 실행 (ROS 2 환경 로드 필요)
python3 test_arm_control.py
```

이 스크립트는 다음 동작을 수행합니다:
1.  **Robot 1 (Picking)**: 서보 ON -> 그리퍼 열기 -> 목표 좌표 이동 -> 작업 명령 -> 이미지 캡처 요청
2.  **Robot 2 (Packing)**: 서보 ON -> 패킹 구역 이동 명령
3.  **상태 모니터링**: 두 로봇의 상태(Status, TCP 좌표)를 실시간으로 출력

