# Robot Arm Server (로봇팔 제어)

메인 서버(Domain 59)가 두 대의 로봇팔(Picking: Domain 60, Packing: Domain 61)을 제어하기 위한 모듈입니다.

## 📂 파일 구조
- **[arm_handler.py](file:///home/addinedu/Desktop/roscamp-repo-3/server/mainserver/robotarm_server/arm_handler.py)**: 메인 서버에서 로봇팔과 통신하는 ROS 2 노드 클래스.
- **[config/bridge_robotarm.yaml](file:///home/addinedu/Desktop/roscamp-repo-3/server/mainserver/config/bridge_robotarm.yaml)**: 도메인 브릿지 설정 파일.

## 📡 통신 구조 (Domain Bridge)

### 1. 명령 (Command) - Downstream
- **메인 서버 (59) -> 로봇팔 (60, 61)**
- 메인 서버는 로봇별로 구분된 토픽을 발행하고, 브릿지가 이를 로봇의 공통 토픽으로 변환하여 전달합니다.

| Main Server Topic | Bridge Action | Robot Topic | Target Robot |
| :--- | :---: | :--- | :--- |
| `/robot1/order_command` | -> | `/order_command` | **Picking Robot (60)** |
| `/robot2/order_command` | -> | `/order_command` | **Packing Robot (61)** |

*   **Type**: `std_msgs/msg/Float64`
*   **Data Example**: `1.4507` (Zone 1, Frame, Legs, Kit)

### 2. 상태 (State) - Upstream
- **로봇팔 (60, 61) -> 메인 서버 (59)**
- 각 로봇이 자신의 상태를 `/robot_state`로 발행하면, 브릿지가 메인 서버의 구분된 토픽으로 전달합니다.

| Robot Topic | Source Robot | Bridge Action | Main Server Topic |
| :--- | :--- | :---: | :--- |
| `/robot_state` | **Picking (60)** | -> | `/robot1/robot_state` |
| `/robot_state` | **Packing (61)** | -> | `/robot2/robot_state` |

*   **Type**: `std_msgs/msg/Int8`
*   **Values**:
    *   `0`: Unknown
    *   `1`: IDLE (대기)
    *   `2`: BUSY (작업 중)
    *   `3`: SUCCESS (완료)

## 💻 사용법 (Main Server)

`app.py` 또는 다른 로직에서 `ArmHandler`를 import하여 사용합니다.

```python
from robotarm_server.arm_handler import ArmHandler
import rclpy

# ROS 2 초기화 (앱 시작 시 1회)
rclpy.init()

# 핸들러 생성
arm_node = ArmHandler()

# 명령 내리기
arm_node.send_picking_command(1.4507) # 피킹 로봇에게 명령
arm_node.send_packing_command(0.0000) # 패킹 로봇에게 명령

# 상태 확인
status1 = arm_node.get_picking_status()
status2 = arm_node.get_packing_status()
```
