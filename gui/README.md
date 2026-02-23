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
ros2 launch lovo_gui domain_bridge.launch.py
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
python3 main_window.py
```

3) (옵션) AI 서버 실행

```bash
cd server/ai_server
python3 main.py
```

경로 변경: 정적 리소스 폴더 `systemData`는 `src/systemData/`로 이동되었습니다. 하드코딩된 경로를 사용 중이면 수정하세요.


문제가 발생하면 알려주시면 구체적으로 도와드리겠습니다.


## 🛠️ Main Server 연동 업데이트 (2026-02-10)

Main Server(FastAPI)와 GUI 간의 데이터 연동 기능이 추가되었습니다.

### 📝 주요 변경 파일 및 내용

#### 1. `src/lovo_gui/lovo_gui/core/api_client.py` (신규 파일)
*   **역할**: Main Server와 HTTP 통신을 담당하는 클래스
*   **기능**: `requests` 라이브러리를 사용하여 서버 API(`/api/orders`, `/api/robots` 등)를 호출하는 함수 구현

#### 2. `src/lovo_gui/package.xml`
*   **변경**: API 통신에 필요한 `python3-requests` 패키지 의존성 추가

#### 3. `src/lovo_gui/lovo_gui/tabs/tab05_communication/communication.py`
*   **변경**:
    *   `APIClient` 모듈 임포트 및 초기화
    *   서버 연결 확인(Connect 버튼) 시, 단순 `ping` 대신 API 헬스 체크(`api_client.check_health()`)를 수행하도록 변경

#### 4. `src/lovo_gui/lovo_gui/main_window.py`
*   **변경**: `MainTab` 생성 시 `communication_manager` 객체를 인자로 전달하도록 수정 (API 클라이언트 접근 권한 부여)

#### 5. `src/lovo_gui/lovo_gui/tabs/tab01_main/tab01_main.py`
*   **변경**:
    *   `QTimer`를 추가하여 2초마다 서버 데이터 갱신
    *   `_update_dashboard_data()`: 서버에서 주문 및 로봇 상태 정보를 가져와 화면(UI)에 반영하는 로직 구현
    *   **주문 로그** 및 **로봇 상태 그리드** 실시간 연동 기능 추가

## 🌐 GUI 원격 실행 가이드 (다른 컴퓨터에서 실행 시)

GUI를 메인 서버(`192.168.0.30`)가 아닌 다른 컴퓨터(예: `192.168.0.13`)에서 실행할 경우, 아래 설정을 반드시 확인해야 합니다.

### 1. 서버 IP 주소 설정
*   **파일**: `src/lovo_gui/lovo_gui/config/robotname.json`
*   **내용**: `server_ip` 값을 실제 메인 서버 주소인 `192.168.0.30`으로 변경합니다.
    ```json
    {
        "server_ip": "192.168.0.30",
        "server_port": 5000
    }
    ```

### 2. ROS 2 도메인 일치 (실시간 데이터 동기화)
로봇 및 시스템 토픽(기본 Domain 59)을 수신하기 위해 실행 전 터미널 환경 변수를 설정해야 합니다.
```bash
export ROS_DOMAIN_ID=59
# 그 후 GUI 실행
ros2 run lovo_gui lovo_gui
```

### 3. 네트워크 접근 권한
현재 메인 서버(`app.py`)는 모든 외부 IP의 접속(`allow_origins=["*"]`)을 허용하고 있으므로, 같은 네트워크(WIFI/LAN)에 있다면 즉시 통신이 가능합니다.

---

## 🛠️ 도메인 변경 가이드 (Domain 59 → 70 통일 시)

시스템 전체 무전 채널(Domain ID)을 59에서 70으로 변경하려면 **서버 PC**에서 다음 파일들을 수정해야 합니다.

### 2. GUI 설정 파일 수정
*   **파일**: `gui/src/lovo_gui/lovo_gui/config/robotname.json`
*   **수정 사항**: `"server_domain": 59` → `"server_domain": 70`

### 3. 실행 환경 변수 변경 (터미널)
서버에서 실행되는 모든 모니터링/제어 스크립트 실행 전 환경 변수를 70으로 맞춰야 합니다.
```bash
# 모든 모니터링/관제 터미널에서 실행
export ROS_DOMAIN_ID=70

# 대상 스크립트:
# - status_robotarm_monitor.py
# - status_pinky_monitor.py
# - pinkytrafficmanger.py
# - robotarmcontrol.py
```

