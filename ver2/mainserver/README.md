# Main Server & GUI 연동 가이드

이 문서는 **Main Server(FastAPI)**와 **GUI(PyQt6)** 간의 연동 방식과 구현 내용에 대해 상세히 설명합니다.

GUI는 REST API를 통해 서버와 통신하며, 실시간으로 주문 정보와 로봇 상태를 동기화합니다.

## 1. 시스템 구조

*   **Server**: FastAPI 기반의 웹 서버 (Port 5000), MySQL 데이터베이스와 직접 통신
*   **GUI**: PyQt6 기반의 데스크탑 앱, `requests` 라이브러리를 사용하여 서버 API 호출
*   **통신 방식**: HTTP REST API (Synchronous)

## 2. 주요 변경 파일 및 기능

GUI 쪽에서 서버와 연동하기 위해 수정 및 추가된 파일들의 상세 내용입니다.

### 2.1 API 클라이언트 모듈 (신규 생성)
*   **파일 위치**: `gui/src/lovo_gui/lovo_gui/core/api_client.py`
*   **기능**: 서버와의 모든 HTTP 통신을 전담하는 클래스입니다.
*   **주요 메소드**:
    *   `check_health()`: 서버가 살아있는지 확인 (단순 Ping이 아닌 실제 웹 서버 응답 확인)
    *   `get_orders()`: `/api/orders` 엔드포인트를 호출하여 최신 주문 내역 조회
    *   `get_robots()`: `/api/robots` 엔드포인트를 호출하여 로봇들의 상태(배터리, 위치 등) 조회
    *   `get_products()`, `get_materials()`: 제품 및 자재 정보 조회

### 2.2 패키지 의존성 추가
*   **파일 위치**: `gui/src/lovo_gui/package.xml`
*   **내용**: HTTP 통신을 위해 `python3-requests` 패키지 의존성을 추가했습니다.

### 2.3 서버 연결 로직 개선
*   **파일 위치**: `gui/src/lovo_gui/lovo_gui/tabs/tab05_communication/communication.py`
*   **내용**:
    *   `CommunicationManager` 초기화 시 `APIClient` 인스턴스를 생성합니다.
    *   **Connect 버튼 로직**: 대상이 '서버'인 경우, 기존의 `ping` 명령 대신 `api_client.check_health()`를 호출하여 실제 서비스 가능 여부를 체크합니다. (초록색 불이 들어오면 API 통신 성공)

### 2.4 데이터 실시간 동기화 (Main 탭)
*   **파일 위치**: `gui/src/lovo_gui/lovo_gui/tabs/tab01_main/tab01_main.py`
*   **내용**:
    *   `QTimer`를 사용하여 **2초마다** 서버 데이터를 갱신합니다.
    *   **주문 로그**: `get_orders()`로 받아온 데이터를 파싱하여 화면 우측 상단의 로그 창에 표시합니다.
    *   **로봇 상태**: `get_robots()`로 받아온 데이터를 기반으로 로봇 상태 그리드(배터리, 상태 텍스트)를 갱신합니다.
    *   *참고*: 로봇 상태 동기화는 DB의 `robot_id`와 GUI 설정의 식별자가 일치해야 정확하게 동작합니다.

## 3. 사용 방법

1.  **Main Server 실행**:
    ```bash
    # server/mainserver 디렉토리에서
    python3 app.py
    ```
    (기본 포트: 5000)

2.  **GUI 실행**:
    ```bash
    ros2 run lovo_gui lovo_gui
    ```

3.  **연결**:
    *   GUI의 **Communication** 탭으로 이동합니다.
    *   서버 행의 IP 주소가 맞는지 확인하고 (기본: `192.168.0.70`), **Connect** 버튼을 누릅니다.
    *   **"🟢 Online"** 상태가 되면 연동 성공입니다.

4.  **확인**:
    *   **Main** 탭으로 이동하면 주문 내역이 서버 DB 내용으로 채워지는 것을 확인할 수 있습니다.
    *   새로운 주문이 들어오면 2초 내에 화면에 반영됩니다.

---

## 4. Pinky 관제 서버 연동 (Pinky Control Server Integration)

메인 서버는 현장의 **핑키 관제 서버(192.168.0.184)**와 통신하여 로봇들의 작업을 조율하고 상태를 수집합니다.

### 4.1 네트워크 구성 (Network Topology)

| 구성 요소 | IP 주소 | 주요 역할 |
| :--- | :--- | :--- |
| **Main Server & GUI** | `192.168.0.30` | 주문 관리, 통합 DB 운영, 사용자 인터페이스 |
| **Pinky Control Server** | `192.168.0.184` | 핑키 로봇(1~3호기) 직접 제어, ROS 2 통신 브릿지 |

### 4.2 주요 연동 엔드포인트

*   **`POST /api/status`**: 관제 서버가 로봇들의 배터리 및 현재 상태를 메인 서버 DB에 보고합니다.
*   **`GET /api/mission/command`**: 관제 서버가 메인 서버로부터 새로운 명령(START/WAIT)이 있는지 폴링합니다.
*   **`POST /api/mission/start`**: 웹/GUI UI에서 미션을 시작할 때 호출하며, `current_mission_command`를 `START`로 변경합니다.

### 4.3 연동 확인 방법
1.  메인 서버(`app.py`)가 `192.168.0.30`에서 실행 중인지 확인합니다.
2.  관제 서버의 `bridge_manager.py`를 실행하여 `192.168.0.30:5000`으로 접속 로그가 남는지 확인합니다.
3.  메인 서버 로그에 `POST /api/status` 요청이 주기적으로 들어오면 연동 성공입니다.
