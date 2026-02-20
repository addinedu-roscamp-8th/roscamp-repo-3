# Smart Factory Main Server & Control System

이 프로젝트는 **Main Server(FastAPI)**를 중심으로 로봇 제어, 주문 관리, 데이터베이스 운영을 통합한 스마트 팩토리 제어 시스템입니다.

## 1. 시스템 개요

*   **Main Server**: FastAPI 기반 웹 서버 (Port 5000), MySQL DB 연동, REST API 제공
*   **Robot Controller**: ROS 2 기반 로봇 제어 노드 (`onlypickingrobotarm.py` 등)
*   **Domain Bridge**: 서로 다른 ROS_DOMAIN_ID 간의 통신 중계
*   **Database**: MySQL (Table: `orders`, `robot`, `material`, `furniture` 등)

---

## 2. 전체 시스템 실행 가이드 (Step-by-Step)

아래 순서대로 **4개의 터미널**을 열어 명령어를 실행하면 전체 시스템을 테스트할 수 있습니다.
*(모든 명령어는 `server3/mainserver` 디렉토리 기준입니다)*

### [Terminal 1] DB 초기화 (주문 생성)
기존 주문 데이터를 삭제하고, 테스트용 주문(침대1, 의자2, 책상2)을 생성합니다.
```bash
cd ~/Desktop/roscamp-repo-3/server3/mainserver
mysql -u lovoDB -p'LovoDB1234!' -D factory_system < DB/orders_seed.sql
```

### [Terminal 2] 도메인 브릿지 (통신 연결)
메인 서버(Domain 59)와 로봇(Domain 60/61) 간의 통신을 연결합니다.
```bash
cd ~/Desktop/roscamp-repo-3/server3/mainserver
source ../../venv/bin/activate
ros2 run domain_bridge domain_bridge config/bridge_robotarm/robotarm1.yaml
```

### [Terminal 3] 메인 서버 앱 (API & 모니터링)
웹 서버를 실행하여 Swagger UI 및 API 서비스를 제공합니다.
```bash
cd ~/Desktop/roscamp-repo-3/server3/mainserver
source ../../venv/bin/activate
python3 app.py
```
*   **Swagger 접속**: [http://localhost:5000/docs](http://localhost:5000/docs)

### [Terminal 4] 피킹 로봇 제어 (주문 처리)
DB에서 `RECEIVED` 상태의 주문을 가져와 로봇(Domain 60)에게 작업을 지시합니다.
```bash
cd ~/Desktop/roscamp-repo-3/server3/mainserver
source ../../venv/bin/activate
export ROS_DOMAIN_ID=59
python3 onlypickingrobotarm.py
```

---

## 3. Swagger UI를 통한 DB 및 시스템 확인 방법

`app.py`를 실행한 후 **[http://localhost:5000/docs](http://localhost:5000/docs)** 에 접속하면 API를 통해 DB 내용을 실시간으로 확인할 수 있습니다.

### 3.1 주문 내역 확인 (Orders)
현재 접수된 주문 목록과 진행 상태(`RECEIVED`, `MAKING`, `DONE`)를 확인합니다.

1.  **`GET /api/orders`** 항목 클릭
2.  **[Try it out]** 버튼 클릭
3.  **[Execute]** 버튼 클릭
4.  **Response Body 확인**:
    ```json
    [
      {
        "order_id": 1,
        "furniture_name": "Bed (Wood/LegA)",
        "quantity": 1,
        "status": "RECEIVED",  <-- (로봇 작업 전)
        "ordered_at": "2026-02-19T10:00:00"
      },
      ...
    ]
    ```
    *   터미널 4(`onlypickingrobotarm.py`) 실행 후 다시 조회하면 `status`가 **`MAKING`** 으로 변경된 것을 볼 수 있습니다.

### 3.2 로봇 상태 확인 (Robots)
로봇들의 현재 상태(IDLE, BUSY), 위치, 배터리 정보를 확인합니다.

1.  **`GET /api/robots`** 항목 클릭
2.  **[Try it out]** -> **[Execute]**
3.  **Response Body 확인**:
    ```json
    [
      {
        "robot_role": "ARM_1",
        "action_state": "IDLE",
        "battery_percent": 100
      },
      ...
    ]
    ```

### 3.3 로봇 커맨드 생성 테스트 (Debugging)
특정 주문 번호에 대해 로봇이 어떤 커맨드(`1.ABCD`)를 생성하는지 테스트할 수 있습니다.
*(주의: 이 API를 호출하면 해당 주문의 상태가 `MAKING`으로 강제 변경됩니다)*

1.  **`GET /api/orders/{order_id}/command`** 항목 클릭
2.  **[Try it out]** 클릭
3.  `order_id`에 **1** 입력 후 **[Execute]**
4.  **Response Body 확인**:
    ```json
    {
      "order_id": 1,
      "command": 1.4507,  <-- (프레임4, 다리5, 바퀴0, 킷7 조합)
      "new_status": "MAKING"
    }
    ```

### 3.4 자재 재고 확인 (Materials)
현재 공장에 남아있는 자재(상판, 다리, 바퀴 등)의 수량을 확인합니다.

1.  **`GET /api/materials`** 항목 클릭
2.  **[Try it out]** -> **[Execute]**
3.  **Response Body 확인**:
    ```json
    [
      {
        "id": 1,
        "name": "Wood Top",
        "quantity": 50
      },
      ...
    ]
    ```

---

## 4. 파일 및 디렉토리 구조

*   `server3/mainserver/`
    *   `app.py`: 메인 API 서버
    *   `onlypickingrobotarm.py`: 피킹 로봇 제어 노드 (DB 연동)
    *   `mock_robot_arm.py`: 테스트용 가상 로봇 (하드웨어 없을 시 사용)
    *   `DB/`: 데이터베이스 관련 파일
        *   `sf.sql`: 전체 DB 스키마
        *   `orders_seed.sql`: 테스트용 시드 데이터 (자동 초기화 포함)
    *   `config/bridge_robotarm/`: 도메인 브릿지 설정 파일
