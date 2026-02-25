from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import mysql.connector
from mysql.connector import Error
import uvicorn
import os
import httpx
from db_manager import db

# ==========================================================
# 0. 초기 설정 및 시스템 구성
# ==========================================================

app = FastAPI(title="Lovo Industrial Intelligence API")

# ==========================================================
# 0. 서버 시작 시 DB 초기화 (원상복귀) 로직
# ==========================================================
@app.on_event("startup")
def reset_db_on_startup():
    print("🔄 [DB RESET] 서버 시작: 데이터베이스를 초기 상태로 리셋합니다...")
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        
        # 1. 모든 주문 상태 초기화
        cursor.execute("UPDATE orders SET status = 'RECEIVED', started_at = NULL, finished_at = NULL")
        
        # 2. 핑키 로봇 상태 초기화 (1: 대기중)
        cursor.execute("UPDATE robot_pinky SET current_order_id = NULL, action_state = 1")
        
        # 3. 로봇팔 상태 초기화 (1: IDLE)
        cursor.execute("UPDATE robot_arm SET current_order_id = NULL, action_state = 1")
        
        # 4. 작업 로그 및 상태 히스토리 삭제
        cursor.execute("DELETE FROM robot_job")
        # 테이블명이 robot_state_history라고 하셨으므로 그대로 반영 (만약 log라면 log로 수정 필요)
        try:
            cursor.execute("DELETE FROM robot_state_history")
        except:
            cursor.execute("DELETE FROM robot_state_log") # 예외 처리용
            
        conn.commit()
        print("✅ [DB RESET] 초기화 완료.")
    except Exception as e:
        print(f"❌ [DB RESET] 초기화 중 오류 발생: {e}")
    finally:
        if 'conn' in locals() and conn.is_connected():
            cursor.close()
            conn.close()

# 모든 Origin에 대해 CORS 허용 (GUI 및 외부 서비스 통신용)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Database helper (using central db_manager)
def get_db_connection():
    """Returns a connection from the global DB pool."""
    return db.get_connection()

# ==========================================================
# 1. 데이터 모델 정의 (Pydantic Models)
# ==========================================================

class OrderItem(BaseModel):
    """주문 항목 상세 정보 (가구 종류, 수량 등)"""
    furnitureType: str
    quantity: int = 1
    components: Optional[dict] = None

class OrderCreate(BaseModel):
    """새 주문 생성을 위한 요청 데이터 구조"""
    items: List[OrderItem]
    customer_phone: Optional[str] = "010-0000-0000"
    statuses: dict

class OrderResponse(BaseModel):
    """주문 목록 응답 구조"""
    order_id: int
    customer_name: str
    furniture_name: str
    quantity: int
    status: str
    picking_command: Optional[float] = None
    packing_command: Optional[float] = None
    ordered_at: str

class OrderCommandResponse(BaseModel):
    """주문 커맨드 단일 조회 응답 구조"""
    order_id: int
    picking_command: Optional[float]
    packing_command: Optional[float]
    new_status: str

class AICoordinateTarget(BaseModel):
    """AI 분석 타겟 좌표 상세 정보"""
    track_id: Optional[int] = None
    conf: Optional[float] = None
    cx: Optional[float] = None
    cy: Optional[float] = None
    bbox: Optional[List[int]] = None
    map_coord: Optional[dict] = None
    base_coord: Optional[dict] = None

class AICoordinateData(BaseModel):
    """AI 서버로부터 전달받는 정밀 분석 데이터 구조"""
    robot_id: str
    t: float
    seq: int
    img: dict
    state: str
    target: Optional[AICoordinateTarget] = None

class AIEventLogData(BaseModel):
    """AI 비전 탐지 이벤트 로그 구조"""
    robot_id: str
    event_type: str
    message: str
    timestamp: str
    seq: int
    label: Optional[str] = None
    map_coord: Optional[dict] = None
    image_path: Optional[str] = None
    image_b64: Optional[str] = None

# ==========================================================
# 2. 로봇 상태 및 매핑 정의 (Integer Codes)
# ==========================================================

# 로봇팔(ARM) 상태 코드 매핑 (0~5)
ARM_STATE_MAP = {
    0: "INIT",
    1: "IDLE",
    2: "BUSY",
    3: "SUCCESS",
    4: "ERROR",
    5: "FIRE"
}

# 운송 로봇(Pinky) 상태 코드 매핑 (0~7)
PINKY_STATE_MAP = {
    0: "충전중",
    1: "출발 대기중",
    2: "운송중 (Nav2)",
    3: "픽업대기 중",
    4: "출고중 (라인트레이싱)",
    5: "패킹대기중",
    6: "복귀중",
    7: "에러 (ERROR)"
}

def format_robot_data(rows):
    """DB의 정수형 상태 코드를 각 로봇 특성에 맞는 한글/영문 텍스트로 변환합니다."""
    for row in rows:
        kind = row.get('robot_kind')
        state_code = row.get('action_state', 0)
        
        if kind == 'ARM':
            row['action_state'] = ARM_STATE_MAP.get(state_code, f"UNKNOWN({state_code})")
        else:
            row['action_state'] = PINKY_STATE_MAP.get(state_code, f"알수없음({state_code})")
            
        # 배터리 소수점 2자리로 고정
        if 'battery_percent' in row and row['battery_percent'] is not None:
            row['battery_percent'] = round(float(row['battery_percent']), 2)
    return rows

# ==========================================================
# 3. 로봇 모니터링 및 상태 업데이트 API
# ==========================================================

@app.get("/api/robots")
def get_robots():
    """모든 로봇(팔 & 핑키)의 현재 상태를 통합 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        # 6축 포즈(ARM)와 3축 포즈(Pinky)를 UNION으로 통합
        query = """
            SELECT 
                arm_id AS robot_id, robot_role, 'ARM' AS robot_kind, 
                pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw,
                action_state, NULL AS battery_percent, current_order_id 
            FROM robot_arm
            UNION ALL
            SELECT 
                pinky_id AS robot_id, robot_role, 'PINKY' AS robot_kind, 
                pose_x, pose_y, 0.0 AS pose_z, 0.0 AS pose_roll, 0.0 AS pose_pitch, pose_yaw,
                action_state, battery_percent, current_order_id 
            FROM robot_pinky
        """
        cursor.execute(query)
        rows = cursor.fetchall()
        return format_robot_data(rows)
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/robots/arms")
def get_robot_arms():
    """로봇팔들의 6축 좌표 및 상태 코드를 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("""
            SELECT 
                arm_id AS robot_id, robot_role, 'ARM' AS robot_kind, 
                pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw,
                action_state, NULL AS battery_percent, current_order_id 
            FROM robot_arm
        """)
        rows = cursor.fetchall()
        return format_robot_data(rows)
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/robots/pinkies")
def get_robot_pinkies():
    """운송 로봇들의 배터리 및 주행 상태를 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("""
            SELECT 
                pinky_id AS robot_id, robot_role, 'PINKY' AS robot_kind, 
                pose_x, pose_y, pose_yaw, battery_percent, action_state, current_order_id 
            FROM robot_pinky
        """)
        rows = cursor.fetchall()
        return format_robot_data(rows)
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.post("/api/status")
def update_robot_status(status_data: dict):
    """[핵심] ROS2 노드로부터 로봇의 실시간 좌표 및 상태 정보(Int/String)를 받아 DB를 동기화합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        
        # ROS 이름 규약과 DB 내 역할 이름 매칭
        name_map = {
            'pinky1': ('PINKY', 'PINKY_1'),
            'pinky2': ('PINKY', 'PINKY_2'),
            'pinky3': ('PINKY', 'PINKY_3'),
            'robot1': ('ARM', 'ARM_1'),
            'robot2': ('ARM', 'ARM_2')
        }
        
        for ros_name, info in status_data.items():
            target = name_map.get(ros_name)
            if not target: continue
            
            kind, db_role = target
            state = info.get('state', 1) # 기본값: IDLE(1)
            
            # 문자열로 들어온 경우 정수형 코드로 매핑
            if isinstance(state, str):
                s = state.upper()
                if kind == 'PINKY':
                    if 'CHARGING' in s: db_state = 0
                    elif 'READY' in s or 'WAIT' in s: db_state = 1
                    elif 'NAV' in s or 'MOVING' in s: db_state = 2
                    elif 'PICKUP' in s: db_state = 3
                    elif 'SHIPPING' in s or 'LINE' in s: db_state = 4
                    elif 'PACK' in s: db_state = 5
                    elif 'RETURN' in s: db_state = 6
                    elif 'ERROR' in s: db_state = 7
                    else: db_state = 1
                else: # ARM
                    if 'INIT' in s: db_state = 0
                    elif 'IDLE' in s: db_state = 1
                    elif 'BUSY' in s or 'MOVING' in s: db_state = 2
                    elif 'SUCCESS' in s: db_state = 3
                    elif 'ERROR' in s: db_state = 4
                    elif 'FIRE' in s: db_state = 5
                    else: db_state = 1
            else:
                db_state = int(state)
                
            battery = info.get('battery', 100.0)
            px, py, pz = info.get('x'), info.get('y'), info.get('z')
            roll, pitch, yaw = info.get('roll'), info.get('pitch'), info.get('yaw')
            
            if kind == 'ARM':
                # 로봇팔은 6축 정보 업데이트
                cursor.execute("""
                    UPDATE robot_arm 
                    SET action_state = %s, pose_x = COALESCE(%s, pose_x), pose_y = COALESCE(%s, pose_y), 
                        pose_z = COALESCE(%s, pose_z), pose_roll = COALESCE(%s, pose_roll), 
                        pose_pitch = COALESCE(%s, pose_pitch), pose_yaw = COALESCE(%s, pose_yaw), 
                        last_seen_at = CURRENT_TIMESTAMP 
                    WHERE robot_role = %s
                """, (db_state, px, py, pz, roll, pitch, yaw, db_role))
            else:
                # 핑키는 배터리 및 주행 좌표(3축) 업데이트
                cursor.execute("""
                    UPDATE robot_pinky 
                    SET action_state = %s, battery_percent = %s, pose_x = COALESCE(%s, pose_x), 
                        pose_y = COALESCE(%s, pose_y), pose_yaw = COALESCE(%s, pose_yaw), 
                        last_seen_at = CURRENT_TIMESTAMP 
                    WHERE robot_role = %s
                """, (db_state, battery, px, py, yaw, db_role))
            
        conn.commit()
        return {"status": "success"}
    except Error as e:
        return {"status": "error", "message": str(e)}
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

# ==========================================================
# 4. 자재, 고객 및 충전소 정보 조회
# ==========================================================

@app.get("/api/products")
def get_products():
    """가구 마스터 정보 및 사양을 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT * FROM furniture ORDER BY furniture_id")
        return cursor.fetchall()
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/materials")
def get_materials():
    """현재 자재 재고 현황을 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT material_id as id, name, qty_on_hand as quantity FROM material")
        rows = cursor.fetchall()
        for r in rows: r.update({'unit': 'ea', 'minStock': 10})
        return rows
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/customers")
def get_customers():
    """등록된 고객 목록을 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT customer_id, name, phone, address FROM customer ORDER BY customer_id")
        return cursor.fetchall()
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/charging-stations")
def get_charging_stations():
    """공장 내 로봇 충전소 위치 및 상태를 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT charger_id, name, x, y, status FROM charging_station")
        return cursor.fetchall()
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

# ==========================================================
# 5. 주문 처리 및 피킹 명령 생성 (MES/CIM Logic)
# ==========================================================

@app.post("/api/orders", status_code=201)
def create_order(order: OrderCreate):
    """새로운 생산 주문을 접수합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        
        # 고객 확인 및 생성 (데모용 단순화)
        phone = order.customer_phone or "010-0000-0000"
        cursor.execute("SELECT customer_id FROM customer WHERE phone = %s", (phone,))
        cust = cursor.fetchone()
        customer_id = cust['customer_id'] if cust else None
        if not customer_id:
            cursor.execute("INSERT INTO customer (name, phone) VALUES ('Demo User', %s)", (phone,))
            customer_id = cursor.lastrowid
            
        # 2. 주문 순번 확인 (X값 결정: 1, 2, 3 순환)
        cursor.execute("SELECT COUNT(*) as count FROM orders")
        order_count = cursor.fetchone()['count']
        
        order_ids = []
        for i, item in enumerate(order.items):
            # i번째 아이템의 X 좌표 결정 (전체 주문 수 + 현재 루프 인덱스)
            x_val = ((order_count + i) % 3) + 1
            
            cat = item.furnitureType.upper()
            # 해당 카테고리의 가구 정보 및 BOM 조회
            cursor.execute("""
                SELECT furniture_id, top_material_id, leg_material_id, wheel_material_id, kit_material_id 
                FROM furniture WHERE category = %s LIMIT 1
            """, (cat,))
            furn = cursor.fetchone()
            
            if furn:
                # BOM 구성에 따른 ABCD 비트 생성
                # A: 프레임(4), B: 다리(5), C: 바퀴(6), D: 작업킷(7) / 없으면 0
                a = "4" if furn['top_material_id'] else "0"
                b = "5" if furn['leg_material_id'] else "0"
                c = "6" if furn['wheel_material_id'] else "0"
                d = "7" if furn['kit_material_id'] else "0"
                
                picking_cmd = float(f"{x_val}.{a}{b}{c}{d}")
                packing_cmd = float(f"0.{a}{b}{c}{d}")
                
                cursor.execute(
                    """INSERT INTO orders (customer_id, furniture_id, quantity, picking_command, packing_command) 
                       VALUES (%s, %s, %s, %s, %s)""",
                    (customer_id, furn['furniture_id'], item.quantity, picking_cmd, packing_cmd)
                )
                order_ids.append(cursor.lastrowid)

        conn.commit()
        return {'message': 'Order created successfully', 'orderIds': order_ids}
    except Error as e:
        if 'conn' in locals() and conn.is_connected(): conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected(): conn.close()

@app.get("/api/orders")
def get_orders():
    """최근 주문 목록 및 생산 진행 현황을 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("""
            SELECT o.order_id, COALESCE(c.name, '알수없는고객') as customer_name, 
                   COALESCE(f.name, '알수없는가구') as furniture_name, 
                   o.quantity, o.status, o.picking_command, o.packing_command, 
                   DATE_FORMAT(o.ordered_at, '%Y-%m-%d %H:%i:%s') as ordered_at 
            FROM orders o 
            LEFT JOIN customer c ON o.customer_id = c.customer_id
            LEFT JOIN furniture f ON o.furniture_id = f.furniture_id 
            ORDER BY o.ordered_at DESC LIMIT 50
        """)
        return cursor.fetchall()
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/orders/{order_id}/command", response_model=OrderCommandResponse)
def get_order_command(order_id: int):
    """주문 ID를 기반으로 로봇 피킹을 위한 비트맵 커맨드(예: 1.4507)를 생성합니다."""
    conn = None
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        conn.start_transaction()

        query = """
            SELECT status, picking_command, packing_command
            FROM orders WHERE order_id = %s
        """
        cursor.execute(query, (order_id,))
        row = cursor.fetchone()
        if not row: raise HTTPException(status_code=404, detail="Order not found")

        # 주문 수락 시 상태 'IN_PROGRESS'로 변경
        if row['status'] == 'RECEIVED':
            cursor.execute("UPDATE orders SET status = 'IN_PROGRESS', started_at = CURRENT_TIMESTAMP WHERE order_id = %s", (order_id,))

        conn.commit()
        return {
            "order_id": order_id, 
            "picking_command": row['picking_command'],
            "packing_command": row['packing_command'],
            "new_status": "IN_PROGRESS"
        }
    except Error as e:
        if conn: conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if conn and conn.is_connected(): conn.close()

# ==========================================================
# 6. 로그 조회 및 이력 관리
# ==========================================================

@app.get("/api/robot-jobs")
def get_robot_jobs():
    """로봇들에게 할당된 작업 이력을 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("""
            SELECT job_id, order_id, COALESCE(arm_id, pinky_id) AS robot_id, job_type, status, created_at FROM robot_job 
            ORDER BY created_at DESC LIMIT 50
        """)
        return cursor.fetchall()
    finally:
        if 'conn' in locals() and conn.is_connected(): conn.close()

@app.get("/api/inventory-logs")
def get_inventory_logs():
    """자재 입출고 및 재고 변동 이력을 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT * FROM inventory_tx ORDER BY created_at DESC LIMIT 50")
        return cursor.fetchall()
    finally:
        if 'conn' in locals() and conn.is_connected(): conn.close()

# ==========================================================
# 7. AI 지능형 관제 및 화재 감지
# ==========================================================

latest_ai_coordinates: Optional[AICoordinateData] = None

@app.post("/api/ai/coordinates")
async def receive_ai_coordinates(data: AICoordinateData):
    """AI 비전 서버로부터 탐지된 객체의 정밀 좌표를 수신합니다."""
    global latest_ai_coordinates
    latest_ai_coordinates = data
    return {"status": "success"}

@app.get("/api/ai/coordinates")
def get_ai_coordinates():
    """저장된 최신 AI 정밀 좌표를 반환합니다 (피킹 보정용)."""
    return latest_ai_coordinates

@app.get("/api/robot-logs")
def get_robot_logs():
    """로봇 상태 변화 히스토리(로그)를 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("""
            SELECT log_id, robot_role as robot_id, state_text as state, 
                   COALESCE(battery_level, 0.0) as battery_level, 
                   COALESCE(pose_info, 'N/A') as location_text,
                   DATE_FORMAT(logged_at, '%Y-%m-%d %H:%i:%s') as logged_at 
            FROM robot_state_history 
            ORDER BY logged_at DESC LIMIT 100
        """)
        return cursor.fetchall()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/fire-logs")
def get_fire_logs():
    """화재 감지 및 이상 징후 이력을 조회합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT * FROM fire_incidents ORDER BY occurred_at DESC LIMIT 50")
        return cursor.fetchall()
    finally:
        if 'conn' in locals() and conn.is_connected(): conn.close()

@app.post("/api/ai/event-log")
async def receive_ai_event(data: AIEventLogData):
    """AI 서버로부터 화재 등 이벤트 로그를 수신하여 DB에 저장"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        # fire_incidents 테이블에 화재 발생 기록 저장
        cursor.execute(
            "INSERT INTO fire_incidents (robot_id, message, occurred_at) VALUES (%s, %s, CURRENT_TIMESTAMP)",
            (data.robot_id, data.message)
        )
        conn.commit()
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}
    finally:
        if 'conn' in locals() and conn.is_connected(): conn.close()

# ==========================================================
# 8. 서버 실행
# ==========================================================

if __name__ == '__main__':
    # 0.0.0.0 포트 5000으로 서버 가동
    uvicorn.run(app, host='0.0.0.0', port=5000)
