from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import mysql.connector
from mysql.connector import Error
import uvicorn
import os
import httpx

app = FastAPI(title="LOVO Factory System API")

# Enable CORS for all origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Database Configuration
db_config = {
    'host': 'localhost',
    'user': 'lovoDB',
    'password': 'LovoDB1234!',
    'database': 'factory_system'
}

def get_db_connection():
    return mysql.connector.connect(**db_config)

# Pydantic Models for validation
class OrderItem(BaseModel):
    furnitureType: str
    quantity: int = 1
    components: Optional[dict] = None

class OrderCreate(BaseModel):
    items: List[OrderItem]
    customer_phone: Optional[str] = "010-0000-0000"

    # This matches the JSON structure from bridge_manager.py
    # { "pinky1": {"state": "...", "location": "...", "battery": 0.0}, ... }
    statuses: dict

class AICoordinateTarget(BaseModel):
    track_id: Optional[int] = None
    conf: Optional[float] = None
    cx: Optional[float] = None
    cy: Optional[float] = None
    bbox: Optional[List[int]] = None
    map_coord: Optional[dict] = None
    base_coord: Optional[dict] = None

class AICoordinateData(BaseModel):
    robot_id: str
    t: float
    seq: int
    img: dict
    state: str
    target: Optional[AICoordinateTarget] = None

# ==========================================================
# 1. 자재 및 제품 정보 조회 (Web UI용)
# ==========================================================

@app.get("/api/products")
def get_products():
    """DB의 가구 마스터 테이블 정보를 모두 가져옵니다"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        
        cursor.execute("SELECT * FROM furniture ORDER BY furniture_id")
        furniture_db_rows = cursor.fetchall()
        
        return furniture_db_rows

    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/materials")
def get_materials():
    """현재 공장 창고에 남은 자재 재고 정보를 가져옵니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT material_id, name, qty_on_hand FROM material")
        rows = cursor.fetchall()
        
        data = []
        for r in rows:
            data.append({
                'id': r['material_id'],
                'name': r['name'],
                'quantity': r['qty_on_hand'],
                'unit': 'ea',
                'minStock': 10
            })
            
        return data
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()



# ==========================================================
# 2. 로봇 상태 관리 (Bridge Manager & Dashboard)
# ==========================================================

@app.get("/api/robots")
def get_robots():
    """모든 로봇의 현재 상태 정보를 가져옵니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("SELECT robot_id, robot_role, robot_kind, pose_x, pose_y, action_state, battery_percent FROM robot")
        rows = cursor.fetchall()
        return rows
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()


@app.post("/api/status")
def update_robot_status(status_data: dict):
   """[핵심] ROS2(Bridge Manager)로부터 로봇의 실시간 정보를 받아 DB를 갱신합니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        
        # Mapping from ROS names to DB roles
        name_map = {
            'pinky1': 'PINKY_TRANS_1',
            'pinky2': 'PINKY_TRANS_2',
            'pinky3': 'PINKY_PATROL'
        }
        
        for ros_name, info in status_data.items():
            db_role = name_map.get(ros_name)
            if not db_role:
                continue
                
            state = info.get('state', 'IDLE')
            # Map ROS state strings to DB ENUM if necessary
            # For now, let's assume they match or are handled gracefully
            
            # Simple conversion if needed: MOVING_TO_PICKUP_1 -> TRANSPORTING
            if 'MOVING' in state:
                db_state = 'TRANSPORTING'
            elif 'AT_PICKUP' in state:
                db_state = 'PICKING'
            elif 'IDLE' in state:
                db_state = 'IDLE'
            else:
                db_state = state
                
            battery = info.get('battery', 100.0)
            
            cursor.execute("""
                UPDATE robot 
                SET action_state = %s, battery_percent = %s, last_seen_at = CURRENT_TIMESTAMP
                WHERE robot_role = %s
            """, (db_state, battery, db_role))
            
        conn.commit()
        return {"status": "success"}
    except Error as e:
        return {"status": "error", "message": str(e)}
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()



# ==========================================================
# 3. 주문 및 작업 할당 (CIM: Computer Integrated Manufacturing)
# ==========================================================

@app.post("/api/orders", status_code=201)
def create_order(order: OrderCreate):
    """고객의 주문을 접수하고 DB에 등록"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        
        # 1. Get or Create Customer
        demo_phone = order.customer_phone or "010-0000-0000"
        cursor.execute("SELECT customer_id FROM customer WHERE phone = %s", (demo_phone,))
        cust = cursor.fetchone()
        if not cust:
            cursor.execute("INSERT INTO customer (name, phone) VALUES ('Demo User', %s)", (demo_phone,))
            customer_id = cursor.lastrowid
        else:
            customer_id = cust['customer_id']
            
        order_ids = []
        # 2. Process Items
        for item in order.items:
            cat = item.furnitureType.upper()
            qty = item.quantity
            
            cursor.execute("SELECT furniture_id FROM furniture WHERE category = %s LIMIT 1", (cat,))
            furn = cursor.fetchone()
            
            if furn:
                furn_id = furn['furniture_id']
                cursor.execute(
                    "INSERT INTO orders (customer_id, furniture_id, quantity) VALUES (%s, %s, %s)",
                    (customer_id, furn_id, qty)
                )
                order_ids.append(cursor.lastrowid)
            else:
                import logging
                logging.debug(f"No furniture found for category {cat}")

        conn.commit()
        return {'message': 'Order created successfully', 'orderIds': order_ids}

    except Error as e:
        if 'conn' in locals() and conn.is_connected():
            conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/orders")
def get_orders():
    """최근 주문 목록을 가져옵니다."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        cursor.execute("""
            SELECT o.order_id, c.name as customer_name, f.name as furniture_name, o.quantity, o.status, o.ordered_at 
            FROM orders o
            JOIN customer c ON o.customer_id = c.customer_id
            JOIN furniture f ON o.furniture_id = f.furniture_id
            ORDER BY o.ordered_at DESC LIMIT 50
        """)
        rows = cursor.fetchall()
        return rows
    except Error as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if 'conn' in locals() and conn.is_connected():
            conn.close()

@app.get("/api/orders/{order_id}/command")
def get_order_command(order_id: int):
    """
    [로봇 연동] 주문 ID를 기반으로 피킹 커맨드를 생성하고, 
    주문 상태를 'RECEIVED'에서 'MAKING'으로 변경합니다.
    """
    conn = None
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        
        # 1. 트랜잭션 시작 (조회와 업데이트를 하나로 묶음)
        conn.start_transaction()

        # 2. 가구의 자재 구성 정보(BOM) 및 현재 주문 상태 조회
        query = """
            SELECT o.status, f.top_material_id, f.leg_material_id, f.wheel_material_id, f.kit_material_id
            FROM orders o
            JOIN furniture f ON o.furniture_id = f.furniture_id
            WHERE o.order_id = %s
        """
        cursor.execute(query, (order_id,))
        row = cursor.fetchone()
        
        if not row:
            raise HTTPException(status_code=404, detail="Order or Furniture not found")

        # 3. 주문 상태 업데이트 (RECEIVED일 때만 MAKING으로 변경)
        if row['status'] == 'RECEIVED':
            cursor.execute("""
                UPDATE orders 
                SET status = 'MAKING', started_at = CURRENT_TIMESTAMP 
                WHERE order_id = %s
            """, (order_id,))

        # 4. 커맨드 생성 로직 (1.[프레임][다리][바퀴][작업킷])
        c4 = "4" if row['top_material_id'] else "0"
        c5 = "5" if row['leg_material_id'] else "0"
        c6 = "6" if row['wheel_material_id'] else "0"
        c7 = "7" if row['kit_material_id'] else "0"
        
        command = f"1.{c4}{c5}{c6}{c7}"

        # 5. 모든 작업 성공 시 커밋
        conn.commit()
        
        return {
            "order_id": order_id, 
            "command": float(command),
            "new_status": "MAKING"
        }

    except Error as e:
        if conn:
            conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if conn and conn.is_connected():
            conn.close()

@app.patch("/api/orders/{order_id}/status")
def update_order_status(order_id: int, status: str):
    """
    로봇이 작업을 완료하면 주문 상태를 업데이트하고, 
    'DONE' 상태일 경우 가구 구성에 맞게 재고를 자동 차감합니다.
    """
    conn = None
    try:
        conn = get_db_connection()
        cursor = conn.cursor(dictionary=True)
        
        # 1. 트랜잭션 시작
        conn.start_transaction()

        # 2. 주문 상태 업데이트 (성공 시 시각 기록 포함)
        cursor.execute("""
            UPDATE orders 
            SET status = %s, finished_at = CURRENT_TIMESTAMP 
            WHERE order_id = %s
        """, (status, order_id))

        # 3. 만약 상태가 'DONE'으로 바뀌는 것이라면 재고 차감 실행
        if status.upper() == 'DONE':
            # 3-1. 해당 주문의 가구 구성(자재 ID 및 수량) 조회
            cursor.execute("""
                SELECT f.top_material_id, f.top_qty_per_unit,
                       f.leg_material_id, f.leg_qty_per_unit,
                       f.wheel_material_id, f.wheel_qty_per_unit,
                       f.kit_material_id, f.kit_qty_per_unit
                FROM orders o
                JOIN furniture f ON o.furniture_id = f.furniture_id
                WHERE o.order_id = %s
            """, (order_id,))
            bom = cursor.fetchone()

            if bom:
                # 3-2. 자재별 차감 쿼리 실행 (자재 ID가 존재하고 수량이 0보다 클 때만)
                materials_to_update = [
                    (bom['top_qty_per_unit'], bom['top_material_id']),
                    (bom['leg_qty_per_unit'], bom['leg_material_id']),
                    (bom['wheel_qty_per_unit'], bom['wheel_material_id']),
                    (bom['kit_qty_per_unit'], bom['kit_material_id'])
                ]

                for qty, m_id in materials_to_update:
                    if m_id and qty > 0:
                        cursor.execute("""
                            UPDATE material 
                            SET qty_on_hand = qty_on_hand - %s 
                            WHERE material_id = %s
                        """, (qty, m_id))

        # 4. 모든 작업 성공 시 커밋
        conn.commit()
        return {"status": "success", "message": f"Order {order_id} updated to {status} and inventory deducted."}

    except Error as e:
        # 오류 발생 시 롤백하여 데이터 보존
        if conn:
            conn.rollback()
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if conn and conn.is_connected():
            conn.close()


# ==========================================================
# 4. 전체 공정 제어 (Mission Control)
# ==========================================================

# Global variable for demo mission control
current_mission_command = "WAIT"

@app.get("/api/mission/command")
def get_mission_command():
    """Endpoint for bridge_manager to check for new commands."""
    global current_mission_command
    return {"command": current_mission_command}

@app.post("/api/mission/start")
def start_global_mission():
    """Endpoint for web UI to trigger mission."""
    global current_mission_command
    current_mission_command = "START"
    return {"status": "Mission started"}

# ==========================================================
# 5. AI 지능형 관제 (AI Microservice Bridge)
# ==========================================================

@app.post("/api/ai/analysis")
async def ai_analysis(request: Request):
    """AI 서버로 분석 요청을 전달하고 결과를 반환"""
    try:
        data = await request.json()
        
        # Call AI Server (Running on port 8000)
        async with httpx.AsyncClient() as client:
            response = await client.post("http://localhost:8000/predict", json=data)
            
        if response.status_code != 200:
            raise HTTPException(status_code=response.status_code, detail="AI Server Error")
            
        return response.json()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# Global storage for latest coordinates
latest_ai_coordinates: Optional[AICoordinateData] = None


@app.post("/api/ai/coordinates")
async def receive_ai_coordinates(data: AICoordinateData):
    """AI 서버로부터 분석 좌표를 수신하여 저장합니다."""
    global latest_ai_coordinates
    try:
        latest_ai_coordinates = data # Update global state
        
        # Log for verification
        import logging
        print(f"Received coordinates from AI: {data.dict()}") 
        logging.info(f"Received coordinates from AI: {data.dict()}")
        return {"status": "success", "received": data.seq}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/ai/coordinates", response_model=Optional[AICoordinateData])
def get_ai_coordinates():
    """저장된 최신 AI 분석 좌표를 가져옵니다 (로봇의 정밀 피킹에 활용)"""
    return latest_ai_coordinates


if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=5000)
