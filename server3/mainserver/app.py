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

@app.get("/api/products")
def get_products():
    """Fetch structured furniture data."""
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
    """Fetch current material stock."""
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

@app.get("/api/robots")
def get_robots():
    """Fetch all robots status."""
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

@app.post("/api/orders", status_code=201)
def create_order(order: OrderCreate):
    """Create a new order."""
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
    """Fetch recent orders."""
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

@app.post("/api/status")
def update_robot_status(status_data: dict):
    """Update robot status in DB received from bridge_manager."""
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

@app.post("/api/ai/analysis")
async def ai_analysis(request: Request):
    """Bridge to AI Microservice."""
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

@app.get("/api/ai/coordinates", response_model=Optional[AICoordinateData])
def get_ai_coordinates():
    """Fetch the latest received AI coordinates."""
    return latest_ai_coordinates

@app.post("/api/ai/coordinates")
async def receive_ai_coordinates(data: AICoordinateData):
    """Receive detection coordinates from AI Server."""
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

if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=5000)
