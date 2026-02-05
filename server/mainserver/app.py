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
    'host': '192.168.0.184',
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
                print(f"No furniture found for category {cat}")

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

@app.post("/api/ai/analysis")
async def ai_analysis(request: Request):
    """Bridge to AI Microservice."""
    try:
        data = await request.json()
        
        # Call AI Server (Running on port 8000)
        async with httpx.AsyncClient() as client:
            response = await client.post("http://192.168.0.184:8000/predict", json=data)
            
        if response.status_code != 200:
            raise HTTPException(status_code=response.status_code, detail="AI Server Error")
            
        return response.json()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=5000)
