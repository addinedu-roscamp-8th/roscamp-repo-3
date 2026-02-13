#!/usr/bin/env python3
import rclpy
import time
import threading
import mysql.connector
from mysql.connector import Error
from arm_handler import ArmHandler # 같은 폴더에 있다고 가정

# --- DB 설정 ---
db_config = {
    'host': 'localhost',
    'user': 'lovoDB',
    'password': 'LovoDB1234!',
    'database': 'factory_system'
}

def get_db_connection():
    # 1. Try localhost
    try:
        config = db_config.copy()
        config['host'] = 'localhost'
        conn = mysql.connector.connect(**config)
        if conn.is_connected():
            return conn
    except Error as e:
        print(f"Warning: Failed to connect to localhost: {e}")

    # 2. Try 127.0.0.1
    try:
        print("Retrying with 127.0.0.1...")
        config = db_config.copy()
        config['host'] = '127.0.0.1'
        conn = mysql.connector.connect(**config)
        if conn.is_connected():
            print("Connected via 127.0.0.1")
            return conn
    except Error as e:
        print(f"Error connecting to MySQL (127.0.0.1): {e}")

    return None

def generate_command(furniture_id, action_type=1):
    """
    furniture_id에 해당하는 가구의 자재 정보를 조회하여
    명령어 포맷 X.ABCD 를 생성함.
    
    X: action_type (1: Picking, 0: Packing)
    A: Frame (4)
    B: Legs (5)
    C: Wheels (6)
    D: Kit (7)
    
    자재가 없으면 0으로 처리.
    """
    conn = get_db_connection()
    if conn is None:
        print("[Warning] DB Connection failed. Using MOCK data (1.4507).")
        # Mock for Frame(4), Legs(5), Kit(7) -> 1.4507
        return float(action_type) + 0.4507
    
    cmd_val = 0.0
    
    try:
        cursor = conn.cursor(dictionary=True)
        # furniture 테이블 조회 (구성 자재 ID 확인)
        query = """
            SELECT 
                top_material_id, 
                leg_material_id, 
                wheel_material_id, 
                kit_material_id
            FROM furniture
            WHERE furniture_id = %s
        """
        cursor.execute(query, (furniture_id,))
        row = cursor.fetchone()
        
        if row:
            # A: Frame (Top) -> 존재하면 4
            a = 4 if row['top_material_id'] else 0
            
            # B: Legs -> 존재하면 5
            b = 5 if row['leg_material_id'] else 0
            
            # C: Wheels -> 존재하면 6 (NULL일 수 있음)
            c = 6 if row['wheel_material_id'] else 0
            
            # D: Kit -> 존재하면 7
            d = 7 if row['kit_material_id'] else 0
            
            # 포맷: X.ABCD 
            # 예: 1.4507 -> 1 + 0.4 + 0.05 + 0.000 + 0.0007 = 1.4507
            # 소수점 자릿수 계산:
            # A는 소수 첫째자리 ( * 0.1 )
            # B는 소수 둘째자리 ( * 0.01 )
            # C는 소수 셋째자리 ( * 0.001 )
            # D는 소수 넷째자리 ( * 0.0001 )
            
            cmd_val = float(action_type) + (a * 0.1) + (b * 0.01) + (c * 0.001) + (d * 0.0001)
            
            print(f"[DB] Furniture ID: {furniture_id} | Components: A={a}, B={b}, C={c}, D={d} => Command: {cmd_val:.4f}")
            
        else:
            print(f"[DB] Furniture ID {furniture_id} not found.")
            
    except Error as e:
        print(f"Error reading data: {e}")
    finally:
        if conn.is_connected():
            cursor.close()
            conn.close()
            
    return cmd_val

# --- Constants ---
ROBOT_IDLE = 1
ROBOT_BUSY = 2
ROBOT_SUCCESS = 3

def update_order_status(order_id, status):
    """
    DB의 orders 테이블에서 해당 order_id의 status를 업데이트함.
    status: 'IN_PROGRESS', 'DONE' (or 'PICKED' if added to Enum, but 'MAKING'/'DONE' are in schema)
    Schema: RECEIVED, MAKING, IN_PROGRESS, DONE, CANCELLED
    """
    conn = get_db_connection()
    if conn is None:
        return
    
    try:
        cursor = conn.cursor()
        query = "UPDATE orders SET status = %s, updated_at = NOW() WHERE order_id = %s"
        # Schema definition might not have updated_at in orders, checking schema... 
        # orders has: ordered_at, started_at, finished_at. 
        # Let's update started_at or finished_at based on status.
        
        if status == 'MAKING':
            query = "UPDATE orders SET status = %s, started_at = NOW() WHERE order_id = %s"
        elif status == 'DONE':
            query = "UPDATE orders SET status = %s, finished_at = NOW() WHERE order_id = %s"
        else:
            query = "UPDATE orders SET status = %s WHERE order_id = %s"
            
        cursor.execute(query, (status, order_id))
        conn.commit()
        print(f"[DB] Order {order_id} updated to {status}")
        
    except Error as e:
        print(f"[DB] Error updating order: {e}")
    finally:
        if conn.is_connected():
            cursor.close()
            conn.close()

def send_command_until_busy(arm_node, robot_role, command):
    """
    로봇이 BUSY(2) 상태가 될 때까지 명령을 반복 전송함.
    robot_role: 1 (Picking), 2 (Packing)
    """
    print(f"[{robot_role}] Sending Command {command:.4f} until BUSY...")
    
    while True:
        # 1. 상태 확인
        current_status = None
        if robot_role == 1:
            current_status = arm_node.r1_state.get('status')
            arm_node.r1_cmd_order(command)
        else:
            current_status = arm_node.r2_state.get('status')
            arm_node.r2_cmd_order(command)
            
        # 2. BUSY 확인
        if current_status == ROBOT_BUSY:
            print(f"[{robot_role}] Robot is BUSY! Stopping command transmission.")
            break
        
        # 3. SUCCESS라면 이미 완료된 것 (이전 명령 등) -> 상황에 따라 처리 필요하지만 여기선 BUSY 대기
        if current_status == ROBOT_SUCCESS:
             print(f"[{robot_role}] Robot is already SUCCESS? (Maybe skipped BUSY or fast). Proceeding.")
             break
             
        time.sleep(0.2) # 5Hz

def wait_for_success(arm_node, robot_role):
    """
    로봇이 SUCCESS(3) 상태가 될 때까지 대기
    """
    print(f"[{robot_role}] Waiting for SUCCESS...")
    while True:
        current_status = None
        if robot_role == 1:
            current_status = arm_node.r1_state.get('status')
        else:
            current_status = arm_node.r2_state.get('status')
            
        if current_status == ROBOT_SUCCESS:
            print(f"[{robot_role}] Job SUCCESS!")
            break
        
        time.sleep(0.5)

def main():
    rclpy.init()
    
    # 1. 핸들러 노드 생성
    arm_node = ArmHandler()
    
    # 별도 스레드에서 spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(arm_node,), daemon=True)
    spin_thread.start()
    
    print("=== Robot Arm Factory Automation Test ===")
    time.sleep(1)

    try:
        # 0. 초기화 (Servo ON)
        print("[Init] Turning Servo ON for both robots...")
        arm_node.r1_cmd_servo(True)
        arm_node.r2_cmd_servo(True)
        time.sleep(2)
        
        # 예시: DB에서 'RECEIVED' 상태인 주문을 하나 가져왔다고 가정
        # 실제로는 DB SELECT 로직이 필요하지만, 여기서는 ID 1번 주문을 가정하고 진행
        # 가구 ID 1 (Bed) -> Command 1.4507
        test_order_id = 999 
        test_furniture_id = 1 
        
        print(f"\n=== Processing Order #{test_order_id} (Furniture {test_furniture_id}) ===")
        
        # --- [Step 1] Picking (Robot 1) ---
        print("\n[Step 1] Picking Process (Robot 1)")
        
        # 1-1. Command 생성
        pick_cmd = generate_command(test_furniture_id, action_type=1) # 1.xxxx
        if pick_cmd is None:
            print("[Error] Failed to generate command. DB Connection or Data error.")
            return

        # 1-2. DB 상태 업데이트 (Making/Picking)
        update_order_status(test_order_id, 'MAKING')

        # 1-3. 명령 전송 (Loop until BUSY)
        send_command_until_busy(arm_node, 1, pick_cmd)
        
        # 1-4. 작업 완료 대기 (Wait for SUCCESS)
        wait_for_success(arm_node, 1)
        
        # 1-5. DB 상태 업데이트 (Picked -> IN_PROGRESS for Packing)
        # Note: Schema doesn't have 'PICKED', using 'IN_PROGRESS' as intermediate
        update_order_status(test_order_id, 'IN_PROGRESS')
        print("[DB] Picking Done. Order status updated to IN_PROGRESS.")


        # --- [Step 2] Packing (Robot 2) ---
        print("\n[Step 2] Packing Process (Robot 2)")
        
        # 2-1. Command 설정 (0.0000)
        pack_cmd = 0.0000
        
        # 2-2. 명령 전송 (Loop until BUSY)
        send_command_until_busy(arm_node, 2, pack_cmd)
        
        # 2-3. 작업 완료 대기 (Wait for SUCCESS)
        wait_for_success(arm_node, 2)
        
        # 2-4. DB 상태 업데이트 (Done/Shipped)
        update_order_status(test_order_id, 'DONE')
        print("[DB] Packing Done. Order status updated to DONE.")
        
        print("\n=== Order Cycle Completed ===")

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        arm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
