#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int8
import mysql.connector
from mysql.connector import Error
import time

class PickingRobotController(Node):
    def __init__(self):
        super().__init__('picking_robot_controller')

        # ---- [1. 로봇 1 (Picking) 통신 설정] ---- 
        # 명령 전송 (Order) 및 상태 수신 (State)
        self.pub_r1_order = self.create_publisher(Float64, '/robot1/order_command', 10)
        self.sub_r1_state = self.create_subscription(Int8, '/robot1/robot_state', self.r1_state_cb, 10)

        # ---- [2. 데이터 설정] ----
        self.r1_idle = False # 초기값 (상태 수신 전까지 대기)
        self.is_working = False # 현재 작업 중인지 확인

        # ---- [3. DB 설정] ----
        self.db_config = {
            'host': 'localhost',
            'user': 'lovoDB',
            'password': 'LovoDB1234!',
            'database': 'factory_system'
        }

        # ---- [4. 제어 루프 타이머 (1초 간격)] ----
        self.timer = self.create_timer(1.0, self.control_loop)

        self.get_logger().info(" === [DB 연동] 로봇팔 1호기 동적 명령 생성기 가동 ===")

    # =============== Callback 함수 ===============
    def r1_state_cb(self, msg):
        """로봇 1의 실제 상태 수신 (1: IDLE, 그 외: BUSY)"""
        # 1: IDLE (준비 완료), 2: BUSY (작업 중), 3: SUCCESS (완료)
        if msg.data == 1:
            self.r1_idle = True
        else:
            self.r1_idle = False

        # 만약 로봇이 다시 IDLE로 돌아왔다면 작업 완료로 간주하고 플래그 초기화
        if self.r1_idle and self.is_working:
            self.is_working = False
            self.get_logger().info("로봇 1: 작업 완료 및 대기 상태 복귀")

    # =============== DB 관련 함수 ===============
    def get_db_connection(self):
        try:
            conn = mysql.connector.connect(**self.db_config)
            if conn.is_connected():
                return conn
        except Error as e:
            self.get_logger().error(f"DB 연결 실패: {e}")
        return None

    def fetch_next_order(self):
        """DB에서 'RECEIVED' 상태인 가장 오래된 주문 하나를 가져와서 처리"""
        conn = self.get_db_connection()
        if not conn: return None, None

        command = None
        order_id = None

        try:
            cursor = conn.cursor(dictionary=True)
            
            # 1. 트랜잭션 시작
            conn.start_transaction()

            # 2. 대기 중인 주문 조회 (오래된 순)
            # 가구의 자재 구성 정보(BOM)도 함께 조인해서 가져옴
            query = """
                SELECT o.order_id, o.status, 
                       f.top_material_id, f.leg_material_id, f.wheel_material_id, f.kit_material_id
                FROM orders o
                JOIN furniture f ON o.furniture_id = f.furniture_id
                WHERE o.status = 'RECEIVED'
                ORDER BY o.ordered_at ASC
                LIMIT 1
                FOR UPDATE
            """
            cursor.execute(query)
            row = cursor.fetchone()

            if row:
                order_id = row['order_id']
                
                # 3. 커맨드 생성 로직 (1.[프레임][다리][바퀴][작업킷])
                # 재료가 있으면 해당 ID가 아니라, 자릿수 위치에 맞는 숫자(4,5,6,7)를 부여하는 방식
                # 예: top이 존재하면 두번째 자리에 4, 없으면 0.
                c4 = "4" if row['top_material_id'] else "0"
                c5 = "5" if row['leg_material_id'] else "0"
                c6 = "6" if row['wheel_material_id'] else "0"
                c7 = "7" if row['kit_material_id'] else "0"
                
                command_str = f"1.{c4}{c5}{c6}{c7}"
                command = float(command_str)

                # 4. 주문 상태를 'MAKING'으로 변경
                update_query = "UPDATE orders SET status = 'MAKING', started_at = CURRENT_TIMESTAMP WHERE order_id = %s"
                cursor.execute(update_query, (order_id,))
                
                conn.commit()
                self.get_logger().info(f"주문 {order_id} 접수 -> Command 생성: {command}")
            else:
                conn.commit() # 조회만 했어도 커밋해서 트랜잭션 종료
        
        except Error as e:
            conn.rollback()
            self.get_logger().error(f"DB 처리 중 에러: {e}")
        finally:
            if conn.is_connected():
                cursor.close()
                conn.close()

        return order_id, command

    # =============== 메인 제어 루프 ===============
    def control_loop(self):
        # 1. 로봇이 준비되었고(IDLE), 2. 현재 작업 중이 아니라면
        if self.r1_idle and not self.is_working:
            
            # DB에서 새 주문 확인
            order_id, command = self.fetch_next_order()

            if command is not None:
                self.get_logger().info(f" >>> 로봇 1 명령 전송: {command} (주문번호: {order_id})")
                
                msg = Float64()
                msg.data = command
                self.pub_r1_order.publish(msg)

                self.is_working = True
            else:
                # 대기 중인 주문이 없음
                pass

def main(args=None):
    rclpy.init(args=args)
    node = PickingRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()