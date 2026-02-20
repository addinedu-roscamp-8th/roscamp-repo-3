#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import mysql.connector
from mysql.connector import Error
import time

class CommandSender(Node):
    def __init__(self):
        super().__init__('command_sender')

        # Only Publisher for robot commands
        self.pub_r1_order = self.create_publisher(Float64, '/robot1/order_command', 10)

        # DB Configuration
        self.db_config = {
            'host': 'localhost',
            'user': 'lovoDB',
            'password': 'LovoDB1234!',
            'database': 'factory_system'
        }

        # Timer: Check for orders every 1 second
        self.timer = self.create_timer(1.0, self.control_loop)

        self.get_logger().info(" === [DB 연동] 단순 명령 전송기 가동 (상태 확인 안함) ===")

    def get_db_connection(self):
        try:
            conn = mysql.connector.connect(**self.db_config)
            if conn.is_connected():
                return conn
        except Error as e:
            self.get_logger().error(f"DB 연결 실패: {e}")
        return None

    def fetch_next_order(self):
        """Fetch oldest 'RECEIVED' order and generate command"""
        conn = self.get_db_connection()
        if not conn: return None, None

        command = None
        order_id = None

        try:
            cursor = conn.cursor(dictionary=True)
            conn.start_transaction()

            # Select oldest RECEIVED order
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
                
                # Generate command (1.[Frame][Leg][Wheel][Kit])
                c4 = "4" if row['top_material_id'] else "0"
                c5 = "5" if row['leg_material_id'] else "0"
                c6 = "6" if row['wheel_material_id'] else "0"
                c7 = "7" if row['kit_material_id'] else "0"
                
                command_str = f"1.{c4}{c5}{c6}{c7}"
                command = float(command_str)

                # Update status to MAKING
                update_query = "UPDATE orders SET status = 'MAKING', started_at = CURRENT_TIMESTAMP WHERE order_id = %s"
                cursor.execute(update_query, (order_id,))
                conn.commit()
                self.get_logger().info(f"주문 {order_id} 접수 -> Command 생성: {command}")
            else:
                conn.commit()
        
        except Error as e:
            conn.rollback()
            self.get_logger().error(f"DB 처리 중 에러: {e}")
        finally:
            if conn.is_connected():
                cursor.close()
                conn.close()

        return order_id, command

    def control_loop(self):
        # Simply fetch and send without checking robot state
        order_id, command = self.fetch_next_order()

        if command is not None:
            self.get_logger().info(f" >>> 명령 전송: {command} (주문번호: {order_id})")
            msg = Float64()
            msg.data = command
            self.pub_r1_order.publish(msg)
        else:
            # No orders pending
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CommandSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()