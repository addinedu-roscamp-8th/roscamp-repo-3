import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int8, String
import mysql.connector
from mysql.connector import Error

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')

        # ---- [1. 로봇팔 명령 전송 및 상태 수신] ---- 
        # 로봇 1 (Picking)
        self.pub_r1_order = self.create_publisher(Float64, '/robot1/order_command', 10)
        self.sub_r1_state = self.create_subscription(Int8, '/robot1/robot_state', self.r1_callback, 10)

        # 로봇 2 (Packing)
        self.pub_r2_order = self.create_publisher(Float64, '/robot2/order_command', 10)
        self.sub_r2_state = self.create_subscription(Int8, '/robot2/robot_state', self.r2_callback, 10)

        # ---- [2. 핑키 로봇 구역 도착 신호 구독] ----
        self.create_subscription(String, '/pinky1/taskzone_arr', lambda msg: self.zone_callback('PINKY_1', msg), 10)
        self.create_subscription(String, '/pinky2/taskzone_arr', lambda msg: self.zone_callback('PINKY_2', msg), 10)
        self.create_subscription(String, '/pinky3/taskzone_arr', lambda msg: self.zone_callback('PINKY_3', msg), 10)
        
        # 핑키 상태(State) 구독 (모니터링용)
        self.create_subscription(Int8, '/pinky1/state', lambda msg: self.p_callback(msg, 0), 10)
        self.create_subscription(Int8, '/pinky2/state', lambda msg: self.p_callback(msg, 1), 10)
        self.create_subscription(Int8, '/pinky3/state', lambda msg: self.p_callback(msg, 2), 10)

        # ---- [3. 설정 및 데이터] ----
        self.db_config = {
            'host': 'localhost',
            'user': 'lovoDB',
            'password': 'LovoDB1234!',
            'database': 'factory_system'
        }
        
        self.p_states = [0, 0, 0]   # 핑키 1, 2, 3 정적 상태
        self.r_states = [1, 1]     # 로봇 1, 2 상태 (1: IDLE)
        
        self.get_logger().info(" 🦾 [메인 서버] 로봇팔 통합 제어 노드 가동 (Event-Driven Mode)")

    # =============== DB 연동 함수 ===============
    def get_db_connection(self):
        try:
            return mysql.connector.connect(**self.db_config)
        except Error as e:
            self.get_logger().error(f"DB 연결 실패: {e}")
            return None

    # =============== Callback 함수 ===============
    def p_callback(self, msg, idx):
        self.p_states[idx] = msg.data

    def r1_callback(self, msg):
        self.r_states[0] = msg.data

    def r2_callback(self, msg):
        self.r_states[1] = msg.data 

    def zone_callback(self, pinky_role, msg):
        """핑키로부터 구역 도착 신호를 받았을 때 수행하는 핵심 로직"""
        event = msg.data.lower()
        self.get_logger().info(f"🚩 [구역 감지] {pinky_role} -> {event} 도착")

        conn = self.get_db_connection()
        if not conn: return

        try:
            cursor = conn.cursor(dictionary=True)
            
            # 1. 해당 핑키가 지금 어떤 주문을 가지고 있는지 확인
            cursor.execute("SELECT current_order_id FROM robot_pinky WHERE robot_role = %s", (pinky_role,))
            robot_row = cursor.fetchone()
            
            if not robot_row or not robot_row['current_order_id']:
                self.get_logger().warn(f"⚠ {pinky_role}에 할당된 주문 정보가 없습니다.")
                return

            order_id = robot_row['current_order_id']

            # 2. 주문 정보에서 피킹/패킹 커맨드 조회
            cursor.execute("SELECT picking_command, packing_command FROM orders WHERE order_id = %s", (order_id,))
            order_row = cursor.fetchone()

            if not order_row:
                self.get_logger().error(f"❌ 주문 {order_id}의 커맨드 정보를 찾을 수 없습니다.")
                return

            # 3. 신호 종류에 따른 로봇팔 명령 전송
            pub_msg = Float64()
            
            if "picking" in event:
                if self.r_states[0] == 1: # Picking 팔이 IDLE일 때만
                    pub_msg.data = float(order_row['picking_command'])
                    self.pub_r1_order.publish(pub_msg)
                    self.get_logger().info(f"🦾 [Picking Arm] 명령 전송: {pub_msg.data} (주문: {order_id})")
                else:
                    self.get_logger().warn("🚨 Picking 로봇이 바쁩니다. 명령을 생략하거나 재시도 로직이 필요합니다.")

            elif "packing" in event:
                if self.r_states[1] == 1: # Packing 팔이 IDLE일 때만
                    pub_msg.data = float(order_row['packing_command'])
                    self.pub_r2_order.publish(pub_msg)
                    self.get_logger().info(f"🦾 [Packing Arm] 명령 전송: {pub_msg.data} (주문: {order_id})")
                else:
                    self.get_logger().warn("🚨 Packing 로봇이 바쁩니다.")

        except Error as e:
            self.get_logger().error(f"DB 처리 중 에러: {e}")
        finally:
            if conn.is_connected():
                cursor.close()
                conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()