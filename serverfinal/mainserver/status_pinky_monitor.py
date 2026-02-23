import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import os
import datetime
import mysql.connector

# DB 설정
DB_CONFIG = {
    'host': 'localhost',
    'user': 'lovoDB',
    'password': 'LovoDB1234!',
    'database': 'factory_system'
}

class MultiTrafficMonitor(Node):
    def __init__(self):
        super().__init__('multi_traffic_monitor')
        
        
        self.robots = ['pinky1', 'pinky2', 'pinky3']
        self.data = {robot: {'req': "None", 'res': "None", 'batt': 0.0, 'req_cnt': 0} for robot in self.robots}

        for robot in self.robots:
            self.create_subscription(String, f'/{robot}/traffic/request', 
                lambda msg, r=robot: self.req_callback(msg, r), 10)
            self.create_subscription(String, f'/{robot}/traffic/response', 
                lambda msg, r=robot: self.res_callback(msg, r), 10)
            self.create_subscription(Float32, f'/{robot}/battery/present', 
                lambda msg, r=robot: self.batt_callback(msg, r), 10)

        self.timer = self.create_timer(0.5, self.display_status)

    def get_db_connection(self):
        try:
            return mysql.connector.connect(**DB_CONFIG)
        except Exception as e:
            self.get_logger().error(f"DB 연결 실패: {e}")
            return None

    def update_db(self, robot):
        conn = self.get_db_connection()
        if not conn:
            return
        
        try:
            cursor = conn.cursor()
            d = self.data[robot]
            role = robot.upper() # pinky1 -> PINKY_1
            if 'PINKY' in role and '_' not in role:
                role = role.replace('PINKY', 'PINKY_')
            
            # Traffic Request 메시지에서 상태 코드 추출 (ex: REQ|P1|1 -> 1)
            state_val = 1 # 기본 대기 상태
            if d['req'] != "None" and '|' in d['req']:
                try:
                    parts = d['req'].split('|')
                    if len(parts) >= 3:
                        state_val = int(parts[-1])
                except:
                    pass
            
            # 상태 텍스트 매핑
            state_text_map = {0:"충전중", 1:"대기중", 2:"이동중", 3:"픽업대기", 4:"출고중", 5:"패킹대기", 6:"복귀중", 7:"에러"}
            state_text = state_text_map.get(state_val, "대기중")

            # 1. 현재 상태 업데이트 (Overwrite)
            query = """
                INSERT INTO robot_pinky (robot_role, battery_percent, action_state)
                VALUES (%s, %s, %s)
                ON DUPLICATE KEY UPDATE
                    battery_percent=VALUES(battery_percent),
                    action_state=VALUES(action_state)
            """
            cursor.execute(query, (role, d['batt'], state_val))
            
            # 2. 히스토리 로그 추가 (New record)
            log_query = """
                INSERT INTO robot_state_history (robot_role, state_text, battery_level)
                VALUES (%s, %s, %s)
            """
            cursor.execute(log_query, (role, state_text, d['batt']))
            
            conn.commit()
            cursor.close()
            conn.close()
        except Exception as e:
            self.get_logger().error(f"DB 업데이트 에러 ({robot}): {e}")

    def req_callback(self, msg, robot):
        self.data[robot]['req'] = msg.data
        self.data[robot]['req_cnt'] += 1
        self.update_db(robot)

    def res_callback(self, msg, robot):
        self.data[robot]['res'] = msg.data

    def batt_callback(self, msg, robot):
        self.data[robot]['batt'] = msg.data
        self.update_db(robot)

    def display_status(self):
        os.system('clear')
        now = datetime.datetime.now().strftime('%H:%M:%S')
        print(f"==============================================================")
        print(f"   [Pinky Multi-Monitor (Domain 59)] - {now}")
        print(f"==============================================================")
        
        for robot in self.robots:
            d = self.data[robot]
            
            # 2. 게이지를 시각적으로 만드는 부분입니다.
            batt_level = int(d['batt'])
            # 10%당 칸 하나씩 (■), 나머지는 빈칸 (□)
            gauge = "■" * (batt_level // 10) + "□" * (10 - (batt_level // 10))
            
            print(f" [{robot.upper()}] Batt: {d['batt']:.1f}% [{gauge}]")
            print(f"  └ Traffic Req ({d['req_cnt']}): {d['req']}")
            print(f"  └ Last Response: {d['res']}")
            print(f"--------------------------------------------------------------")
            
        print(f"  Press Ctrl+C to exit monitor.")

def main(args=None):
    rclpy.init(args=args)
    monitor = MultiTrafficMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()