import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import os
import datetime
from db_manager import db

class MultiTrafficMonitor(Node):
    def __init__(self):
        super().__init__('multi_traffic_monitor')
        
        self.robots = ['pinky1', 'pinky2', 'pinky3']
        # 'state'를 추가하여 로봇의 현재 상태 코드를 추적합니다 (기본값 1: 대기중)
        self.data = {robot: {'req': "None", 'res': "None", 'batt': 0.0, 'req_cnt': 0, 'state': 1} for robot in self.robots}

        for robot in self.robots:
            # 트래픽 요청 구독
            self.create_subscription(String, f'/{robot}/traffic/request', 
                lambda msg, r=robot: self.req_callback(msg, r), 10)
            # 트래픽 응답 구독
            self.create_subscription(String, f'/{robot}/traffic/response', 
                lambda msg, r=robot: self.res_callback(msg, r), 10)
            # 배터리 구독
            self.create_subscription(Float32, f'/{robot}/battery/present', 
                lambda msg, r=robot: self.batt_callback(msg, r), 10)
            # [수정] YAML 변경에 맞춰 토픽 이름 수정 (/taskzone_arr -> /task_zone/arrived)
            self.create_subscription(String, f'/{robot}/task_zone/arrived', 
                lambda msg, r=robot: self.zone_callback(msg, r), 10)

        self.timer = self.create_timer(0.5, self.display_status)
        self.get_logger().info("📡 [Pinky Monitor] Listening to traffic and zone signals...")

    def get_db_connection(self):
        return db.get_connection()

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
            
            # 상태 텍스트 매핑 (로그용)
            state_text_map = {0:"충전중", 1:"대기중", 2:"이동중", 3:"픽업대기", 4:"출고중", 5:"패킹대기", 6:"복귀중", 7:"에러"}
            state_text = state_text_map.get(d['state'], "알수없음")

            # 1. robot_pinky 테이블 업데이트 (현재 상태 상시 반영)
            query = """
                INSERT INTO robot_pinky (robot_role, battery_percent, action_state)
                VALUES (%s, %s, %s)
                ON DUPLICATE KEY UPDATE
                    battery_percent=VALUES(battery_percent),
                    action_state=VALUES(action_state),
                    last_seen_at=CURRENT_TIMESTAMP
            """
            cursor.execute(query, (role, d['batt'], d['state']))
            
            # 2. 히스토리 로그 추가
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
        
        # 메시지에서 상태 코드 추출 시도 (ex: REQ|P1|2 -> 상태 2로 변경)
        if '|' in msg.data:
            try:
                parts = msg.data.split('|')
                if len(parts) >= 3:
                    self.data[robot]['state'] = int(parts[-1])
            except:
                pass
        self.update_db(robot)

    def res_callback(self, msg, robot):
        self.data[robot]['res'] = msg.data

    def batt_callback(self, msg, robot):
        self.data[robot]['batt'] = msg.data
        self.update_db(robot)

    def zone_callback(self, msg, robot):
        """도착 신호(Ready)를 수신하여 로봇 상태를 변경합니다."""
        # [수정] 대소문자 구분 없이 확인하기 위해 lower() 사용
        status = msg.data.lower()
        if 'pick_ready' in status or 'pickingready' in status:
            self.data[robot]['state'] = 3 # 픽업 대기 중
            self.get_logger().info(f"✅ {robot} arrived at PICKING zone.")
        elif 'pack_ready' in status or 'packingready' in status:
            self.data[robot]['state'] = 5 # 패킹 대기 중
            self.get_logger().info(f"✅ {robot} arrived at PACKING zone.")
        
        self.update_db(robot)

    def display_status(self):
        os.system('clear')
        now = datetime.datetime.now().strftime('%H:%M:%S')
        print(f"==============================================================")
        print(f"   [Pinky Multi-Monitor (Domain 59)] - {now}")
        print(f"==============================================================")
        
        state_text_map = {0:"충전중", 1:"대기중", 2:"이동중", 3:"픽업대기", 4:"출고중", 5:"패킹대기", 6:"복귀중", 7:"에러"}
        
        for robot in self.robots:
            d = self.data[robot]
            batt_level = int(d['batt'])
            gauge = "■" * (batt_level // 10) + "□" * (10 - (batt_level // 10))
            state_str = state_text_map.get(d['state'], "Unknown")
            
            print(f" [{robot.upper()}] Batt: {d['batt']:.2f}% [{gauge}] | State: {state_str}")
            print(f"  └ Last Req: {d['req']} (Total: {d['req_cnt']})")
            print(f"  └ Last Res: {d['res']}")
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