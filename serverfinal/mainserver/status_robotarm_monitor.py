import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int8, Float64MultiArray
import os
import datetime
from db_manager import db

class RobotArmMonitor(Node):
    def __init__(self):
        super().__init__('robot_arm_monitor')
        
        # 관리할 로봇팔 목록
        self.arms = ['robot1', 'robot2']
        
        # 실시간 데이터 저장소
        self.data = {
            arm: {
                'state': 0,
                'last_command': 0.0,
                'tcp_pose': [0.0] * 6,
                'update_cnt': 0
            } for arm in self.arms
        }

        # 로봇 상태 코드 매핑 (0~5)
        self.state_map = {
            0: "INIT",
            1: "IDLE",
            2: "BUSY",
            3: "SUCCESS",
            4: "ERROR",
            5: "FIRE"
        }

        # 토픽 구독 설정
        for arm in self.arms:
            # 1. 로봇 상태 수신 (Int8)
            self.create_subscription(
                Int8, 
                f'/{arm}/robot_state', 
                lambda msg, a=arm: self.state_callback(msg, a), 
                10
            )
            # 2. 로봇 명령 출력 모니터링 (Float64)
            self.create_subscription(
                Float64, 
                f'/{arm}/order_command', 
                lambda msg, a=arm: self.command_callback(msg, a), 
                10
            )
            # 3. TCP 포즈 정보 수신 (Float64MultiArray: [x, y, z, r, p, y])
            self.create_subscription(
                Float64MultiArray, 
                f'/{arm}/PTP_tcp_pose', 
                lambda msg, a=arm: self.pose_callback(msg, a), 
                10
            )

        # 0.5초마다 화면 갱신
        self.timer = self.create_timer(0.5, self.display_status)
        self.get_logger().info("🦾 [Robot Arm Monitor] Monitoring started...")

    def get_db_connection(self):
        return db.get_connection()

    def update_db(self, arm):
        conn = self.get_db_connection()
        if not conn:
            return
        
        try:
            cursor = conn.cursor()
            d = self.data[arm]
            p = d['tcp_pose']
            role = arm.upper().replace('ROBOT', 'ARM_') # robot1 -> ARM_1
            state_text = self.state_map.get(d['state'], "UNKNOWN")
            
            # 1. 현재 상태 업데이트 (Overwrite)
            query = """
                INSERT INTO robot_arm (robot_role, pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw, action_state)
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                ON DUPLICATE KEY UPDATE
                    pose_x=VALUES(pose_x), pose_y=VALUES(pose_y), pose_z=VALUES(pose_z),
                    pose_roll=VALUES(pose_roll), pose_pitch=VALUES(pose_pitch), pose_yaw=VALUES(pose_yaw),
                    action_state=VALUES(action_state)
            """
            cursor.execute(query, (role, p[0], p[1], p[2], p[3], p[4], p[5], d['state']))
            
            # 2. 히스토리 로그 추가 (New record)
            log_query = """
                INSERT INTO robot_state_history (robot_role, state_text, pose_info)
                VALUES (%s, %s, %s)
            """
            pose_str = f"X:{p[0]:.1f}, Y:{p[1]:.1f}, Z:{p[2]:.1f}"
            cursor.execute(log_query, (role, state_text, pose_str))
            
            conn.commit()
            cursor.close()
            conn.close()
        except Exception as e:
            self.get_logger().error(f"DB 업데이트 에러 ({arm}): {e}")

    def state_callback(self, msg, arm):
        self.data[arm]['state'] = msg.data
        self.data[arm]['update_cnt'] += 1
        self.update_db(arm)

    def command_callback(self, msg, arm):
        self.data[arm]['last_command'] = msg.data

    def pose_callback(self, msg, arm):
        # 배열 길이에 상관없이 안전하게 복사 (최대 6개)
        for i in range(min(len(msg.data), 6)):
            self.data[arm]['tcp_pose'][i] = msg.data[i]
        self.update_db(arm)

    def display_status(self):
        os.system('clear')
        now = datetime.datetime.now().strftime('%H:%M:%S')
        print(f"==============================================================")
        print(f"   [Robot Arm Multi-Monitor (Domain 59)] - {now}")
        print(f"==============================================================")
        
        for arm in self.arms:
            d = self.data[arm]
            state_text = self.state_map.get(d['state'], f"UNKNOWN({d['state']})")
            
            # 상태에 따른 색상/기호 (IDLE: ⚪, BUSY: 🔵, SUCCESS: 🟢, ERROR/FIRE: 🔴)
            status_icon = "🔵" if d['state'] == 2 else "⚪"
            if d['state'] == 3: status_icon = "🟢"
            if d['state'] in [4, 5]: status_icon = "🔴"

            print(f" [{arm.upper()}] Status: {status_icon} {state_text}")
            print(f"  ├ Last Command: {d['last_command']:.4f}")
            
            p = d['tcp_pose']
            # p[0]:X, p[1]:Y, p[2]:Z, p[3]:Roll, p[4]:Pitch, p[5]:Yaw
            print(f"  ├ TCP Pose (XYZ): [{p[0]:8.2f}, {p[1]:8.2f}, {p[2]:8.2f}]")
            print(f"  └ TCP Pose (RPY): [{p[3]:8.2f}, {p[4]:8.2f}, {p[5]:8.2f}]")
            print(f"--------------------------------------------------------------")
            
        print(f"  Press Ctrl+C to exit monitor.")

def main(args=None):
    rclpy.init(args=args)
    monitor = RobotArmMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
