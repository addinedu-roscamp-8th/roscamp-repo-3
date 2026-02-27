import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import requests
import time
import os

# ==========================================
# [설정] 명령 발송 횟수를 여기에서 정하세요
# ==========================================
REPEAT_COUNT = 1  # 예: 1번만 보내고 중단
# ==========================================

class RobotController:
    """사용자 요청 규격(self.controller.publish_goal_pose) 지원용"""
    def __init__(self, publisher, logger):
        self.publisher = publisher
        self.get_logger = lambda: logger

    def publish_goal_pose(self, pose):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in pose]
        self.publisher.publish(msg)

class FireEmergencyPublisher(Node):
    def __init__(self):
        super().__init__('fire_emergency_publisher')
        
        # 1. 로봇팔 1호기 목표 좌표 토픽 설정 (도메인 59 -> 브릿지 -> 60)
        self.pub_goal_pose = self.create_publisher(Float64MultiArray, '/robot1/PTP_goal_pose', 10)
        self.controller = RobotController(self.pub_goal_pose, self.get_logger())
        
        # 2. 메인 서버 설정
        self.main_server_ip = os.environ.get("MAIN_SERVER_IP", "localhost")
        self.coord_api_url = f"http://{self.main_server_ip}:5000/api/ai/coordinates"
        
        # 3. 사용자 지정 긴급 좌표
        self.home_pose = [-18.14, 19.73, 24.44, -7.11, -41.87, 46.12] 
        
        # 4. 상태 및 횟수 관리
        self.is_emergency_active = False
        self.sent_count = 0
        self.max_repeat = REPEAT_COUNT
        
        # 5. [중요] 시작 시점의 시퀀스 번호 기록 (과거 기록 무시용)
        self.start_seq = -1
        try:
            resp = requests.get(self.coord_api_url, timeout=0.5)
            if resp.status_code == 200:
                data = resp.json()
                if data and 'seq' in data:
                    self.start_seq = data['seq']
                    self.get_logger().info(f"⏳ [READY] Ignoring old data (seq <= {self.start_seq}). Waiting for new detection...")
        except Exception:
            self.get_logger().info("⏳ [READY] No existing data found. Waiting for new detection...")

        # 0.1초 간격으로 체크 및 실행
        self.timer = self.create_timer(0.1, self.emergency_loop)
        
        self.get_logger().info(f"📡 [MONITORING] Watching AI Data. Will repeat {self.max_repeat} times upon detection.")

    def emergency_loop(self):
        # 1. 감지 전: API 실시간 모니터링
        if not self.is_emergency_active:
            try:
                response = requests.get(self.coord_api_url, timeout=0.1)
                if response.status_code == 200:
                    data = response.json()
                    if data and 'seq' in data:
                        current_seq = data['seq']
                        # 시작 시점보다 큰(새로운) 시퀀스 번호가 들어와야 실행
                        if current_seq > self.start_seq:
                            self.get_logger().error(f"🔥 NEW DETECTION! (seq: {current_seq}) Starting to send command.")
                            self.is_emergency_active = True
            except Exception:
                pass
        
        # 2. 감지 후: 정해진 횟수만큼 반복 발송
        if self.is_emergency_active and self.sent_count < self.max_repeat:
            self.sent_count += 1
            self.controller.publish_goal_pose(self.home_pose)
            self.get_logger().info(f"🚨 [PROGRESS] Sending Pose... ({self.sent_count}/{self.max_repeat})")
            
            if self.sent_count >= self.max_repeat:
                self.get_logger().info("✅ Finished sending all commands. System standby.")

def main(args=None):
    rclpy.init(args=args, domain_id=59)
    node = FireEmergencyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
