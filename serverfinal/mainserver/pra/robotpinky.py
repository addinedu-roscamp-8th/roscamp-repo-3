import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int8, String
from db_manager import db

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

        # ---- [3. 전역 관리 데이터] ----
        self.r_states = [1, 1]     # 로봇 1, 2 상태 (1: IDLE)
        
        # 현재 각 구역에서 작업 중인 핑키 ID 기억 (신호 전송용)
        self.active_pinkies = {
            'picking': None, # e.g., 'PINKY_1'
            'packing': None
        }

        # ---- [4. 핑키 전용 응답 퍼블리셔 (명령 하달용)] ----
        # [수정] YAML 변경에 맞춰 토픽 경로 수정 (/pinkyX/arm_done -> /pinkyX/arm_working/done)
        self.done_pubs = {
            'PINKY_1': self.create_publisher(String, '/pinky1/arm_working/done', 10),
            'PINKY_2': self.create_publisher(String, '/pinky2/arm_working/done', 10),
            'PINKY_3': self.create_publisher(String, '/pinky3/arm_working/done', 10)
        }
        
        # ---- [5. 핑키 구역 진입 구독자 (수정: /pinkyX/task_zone/arrived)] ----
        self.create_subscription(String, '/pinky1/task_zone/arrived', lambda msg: self.zone_callback('PINKY_1', msg), 10)
        self.create_subscription(String, '/pinky2/task_zone/arrived', lambda msg: self.zone_callback('PINKY_2', msg), 10)
        self.create_subscription(String, '/pinky3/task_zone/arrived', lambda msg: self.zone_callback('PINKY_3', msg), 10)

        self.get_logger().info(" 🦾 [Main Server] Robot Arm Collaborative Controller Active (Fixed Commands Mode)")
        self.get_logger().info(" 📡 Listening for 'PICK_READY' and 'PACK_READY' on /pinkyX/task_zone/arrived")

    # =============== Callback 함수 ===============
    def r1_callback(self, msg):
        # 로봇팔 1(Picking)이 SUCCESS(3)로 바뀌는 순간 감지
        if self.r_states[0] != 3 and msg.data == 3:
            pinky_role = self.active_pinkies['picking']
            if pinky_role and pinky_role in self.done_pubs:
                done_msg = String()
                done_msg.data = "PICK_DONE"
                self.done_pubs[pinky_role].publish(done_msg)
                self.get_logger().info(f"✅ [Picking Arm] Work Complete! Sent 'PICK_DONE' to {pinky_role}")
                
                # [수정] 핑키 상태 업데이트 (3: 출고중/SHIPPING)
                db.execute_query("UPDATE robot_pinky SET action_state = 3 WHERE robot_role = %s", (pinky_role,))
                self.active_pinkies['picking'] = None 

        self.r_states[0] = msg.data

    def r2_callback(self, msg):
        # 로봇팔 2(Packing)이 SUCCESS(3)로 바뀌는 순간 감지
        if self.r_states[1] != 3 and msg.data == 3:
            pinky_role = self.active_pinkies['packing']
            if pinky_role and pinky_role in self.done_pubs:
                done_msg = String()
                done_msg.data = "PACK_DONE"
                self.done_pubs[pinky_role].publish(done_msg)
                self.get_logger().info(f"✅ [Packing Arm] Work Complete! Sent 'PACK_DONE' to {pinky_role}")
                
                # [수정] 핑키 상태 업데이트 (5: 복귀중/RETURN)
                db.execute_query("UPDATE robot_pinky SET action_state = 5 WHERE robot_role = %s", (pinky_role,))
                self.active_pinkies['packing'] = None 

        self.r_states[1] = msg.data 

    def zone_callback(self, pinky_role, msg):
        """핑키로부터 구역 도착 신호(Ready)를 받았을 때 고정 커맨드 발송 (DB 연동 안함)"""
        event = msg.data.lower()
        self.get_logger().info(f"🚩 [Zone Event] {pinky_role} reported '{msg.data}'")
        
        pub_msg = Float64()
        
        if "pick_ready" in event or "pickingready" in event:
            self.active_pinkies['picking'] = pinky_role
            if self.r_states[0] == 1: # Picking 팔이 IDLE(1)일 때만
                pub_msg.data = 2.4675
                self.pub_r1_order.publish(pub_msg)
                self.get_logger().info(f"🦾 [Picking Arm] FIXED CMD Sent: {pub_msg.data}")
            else:
                self.get_logger().warn("🚨 Picking Arm is BUSY. Command skipped.")

        elif "pack_ready" in event or "packingready" in event:
            self.active_pinkies['packing'] = pinky_role
            if self.r_states[1] == 1: # Packing 팔이 IDLE(1)일 때만
                pub_msg.data = 0.4675
                self.pub_r2_order.publish(pub_msg)
                self.get_logger().info(f"🦾 [Packing Arm] FIXED CMD Sent: {pub_msg.data}")
            else:
                self.get_logger().warn("🚨 Packing Arm is BUSY. Command skipped.")


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