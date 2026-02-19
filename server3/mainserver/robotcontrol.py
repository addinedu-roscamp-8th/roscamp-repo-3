#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int8, String
from collections import deque


# ================================[교통 관리 클래스] ===================================
class TrafficManager:
    def __init__(self):

        # 현재 WP 작업 중인 핑키
        self.occupancy = {}   #{'WP_ID': 'pinky_id'}
        #대기 핑키들의 명단 
        self.waiting_queues = {} #{'WP_ID': deque([pinky_id, ...])}


    # ======== 경로 검토 함수 ==========
    def check_path_and_reserve(self, pinky_id, path_list):

        # S1. 경로상에 다른 핑키가 점유 중인 WP가 있는지 확인
        for wp_num in path_list:
            wp_id = f"WP_{int(wp_num)}"
            current_owner = self.occupancy.get(wp_id)
            # 다른 핑키가 점유 중 -> WAIT return
            if current_owner is not None and current_owner != pinky_id:
                return "WAIT", wp_id
        
        # S2. 든 경로가 비어있다면, 경로상의 모든 WP를 이 핑키 이름으로 예약
        for wp_num in path_list:
            wp_id = f"WP_{int(wp_num)}"
            self.occupancy[wp_id] = pinky_id
        return "OK", None

    
     # ======== 구역에서 벗어날 때 점유 해제 ==========
    def handle_clear(self, pinky_id, cleared_wp_id):
        if self.occupancy.get(cleared_wp_id) == pinky_id:
            self.occupancy[cleared_wp_id] = None
        
# ============================== [Core] 통합 제어 노드 ==============================
class IntegratedControl(Node):
    def __init__(self):
        super().__init__('integrated_control_node')
        self.tm = TrafficManager()



        # ======== Robotarm ===========
        # ---- Robot 1 (Picking / Domain 60) ---
        self.pub_r1_order = self.create_publisher(Float64, '/robot1/order_command', 10)
        self.sub_r1_state = self.create_subscription(Int8, '/robot1/robot_state', self.r1_state_cb, 10)

        # --- Robot 2 (Packing / Domain 61) ---
        self.pub_r2_order = self.create_publisher(Float64, '/robot2/order_command', 10)
        self.sub_r2_state = self.create_subs

        # =========== pinky ===========
        # --- [1. Pinky state  ] ---
        self.sub_p1_state = self.create_subscription(Int8, '/pinky1/state', self.p1_state_cb, 10)
        self.sub_p2_state = self.create_subscription(Int8, '/pinky2/state', self.p2_state_cb, 10)
        self.sub_p3_state = self.create_subscription(Int8, '/pinky3/state', self.p3_state_cb, 10)
        
        # --- [2.Pinky 경로 수신] ---
        self.sub_p1_path = self.create_subscription(String, '/pinky1/path', self.p1_path_cb, 10)
        self.sub_p2_path = self.create_subscription(String, '/pinky2/path', self.p2_path_cb, 10)
        self.sub_p3_path = self.create_subscription(String, '/pinky3/path', self.p3_path_cb, 10)
        
        # --- [3. pinky 경로 응답] ---
        self.pub_p1_res = self.create_publisher(String, '/pinky1/path_response', 10)
        self.pub_p2_res = self.create_publisher(String, '/pinky2/path_response', 10)
        self.pub_p3_res = self.create_publisher(String, '/pinky3/path_response', 10)

        # --- [4. 상태 데이터 변수] ---]
        self.p_states = [0,0,0]   # 핑키 1, 2, 3 상태
        self.r_states = [0,0]     # 로봇arm 1, 2 상태
        self.current_partners = [None, None] # [로봇1과 작업중인 핑키, 로봇2와 작업중인 핑키]

        self.timer = self.create_timer(0.5, self.monitor_and_control)
        self.get_logger().info(" === 로봇 2대 & 핑키 3대 통합 관제소 가동 ===")


    # ============== Callbacks ===============
    def p1_state_cb(self, msg): self.p_states[0] = msg.data
    def p2_state_cb(self, msg): self.p_states[1] = msg.data
    def p3_state_cb(self, msg): self.p_states[2] = msg.data
    def r1_state_cb(self, msg): self.r_states[0] = msg.data
    def r2_state_cb(self, msg): self.r_states[1] = msg.data

    # --- 경로 요청 처리 (String 버전) ---
    def p1_path_cb(self, msg): self.process_request(0, msg.data, self.pub_p1_res)
    def p2_path_cb(self, msg): self.process_request(1, msg.data, self.pub_p2_res)
    def p3_path_cb(self, msg): self.process_request(2, msg.data, self.pub_p3_res)

    def process_request(self, p_idx, path_str, response_pub):
        pinky_id = f"pinky{p_idx+1}"
        # 핑키 상태가 2(READY)일 때만 경로 검토
        if self.p_states[p_idx] == 2:
            result, blocked_wp = self.tm.check_path_and_reserve(pinky_id, path_str)
            res_msg = String()
            res_msg.data = "GO" if result == "GO" else f"WAIT:{blocked_wp}"
            response_pub.publish(res_msg)
            if result == "GO":
                self.get_logger().info(f"[{pinky_id}] 경로 승인: {path_str}")
        else:
            # 준비 안 된 핑키는 거절
            msg = String(); msg.data = "REJECT:NOT_READY"
            response_pub.publish(msg)

            

    # =========== [Core] 관제 루프 robotarm & pinky ===========
    def monitor_and_control(self):
        # 어떤 핑키라도 피킹 존에 도착했는지 확인 
        any_pinky_ready = any(self.pinky_at_picking_zone)
        
        # [시나리오] 로봇 1 (Picking) 제어 로직
        if self.r1_state == 1: # IDLE 상태
            if any_pinky_ready:
                target_command = 1.4507
                arrived_idx = self.pinky_at_picking_zone.index(True) + 1
                
                
                self.get_logger().info(f"Pinky {arrived_idx} 도착 확인 명령 {target_command} 전송")
                self.send_picking_order(target_command)
            else:
                # 핑키 대기 중일 때 로그 (필요 시 주석 해제)
                # self.get_logger().info("Robot 1 IDLE - Waiting for Pinky...")
                pass

        elif self.r1_state == 3: # SUCCESS 상태
            self.get_logger().info("Robot 1 작업 성공! SUCCESS 보고 중")
            # TODO: 핑키에게 출발(Release) 신호를 보내는 로직을 여기에 구현
            pass

        # [시나리오] 로봇 2 (Packing) 제어 로직
        if self.r2_state == 1:
            # self.get_logger().info("Robot 2 IDLE")
            pass
        elif self.r2_state == 3:
            self.get_logger().info("Robot 2 SUCCESS")
            pass

    # ---[Command] robotarm Commands ---
    def send_picking_order(self, order_command: float):
        msg = Float64()
        msg.data = order_command
        self.pub_r1_order.publish(msg)
        self.get_logger().info(f"Published Picking Order: {order_command}")

    def send_packing_order(self, order_command: float):
        msg = Float64()
        msg.data = order_command
        self.pub_r2_order.publish(msg)
        self.get_logger().info(f"Published Packing Order: {order_command}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()