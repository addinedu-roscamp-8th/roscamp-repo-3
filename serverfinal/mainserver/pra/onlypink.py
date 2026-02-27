#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
import time
import threading
import httpx  # API 연동을 위한 라이브러리

class AdvancedTrafficManager(Node):
    def __init__(self):
        super().__init__('advanced_traffic_manager')
        
        # 1. ROS2 인터페이스 설정
        self.sub = self.create_subscription(String, '/traffic/request', self.handle_request, 10)
        self.pub = self.create_publisher(String, '/traffic/response', 10)
        
        # 외부(GUI/API)에서 수동으로 '작업 완료' 신호를 줄 때 사용하는 토픽 추가
        self.complete_sub = self.create_subscription(String, '/task/manual_complete', self.handle_manual_complete, 10)
        
        # 로봇별 개별 작업 할당 퍼블리셔 (필요 시 유지)
        self.task_pubs = {
            'pinky1': self.create_publisher(String, '/pinky1/taskassign', 10),
            'pinky2': self.create_publisher(String, '/pinky2/taskassign', 10),
            'pinky3': self.create_publisher(String, '/pinky3/taskassign', 10)
        }
        
        # 2. 로컬 상태 관리 (교통 제어용)
        self.locked_edges = {}
        self.locked_nodes = {}
        self.waiting_queues = {}
        self.charging_slots = {1: "pinky3", 2: "pinky2", 3: "pinky1"}
        self.robot_states = {}
        self.assigned_tasks = {}      
        
        # 3. API 설정
        self.api_base_url = "http://localhost:5000" # app.py 주소
        
        self.get_logger().info("🚦 [V12] 교통 관제 센터 가동 (REST API 연동 모드 - No Robot Arm)")

    def call_api(self, method, endpoint, data=None):
        """app.py API를 호출하는 공통 메서드"""
        try:
            url = f"{self.api_base_url}{endpoint}"
            with httpx.Client(timeout=2.0) as client:
                if method == 'GET':
                    response = client.get(url)
                elif method == 'POST':
                    response = client.post(url, json=data)
                
                if response and response.status_code in [200, 201]:
                    return response.json()
        except Exception as e:
            self.get_logger().error(f"API Connection Failed: {e}")
        return None

    def handle_request(self, msg):
        try:
            parts = msg.data.split('|')
            action = parts[0]
            robot_id = parts[1]
            
            if action in ["REQUEST", "RELEASE"]:
                self.handle_navigation_traffic(action, robot_id, parts[2], parts[3])

            elif action == "REPORT_STATE":
                state_num = int(parts[2])
                self.robot_states[robot_id] = state_num
                
                # API로 로봇 상태 실시간 업데이트 보냄
                self.call_api('POST', '/api/status', {
                    robot_id: {"state": state_num, "battery": 100.0}
                })
                
                if state_num == 4: 
                    self.free_all_locks_for_robot(robot_id)
                
                self.respond_custom(f"{robot_id}|REPORT_ACK|{state_num}")

            elif action == "REQ_TASK":
                self.handle_task_dispatch(robot_id)

            elif action == "PICK_START":
                self.get_logger().info(f"📍 {robot_id}: 피킹 시작 (5초 후 자동 완료)")
                threading.Timer(5.0, lambda: self.respond_custom(f"{robot_id}|PICK_DONE|0")).start()

            elif action == "PACKING_START":
                self.get_logger().info(f"📍 {robot_id}: 패킹 시작 (5초 후 자동 완료)")
                threading.Timer(5.0, lambda: self.respond_custom(f"{robot_id}|PACKING_DONE|0")).start()

            elif action == "CHARGING_REQ":
                self.handle_charging_request(robot_id)

        except Exception as e:
            self.get_logger().error(f"Msg Error: {e}")

    def handle_task_dispatch(self, robot_id):
        """DB 직접 접근 대신 app.py의 assign-order API를 호출합니다."""
        res = self.call_api('POST', f'/api/robots/pinky/{robot_id}/assign-order')
        
        if res and res.get('status') == 'success':
            order_id = res['order_id']
            cmd_res = self.call_api('GET', f'/api/orders/{order_id}/command')
            if cmd_res:
                cmd_val = cmd_res['picking_command']
                x_type = int(float(cmd_val))
                mapping = {1: 'WP4', 2: 'WP8', 3: 'WP6'}
                wp = mapping.get(x_type, 'WP4')
                
                self.assigned_tasks[robot_id] = wp
                self.get_logger().info(f"📋 [API 작업할당] {robot_id} -> {wp} (주문: {order_id})")
                
                self.respond_custom(f"{robot_id}|DISPATCH|{wp}")
                
                if robot_id in self.task_pubs:
                    t_msg = String()
                    t_msg.data = str(cmd_val)
                    self.task_pubs[robot_id].publish(t_msg)
        else:
            self.get_logger().info(f"⏳ {robot_id}: 할당 가능한 주문이 없습니다.")

    def handle_manual_complete(self, msg):
        try:
            robot_id, action = msg.data.split('|')
            self.get_logger().info(f"🔘 [수동 제어] {robot_id} -> {action}")
            
            if action == "PICK_DONE":
                self.respond_custom(f"{robot_id}|PICK_DONE|0")
            elif action == "PACK_DONE":
                self.respond_custom(f"{robot_id}|PACKING_DONE|0")
        except:
            pass

    def handle_navigation_traffic(self, action, robot_id, current_wp, next_wp):
        edge_key = self.get_edge_key(current_wp, next_wp)
        if action == "REQUEST":
            edge_free = (edge_key not in self.locked_edges) or (self.locked_edges[edge_key] == robot_id)
            node_free = (next_wp not in self.locked_nodes) or (self.locked_nodes[next_wp] == robot_id)
            if edge_free and node_free:
                self.locked_edges[edge_key] = robot_id
                self.locked_nodes[next_wp] = robot_id
                self.respond_custom(f"{robot_id}|APPROVED|{current_wp}|{next_wp}")
            else:
                self.respond_custom(f"{robot_id}|WAIT|{current_wp}|{next_wp}")
        elif action == "RELEASE":
            if edge_key in self.locked_edges and self.locked_edges[edge_key] == robot_id: del self.locked_edges[edge_key]
            if current_wp in self.locked_nodes and self.locked_nodes[current_wp] == robot_id: del self.locked_nodes[current_wp]
            self.respond_custom(f"{robot_id}|RELEASE_ACK|{current_wp}|{next_wp}")

    def handle_charging_request(self, robot_id):
        for slot in [3, 2, 1]:
            if self.charging_slots[slot] is None:
                self.charging_slots[slot] = robot_id
                self.respond_custom(f"{robot_id}|CHARGING_ASSIGN|{slot}")
                return
        self.respond_custom(f"{robot_id}|CHARGING_WAIT|0")

    def free_all_locks_for_robot(self, robot_id):
        for e in [e for e, r in self.locked_edges.items() if r == robot_id]: del self.locked_edges[e]
        for n in [n for n, r in self.locked_nodes.items() if r == robot_id]: del self.locked_nodes[n]

    def get_edge_key(self, wp_a, wp_b):
        return "-".join(sorted([wp_a, wp_b]))

    def respond_custom(self, raw_string):
        msg = String()
        msg.data = raw_string
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = AdvancedTrafficManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()