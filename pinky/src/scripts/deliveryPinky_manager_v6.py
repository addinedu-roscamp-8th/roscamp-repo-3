#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
import time
import threading

class AdvancedTrafficManager(Node):
    def __init__(self):
        super().__init__('advanced_traffic_manager')
        
        self.sub = self.create_subscription(String, '/traffic/request', self.handle_request, 10)
        self.pub = self.create_publisher(String, '/traffic/response', 10)
        
        # 자원 관리
        self.locked_edges = {}
        self.locked_nodes = {}
        self.waiting_queues = {}
        self.charging_slots = {1: "robot3", 2: "robot2", 3: "robot1"}
        self.robot_states = {}
        
        # [NEW] 통신 신뢰성을 위한 중복 방지 캐시
        self.task_queue = deque(['WP4', 'WP6', 'WP8', 'WP4', 'WP6', 'WP8']) 
        self.assigned_tasks = {}      # 로봇별 할당된 픽업지 저장 (중복 요청 방어)
        self.packing_active = set()   # 현재 패킹 스레드가 돌고 있는 로봇
        self.packing_done = set()     # 패킹이 완료된 로봇

        self.get_logger().info("🚦 [V5 신뢰성 보장] 교통 관제 센터 가동 (100% ACK 시스템)")

    def get_edge_key(self, wp_a, wp_b):
        wps = sorted([wp_a, wp_b])
        return f"{wps[0]}-{wps[1]}"

    def handle_request(self, msg):
        try:
            parts = msg.data.split('|')
            action = parts[0]
            robot_id = parts[1]
            
            # --- [A] 주행 관제 ---
            if action in ["REQUEST", "RELEASE"]:
                self.handle_navigation_traffic(action, robot_id, parts[2], parts[3])

            # --- [B] 상태 보고 ---
            elif action == "REPORT_STATE":
                state_num = int(parts[2])
                self.robot_states[robot_id] = state_num
                
                # 상태 전환 시 이전 캐시 정리
                if state_num == 3 and robot_id in self.assigned_tasks:
                    del self.assigned_tasks[robot_id] # 픽업 완료했으니 캐시 삭제
                if state_num == 5 and robot_id in self.packing_done:
                    self.packing_done.remove(robot_id) # 복귀 시작했으니 캐시 삭제
                if state_num == 2:
                    self.free_charging_slot(robot_id) # 충전소 이탈

                self.respond_custom(f"{robot_id}|REPORT_ACK|{state_num}")

            # --- [C] 작업 할당 ---
            elif action == "REQ_TASK":
                self.handle_task_dispatch(robot_id)

            # --- [D] 패킹 구역/작업 ---
            elif action == "CHECK_PACKING":
                self.handle_check_packing(robot_id)
            elif action == "PACKING_START":
                self.handle_packing_process(robot_id)

            # --- [E] 충전소 관련 ---
            elif action == "CHARGING_REQ":
                self.handle_charging_request(robot_id)
            elif action == "CHECK_ADVANCE":
                current_slot = int(parts[2])
                self.handle_check_advance(robot_id, current_slot)

        except Exception as e:
            self.get_logger().error(f"Msg Error: {e}")

    # ----------------------------------------------------------------
    # [NEW] 견고해진 서버 로직
    # ----------------------------------------------------------------
    def handle_task_dispatch(self, robot_id):
        # 이미 할당받았는데 네트워크 문제로 또 요청한 경우 (동일 목적지 재전송)
        if robot_id in self.assigned_tasks:
            wp = self.assigned_tasks[robot_id]
            self.respond_custom(f"{robot_id}|DISPATCH|{wp}")
        else:
            if not self.task_queue:
                self.task_queue.extend(['WP4', 'WP6', 'WP8']) # 큐가 비면 채움
            wp = self.task_queue.popleft()
            self.assigned_tasks[robot_id] = wp
            self.get_logger().info(f"📋 [작업할당] {robot_id} -> {wp}")
            self.respond_custom(f"{robot_id}|DISPATCH|{wp}")

    def handle_packing_process(self, robot_id):
        # 1. 이미 끝난 작업 재요청 시
        if robot_id in self.packing_done:
            self.respond_custom(f"{robot_id}|PACKING_DONE|0")
            return
        # 2. 이미 작업 중인 경우 무시 (대기)
        if robot_id in self.packing_active:
            return 

        # 3. 처음 요청인 경우
        self.packing_active.add(robot_id)
        self.get_logger().info(f"📦 [패킹진행] {robot_id} 작업 시작...")
        
        def packing_job():
            time.sleep(3.0) 
            self.packing_active.remove(robot_id)
            self.packing_done.add(robot_id)
            self.get_logger().info(f"✅ [패킹완료] {robot_id}")
            self.respond_custom(f"{robot_id}|PACKING_DONE|0")
            
        threading.Thread(target=packing_job).start()

    def handle_check_advance(self, robot_id, current_slot):
        """클라이언트가 스스로 '앞으로 가도 돼?' 물어보는 구조"""
        target_slot = current_slot + 1
        
        if target_slot <= 3 and self.charging_slots[target_slot] is None:
            # 이동 승인 및 서버 내 장부 수정
            self.charging_slots[target_slot] = robot_id
            self.charging_slots[current_slot] = None
            self.get_logger().info(f"📢 [자동정렬 승인] {robot_id}: {current_slot} -> {target_slot}")
            self.respond_custom(f"{robot_id}|ADVANCE_OK|{target_slot}")
        else:
            self.respond_custom(f"{robot_id}|STAY|{current_slot}")

    def free_charging_slot(self, robot_id):
        for slot, occupant in self.charging_slots.items():
            if occupant == robot_id:
                self.charging_slots[slot] = None
                self.get_logger().info(f"🔋 {robot_id} 이탈. 슬롯 {slot} 비움.")
                break

    def handle_charging_request(self, robot_id):
        for slot in [3, 2, 1]:
            if self.charging_slots[slot] is None:
                self.charging_slots[slot] = robot_id
                self.respond_custom(f"{robot_id}|CHARGING_ASSIGN|{slot}")
                return
        self.respond_custom(f"{robot_id}|CHARGING_WAIT|0")

    def handle_check_packing(self, requestor_id):
        is_busy = any((rid != requestor_id and state == 4) for rid, state in self.robot_states.items())
        if is_busy: self.respond_custom(f"{requestor_id}|PACKING_AREA_BUSY|0")
        else: self.respond_custom(f"{requestor_id}|PACKING_AREA_FREE|0")

    def handle_navigation_traffic(self, action, robot_id, current_wp, next_wp):
        edge_key = self.get_edge_key(current_wp, next_wp)
        target_node = next_wp
        prev_node = current_wp
        
        if action == "REQUEST":
            edge_free = (edge_key not in self.locked_edges) or (self.locked_edges[edge_key] == robot_id)
            node_free = (target_node not in self.locked_nodes) or (self.locked_nodes[target_node] == robot_id)
            queue_ok = True
            if target_node in self.waiting_queues and self.waiting_queues[target_node]:
                if self.waiting_queues[target_node][0] != robot_id: queue_ok = False
            
            if edge_free and node_free and queue_ok:
                self.locked_edges[edge_key] = robot_id
                self.locked_nodes[target_node] = robot_id
                if target_node in self.waiting_queues and robot_id in self.waiting_queues[target_node]:
                    self.waiting_queues[target_node].remove(robot_id)
                self.respond_custom(f"{robot_id}|APPROVED|{current_wp}|{next_wp}")
            else:
                if target_node not in self.waiting_queues: self.waiting_queues[target_node] = deque()
                if robot_id not in self.waiting_queues[target_node]: self.waiting_queues[target_node].append(robot_id)
                self.respond_custom(f"{robot_id}|WAIT|{current_wp}|{next_wp}")
                
        elif action == "RELEASE":
            if edge_key in self.locked_edges and self.locked_edges[edge_key] == robot_id:
                del self.locked_edges[edge_key]
            if prev_node in self.locked_nodes and self.locked_nodes[prev_node] == robot_id:
                if prev_node != target_node: del self.locked_nodes[prev_node]
            # [NEW] 해제도 ACK 응답
            self.respond_custom(f"{robot_id}|RELEASE_ACK|{current_wp}|{next_wp}")

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