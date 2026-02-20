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
        
        # 1. 자원 관리 (기존)
        self.locked_edges = {}
        self.locked_nodes = {}
        self.waiting_queues = {}
        self.charging_slots = {1: None, 2: None, 3: None}

        # 2. [NEW] 작업 큐 (라운드 로빈 방식으로 픽업 장소 할당 예시)
        self.task_queue = deque(['WP4', 'WP6', 'WP8']) 

        self.get_logger().info("🚦 [State Machine 지원] 교통 관제 센터 가동")

    def get_edge_key(self, wp_a, wp_b):
        wps = sorted([wp_a, wp_b])
        return f"{wps[0]}-{wps[1]}"

    def handle_request(self, msg):
        try:
            parts = msg.data.split('|')
            action = parts[0]
            robot_id = parts[1]
            
            # --- [A] 주행 관제 (REQUEST/RELEASE) ---
            if action in ["REQUEST", "RELEASE"]:
                self.handle_navigation_traffic(action, robot_id, parts[2], parts[3])

            # --- [B] 충전소 관련 (CHARGING) ---
            elif action == "CHARGING_REQ":
                self.handle_charging_request(robot_id)

            # --- [C] 작업 할당 요청 (State 1 -> 2) ---
            elif action == "REQ_TASK":
                self.handle_task_dispatch(robot_id)

            # --- [D] 패킹 작업 요청 (State 3 -> 4 -> 5) ---
            elif action == "PACKING_START":
                self.handle_packing_process(robot_id)

        except Exception as e:
            self.get_logger().error(f"Msg Error: {e}")

    # ----------------------------------------------------------------
    # [NEW] 작업 할당 및 패킹 공정 로직
    # ----------------------------------------------------------------
    def handle_task_dispatch(self, robot_id):
        # 작업 큐에서 하나 꺼내서 할당 (없으면 WP8 기본값)
        pickup_wp = self.task_queue[0]
        self.task_queue.rotate(-1) # 큐 회전 (다음 로봇은 다른 곳)
        
        self.get_logger().info(f"📋 [작업할당] {robot_id} -> 픽업지: {pickup_wp}")
        self.respond_custom(f"{robot_id}|DISPATCH|{pickup_wp}")

    def handle_packing_process(self, robot_id):
        self.get_logger().info(f"📦 [패킹시작] {robot_id} 작업 대기 중...")
        
        # 비동기로 처리 (메인 스레드 차단 방지)
        def packing_job():
            time.sleep(3.0) # 패킹 작업 시간 시뮬레이션
            self.get_logger().info(f"✅ [패킹완료] {robot_id} 작업 종료 신호 전송")
            self.respond_custom(f"{robot_id}|PACKING_DONE|0")
            
        threading.Thread(target=packing_job).start()

    # ----------------------------------------------------------------
    # [기존] 주행 및 충전소 로직
    # ----------------------------------------------------------------
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
                self.respond(robot_id, "APPROVED", current_wp, next_wp)
            else:
                if target_node not in self.waiting_queues: self.waiting_queues[target_node] = deque()
                if robot_id not in self.waiting_queues[target_node]: self.waiting_queues[target_node].append(robot_id)
                self.respond(robot_id, "WAIT", current_wp, next_wp)
                
        elif action == "RELEASE":
            if edge_key in self.locked_edges and self.locked_edges[edge_key] == robot_id:
                del self.locked_edges[edge_key]
            if prev_node in self.locked_nodes and self.locked_nodes[prev_node] == robot_id:
                if prev_node != target_node: del self.locked_nodes[prev_node]

    def handle_charging_request(self, robot_id):
        assigned_slot = None
        for slot in [3, 2, 1]:
            if self.charging_slots[slot] is None:
                assigned_slot = slot
                break
        
        if assigned_slot:
            self.charging_slots[assigned_slot] = robot_id
            self.respond_custom(f"{robot_id}|CHARGING_ASSIGN|{assigned_slot}")
            
            # 만약 내가 1번이나 2번에 들어갔는데 내 앞자리(2번이나 3번)가 비어있다면?
            # -> 로직 단순화를 위해 일단 배정만 하고, '앞자리 비움' 이벤트 발생 시 process_slot_move_up 호출
        else:
            self.respond_custom(f"{robot_id}|CHARGING_WAIT|0")

    def process_slot_move_up(self, freed_slot_idx):
        check_slot = freed_slot_idx - 1
        if check_slot >= 1 and self.charging_slots[check_slot] is not None:
            robot_to_move = self.charging_slots[check_slot]
            self.charging_slots[freed_slot_idx] = robot_to_move
            self.charging_slots[check_slot] = None
            self.get_logger().info(f"📢 [자동정렬] {robot_to_move}: {check_slot}->{freed_slot_idx} 이동")
            self.respond_custom(f"{robot_to_move}|MOVE_UP|1")
            self.process_slot_move_up(check_slot)

    def respond(self, robot_id, status, wp1, wp2):
        self.respond_custom(f"{robot_id}|{status}|{wp1}|{wp2}")

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