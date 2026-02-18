#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque

class AdvancedTrafficManager(Node):
    def __init__(self):
        super().__init__('advanced_traffic_manager')
        
        self.sub = self.create_subscription(String, '/traffic/request', self.handle_request, 10)
        self.pub = self.create_publisher(String, '/traffic/response', 10)
        
        # 1. 주행 구역 자원 (기존 유지)
        self.locked_edges = {}
        self.locked_nodes = {}
        self.waiting_queues = {}

        # 2. [NEW] 충전소 슬롯 관리 (1:입구쪽 ~ 3:가장 안쪽)
        # Value: 로봇ID 또는 None
        self.charging_slots = {1: None, 2: None, 3: None}
        
        self.get_logger().info("🚦 [통합] 교통 관제 센터 (주행 + 충전소 대기열 관리)")

    def get_edge_key(self, wp_a, wp_b):
        wps = sorted([wp_a, wp_b])
        return f"{wps[0]}-{wps[1]}"

    def handle_request(self, msg):
        try:
            parts = msg.data.split('|')
            action = parts[0]
            robot_id = parts[1]
            
            # --- [A] 일반 주행 요청/반납 처리 ---
            if action in ["REQUEST", "RELEASE"]:
                current_wp = parts[2]
                next_wp = parts[3]
                self.handle_navigation_traffic(action, robot_id, current_wp, next_wp)

            # --- [B] 충전소 관련 처리 ---
            elif action == "CHARGING_REQ":
                self.handle_charging_request(robot_id)
                
            elif action == "CHARGING_LEAVE":
                # 로봇이 충전소(또는 대기열)를 완전히 떠날 때 (예: 다시 일하러 감)
                # 현재는 시나리오상 '복귀'까지이므로 이 기능은 로봇이 충전 완료 후 
                # 다시 맵으로 나갈 때 사용됨. 여기서는 슬롯 비우기 로직으로 사용.
                # 하지만 로봇이 '앞으로 당겨지는 것'은 내부적으로 처리됨.
                pass 

        except Exception as e:
            self.get_logger().error(f"메시지 처리 오류: {e}")

    # =================================================================
    # [A] 일반 주행 관제 로직 (기존 코드 유지)
    # =================================================================
    def handle_navigation_traffic(self, action, robot_id, current_wp, next_wp):
        edge_key = self.get_edge_key(current_wp, next_wp)
        target_node = next_wp
        prev_node = current_wp
        
        if action == "REQUEST":
            edge_free = (edge_key not in self.locked_edges) or (self.locked_edges[edge_key] == robot_id)
            node_free = (target_node not in self.locked_nodes) or (self.locked_nodes[target_node] == robot_id)
            
            queue_ok = True
            if target_node in self.waiting_queues and self.waiting_queues[target_node]:
                if self.waiting_queues[target_node][0] != robot_id:
                    queue_ok = False
            
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
                if prev_node != target_node:
                    del self.locked_nodes[prev_node]

    # =================================================================
    # [B] 충전소 슬롯 배정 및 자동 정렬(Shift-Up) 로직
    # =================================================================
    def handle_charging_request(self, robot_id):
        # 1. 가장 깊은 슬롯(3번)부터 비어있는지 확인
        assigned_slot = None
        for slot in [3, 2, 1]:
            if self.charging_slots[slot] is None:
                assigned_slot = slot
                break # 가장 안쪽 자리를 찾으면 중단
        
        if assigned_slot:
            self.charging_slots[assigned_slot] = robot_id
            self.get_logger().info(f"🔋 {robot_id} -> 충전소 {assigned_slot}번 슬롯 배정")
            # 응답: CHARGING_ASSIGN|슬롯번호
            self.respond_custom(f"{robot_id}|CHARGING_ASSIGN|{assigned_slot}")
        else:
            # 꽉 찼을 경우 대기 (이번 시나리오엔 없지만 예외처리)
            self.respond_custom(f"{robot_id}|CHARGING_WAIT|0")

    def process_slot_move_up(self, freed_slot_idx):
        """
        특정 슬롯이 비었을 때, 그 뒤(입구 쪽)에 있는 로봇들을 앞으로 당김
        예: 3번이 비면 -> 2번 로봇을 3번으로 이동 -> 2번이 비면 -> 1번 로봇을 2번으로 이동
        """
        # 비워진 슬롯보다 입구 쪽(작은 번호)에 있는 슬롯들을 검사
        # 예: freed_slot=3 이면 check_slot=2
        check_slot = freed_slot_idx - 1
        
        if check_slot >= 1 and self.charging_slots[check_slot] is not None:
            # 이동할 로봇 식별
            robot_to_move = self.charging_slots[check_slot]
            
            # 장부 업데이트 (이동)
            self.charging_slots[freed_slot_idx] = robot_to_move
            self.charging_slots[check_slot] = None
            
            self.get_logger().info(f"📢 [자동정렬] {robot_to_move}: {check_slot}번 -> {freed_slot_idx}번으로 이동 명령")
            
            # 로봇에게 이동 명령 전송
            self.respond_custom(f"{robot_to_move}|MOVE_UP|1") # 1칸 이동해라
            
            # 재귀적으로 연쇄 이동 체크 (방금 비워진 check_slot에 대해 다시 수행)
            self.process_slot_move_up(check_slot)

    def respond(self, robot_id, status, wp1, wp2):
        msg = String()
        msg.data = f"{robot_id}|{status}|{wp1}|{wp2}"
        self.pub.publish(msg)

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