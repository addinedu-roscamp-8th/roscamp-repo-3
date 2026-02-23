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
        
        self.task_queue = deque() # DB 기반으로 변경되므로 초기 큐는 비워둠
        self.assigned_tasks = {}      
        self.packing_active = set()   
        self.packing_done = set()     

        # DB 설정
        self.db_config = {
            'host': 'localhost',
            'user': 'lovoDB',
            'password': 'LovoDB1234!',
            'database': 'factory_system'
        }

        self.get_logger().info("🚦 [V11] 교통 관제 센터 가동 (DB 연동 작업 할당 모드)")

    def get_db_connection(self):
        try:
            import mysql.connector
            return mysql.connector.connect(**self.db_config)
        except Exception as e:
            self.get_logger().error(f"DB 연결 실패: {e}")
            return None

    def get_edge_key(self, wp_a, wp_b):
        wps = sorted([wp_a, wp_b])
        return f"{wps[0]}-{wps[1]}"

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
                
                if state_num == 4:
                    self.free_all_locks_for_robot(robot_id)
                
                if state_num == 3 and robot_id in self.assigned_tasks:
                    del self.assigned_tasks[robot_id] 
                
                if state_num == 5 and robot_id in self.packing_done:
                    self.packing_done.remove(robot_id) 
                
                if state_num == 6:
                    # 복귀 시작 시 DB에서 할당된 주문 정보 삭제
                    conn = self.get_db_connection()
                    if conn:
                        try:
                            cursor = conn.cursor()
                            cursor.execute("UPDATE robot_pinky SET current_order_id = NULL WHERE robot_role = %s", (robot_id.upper(),))
                            conn.commit()
                            conn.close()
                        except Exception as e:
                            self.get_logger().error(f"주문 ID 초기화 DB 에러: {e}")

                self.respond_custom(f"{robot_id}|REPORT_ACK|{state_num}")

            elif action == "RELEASE_SLOT":
                slot_num = int(parts[2])
                if self.charging_slots.get(slot_num) == robot_id:
                    self.charging_slots[slot_num] = None
                    self.get_logger().info(f"🔋 {robot_id} 물리적 이탈 확인 완료. 슬롯 {slot_num} 비움.")
                self.respond_custom(f"{robot_id}|RELEASE_SLOT_ACK|{slot_num}")

            elif action == "REQ_TASK":
                self.handle_task_dispatch(robot_id)
            elif action == "CHECK_PACKING":
                self.handle_check_packing(robot_id)
            elif action == "PACKING_START":
                self.handle_packing_process(robot_id)
            elif action == "CHARGING_REQ":
                self.handle_charging_request(robot_id)
            elif action == "CHECK_ADVANCE":
                current_slot = int(parts[2])
                self.handle_check_advance(robot_id, current_slot)

        except Exception as e:
            self.get_logger().error(f"Msg Error: {e}")


    # 로롯이 물고 있는 모든 맵 노드 점유권 일괄 압수 (WP3 반납 문제 해결)
    def free_all_locks_for_robot(self, robot_id):
        edges_to_del = [e for e, r in self.locked_edges.items() if r == robot_id]
        nodes_to_del = [n for n, r in self.locked_nodes.items() if r == robot_id]
        
        for e in edges_to_del: 
            del self.locked_edges[e]
        for n in nodes_to_del: 
            del self.locked_nodes[n]
            self.get_logger().info(f"🔓 [노드 안전 반납] {robot_id}가 점유중이던 {n} 해제 완료")
            
        for q in self.waiting_queues.values():
            if robot_id in q: q.remove(robot_id)

    def handle_task_dispatch(self, robot_id):
        # 이미 할당된 작업이 있으면 재전송
        if robot_id in self.assigned_tasks:
            wp = self.assigned_tasks[robot_id]
            self.respond_custom(f"{robot_id}|DISPATCH|{wp}")
            return

        conn = self.get_db_connection()
        if not conn: return

        wp = None
        try:
            cursor = conn.cursor(dictionary=True)
            # 1. 'RECEIVED' 상태인 주문 조회
            cursor.execute("""
                SELECT order_id, picking_command 
                FROM orders 
                WHERE status = 'RECEIVED' 
                ORDER BY ordered_at ASC 
                LIMIT 1
                FOR UPDATE
            """)
            row = cursor.fetchone()

            if row:
                order_id = row['order_id']
                cmd_val = row['picking_command']
                
                # 2. X값에 따른 WP 매핑 (1->WP4, 2->WP8, 3->WP6)
                x_type = int(cmd_val)
                mapping = {1: 'WP4', 2: 'WP8', 3: 'WP6'}
                wp = mapping.get(x_type, 'WP4') # 기본값 WP4

                # 3. 상태 업데이트 (MAKING) 및 로봇에게 주문 ID 할당
                cursor.execute("UPDATE orders SET status = 'MAKING' WHERE order_id = %s", (order_id,))
                cursor.execute("UPDATE robot_pinky SET current_order_id = %s WHERE robot_role = %s", (order_id, robot_id.upper()))
                conn.commit()
                
                self.assigned_tasks[robot_id] = wp
                self.get_logger().info(f"📋 [내부작업할당] {robot_id} -> {wp} (주문:{order_id}, 커맨드:{cmd_val})")
            else:
                conn.commit()
                # 대기 중인 주문이 없으면 기본 장소로 보내거나 대기
                self.get_logger().info(f"⏳ {robot_id}: 할당할 주문이 없습니다.")
                return

        except Exception as e:
            if conn: conn.rollback()
            self.get_logger().error(f"작업 할당 DB 에러: {e}")
        finally:
            if conn: conn.close()

        if wp:
            self.respond_custom(f"{robot_id}|DISPATCH|{wp}")

    def handle_packing_process(self, robot_id):
        if robot_id in self.packing_done:
            self.respond_custom(f"{robot_id}|PACKING_DONE|0")
            return
        if robot_id in self.packing_active:
            return 
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
        target_slot = current_slot + 1
        if target_slot <= 3 and self.charging_slots[target_slot] is None:
            self.charging_slots[target_slot] = robot_id
            self.charging_slots[current_slot] = None
            self.get_logger().info(f"📢 [자동정렬 승인] {robot_id}: {current_slot} -> {target_slot}")
            self.respond_custom(f"{robot_id}|ADVANCE_OK|{target_slot}")
        else:
            self.respond_custom(f"{robot_id}|STAY|{current_slot}")

    def handle_charging_request(self, robot_id):
        for slot in [3, 2, 1]:
            if self.charging_slots[slot] is None:
                self.charging_slots[slot] = robot_id
                self.respond_custom(f"{robot_id}|CHARGING_ASSIGN|{slot}")
                return
        self.respond_custom(f"{robot_id}|CHARGING_WAIT|0")

    def handle_check_packing(self, requestor_id):
        is_busy = any((rid != requestor_id and (state == 3 or state == 4)) for rid, state in self.robot_states.items())
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