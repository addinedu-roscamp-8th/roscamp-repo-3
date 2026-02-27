#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int8, Float32
from collections import deque
import time
import threading


class AdvancedTrafficManager(Node):
    def __init__(self):
        super().__init__('advanced_traffic_manager')
        
        # ---- [1. 기본 설정 및 데이터 저장소] ----
        self.robots = ['pinky1', 'pinky2', 'pinky3']
        self.robot_states = {}    # 핑키 상태
        self.robot_batteries = {} # 배터리
        self.arm_states = [1, 1]  # 로봇팔 상태 (0: robot1, 1: robot2)

        
        # 자원 관리 (교통 제어)
        self.locked_edges = {}
        self.locked_nodes = {}
        self.waiting_queues = {}
        self.charging_slots = {1: None, 2: None, 3: "pinky1"}
        
        # 작업 관리
        self.task_queue = deque(['WP4', 'WP8', 'WP6']) 
        self.assigned_tasks = {}      
        self.active_pinkies = {
            'picking': None, # 현재 피킹 구역에서 작업 중인 핑키 명
            'packing': None  # 현재 패킹 구역에서 작업 중인 핑키 명
        }

        # ---- [2. ROS2 퍼블리셔 & 구독자 설정] ----
        
        # 로봇팔 제어용
        self.pub_r1_order = self.create_publisher(Float64, '/robot1/order_command', 10)
        self.sub_r1_state = self.create_subscription(Int8, '/robot1/robot_state', 
                                                       lambda msg: self.arm_state_callback(0, msg), 10)
        
        self.pub_r2_order = self.create_publisher(Float64, '/robot2/order_command', 10)
        self.sub_r2_state = self.create_subscription(Int8, '/robot2/robot_state', 
                                                       lambda msg: self.arm_state_callback(1, msg), 10)


        # 핑키 개별 토픽 구독 (Remapped 토픽들)
        for robot in self.robots:
            # 트래픽 요청
            self.create_subscription(String, f'/{robot}/traffic/request', 
                                     lambda msg, r=robot: self.handle_traffic_request(msg, r), 10)
            # 도착 알림 (Pick/Pack Ready)
            self.create_subscription(String, f'/{robot}/task_zone/arrived', 
                                     lambda msg, r=robot: self.handle_task_zone(msg, r), 10)
            # 배터리 정보
            self.create_subscription(Float32, f'/{robot}/battery/present', 
                                     lambda msg, r=robot: self.handle_battery(msg, r), 10)

        # 트래픽 응답 및 핑키 명령 하달용 퍼블리셔
        self.traffic_pub =self.create_publisher(String, '/traffic/response', 10)
    
        self.done_pubs = {
            'pinky1': self.create_publisher(String, '/pinky1/arm_working/done', 10),
            'pinky2': self.create_publisher(String, '/pinky2/arm_working/done', 10),
            'pinky3': self.create_publisher(String, '/pinky3/arm_working/done', 10)
        }

        self.get_logger().info("🚀 [통합 관제 서버] 교통 제어 + 로봇팔 협업 시스템 가동")

    # =========================================================
    # [A] 로봇팔 협업 로직 (robotpinky 연동)
    # =========================================================
    
    def handle_task_zone(self, msg, robot_id):
        """핑키가 구역에 도착했을 때 로봇팔에게 명령을 내림"""
        event = msg.data.upper()
        
        pub_msg = Float64()
        
        if "PICK_READY" in event:
            # 이미 피킹 완료 상태(3: SHIPPING)라면 지연된 중복 신호 무시
            if self.robot_states.get(robot_id) == 3:
                return
            # 중복 명령 방지: 이미 해당 로봇이 피킹 구역에서 작업 중으로 등록되어 있다면 무시
            if self.active_pinkies['picking'] == robot_id:
                return
                
            self.get_logger().info(f"🚩 [도착 알림] {robot_id} -> {event}")
            self.active_pinkies['picking'] = robot_id
            
            if self.arm_states[0] == 1: # IDLE
                pub_msg.data = 2.4675
                self.pub_r1_order.publish(pub_msg)
                self.get_logger().info(f"🦾 [Picking Arm] 명령 전송: {pub_msg.data}")
            else:
                self.get_logger().warn("⚠️ Picking Arm이 작업 중입니다.")

        elif "PACK_READY" in event:
            # 이미 패킹 완료 상태(5: RETURN)라면 지연된 중복 신호 무시
            if self.robot_states.get(robot_id) == 5:
                return
            # 중복 명령 방지: 이미 해당 로봇이 패킹 구역에서 작업 중으로 등록되어 있다면 무시
            if self.active_pinkies['packing'] == robot_id:
                return

            self.get_logger().info(f"🚩 [도착 알림] {robot_id} -> {event}")
            self.active_pinkies['packing'] = robot_id
            
            if self.arm_states[1] == 1: # IDLE
                pub_msg.data = 0.4675
                self.pub_r2_order.publish(pub_msg)
                self.get_logger().info(f"🦾 [Packing Arm] 명령 전송: {pub_msg.data}")
            else:
                self.get_logger().warn("⚠️ Packing Arm이 작업 중입니다.")

    def arm_state_callback(self, arm_idx, msg):
        """로봇팔의 작업 상태(SUCCESS=3)를 감지하여 핑키에게 완료 신호를 보냄"""
        prev_state = self.arm_states[arm_idx]
        current_state = msg.data
        self.arm_states[arm_idx] = current_state

        if prev_state != 3 and current_state == 3:
            done_msg = String()
            if arm_idx == 0: # Picking
                robot_id = self.active_pinkies['picking']
                if robot_id:
                    done_msg.data = "PICK_DONE"
                    self.done_pubs[robot_id].publish(done_msg)
                    self.robot_states[robot_id] = 3  # 출고중
                    self.get_logger().info(f"✅ [Picking 완료] {robot_id}에게 PICK_DONE 전송")
                    self.active_pinkies['picking'] = None
            else: # Packing
                robot_id = self.active_pinkies['packing']
                if robot_id:
                    done_msg.data = "PACK_DONE"
                    self.done_pubs[robot_id].publish(done_msg)
                    self.robot_states[robot_id] = 5  # 복귀중
                    self.get_logger().info(f"✅ [Packing 완료] {robot_id}에게 PACK_DONE 전송")
                    self.get_logger().info(f"🚚 {robot_id}: 모든 공정 완료. 이제 '복귀중' 상태로 전환됩니다.")
                    self.active_pinkies['packing'] = None



    # =========================================================
    # [B] 교통 제어 로직 (fullll.py 베이스)
    # =========================================================
    
    def handle_traffic_request(self, msg, robot_id):
        try:
            parts = msg.data.split('|')
            action = parts[0]
            
            if action in ["REQUEST", "RELEASE"]:
                self.handle_navigation_traffic(action, robot_id, parts[2], parts[3])

            elif action == "PICK_START":
                # 로봇이 픽업지에 도착해 팔 동작을 요청함 (상태 3: 픽업대기)

                self.robot_states[robot_id] = 3
                self.handle_task_zone(String(data="PICK_READY"), robot_id)

            elif action == "PACKING_START":
                # 로봇이 패킹지에 도착해 팔 동작을 요청함 (상태 5: 패킹대기)

                self.robot_states[robot_id] = 4
                self.handle_task_zone(String(data="PACK_READY"), robot_id)

            elif action == "REPORT_STATE":
                state_num = int(parts[2])
                self.robot_states[robot_id] = state_num
                if state_num == 4: self.free_all_locks_for_robot(robot_id)
                if state_num == 3:
                     if robot_id in self.assigned_tasks: del self.assigned_tasks[robot_id] 
                self.respond_traffic(f"{robot_id}|REPORT_ACK|{state_num}")

            elif action == "REQ_TASK":
                self.handle_task_dispatch(robot_id)
            elif action == "CHECK_PACKING":
                self.handle_check_packing(robot_id)
            elif action == "CHARGING_REQ":
                self.handle_charging_request(robot_id)

        except Exception as e:
            self.get_logger().error(f"Traffic Msg Error: {e}")

    def handle_navigation_traffic(self, action, robot_id, current_wp, next_wp):
        edge_key = self.get_edge_key(current_wp, next_wp)
        if action == "REQUEST":
            edge_free = (edge_key not in self.locked_edges) or (self.locked_edges[edge_key] == robot_id)
            node_free = (next_wp not in self.locked_nodes) or (self.locked_nodes[next_wp] == robot_id)
            
            if edge_free and node_free:
                self.locked_edges[edge_key] = robot_id
                self.locked_nodes[next_wp] = robot_id
                self.respond_traffic(f"{robot_id}|APPROVED|{current_wp}|{next_wp}")
            else:
                self.respond_traffic(f"{robot_id}|WAIT|{current_wp}|{next_wp}")
                
        elif action == "RELEASE":
            if edge_key in self.locked_edges and self.locked_edges[edge_key] == robot_id:
                del self.locked_edges[edge_key]
            if current_wp in self.locked_nodes and self.locked_nodes[current_wp] == robot_id:
                del self.locked_nodes[current_wp]
            self.respond_traffic(f"{robot_id}|RELEASE_ACK|{current_wp}|{next_wp}")

    def handle_task_dispatch(self, robot_id):
        if robot_id in self.assigned_tasks:
            wp = self.assigned_tasks[robot_id]
        else:
            if not self.task_queue: self.task_queue.extend(['WP4', 'WP8', 'WP6']) 
            wp = self.task_queue.popleft()
            self.assigned_tasks[robot_id] = wp
        self.respond_traffic(f"{robot_id}|DISPATCH|{wp}")

    def handle_charging_request(self, robot_id):
        for slot in [3, 2, 1]:
            if self.charging_slots[slot] is None:
                self.charging_slots[slot] = robot_id
                # [수정] 성급한 상태 변경 제거: 로봇이 실제 도착 후 REPORT_STATE(6)를 보낼 때까지 기다림
                self.respond_traffic(f"{robot_id}|CHARGING_ASSIGN|{slot}")
                return
        self.respond_traffic(f"{robot_id}|CHARGING_WAIT|0")

    def handle_check_packing(self, requestor_id):
        is_busy = any((rid != requestor_id and (state == 3 or state == 4)) for rid, state in self.robot_states.items())
        if is_busy: self.respond_traffic(f"{requestor_id}|PACKING_AREA_BUSY|0")
        else: self.respond_traffic(f"{requestor_id}|PACKING_AREA_FREE|0")

    def handle_battery(self, msg, robot_id):
        try:
            self.robot_batteries[robot_id] = float(msg.data)
        except: pass

    def free_all_locks_for_robot(self, robot_id):
        for e in [e for e, r in self.locked_edges.items() if r == robot_id]: del self.locked_edges[e]
        for n in [n for n, r in self.locked_nodes.items() if r == robot_id]: del self.locked_nodes[n]

    def get_edge_key(self, wp_a, wp_b):
        return "-".join(sorted([wp_a, wp_b]))

    def respond_traffic(self, raw_string):
        msg = String()
        msg.data = raw_string
        self.traffic_pub.publish(msg)

def main():
    rclpy.init()
    node = AdvancedTrafficManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()