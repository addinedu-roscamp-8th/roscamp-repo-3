#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AdvancedTrafficManager(Node):
    def __init__(self):
        super().__init__('advanced_traffic_manager')
        
        self.sub = self.create_subscription(String, '/traffic/request', self.handle_request, 10)
        self.pub = self.create_publisher(String, '/traffic/response', 10)
        
        # 1. 통로(Edge) 점유 장부 (Key: 'WP1-WP2', Value: 'robot1')
        self.locked_edges = {}
        
        # 2. [추가] 노드(Node) 점유 장부 (Key: 'WP2', Value: 'robot1')
        self.locked_nodes = {}
        
        self.get_logger().info("🚦 [고급] 교통 관제 센터 가동 (Edge + Node 이중 잠금)")

    def get_edge_key(self, wp_a, wp_b):
        """통로 키 생성 (순서 무관)"""
        wps = sorted([wp_a, wp_b])
        return f"{wps[0]}-{wps[1]}"

    def handle_request(self, msg):
        try:
            parts = msg.data.split('|')
            action = parts[0]
            robot_id = parts[1]
            current_wp = parts[2] # 출발지 (또는 이전 위치)
            next_wp = parts[3]    # 목적지 (또는 현재 위치)
            
            edge_key = self.get_edge_key(current_wp, next_wp)
            target_node = next_wp  # 내가 들어가려는 목적지
            prev_node = current_wp # 내가 비워줄 이전 위치
            
            if action == "REQUEST":
                # === [진입 요청 처리] ===
                # 조건 1: 통로(Edge)가 비어있거나 내가 쓰고 있는가?
                edge_free = (edge_key not in self.locked_edges) or (self.locked_edges[edge_key] == robot_id)
                
                # 조건 2: 목적지 노드(Node)가 비어있거나 내가 쓰고 있는가?
                node_free = (target_node not in self.locked_nodes) or (self.locked_nodes[target_node] == robot_id)
                
                if edge_free and node_free:
                    # [승인] 두 자원 모두 잠금 실행
                    self.locked_edges[edge_key] = robot_id
                    self.locked_nodes[target_node] = robot_id
                    
                    self.get_logger().info(f"✅ [승인] {robot_id}: {current_wp}->{next_wp} (Edge & Node 잠금)")
                    self.respond(robot_id, "APPROVED", current_wp, next_wp)
                else:
                    # [거절] 사유 로깅
                    reason = []
                    if not edge_free: reason.append(f"통로({edge_key}) 점유됨")
                    if not node_free: reason.append(f"목적지({target_node}) 점유됨")
                    
                    # self.get_logger().info(f"⛔ [대기] {robot_id}: {', '.join(reason)}")
                    self.respond(robot_id, "WAIT", current_wp, next_wp)
                    
            elif action == "RELEASE":
                # === [도착 후 반납 처리] ===
                # 상황: 로봇이 current_wp -> next_wp 이동을 마치고 next_wp에 도착함
                
                # 1. 통로(Edge) 반납
                if edge_key in self.locked_edges and self.locked_edges[edge_key] == robot_id:
                    del self.locked_edges[edge_key]
                    
                # 2. [핵심] '이전 노드(prev_node)' 반납
                # 주의: 도착한 노드(target_node/next_wp)는 반납하면 안 됨! (내가 서 있으니까)
                if prev_node in self.locked_nodes and self.locked_nodes[prev_node] == robot_id:
                    del self.locked_nodes[prev_node]
                    
                self.get_logger().info(f"🔓 [해제] {robot_id}: 통로({edge_key}) 및 이전노드({prev_node}) 반납 완료")

        except Exception as e:
            self.get_logger().error(f"메시지 처리 오류: {e}")

    def respond(self, robot_id, status, wp1, wp2):
        msg = String()
        msg.data = f"{robot_id}|{status}|{wp1}|{wp2}"
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = AdvancedTrafficManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()