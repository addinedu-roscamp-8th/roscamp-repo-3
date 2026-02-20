#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String, Float64
from collections import deque
import time
import threading

class FactoryIntegratedControl(Node):
    def __init__(self):
        super().__init__('factory_integrated_control_node')

        # --- [1. 통신 설정] ---
        
        for i in range(1, 4):
            # 핑키 상태(1~7) 수신
            self.create_subscription(Int8, f'/pinky{i}/state', lambda msg, idx=i-1: self.p_state_cb(msg, idx), 10)
            # 핑키 명령 하달 (이동 승인 및 모드 제어)
            setattr(self, f'pub_p{i}_res', self.create_publisher(String, f'/pinky{i}/response', 10))

        # 로봇팔 상태 및 명령
        self.pub_r1_order = self.create_publisher(Float64, '/robot1/order_command', 10)
        self.sub_r1_state = self.create_subscription(Int8, '/robot1/robot_state', self.r1_state_cb, 10)
        self.pub_r2_order = self.create_publisher(Float64, '/robot2/order_command', 10)
        self.sub_r2_state = self.create_subscription(Int8, '/robot2/robot_state', self.r2_state_cb, 10)

        # --- [2. 내부 변수] ---
        self.p_states = [0, 0, 0]
        self.r_states = [1, 1]  # 1: IDLE, 3: SUCCESS
        self.r1_target = None
        self.r2_target = None

        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info("=== 라인트레이싱 패킹존 통합 관제 시작 ===")

    def p_state_cb(self, msg, idx): self.p_states[idx] = msg.data
    def r1_state_cb(self, msg): self.r_states[0] = msg.data
    def r2_state_cb(self, msg): self.r_states[1] = msg.data

    # --- [3. 핵심 제어 루프] ---
    def control_loop(self):
        # (1) 로봇 1 (Picking) - 피킹존 대기
        if self.r_states[0] == 1:
            for i in range(3):
                if self.p_states[i] == 4: # 4: 픽업대기중
                    self.r1_target = i
                    self.pub_r1_order.publish(Float64(data=1.4507))
                    self.get_logger().info(f"로봇 1: 핑키 {i+1} 피킹 시작")
                    break

        # (2) 로봇 1 완료 -> WP 3으로 출발 (Nav2 구간의 끝으로 이동)
        if self.r_states[0] == 3 and self.r1_target is not None:
            idx = self.r1_target
            getattr(self, f'pub_p{idx+1}_res').publish(String(data="MOVE_TO_WP3"))
            self.r1_target = None

        # (3) 라인트레이싱 중 패킹존 도착 감지
        # 핑키가 라인트레이싱(5) 중 패킹 센서를 만나면 상태를 6으로 바꿨다고 가정
        if self.r_states[1] == 1:
            for i in range(3):
                if self.p_states[i] == 6: # 6: 패킹대기중 (라인트레이싱 중간 정지)
                    self.r2_target = i
                    self.pub_r2_order.publish(Float64(data=0.0000))
                    self.get_logger().info(f"로봇 2: 핑키 {i+1} 패킹 시작 (라인 중단)")
                    break

        # (4) 로봇 2 완료 -> 라인트레이싱 재개 또는 복귀
        if self.r_states[1] == 3 and self.r2_target is not None:
            idx = self.r2_target
            self.get_logger().info(f"패킹 완료! 핑키 {idx+1} 다시 출발 및 복귀 지시")
            # 라인 끝까지 가게 하거나 바로 복귀(7) 시킴
            getattr(self, f'pub_p{idx+1}_res').publish(String(data="RESUME_AND_GO_HOME"))
            self.r2_target = None

def main(args=None):
    rclpy.init(args=args)
    node = FactoryIntegratedControl()
    try: rclpy.spin(node)
    except: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()