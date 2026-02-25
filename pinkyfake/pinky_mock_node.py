#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import threading
import sys
import os

class PinkyMockNode(Node):
    def __init__(self, robot_id):
        node_name = f'pinky_mock_node_{robot_id}'
        super().__init__(node_name)
        
        self.robot_id = robot_id
        domain_id = os.environ.get('ROS_DOMAIN_ID', '???')
        
        self.battery_pub = self.create_publisher(Float32, '/battery/present', 10)
        # [수정] YAML 변경에 맞춘 토픽 이름 변경
        self.zone_pub = self.create_publisher(String, '/task_zone/arrived', 10)
        
        # [수정] YAML 변경에 맞춘 토픽 이름 변경 (동적 구독)
        self.arm_done_sub = self.create_subscription(
            String, '/arm_working/done', self.arm_done_callback, 10)
            
        self.battery_level = 95.0 # 초기 배터리 설정
        
        self.timer = self.create_timer(5.0, self.publish_battery)
        
        self.get_logger().info('=========================================')
        self.get_logger().info(f'🚀 Pinky Mock Node #{robot_id} started')
        self.get_logger().info(f'🌐 Current ROS_DOMAIN_ID: {domain_id}')
        self.get_logger().info('-----------------------------------------')
        self.get_logger().info('Status: [DRIVING]')
        self.get_logger().info('Commands: [1] PICK_READY, [2] PACK_READY, [q] Quit')
        self.get_logger().info('=========================================')

    def publish_battery(self):
        msg = Float32()
        # 단순히 배터리 소모 시뮬레이션 (충전 기능 없음)
        self.battery_level -= 0.05
        if self.battery_level < 0:
            self.battery_level = 0.0
                
        msg.data = round(float(self.battery_level), 2)
        self.battery_pub.publish(msg)

    def arm_done_callback(self, msg):
        self.get_logger().info(f'📥 [SIGNAL RECV] /arm_working/done: "{msg.data}"')

    def send_arrival_signal(self, status):
        msg = String()
        msg.data = status
        self.zone_pub.publish(msg)
        self.get_logger().info(f'📤 [SIGNAL SENT] /task_zone/arrived: "{status}"')

def main():
    if len(sys.argv) < 2:
        print("\n❌ Error: Robot ID missing.\n")
        return

    robot_id = sys.argv[1]
    rclpy.init()
    node = PinkyMockNode(robot_id)
    
    def keyboard_listener():
        while rclpy.ok():
            try:
                key = sys.stdin.read(1)
                # [수정] 메시지 데이터 값 규격화 (대문자)
                if key == '1':
                    node.send_arrival_signal('PICK_READY')
                elif key == '2':
                    node.send_arrival_signal('PACK_READY')
                elif key == 'q':
                    rclpy.shutdown()
                    break
            except: break

    thread = threading.Thread(target=keyboard_listener, daemon=True)
    thread.start()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
