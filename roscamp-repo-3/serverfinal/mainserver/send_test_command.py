#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import time

class SimpleCommandSender(Node):
    def __init__(self):
        super().__init__('simple_command_sender')
        # 토픽 이름 확인: /robot1/order_command
        self.publisher_ = self.create_publisher(Float64, '/robot1/order_command', 10)

    def send_command(self, command_value):
        msg = Float64()
        msg.data = command_value
        
        # 연결될 때까지 잠시 대기 (퍼블리셔 등록 시간 확보)
        # 실제 환경에서는 subscriber가 없을 수도 있으니 그냥 보냄
        # 하지만 안정성을 위해 여러번 보내거나 잠시 기다림
        time.sleep(1) 
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'>>> [명령 전송] : {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCommandSender()

    # 기본값: 2.4567 (예시)
    # 실행 시 인자로 값을 주면 그 값을 사용 (예: python3 send_test_command.py 1.0000)
    command = 2.46
    if len(sys.argv) > 1:
        try:
            command = float(sys.argv[1])
        except ValueError:
            print("유효한 숫자가 아닙니다. 기본값(1.4567)을 전송합니다.")

    try:
        node.send_command(command)
        # 메시지가 전송될 시간을 주기 위해 잠시 대기 후 종료
        time.sleep(1)
        print("전송 완료.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
