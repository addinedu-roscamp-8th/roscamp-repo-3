import rclpy
from rclpy.node import Node
# DatabaseManager 클래스가 없으므로 log_work_status 함수만 가져옵니다.
from database_manager import log_work_status 
#from lovo_interfaces.msg import LovoStatus

class LovoMainServer(Node):
    def __init__(self):
        super().__init__('lovo_main_server')
        self.get_logger().info('Lovo Main Server Node가 시작되었습니다.')
        
        # 1. 서버 시작 시 DB에 로그 기록 (함수 직접 호출)
        log_work_status(0, "START", "Lovo Main Server Node가 시작되었습니다.")

        # GUI 명령을 받기 위한 Subscriber 설정
    #   self.subscription = self.create_subscription(
    #       LovoStatus,
    #       'gui_command',
    #       self.gui_command_callback,
    #       10)

    def gui_command_callback(self, msg):
        self.get_logger().info(f'GUI로부터 명령 수신: ID={msg.work_id}, Status={msg.status}')
        
        # 2. 수신된 명령을 DB에 기록
        log_work_status(msg.work_id, msg.status, f"GUI 메시지: {msg.message}")
        
        self.get_logger().info(f'메시지 내용: {msg.message}')

    def destroy_node(self):
        # 3. 노드 종료 시 로그 기록
        log_work_status(0, "SHUTDOWN", "Lovo Main Server Node가 종료되었습니다.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LovoMainServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('서버가 사용자에 의해 중단되었습니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()