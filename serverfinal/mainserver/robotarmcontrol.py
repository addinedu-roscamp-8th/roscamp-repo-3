import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int8, String

class RobotArmController(Node):
    def __init__(self):
        super().__init__('only_robot_arm_controller')


        # ---- [1. robotarm order ] ---- 
        # picking 
        self.pub_r1_order = self.create_publisher(Float64, '/robot1/order_command', 10)
        self.sub_r1_state = self.create_subscription(Int8, '/robot1/robot_state', self.r1_state_cb, 10)

        # packing
        self.pub_r2_order = self.create_publisher(Float64, '/robot2/order_command', 10)
        self.sub_r2_state = self.create_subscription(Int8, '/robot2/robot_state', self.r2_state_cb, 10)

        # ---- [2. pinky 상태  수신  ]
        self.create_subscription(Int8, '/pinky1/state', lambda msg: self.p_state_cb(msg, 0), 10)
        self.create_subscription(Int8, '/pinky2/state', lambda msg: self.p_state_cb(msg, 1), 10)
        self.create_subscription(Int8, '/pinky3/state', lambda msg: self.p_state_cb(msg, 2), 10)
        

        # ---- [3. data] ----
        self.p_states = [0,0,0]   # 핑키 1, 2, 3 상태
        self.r_states = [1,1]     # 로봇arm 1, 2 상태
        
        # ---- [4. timer] ----
        self.timer = self.create_timer(0.5, self.control_loop)

        self.get_logger().info(" === 로봇 2대 통합 관제소 가동 ===")
        

    # =============== Callbacks ===============
    def p_callback(self, msg, idx):
        self.p_states[idx] = msg.data

    def r1_callback(self, msg):
        self.r_states[0] = msg.data

    def r2_callback(self, msg):
        self.r_states[1] = msg.data 


    # =============== Control Loop ===============
    def control_loop(self):
        # [picking] 로봇arm 1이 IDLE 상태이고 핑키 상태가 4일 때 가동
        if self.r_states[0] == 1:
            for i in range(3):
                if self.p_states[i] == 4:
                    self.get_logger().info(f"Pinky {i+1} 도착 확인 명령 {target_command} 전송")
                    msg = Float64()
                    msg.data = 1.4507

                    self.pub_r1_order.publish(msg)
                    # 중복 명령 방지를 위해 서버 메모리 상의 핑키 상태 초기화
                    self.p_states[i] = 0
                    break


        # [packing] 로봇arm 2가 IDLE 상태이고 핑키 상태가 6일 때 가동
        if self.r_states[1] == 1:
            for i in range(3):
                if self.p_states[i] == 6:
                    self.get_logger().info(f"Pinky {i+1} 도착 확인 포장시작")
                    #fix
                    msg = Float64(); msg.data = 0.0000  # 패킹용 명령값
                    self.pub_r2_order.publish(msg)
                    # 중복 명령 방지를 위해 서버 메모리 상의 핑키 상태 초기화
                    self.p_states[i] = 0
                    break



def main(args=None):
    rclpy.init(args=args)
    node = FactoryArmControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()