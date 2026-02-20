import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32  # 1. 여기서 가져와야 합니다.
import os
import datetime

class MultiTrafficMonitor(Node):
    def __init__(self):
        super().__init__('multi_traffic_monitor')
        
        # YAML의 remap 설정에 맞춰 언더바(_)를 뺐습니다. (pinky1, pinky2...)
        self.robots = ['pinky1', 'pinky2', 'pinky3']
        self.data = {robot: {'req': "None", 'res': "None", 'batt': 0.0, 'req_cnt': 0} for robot in self.robots}

        for robot in self.robots:
            self.create_subscription(String, f'/{robot}/traffic/request', 
                lambda msg, r=robot: self.req_callback(msg, r), 10)
            self.create_subscription(String, f'/{robot}/traffic/response', 
                lambda msg, r=robot: self.res_callback(msg, r), 10)
            self.create_subscription(Float32, f'/{robot}/battery/present', 
                lambda msg, r=robot: self.batt_callback(msg, r), 10)

        self.timer = self.create_timer(0.5, self.display_status)

    def req_callback(self, msg, robot):
        self.data[robot]['req'] = msg.data
        self.data[robot]['req_cnt'] += 1

    def res_callback(self, msg, robot):
        self.data[robot]['res'] = msg.data

    def batt_callback(self, msg, robot):
        self.data[robot]['batt'] = msg.data

    def display_status(self):
        os.system('clear')
        now = datetime.datetime.now().strftime('%H:%M:%S')
        print(f"==============================================================")
        print(f"   [Pinky Multi-Monitor (Domain 59)] - {now}")
        print(f"==============================================================")
        
        for robot in self.robots:
            d = self.data[robot]
            
            # 2. 게이지를 시각적으로 만드는 부분입니다.
            batt_level = int(d['batt'])
            # 10%당 칸 하나씩 (■), 나머지는 빈칸 (□)
            gauge = "■" * (batt_level // 10) + "□" * (10 - (batt_level // 10))
            
            print(f" [{robot.upper()}] Batt: {d['batt']:.1f}% [{gauge}]")
            print(f"  └ Traffic Req ({d['req_cnt']}): {d['req']}")
            print(f"  └ Last Response: {d['res']}")
            print(f"--------------------------------------------------------------")
            
        print(f"  Press Ctrl+C to exit monitor.")

def main(args=None):
    rclpy.init(args=args)
    monitor = MultiTrafficMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()