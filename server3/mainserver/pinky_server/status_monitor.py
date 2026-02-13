#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Float32
import os
import time

class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')
        
        self.robot_data = {
            'pinky1': {'state': 'Unknown', 'battery': 0.0, 'x': 0.0, 'y': 0.0},
            'pinky2': {'state': 'Unknown', 'battery': 0.0, 'x': 0.0, 'y': 0.0},
            'pinky3': {'state': 'Unknown', 'battery': 0.0, 'x': 0.0, 'y': 0.0}
        }

        # Subscribers for all robots
        for i in range(1, 4):
            r_name = f'pinky{i}'
            ns = f'/pinky_{i}'
            
            # Using closures to pass robot_name to callbacks
            self.create_subscription(PoseWithCovarianceStamped, f'{ns}/pose', self.make_pose_callback(r_name), 10)
            self.create_subscription(String, f'{ns}/status', self.make_status_callback(r_name), 10)
            self.create_subscription(Float32, f'{ns}/battery', self.make_battery_callback(r_name), 10)

        # Print timer (2Hz)
        self.timer = self.create_timer(0.5, self.draw_dashboard)
        print("Starting Status Monitor Dashboard...")

    def make_pose_callback(self, name):
        return lambda msg: self.update_pose(name, msg)
    
    def make_status_callback(self, name):
        return lambda msg: self.update_status(name, msg)
        
    def make_battery_callback(self, name):
        return lambda msg: self.update_battery(name, msg)

    def update_pose(self, name, msg):
        self.robot_data[name]['x'] = msg.pose.pose.position.x
        self.robot_data[name]['y'] = msg.pose.pose.position.y

    def update_status(self, name, msg):
        self.robot_data[name]['state'] = msg.data

    def update_battery(self, name, msg):
        self.robot_data[name]['battery'] = float(msg.data)

    def draw_dashboard(self):
        # Clear terminal
        os.system('clear')
        print("="*60)
        print(f"   PINKY ROBOT REAL-TIME MONITOR (Domain 59)   {time.strftime('%H:%M:%S')}")
        print("="*60)
        print(f"{'Robot':<10} | {'Status':<15} | {'Battery':<10} | {'Position (x,y)':<20}")
        print("-"*60)
        
        for name in sorted(self.robot_data.keys()):
            d = self.robot_data[name]
            pos = f"({d['x']:>6.2f}, {d['y']:>6.2f})"
            batt = f"{d['battery']:>5.1f}%"
            print(f"{name:<10} | {d['state']:<15} | {batt:<10} | {pos:<20}")
        
        print("-"*60)
        print(" Press Ctrl+C to exit monitor ")

def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
