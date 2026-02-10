#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json
import threading
import time
from std_msgs.msg import String

# --- Configuration ---
MAIN_SERVER_URL = "http://192.168.0.30:5000/api"
STATUS_EP = f"{MAIN_SERVER_URL}/status"
COMMAND_EP = f"{MAIN_SERVER_URL}/mission/command"

class BridgeManager(Node):
    def __init__(self):
        super().__init__('bridge_manager')
        
        # Internal state to hold the latest statuses from MultiPinkyController
        self.latest_robot_statuses = {}
        self.lock = threading.Lock()
        
        # Subscribe to unified status topic
        self.status_sub = self.create_subscription(
            String,
            '/robot_statuses',
            self.status_callback,
            10
        )
        
        # Publisher to trigger mission
        self.start_pub = self.create_publisher(String, '/start_mission', 10)
        
        # Last reported state for ARRIVED_EVENT detection
        self.last_known_locations = {
            'pinky1': 'CHARGING_STATION',
            'pinky2': 'CHARGING_STATION',
            'pinky3': 'CHARGING_STATION'
        }
        
        self.get_logger().info('BridgeManager initialized.')
        
        # Start background threads
        threading.Thread(target=self.status_reporter_loop, daemon=True).start()
        threading.Thread(target=self.order_polling_loop, daemon=True).start()

    def status_callback(self, msg):
        with self.lock:
            self.latest_robot_statuses = json.loads(msg.data)
            
            # Detect ARRIVED_EVENT for Picking Wait Zone
            for name, status in self.latest_robot_statuses.items():
                curr_loc = status.get('location', '')
                state = status.get('state', '')
                
                if curr_loc == 'PICKING_WAIT_ZONE' and state == 'AT_PICKING_ZONE' and self.last_known_locations[name] != 'PICKING_WAIT_ZONE':
                    self.send_arrived_event(name)
                
                self.last_known_locations[name] = curr_loc

    def send_arrived_event(self, robot_id):
        self.get_logger().info(f'Sending ARRIVED_EVENT for {robot_id}')
        payload = {
            'robot_id': robot_id,
            'event': 'ARRIVED_EVENT',
            'location': 'PICKING_WAIT_ZONE'
        }
        try:
            requests.post(STATUS_EP, json=payload, timeout=0.5)
        except Exception as e:
            self.get_logger().error(f'Failed to send ARRIVED_EVENT: {e}')

    def status_reporter_loop(self):
        """Thread 1: Report status every 1 second"""
        while rclpy.ok():
            with self.lock:
                if self.latest_robot_statuses:
                    try:
                        # Reformat or send as is
                        requests.post(STATUS_EP, json=self.latest_robot_statuses, timeout=1.0)
                        # self.get_logger().debug('Reported status to main server.')
                    except Exception as e:
                        self.get_logger().error(f'Status report failed: {e}')
            time.sleep(1.0)

    def order_polling_loop(self):
        """Thread 2: Poll for mission commands every 2 seconds"""
        while rclpy.ok():
            try:
                response = requests.get(COMMAND_EP, timeout=1.0)
                if response.status_code == 200:
                    data = response.json()
                    if data.get('command') == 'START':
                        self.get_logger().info('Received START command from main server!')
                        msg = String()
                        msg.data = 'START'
                        self.start_pub.publish(msg)
            except Exception as e:
                # self.get_logger().error(f'Command polling failed: {e}')
                pass 
            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    bridge = BridgeManager()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
