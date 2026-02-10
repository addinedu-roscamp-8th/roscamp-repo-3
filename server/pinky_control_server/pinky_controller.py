#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import json
from std_msgs.msg import String

# --- Coordinate Configuration (x, y, orientation_z, orientation_w) ---
# NOTE: Update these with accurate values from Rviz /clicked_point
WAYPOINTS = {
    'CHARGING_STATION': {'x': 0.0, 'y': 0.0, 'oz': 0.0, 'ow': 1.0},
    'PICKUP_1': {'x': 1.0, 'y': 0.0, 'oz': 0.0, 'ow': 1.0},
    'PICKUP_2': {'x': 2.0, 'y': 0.0, 'oz': 0.0, 'ow': 1.0},
    'PICKUP_3': {'x': 3.0, 'y': 0.0, 'oz': 0.0, 'ow': 1.0},
    'PICKING_WAIT_ZONE': {'x': 4.0, 'y': 0.0, 'oz': 0.0, 'ow': 1.0},
}

# --- State Constants ---
STATES = {
    'IDLE': 'IDLE',
    'MOVING_TO_PICKUP_1': 'MOVING_TO_PICKUP_1',
    'AT_PICKUP_1': 'AT_PICKUP_1',
    'MOVING_TO_PICKUP_2': 'MOVING_TO_PICKUP_2',
    'AT_PICKUP_2': 'AT_PICKUP_2',
    'MOVING_TO_PICKUP_3': 'MOVING_TO_PICKUP_3',
    'AT_PICKUP_3': 'AT_PICKUP_3',
    'MOVING_TO_PICKING_ZONE': 'MOVING_TO_PICKING_ZONE',
    'AT_PICKING_ZONE': 'AT_PICKING_ZONE',
    'MOVING_TO_CHARGING': 'MOVING_TO_CHARGING',
    'AT_CHARGING': 'AT_CHARGING',
    'ERROR': 'ERROR'
}

import threading
import time

class MultiPinkyController(Node):
    def __init__(self):
        super().__init__('multi_pinky_controller')
        self.robots = ['pinky1', 'pinky2', 'pinky3']
        self.navigators = {name: BasicNavigator(namespace=name) for name in self.robots}
        
        # State tracking: {robot_name: {'state': STATE, 'location': LOC, 'battery': 0.0}}
        self.robot_statuses = {name: {'state': STATES['IDLE'], 'location': 'CHARGING_STATION', 'battery': 100.0} for name in self.robots}
        
        # Publishers for bridge_manager
        self.status_pub = self.create_publisher(String, '/robot_statuses', 10)
        
        # Subscribers for batteries
        self.create_subscription(Float32, '/pinky1/battery/present', lambda msg: self.battery_callback('pinky1', msg), 10)
        self.create_subscription(Float32, '/pinky2/battery/present', lambda msg: self.battery_callback('pinky2', msg), 10)
        self.create_subscription(Float32, '/pinky3/battery/present', lambda msg: self.battery_callback('pinky3', msg), 10)
        
        # Subscriber for mission start
        self.create_subscription(String, '/start_mission', self.start_mission_callback, 10)
        
        # Timer for state reporting
        self.create_timer(1.0, self.report_statuses)
        
        self.get_logger().info('MultiPinkyController initialized.')

    def battery_callback(self, name, msg):
        self.robot_statuses[name]['battery'] = msg.data

    def start_mission_callback(self, msg):
        if msg.data == 'START':
            self.get_logger().info('Global mission started!')
            threading.Thread(target=self.run_global_sequence).start()

    def report_statuses(self):
        msg = String()
        msg.data = json.dumps(self.robot_statuses)
        self.status_pub.publish(msg)

    def create_pose(self, wp_name):
        wp = WAYPOINTS[wp_name]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.orientation.z = wp['oz']
        pose.pose.orientation.w = wp['ow']
        return pose

    def move_robot_to(self, name, wp_name, next_state_moving, next_state_at):
        self.robot_statuses[name]['state'] = next_state_moving
        self.robot_statuses[name]['location'] = wp_name
        target_pose = self.create_pose(wp_name)
        
        nav = self.navigators[name]
        nav.goToPose(target_pose)
        
        while not nav.isTaskComplete():
            time.sleep(0.5)
            
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.robot_statuses[name]['state'] = next_state_at
            self.get_logger().info(f'{name} arrived at {wp_name}')
            return True
        else:
            self.robot_statuses[name]['state'] = STATES['ERROR']
            self.get_logger().error(f'{name} failed to reach {wp_name}')
            return False

    def run_robot_sequence(self, name):
        """Individual robot sequence: Charge -> P1 -> P2 -> P3 -> Waiting"""
        sequence = [
            ('PICKUP_1', STATES['MOVING_TO_PICKUP_1'], STATES['AT_PICKUP_1']),
            ('PICKUP_2', STATES['MOVING_TO_PICKUP_2'], STATES['AT_PICKUP_2']),
            ('PICKUP_3', STATES['MOVING_TO_PICKUP_3'], STATES['AT_PICKUP_3']),
            ('PICKING_WAIT_ZONE', STATES['MOVING_TO_PICKING_ZONE'], STATES['AT_PICKING_ZONE']),
        ]
        
        for wp, s_moving, s_at in sequence:
            if wp == 'PICKUP_1':
                # Special condition: Notify next robot to start if needed
                pass
            
            if not self.move_robot_to(name, wp, s_moving, s_at):
                return False
        return True

    def run_global_sequence(self):
        """Coordination: Pinky 1 starts, when Pinky 1 reaches P1, Pinky 2 starts..."""
        
        # Start Pinky 1
        p1_thread = threading.Thread(target=self.run_robot_sequence, args=('pinky1',))
        p1_thread.start()
        
        # Wait for Pinky 1 to reach PICKUP_1
        while self.robot_statuses['pinky1']['state'] != STATES['AT_PICKUP_1']:
            if self.robot_statuses['pinky1']['state'] == STATES['ERROR']: return
            time.sleep(0.5)
            
        # Start Pinky 2
        self.get_logger().info('Pinky 1 reached Pickup 1. Starting Pinky 2.')
        p2_thread = threading.Thread(target=self.run_robot_sequence, args=('pinky2',))
        p2_thread.start()
        
        # Wait for Pinky 2 to reach PICKUP_1
        while self.robot_statuses['pinky2']['state'] != STATES['AT_PICKUP_1']:
            if self.robot_statuses['pinky2']['state'] == STATES['ERROR']: return
            time.sleep(0.5)
            
        # Start Pinky 3
        self.get_logger().info('Pinky 2 reached Pickup 1. Starting Pinky 3.')
        p3_thread = threading.Thread(target=self.run_robot_sequence, args=('pinky3',))
        p3_thread.start()

from std_msgs.msg import Float32

def main(args=None):
    rclpy.init(args=args)
    controller = MultiPinkyController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
