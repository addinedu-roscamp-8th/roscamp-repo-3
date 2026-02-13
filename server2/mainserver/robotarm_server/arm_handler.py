#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int8

class ArmHandler(Node):
    def __init__(self):
        super().__init__('arm_handler_node')
        
        # --- Publishers (Command Downstream) ---
        # Main Server -> Bridge -> Robot
        self.pub_picking = self.create_publisher(Float64, '/robot1/order_command', 10)
        self.pub_packing = self.create_publisher(Float64, '/robot2/order_command', 10)
        
        # --- Subscribers (State Upstream) ---
        # Robot -> Bridge -> Main Server
        self.sub_picking = self.create_subscription(
            Int8, 
            '/robot1/robot_state', 
            self.picking_state_callback, 
            10
        )
        self.sub_packing = self.create_subscription(
            Int8, 
            '/robot2/robot_state', 
            self.packing_state_callback, 
            10
        )
        
        # Internal State Storage
        self.picking_robot_state = 0 # 0: Unknown, 1: Idle, 2: Busy, 3: Success, etc.
        self.packing_robot_state = 0
        
        self.get_logger().info("Arm Handler Node Started")

    # --- Callbacks ---
    def picking_state_callback(self, msg):
        self.picking_robot_state = msg.data
        # self.get_logger().debug(f"Picking Robot State Updated: {msg.data}")

    def packing_state_callback(self, msg):
        self.packing_robot_state = msg.data
        # self.get_logger().debug(f"Packing Robot State Updated: {msg.data}")

    # --- Command Methods ---
    def send_picking_command(self, command_value: float):
        """
        Send command to Picking Robot (Robot 1)
        Example: 1.4507 (Zone 1, Frame/Legs/Kit)
        """
        msg = Float64()
        msg.data = command_value
        self.pub_picking.publish(msg)
        self.get_logger().info(f"Sent Picking Command: {command_value}")

    def send_packing_command(self, command_value: float):
        """
        Send command to Packing Robot (Robot 2)
        Example: 0.0000 (Move to Packing Zone)
        """
        msg = Float64()
        msg.data = command_value
        self.pub_packing.publish(msg)
        self.get_logger().info(f"Sent Packing Command: {command_value}")

    # --- Status Methods (for external polling) ---
    def get_picking_status(self):
        return self.picking_robot_state

    def get_packing_status(self):
        return self.packing_robot_state

def main(args=None):
    rclpy.init(args=args)
    node = ArmHandler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
