#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, Int8, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from lovo_interfaces.msg import CaptureImageWithCommand

class ArmHandler(Node):
    def __init__(self):
        super().__init__('arm_handler_node')
        
        # --- Robot 1 (Picking) ---
        self.pub_r1_gripper = self.create_publisher(Int32, '/robot1/PTP_gripper_command', 10)
        self.pub_r1_servo = self.create_publisher(Bool, '/robot1/PTP_servo_status', 10)
        self.pub_r1_goal = self.create_publisher(Float64MultiArray, '/robot1/PTP_goal_pose', 10)
        self.pub_r1_order = self.create_publisher(Float64, '/robot1/order_command', 10)
        self.pub_r1_capture = self.create_publisher(CaptureImageWithCommand, '/robot1/PTP_capture_image_with_command/compressed', 10)
        
        self.sub_r1_joint = self.create_subscription(JointState, '/robot1/joint_states', self.r1_joint_cb, 10)
        self.sub_r1_tcp = self.create_subscription(Float64MultiArray, '/robot1/PTP_tcp_pose', self.r1_tcp_cb, 10)
        self.sub_r1_state = self.create_subscription(Int8, '/robot1/robot_state', self.r1_state_cb, 10)

        # --- Robot 2 (Packing) ---
        self.pub_r2_gripper = self.create_publisher(Int32, '/robot2/PTP_gripper_command', 10)
        self.pub_r2_servo = self.create_publisher(Bool, '/robot2/PTP_servo_status', 10)
        self.pub_r2_goal = self.create_publisher(Float64MultiArray, '/robot2/PTP_goal_pose', 10)
        self.pub_r2_order = self.create_publisher(Float64, '/robot2/order_command', 10)
        self.pub_r2_capture = self.create_publisher(CaptureImageWithCommand, '/robot2/PTP_capture_image_with_command/compressed', 10)
        
        self.sub_r2_joint = self.create_subscription(JointState, '/robot2/joint_states', self.r2_joint_cb, 10)
        self.sub_r2_tcp = self.create_subscription(Float64MultiArray, '/robot2/PTP_tcp_pose', self.r2_tcp_cb, 10)
        self.sub_r2_state = self.create_subscription(Int8, '/robot2/robot_state', self.r2_state_cb, 10)
        
        # State Storage
        self.r1_state = {'joint': None, 'tcp': None}
        self.r2_state = {'joint': None, 'tcp': None}
        
        self.get_logger().info("Arm Handler Node Started (Domain 59)")

    # --- Callbacks ---
    def r1_joint_cb(self, msg): self.r1_state['joint'] = msg
    def r1_tcp_cb(self, msg): self.r1_state['tcp'] = msg.data
    def r1_state_cb(self, msg): self.r1_state['status'] = msg.data
    
    def r2_joint_cb(self, msg): self.r2_state['joint'] = msg
    def r2_tcp_cb(self, msg): self.r2_state['tcp'] = msg.data
    def r2_state_cb(self, msg): self.r2_state['status'] = msg.data

    # --- Commands (Robot 1) ---
    def r1_cmd_gripper(self, val: int):
        self.pub_r1_gripper.publish(Int32(data=val))
        
    def r1_cmd_servo(self, enable: bool):
        self.pub_r1_servo.publish(Bool(data=enable))
        
    def r1_cmd_goal(self, pose: list):
        self.pub_r1_goal.publish(Float64MultiArray(data=pose))
        
    def r1_cmd_order(self, val: float):
        self.pub_r1_order.publish(Float64(data=val))

    def r1_cmd_capture(self, cmd: str):
        msg = CaptureImageWithCommand()
        msg.command = cmd
        self.pub_r1_capture.publish(msg)

    # --- Commands (Robot 2) ---
    def r2_cmd_gripper(self, val: int):
        self.pub_r2_gripper.publish(Int32(data=val))
        
    def r2_cmd_servo(self, enable: bool):
        self.pub_r2_servo.publish(Bool(data=enable))
        
    def r2_cmd_goal(self, pose: list):
        self.pub_r2_goal.publish(Float64MultiArray(data=pose))
        
    def r2_cmd_order(self, val: float):
        self.pub_r2_order.publish(Float64(data=val))
        
    def r2_cmd_capture(self, cmd: str):
        msg = CaptureImageWithCommand()
        msg.command = cmd
        self.pub_r2_capture.publish(msg)

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
