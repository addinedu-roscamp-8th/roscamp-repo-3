#!/usr/bin/env python3
"""
Description:
    JetCobot hardware driver that combines MoveIt2 trajectory execution
    with direct joint/coordinate control interfaces for the MyCobot280 robot arm.

------
Publishing Topics:
    Joint states for ROS2 control and MoveIt2 
    /joint_states - sensor_msgs/JointState

Subscription Topics:
    Servo power control
    /servo_status - std_msgs/Bool
    
    Gripper control (MoveIt2 interface)
    /gripper_command - std_msgs/Float32

Action Servers:
    MoveIt2 trajectory execution
    /arm_controller/follow_joint_trajectory - control_msgs/action/FollowJointTrajectory

------
Author: Taehyun Seo
Date: February 9, 2026
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Int32
from control_msgs.action import FollowJointTrajectory
from pymycobot.mycobot280 import MyCobot280


class JetCobotUnifiedDriver(Node):
    """
    Unified JetCobot driver combining trajectory action server and direct control
    """

    def __init__(self):
        super().__init__("jetcobot_HW_node")

        # ---------------- parameters ----------------

        # 파라미터 선언
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baud", 1000000)
        self.declare_parameter("sdk_speed", 50)
        self.declare_parameter("gripper_speed", 30)
        self.declare_parameter("state_rate", 20.0)
        self.declare_parameter("max_rate_hz", 10.0)

        # 파라미터 읽기
        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.sdk_speed = int(self.get_parameter("sdk_speed").value)
        self.gripper_speed = int(self.get_parameter("gripper_speed").value)
        self.max_rate = float(self.get_parameter("max_rate_hz").value)
        self.state_rate = float(self.get_parameter("state_rate").value)

        # ---------------- hardware initialization ----------------
        try:
            self.mc = MyCobot280(port, baud)
            self.mc.power_on()
            self.get_logger().info(f"★★★ JetCobot Unified Driver Online on {port} ★n★★")
        except Exception as e:
            self.get_logger().error(f"Connection Error: {e}")
            raise

        self.joint_names = [
            "link1_to_link2",
            "link2_to_link3",
            "link3_to_link4",
            "link4_to_link5",
            "link5_to_link6",
            "link6_to_link6_flange",
        ]

        # ---------------- Publishers ----------------
        # ROS2 control joint states
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        # State publishing timer
        self.create_timer(1.0 / self.state_rate, self.publish_states)


        # ---------------- Subscribers ----------------
        # Servo power control
        self.create_subscription(Bool, "/servo_status", self.servo_cb, 10)
        
        # Gripper control
        self.create_subscription(Int32, "/gripper_command", self.gripper_cb, 10)
        

        # ---------------- Action Server ----------------
        # MoveIt2 trajectory execution
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.execute_trajectory_cb,
        )

        self.get_logger().info("JetCobot Driver ready - all interfaces active")

    # =================================================
    # State Publishing
    # =================================================
    def publish_states(self):
        """Publish both JointState (for ROS2 control) and legacy angle/coord feedback"""
        try:
            angles = self.mc.get_angles()
            if not angles or not isinstance(angles, (list, tuple)):
                return

            # Publish standard JointState
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = self.joint_names
            js_msg.position = [math.radians(a) for a in angles[:6]]
            self.js_pub.publish(js_msg)

        except Exception as e:
            error_msg = str(e)
            if "device reports readiness" not in error_msg and "not subscriptable" not in error_msg:
                self.get_logger().warn(f"State publishing error: {e}")

    # =================================================
    # Direct Control Callbacks
    # =================================================
    
    def target_coords_cb(self, msg):
        """Direct coordinate movement to desired position"""
        coords = list(msg.data)
        self.get_logger().info(f"Moving to coordinates: {coords}")
        try:
            self.mc.send_coords(coords, self.sdk_speed, 0)
        except Exception as e:
            self.get_logger().error(f"Coordinate move error: {e}")
    
    def servo_cb(self, msg):
        """Servo power on/off control"""
        if msg.data:
            self.mc.power_on()
            self.get_logger().info("Servo ON")
        else:
            self.mc.release_all_servos()
            self.get_logger().info("Servo OFF")


    def gripper_cb(self, msg):
        """Gripper control with Int32 (0-100)"""
        try:
            val = int(msg.data)
            val = max(0, min(100, val))  # Clamp to 0-100
            
            speed = int(self.gripper_speed)
            
            self.get_logger().info(f"Gripper command (Int32): value={val}, speed={speed}")
            self.mc.set_gripper_value(val, speed)
            
        except Exception as e:
            self.get_logger().warn(f"Gripper Int32 command error: {e}")

    # =================================================
    # Trajectory Execution Action Server
    # =================================================
    def execute_trajectory_cb(self, goal_handle):
        """Execute FollowJointTrajectory action from MoveIt2"""
        traj = goal_handle.request.trajectory
        points = traj.points

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Map trajectory joint names to our joint order
        name_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
        idx = [name_to_idx[n] for n in self.joint_names]

        def tsec(p):
            return p.time_from_start.sec + p.time_from_start.nanosec * 1e-9

        min_dt = 1.0 / self.max_rate
        last_send = time.monotonic()
        last_t = 0.0

        self.get_logger().info(f"Executing trajectory: {len(points)} points")

        # Send each trajectory point
        for i, p in enumerate(points):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Trajectory execution canceled")
                return FollowJointTrajectory.Result()

            # Time synchronization
            dt = tsec(p) - last_t
            if dt > 0:
                time.sleep(dt)
            last_t = tsec(p)

            # Rate limiting
            now = time.monotonic()
            if now - last_send < min_dt and i < len(points) - 1:
                continue

            # Send position command (angles in degrees)
            pos_deg = [math.degrees(p.positions[j]) for j in idx]
            self.mc.send_angles(pos_deg, self.sdk_speed, _async=True)

            last_send = time.monotonic()

        goal_handle.succeed()
        self.get_logger().info("Trajectory execution completed")
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = JetCobotUnifiedDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
