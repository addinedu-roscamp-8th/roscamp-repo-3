#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from control_msgs.action import FollowJointTrajectory
from pymycobot.mycobot280 import MyCobot280


class JetCobotHWNodeTraj(Node):
    """
    JetCobot trajectory player.
    - Receives FollowJointTrajectory
    - Replays trajectory.points using time_from_start
    - Uses pymycobot SDK as actuator driver
    - Handles gripper control via /gripper_command topic
    """

    def __init__(self):
        super().__init__("jetcobot_hw_node_traj")

        # ---------------- parameters ----------------
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baud", 1000000)
        self.declare_parameter("sdk_speed", 85)
        self.declare_parameter("state_rate", 20.0)
        self.declare_parameter("max_rate_hz", 15.0)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.sdk_speed = int(self.get_parameter("sdk_speed").value)
        self.max_rate = float(self.get_parameter("max_rate_hz").value)
        self.state_rate = float(self.get_parameter("state_rate").value)

        # ---------------- hardware ----------------
        self.mc = MyCobot280(port, baud)
        self.get_logger().info(f"Connected to JetCobot on {port}")

        self.joint_names = [
            "link1_to_link2",
            "link2_to_link3",
            "link3_to_link4",
            "link4_to_link5",
            "link5_to_link6",
            "link6_to_link6_flange",
        ]

        # ---------------- joint states ----------------
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / self.state_rate, self.publish_joint_states)

        # ---------------- gripper control ----------------
        self.gripper_sub = self.create_subscription(
            Float32,
            "/gripper_command",
            self.on_gripper_command,
            10
        )
        self.get_logger().info("Gripper command subscriber created")

        # ---------------- action server ----------------
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.execute_trajectory,
        )

        self.get_logger().info("JetCobot trajectory HW node ready")

    # =================================================
    def publish_joint_states(self):
        try:
            angles = self.mc.get_angles()
            if not angles:
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [math.radians(a) for a in angles[:6]]
            self.js_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"joint_states error: {e}")

    # =================================================
    def on_gripper_command(self, msg: Float32):
        """Handle gripper command from /gripper_command topic"""
        gripper_value = msg.data  # 0~100
        
        try:
            self.get_logger().info(f"Gripper command received: {gripper_value}")
            # set_gripper_value(value, speed, gripper_type)
            # gripper_type: 1=Adaptive gripper
            self.mc.set_gripper_value(int(gripper_value), 50, 1)
            self.get_logger().info(f"Gripper set to {gripper_value}")
        except Exception as e:
            self.get_logger().error(f"Gripper control error: {e}")

    # =================================================
    def execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        points = traj.points

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        name_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
        idx = [name_to_idx[n] for n in self.joint_names]

        def tsec(p):
            return p.time_from_start.sec + p.time_from_start.nanosec * 1e-9

        min_dt = 1.0 / self.max_rate
        last_send = time.monotonic()
        last_t = 0.0

        self.get_logger().info(
            f"Execute trajectory: {len(points)} points"
        )

        for i, p in enumerate(points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            # time sync
            dt = tsec(p) - last_t
            if dt > 0:
                time.sleep(dt)
            last_t = tsec(p)

            # rate limit
            now = time.monotonic()
            if now - last_send < min_dt and i < len(points) - 1:
                continue

            pos_deg = [math.degrees(p.positions[j]) for j in idx]
            self.mc.send_angles(pos_deg, self.sdk_speed, _async=True)

            last_send = time.monotonic()

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = JetCobotHWNodeTraj()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
