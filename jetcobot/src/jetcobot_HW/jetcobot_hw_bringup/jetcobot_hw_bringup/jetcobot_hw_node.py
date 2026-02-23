#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from pymycobot.mycobot280 import MyCobot280


class JetCobotHW(Node):
    def __init__(self):
        super().__init__("jetcobot_hw_node")

        # params
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baud", 1000000)
        self.declare_parameter("speed", 50)
        self.declare_parameter("state_rate", 20.0)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.speed = int(self.get_parameter("speed").value)
        rate = float(self.get_parameter("state_rate").value)

        # single serial connection
        self.mc = MyCobot280(port, baud)
        self.get_logger().info(f"Connected to JetCobot on {port}")

        # joint names (URDF 순서와 반드시 일치)
        self.joint_names = [
            "link1_to_link2",
            "link2_to_link3",
            "link3_to_link4",
            "link4_to_link5",
            "link5_to_link6",
            "link6_to_link6_flange",
        ]

        # publisher
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_states)

        # action server
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.execute_cb,
        )

        self.get_logger().info("HW node ready (state + trajectory)")

    # ---------------- joint states ----------------
    def publish_joint_states(self):
        try:
            angles_deg = self.mc.get_angles()
            if not angles_deg or len(angles_deg) < 6:
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [math.radians(a) for a in angles_deg[:6]]

            self.js_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Joint state error: {e}")

    # ---------------- trajectory ----------------
    def execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        if not traj.points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        p = traj.points[-1]
        angles_deg = [math.degrees(x) for x in p.positions]

        self.get_logger().info(f"Execute: {angles_deg}")
        self.mc.send_angles(angles_deg, self.speed)

        time.sleep(1.5)
        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = JetCobotHW()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()