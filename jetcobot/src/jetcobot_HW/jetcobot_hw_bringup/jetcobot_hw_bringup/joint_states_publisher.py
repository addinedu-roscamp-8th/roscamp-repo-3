#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymycobot.mycobot280 import MyCobot280


class JointStatesPublisher(Node):
    def __init__(self):
        super().__init__("jetcobot_joint_states_publisher")

        # ---- params ----
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baud", 1000000)
        self.declare_parameter("rate_hz", 20.0)
        # URDF joint 이름과 반드시 동일해야 함 (필요시 바꿔)
        self.declare_parameter(
    "joint_names",
    [
        "link1_to_link2",
        "link2_to_link3",
        "link3_to_link4",
        "link4_to_link5",
        "link5_to_link6",
        "link6_to_link6_flange",
    ])
        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.joint_names = list(self.get_parameter("joint_names").value)

        # ---- connect ----
        self.mc = MyCobot280(self.port, self.baud)
        test = self.mc.get_angles()
        self.get_logger().info(f"Connected. First angles(deg)={test}")

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1e-6), self.tick)

    def tick(self):
        angles_deg = self.mc.get_angles()
        if not angles_deg or len(angles_deg) < len(self.joint_names):
            self.get_logger().warn(f"Invalid angles: {angles_deg}")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        # pymycobot은 보통 deg -> ROS 표준 rad로 변환
        msg.position = [math.radians(a) for a in angles_deg[:len(self.joint_names)]]


        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointStatesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
