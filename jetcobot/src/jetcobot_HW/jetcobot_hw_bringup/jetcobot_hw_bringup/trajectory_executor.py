#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory
from pymycobot.mycobot280 import MyCobot280


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__("jetcobot_trajectory_executor")

        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baud", 1000000)
        self.declare_parameter("speed", 50)
        self.declare_parameter("settle_sec", 1.5)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.speed = int(self.get_parameter("speed").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)

        self.mc = MyCobot280(self.port, self.baud)
        self.get_logger().info(f"Connected to JetCobot on {self.port} @ {self.baud}")

        # MoveIt이 기본으로 쓰는 FollowJointTrajectory 액션 서버
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.execute_callback,
        )

        self.get_logger().info("Action server ready: /arm_controller/follow_joint_trajectory")

    def execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory

        if not traj.points:
            self.get_logger().error("Empty trajectory")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # 1차 bringup: 마지막 포인트만 보내서 동작 확인
        p = traj.points[-1]
        positions_rad = list(p.positions)
        positions_deg = [math.degrees(x) for x in positions_rad]

        self.get_logger().info(f"Received goal with {len(traj.points)} points")
        self.get_logger().info(f"Goal joints: {traj.joint_names}")
        self.get_logger().info(f"Sending angles (deg): {positions_deg}")

        self.mc.send_angles(positions_deg, self.speed)
        time.sleep(self.settle_sec)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = TrajectoryExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
