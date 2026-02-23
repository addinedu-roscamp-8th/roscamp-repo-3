#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from pymycobot.mycobot280 import MyCobot280





class JetCobotHWNodeTraj(Node):
    """
    JetCobot trajectory player.
    - Receives FollowJointTrajectory
    - Replays trajectory.points using time_from_start
    - Uses pymycobot SDK as actuator driver
    """

    def __init__(self):
        super().__init__("jetcobot_hw_node_traj")

        # ---------------- parameters ----------------
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baud", 1000000)
        self.declare_parameter("sdk_speed", 85)
        self.declare_parameter("gripper_speed", 50)
        self.declare_parameter("state_rate", 20.0)
        self.declare_parameter("max_rate_hz", 15.0)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.sdk_speed = int(self.get_parameter("sdk_speed").value)
        self.gripper_speed = int(self.get_parameter("gripper_speed").value)
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

        # ---------------- gripper command subscriber ----------------
        # expects Float32 (0.0..100.0) and forwards to pymycobot.set_gripper_value
        from std_msgs.msg import Float32
        self.create_subscription(Float32, "/gripper_command", self._on_gripper_command, 10)

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
            
            # angles가 리스트/튜플인지 확인 (가끔 에러 코드로 정수를 반환할 수 있음)
            if not isinstance(angles, (list, tuple)):
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [math.radians(a) for a in angles[:6]]
            self.js_pub.publish(msg)

        except Exception as e:
            # 일시적인 시리얼 통신 에러는 무시 (로그 스팸 방지)
            error_msg = str(e)
            if "device reports readiness" not in error_msg and "not subscriptable" not in error_msg:
                self.get_logger().warn(f"joint_states error: {e}")

    # =================================================
    def _on_gripper_command(self, msg):
        try:
            val = float(msg.data)
            # clamp
            if val < 0.0:
                val = 0.0
            if val > 100.0:
                val = 100.0

            speed = int(self.gripper_speed)
            gv = int(round(val))
            # pymycobot expects (gripper_value, speed)
            self.get_logger().info(f"Gripper command received: value={gv}, speed={speed}")
            try:
                self.mc.set_gripper_value(gv, speed)
            except Exception as e:
                self.get_logger().warn(f"set_gripper_value failed: {e}")

        except Exception as e:
            self.get_logger().warn(f"gripper_command callback error: {e}")

    # =================================================
    def execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory   ## Moveit 에서 보낸 FollowJointTrajectory Goal
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
            self.mc.send_angles(pos_deg, self.sdk_speed, _async=True)  ## 비동기적으로 명령 전송

            last_send = time.monotonic() # 업데이트

        goal_handle.succeed() # 완료
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
