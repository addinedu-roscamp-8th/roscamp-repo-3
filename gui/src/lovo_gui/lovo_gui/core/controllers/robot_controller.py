"""
로봇팔 제어 컨트롤러 (ROS2 통신)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Int32, Float32
from sensor_msgs.msg import JointState
from lovo_interfaces.msg import CaptureImageWithCommand
from PyQt6.QtCore import QObject, pyqtSignal
import time


class RobotArmController(Node, QObject):
    """로봇팔 제어 컨트롤러 (ROS2 Node)"""
    
    # PyQt Signals
    angles_updated = pyqtSignal(list)      # 각도 업데이트
    pose_updated = pyqtSignal(list)        # 좌표 업데이트
    # Keep coords_updated for backward compatibility with UI (build) code
    coords_updated = pyqtSignal(list)      # 현재 좌표 업데이트
    connection_changed = pyqtSignal(bool)  # 연결 상태 변경
    controller_log = pyqtSignal(str)       # controller 로그 메시지 (GUI로 전달)
    
    def __init__(self, robot_name, robot_domain, context=None):
        Node.__init__(self, f'robot_arm_controller_{robot_domain}', context=context)
        QObject.__init__(self)
        
        self.robot_name = robot_name
        self.robot_domain = robot_domain
        
        # 도메인 ID 기반으로 로봇 ID 생성 (한글 이름 문제 해결)
        domain_to_robot_id = {60: 'robot1', 61: 'robot2'}
        robot_id = domain_to_robot_id.get(robot_domain, f'robot{robot_domain}')
        
        # 데이터
        self.current_angles = [0.0] * 6
        self.current_coords = [0.0] * 6
        self.current_pose = [0.0] * 6
        self.robot_connected = False
        self.last_encoder_time = time.time()
        self.connection_timeout = 1.0
        
        # Initialize publishers/subscribers centrally
        self._init_topics(robot_id)
       
       
        # 연결 상태 체크 타이머
        self.connection_timer = self.create_timer(
            0.5, self.check_connection
        )
        
        self.get_logger().info(
            f'RobotArmController initialized for {robot_name} (Domain: {robot_domain})'
        )
    
    def joint_states_callback(self, msg):
        """각도 데이터 수신"""
        if len(msg.position) == 6:
            self.last_encoder_time = time.time()
            # joint_states.position is typically in radians; convert to degrees for GUI
            try:
                from math import degrees
                angles_deg = [degrees(p) for p in msg.position]
            except Exception:
                angles_deg = list(msg.position)

            self.current_angles = angles_deg
            self.angles_updated.emit(self.current_angles)

    def current_coords_callback(self, msg):
        """Float64MultiArray로 오는 현재 좌표 수신 (Actual 필드 업데이트용)"""
        try:
            if hasattr(msg, 'data') and len(msg.data) == 6:
                self.last_encoder_time = time.time()
                # convert to float and keep incoming order
                vals = [float(x) for x in msg.data]
                self.current_coords = vals
                # emit coords update for GUI without verbose logging
                self.coords_updated.emit(self.current_coords)
        except Exception:
            pass

    # Note: callbacks for incoming Float64MultiArray/pose/coords were removed
    # because the corresponding subscriptions/publishers are no longer used.
    
    def check_connection(self):
        """로봇 연결 상태 체크"""
        current_time = time.time()
        is_connected = (current_time - self.last_encoder_time) <= self.connection_timeout
        
        if is_connected != self.robot_connected:
            self.robot_connected = is_connected
            self.connection_changed.emit(is_connected)
    
    # 제어 명령
    def publish_angles(self, angles):
        """각도 명령 전송"""
        msg = Float64MultiArray()
        msg.data = [float(a) for a in angles]
        self.pub_angles.publish(msg)
        self.current_angles = list(angles)
    # publish_coords removed because its publisher was deleted from initialization
    
    def send_servo(self, on):
        """서보 ON/OFF"""
        msg = Bool()
        msg.data = on
        self.pub_servo.publish(msg)
    
    def send_gripper(self, state):
        """그리퍼 제어 (0: UNGRIP, 1: GRIP) - publish as Int32"""
        msg = Int32()
        msg.data = int(state)
        self.pub_gripper.publish(msg)
    
    def send_gripper_command(self, value):
        """Send gripper command with a specific value (100 for GRIP, 0 for UNGRIP) as Int32"""
        msg = Int32()
        msg.data = int(value)
        self.pub_gripper.publish(msg)
        self.get_logger().info(f"Gripper command sent: {value}")

    def publish_goal_pose(self, pose):
        """Publish a goal pose (Float64MultiArray).

        Args:
            pose: iterable of 6 numeric values (assumed already in the correct unit)
        """
        try:
            msg = Float64MultiArray()
            msg.data = [float(x) for x in pose]
            self.pub_goal_pose.publish(msg)
            self.get_logger().info(f"Goal pose published: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish goal pose: {e}")
    
    def go_home(self):
        """홈 위치로 이동"""
        self.publish_angles([0.0] * 6)

    def _init_topics(self, robot_id: str):
        """Centralized initialization of publishers and subscribers.

        Creates `self.publishers` and `self.subscribers` dicts and sets
        backward-compatible attributes like `pub_gripper`, `subs_joint_states`, etc.
        """
        # Publishers (use unique registry name to avoid rclpy internal attribute collisions)
        self._topic_publishers = {}
        self._topic_publishers['gripper'] = self.create_publisher(Int32, f'/{robot_id}/PTP_gripper_command', 10)
        self._topic_publishers['servo'] = self.create_publisher(Bool, f'/{robot_id}/PTP_servo_status', 10)
        self._topic_publishers['goal_pose'] = self.create_publisher(Float64MultiArray, f'/{robot_id}/PTP_goal_pose', 10)
        # legacy publisher for explicit angles if used elsewhere
        self._topic_publishers['angles'] = self.create_publisher(Float64MultiArray, f'/{robot_id}/PTP_angles', 10)
        # packing order command
        self._topic_publishers['order_command'] = self.create_publisher(Float32, f'/{robot_id}/picking/order_command', 10)
        # Camera payload publisher: custom message (image + command)
        capture_pub = self.create_publisher(
            CaptureImageWithCommand,
            f'/{robot_id}/PTP_capture_image_with_command/compressed',
            5
        )
        self._topic_publishers['PTP_capture_image_with_command_compressed'] = capture_pub
        # Backward-compatible alias
        self._topic_publishers['PTP_capture_image_compressed'] = capture_pub

        # Subscribers (private registry with unique name)
        self._topic_subscribers = {}
        self._topic_subscribers['joint_states'] = self.create_subscription(JointState, f'/{robot_id}/joint_states', self.joint_states_callback, 10)
        self._topic_subscribers['tcp_pose'] = self.create_subscription(Float64MultiArray, f'/{robot_id}/PTP_tcp_pose', self.current_coords_callback, 10)

        # Backwards-compatible attributes used by other code
        self.pub_gripper = self._topic_publishers['gripper']
        self.pub_servo = self._topic_publishers['servo']
        self.pub_goal_pose = self._topic_publishers['goal_pose']
        self.pub_angles = self._topic_publishers['angles']

        self.subs_joint_states = self._topic_subscribers['joint_states']
        self.subs_tcp_Pose = self._topic_subscribers['tcp_pose']

    def get_publisher(self, name: str):
        """Return a publisher by logical name from the centralized registry.

        Args:
            name: one of the keys in `self.publishers` (e.g. 'gripper', 'goal_pose')
        """
        return self._topic_publishers.get(name)

    def get_subscriber(self, name: str):
        """Return a subscriber by logical name from the centralized registry."""
        return self._topic_subscribers.get(name)
