"""
로봇 제어 컨트롤러 (ROS2 통신)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Int32
from geometry_msgs.msg import Pose
from PyQt6.QtCore import QObject, pyqtSignal
import socket
import threading
import cv2
import numpy as np
import time


class RobotArmController(Node, QObject):
    """로봇팔 제어 컨트롤러 (ROS2 Node)"""
    
    # PyQt Signals
    angles_updated = pyqtSignal(list)      # 각도 업데이트
    pose_updated = pyqtSignal(list)        # 좌표 업데이트
    coords_updated = pyqtSignal(list)      # 현재 좌표 업데이트
    connection_changed = pyqtSignal(bool)  # 연결 상태 변경
    
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
        
        # Publishers (domain_bridge가 robot_id를 prefix로 사용)
        self.pub_angles = self.create_publisher(
            Float64MultiArray, f'/{robot_id}/target_angles', 10
        )
        self.pub_servo = self.create_publisher(
            Bool, f'/{robot_id}/servo_status', 10
        )
        self.pub_gripper = self.create_publisher(
            Int32, f'/{robot_id}/gripper_control', 10
        )
        self.pub_target_coords = self.create_publisher(
            Float64MultiArray, f'/{robot_id}/target_coords', 10
        )
        self.pub_pose_target = self.create_publisher(
            Pose, f'/{robot_id}/goal_pose', 10
        )
        
        # Subscribers (domain_bridge를 통해 전달받음)
        self.sub_current = self.create_subscription(
            Float64MultiArray, f'/{robot_id}/current_angles', 
            self.receive_angles_callback, 10
        )
        self.sub_pose = self.create_subscription(
            Pose, f'/{robot_id}/current_pose', 
            self.pose_callback, 10
        )
        self.sub_current_coords = self.create_subscription(
            Float64MultiArray, f'/{robot_id}/current_coords',
            self.coords_callback, 10
        )
        
        # 연결 상태 체크 타이머
        self.connection_timer = self.create_timer(
            0.5, self.check_connection
        )
        
        self.get_logger().info(
            f'RobotArmController initialized for {robot_name} (Domain: {robot_domain})'
        )
    
    def receive_angles_callback(self, msg):
        """각도 데이터 수신"""
        if len(msg.data) == 6:
            self.last_encoder_time = time.time()
            self.current_angles = list(msg.data)
            self.angles_updated.emit(self.current_angles)
    
    def pose_callback(self, msg):
        """포즈 데이터 수신"""
        from scipy.spatial.transform import Rotation as R
        
        x = msg.position.x * 1000  # m to mm
        y = msg.position.y * 1000
        z = msg.position.z * 1000
        
        r = R.from_quat([
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        ])
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        
        self.current_pose = [x, y, z, roll, pitch, yaw]
        self.pose_updated.emit(self.current_pose)
    
    def coords_callback(self, msg):
        """좌표 데이터 수신"""
        if len(msg.data) == 6:
            self.current_coords = list(msg.data)
            self.coords_updated.emit(self.current_coords)
    
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
    
    def publish_coords(self, coords):
        """좌표 명령 전송"""
        msg = Float64MultiArray()
        msg.data = [float(c) for c in coords]
        print(f"[RobotController] publish_coords: {msg.data}")
        self.pub_target_coords.publish(msg)
        print(f"[RobotController] 토픽 발행 완료: {self.pub_target_coords.topic_name}")
    
    def send_servo(self, on):
        """서보 ON/OFF"""
        msg = Bool()
        msg.data = on
        self.pub_servo.publish(msg)
    
    def send_gripper(self, state):
        """그리퍼 제어 (0: UNGRIP, 1: GRIP)"""
        msg = Int32()
        msg.data = state
        self.pub_gripper.publish(msg)
    
    def go_home(self):
        """홈 위치로 이동"""
        self.publish_angles([0.0] * 6)


class CameraController(QObject):
    """UDP 카메라 스트리밍 컨트롤러"""
    
    frame_updated = pyqtSignal(object)  # numpy array (OpenCV frame)
    connection_changed = pyqtSignal(bool)
    
    def __init__(self, robot_ip, udp_port=9505):
        super().__init__()
        
        self.robot_ip = robot_ip
        self.udp_port = udp_port
        self.command_port = udp_port + 1  # 9506
        self.is_streaming = False
        self.latest_frame = None
        self.sock = None
        self.command_sock = None
        self.thread = None
    
    def start(self):
        """스트리밍 시작"""
        if not self.is_streaming:
            self._send_command("START")
            self.is_streaming = True
            self.connection_changed.emit(True)
            self.thread = threading.Thread(
                target=self._udp_receiver_thread, 
                daemon=True
            )
            self.thread.start()
    
    def stop(self):
        """스트리밍 중지"""
        self._send_command("STOP")
        self.is_streaming = False
        self.connection_changed.emit(False)
    
    def capture(self, filename):
        """이미지 캡처"""
        if self.latest_frame is not None:
            try:
                from pathlib import Path
                # 파일 경로 생성
                filepath = Path(filename)
                # 폴더 생성
                filepath.parent.mkdir(parents=True, exist_ok=True)
                # 이미지 저장
                cv2.imwrite(str(filepath), self.latest_frame)
                print(f"✅ 캡처 저장됨: {filepath}")
                return str(filepath)
            except Exception as e:
                print(f"❌ 캡처 저장 실패: {e}")
                return None
        else:
            print(f"⚠️ 캡처할 프레임이 없습니다")
            return None
    
    def _send_command(self, command):
        """로봇에 명령 전송 (START/STOP)"""
        try:
            if self.command_sock is None:
                self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            self.command_sock.sendto(
                command.encode("utf-8"),
                (self.robot_ip, self.command_port)
            )
            print(f"📤 명령 전송: {command} -> {self.robot_ip}:{self.command_port}")
        except Exception as e:
            print(f"⚠️ 명령 전송 실패: {e}")
    
    def _udp_receiver_thread(self):
        """UDP 수신 스레드"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.udp_port))
        self.sock.settimeout(2.0)
        
        print(f"📡 UDP 수신 시작: 0.0.0.0:{self.udp_port}")
        
        consecutive_timeouts = 0
        
        while self.is_streaming:
            try:
                data, addr = self.sock.recvfrom(65507)
                
                # JPEG 디코딩
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    # 프레임 복사하여 저장 및 emit (메모리 안전성)
                    self.latest_frame = frame.copy()
                    self.frame_updated.emit(self.latest_frame.copy())
                    consecutive_timeouts = 0  # 리셋
                    
            except socket.timeout:
                consecutive_timeouts += 1
                if consecutive_timeouts >= 3:  # 6초 타임아웃
                    print("⚠️ 카메라 연결 끊김 (타임아웃)")
                    self._handle_disconnection()
                    break
            except Exception as e:
                print(f"⚠️ 프레임 수신 오류: {e}")
                continue
        
        if self.sock:
            self.sock.close()
            self.sock = None
    
    def _handle_disconnection(self):
        """연결 끊김 처리"""
        self.is_streaming = False
        self.connection_changed.emit(False)
        print("🔄 5초 후 재연결 시도...")
        # 5초 후 재연결 시도
        threading.Timer(5.0, self.start).start()
