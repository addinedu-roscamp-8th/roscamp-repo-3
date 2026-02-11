"""
카메라 스트리밍 컨트롤러 (UDP 통신)
"""
import socket
import threading
import cv2
import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal


class CameraController(QObject):
    """UDP 카메라 스트리밍 컨트롤러"""
    
    frame_updated = pyqtSignal(object)  # numpy array (OpenCV frame)
    connection_changed = pyqtSignal(bool)
    
    def __init__(self, robot_ip, udp_port=9505, robot_controller=None):
        super().__init__()

        # Optional reference to RobotArmController (for centralized ROS publishers)
        self.robot_controller = robot_controller

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

    def publish_frame_with_command_ros(self, jpg_bytes, command_value=0, robot_controller=None):
        """Publish image + command to the with-command test topic."""
        rc = robot_controller or self.robot_controller
        if rc is None:
            print("⚠️ ROS publisher not available (no robot_controller provided)")
            return False

        pub = None
        try:
            pub = rc.get_publisher('PTP_capture_image_with_command_compressed')
        except Exception as e:
            print(f"publish_frame_with_command_ros: get_publisher raised: {e}")
            pub = None

        if pub is None:
            print("⚠️ PTP_capture_image_with_command_compressed publisher not found in robot controller")
            return False

        try:
            from lovo_interfaces.msg import CaptureImageWithCommand
            msg = CaptureImageWithCommand()
            msg.image.format = 'jpeg'
            msg.image.data = bytearray(jpg_bytes)
            msg.command = int(command_value)
            print(
                f"publish_frame_with_command_ros: publishing image={len(msg.image.data)} bytes, "
                f"command={msg.command} to publisher {pub}"
            )
            pub.publish(msg)
            print("publish_frame_with_command_ros: publish succeeded")
            return True
        except Exception as e:
            print(f"❌ Failed to publish capture payload: {repr(e)}")
            return False
    
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
