"""
화재 경보 다이얼로그
"""
import socket
import numpy as np
import cv2
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QWidget
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap


class UDPVideoReceiver(QThread):
    """UDP로 영상을 수신하는 스레드"""
    frame_updated = pyqtSignal(QImage)
    
    def __init__(self, port: int):
        super().__init__()
        self.port = port
        self.running = False
    
    def run(self):
        self.running = True
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            sock.bind(("0.0.0.0", self.port))
            sock.settimeout(1.0)
        except Exception as e:
            print(f"UDP 포트 {self.port} 바인딩 실패: {e}")
            sock.close()
            return
        
        while self.running:
            try:
                data, addr = sock.recvfrom(65507)
                
                # JPEG 디코딩
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    # BGR -> RGB 변환
                    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb_image.shape
                    bytes_per_line = ch * w
                    qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                    self.frame_updated.emit(qt_image.copy())
                    
            except socket.timeout:
                continue
            except Exception as e:
                continue
        
        sock.close()
    
    def stop(self):
        self.running = False
        self.wait()


class FireAlertDialog(QDialog):
    """화재 경보 다이얼로그 - 2개의 UDP 영상 표시"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("🔥 화재 상황 발생")
        self.setFixedSize(1640, 1000)
        self.setModal(False)  # 논모달로 설정하여 메인 창 조작 가능
        
        # UDP 수신기
        self.receiver_9730 = UDPVideoReceiver(9730)
        self.receiver_9710 = UDPVideoReceiver(9710)
        
        self._setup_ui()
        
        # 영상 수신 시작
        self.receiver_9730.frame_updated.connect(self._update_left_video)
        self.receiver_9710.frame_updated.connect(self._update_right_video)
        self.receiver_9730.start()
        self.receiver_9710.start()
    
    def _setup_ui(self):
        """UI 구성"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 제목
        title = QLabel("화재 상황 발생")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                background-color: #d32f2f;
                color: white;
                font-size: 144px;
                font-weight: bold;
                padding: 15px;
                border-radius: 5px;
            }
        """)
        layout.addWidget(title)
        
        # 영상 표시 영역
        video_layout = QHBoxLayout()
        video_layout.setSpacing(10)
        
        # 왼쪽 영상 (9730 포트)
        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(5)
        
        left_title = QLabel("📹 USB 카메라 (가공)")
        left_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_title.setStyleSheet("""
            QLabel {
                background-color: #424242;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 8px;
                border-radius: 3px;
            }
        """)
        
        self.left_video_label = QLabel("영상 수신 대기 중...")
        self.left_video_label.setFixedSize(960, 480)
        self.left_video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.left_video_label.setStyleSheet("""
            QLabel {
                background-color: black;
                color: white;
                border: 2px solid #d32f2f;
                border-radius: 5px;
            }
        """)
        self.left_video_label.setScaledContents(True)
        
        left_layout.addWidget(left_title)
        left_layout.addWidget(self.left_video_label)
        
        # 오른쪽 영상 (9710 포트)
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(5)
        
        right_title = QLabel("📹 로봇팔 카메라 (가공)")
        right_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right_title.setStyleSheet("""
            QLabel {
                background-color: #424242;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 8px;
                border-radius: 3px;
            }
        """)
        
        self.right_video_label = QLabel("영상 수신 대기 중...")
        self.right_video_label.setFixedSize(640, 480)
        self.right_video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.right_video_label.setStyleSheet("""
            QLabel {
                background-color: black;
                color: white;
                border: 2px solid #d32f2f;
                border-radius: 5px;
            }
        """)
        self.right_video_label.setScaledContents(True)
        
        right_layout.addWidget(right_title)
        right_layout.addWidget(self.right_video_label)
        
        video_layout.addWidget(left_container)
        video_layout.addWidget(right_container)
        
        layout.addLayout(video_layout)
        
        # 닫기 버튼
        close_btn = QPushButton("닫기")
        close_btn.setFixedHeight(50)
        close_btn.setStyleSheet("""
            QPushButton {
                background-color: #616161;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #757575;
            }
            QPushButton:pressed {
                background-color: #424242;
            }
        """)
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)
    
    def _update_left_video(self, qimage: QImage):
        """왼쪽 영상 업데이트"""
        pixmap = QPixmap.fromImage(qimage)
        self.left_video_label.setPixmap(pixmap)
    
    def _update_right_video(self, qimage: QImage):
        """오른쪽 영상 업데이트"""
        pixmap = QPixmap.fromImage(qimage)
        self.right_video_label.setPixmap(pixmap)
    
    def closeEvent(self, event):
        """다이얼로그 닫을 때 수신기 정지"""
        self.receiver_9730.stop()
        self.receiver_9710.stop()
        event.accept()
