"""
비전 전용 위젯 (운송/청소 로봇용)
"""
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox
)
from PyQt6.QtCore import Qt


class VisionOnlyWidget(QWidget):
    """비전 시스템만 있는 위젯 (바퀴 제어 로봇용)"""
    
    def __init__(self, robot_name, robot_key, parent=None):
        super().__init__(parent)
        self.robot_name = robot_name
        self.robot_key = robot_key
        self._setup_ui()
    
    def _setup_ui(self):
        """UI 구성"""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(10)
        
        # 카메라 비전 그룹
        vision_group = QGroupBox("📷 Camera Vision")
        vision_layout = QVBoxLayout()
        
        cam_view = QLabel("카메라 대기 중...")
        cam_view.setFixedSize(640, 480)
        cam_view.setStyleSheet("background-color: black; border: 2px solid #555; border-radius: 4px;")
        cam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        cam_ctrl_layout = QHBoxLayout()
        for text, func in [
            ("🔌 CONNECT", lambda: None),
            ("❌ DISCONNECT", lambda: None),
            ("📸 CAPTURE", lambda: None)
        ]:
            btn = QPushButton(text)
            btn.setFixedSize(120, 40)
            btn.clicked.connect(func)
            cam_ctrl_layout.addWidget(btn)
        cam_ctrl_layout.addStretch()
        
        vision_layout.addWidget(cam_view)
        vision_layout.addLayout(cam_ctrl_layout)
        vision_group.setLayout(vision_layout)
        
        main_layout.addWidget(vision_group, alignment=Qt.AlignmentFlag.AlignTop)
        main_layout.addStretch()
        
        # 안내 메시지
        info_label = QLabel("💡 바퀴 제어 시스템은 추후 추가 예정입니다.")
        info_label.setStyleSheet("color: #888; font-size: 12px; font-style: italic;")
        main_layout.addWidget(info_label, alignment=Qt.AlignmentFlag.AlignCenter)
