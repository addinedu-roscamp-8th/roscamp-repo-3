"""
Monitoring 탭
"""
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QFrame, QLabel
from PyQt6.QtCore import Qt


class MonitoringTab(QWidget):
    """Monitoring 탭 - 시스템 모니터링"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()
    
    def _setup_ui(self):
        """UI 구성"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        
        # 모니터링 맵
        monitoring_map = QFrame()
        monitoring_map.setFixedSize(1600, 800)
        monitoring_map.setStyleSheet("""
            QFrame {
                background-color: #e8e8e8;
                border: 2px solid #999;
                border-radius: 4px;
            }
        """)
        
        map_layout = QVBoxLayout(monitoring_map)
        map_label = QLabel("모니터링 맵", alignment=Qt.AlignmentFlag.AlignCenter)
        map_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #555;")
        map_layout.addWidget(map_label)
        
        layout.addWidget(monitoring_map)
        layout.addStretch()
