"""
Log 탭
"""
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QCalendarWidget,
    QTextEdit, QSizePolicy, QTableWidget, QTableWidgetItem, QHeaderView
)
from PyQt6.QtCore import Qt


class LogTab(QWidget):
    """Log 탭 - 시스템 로그"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.log_table = None
        self._setup_ui()
    
    def _setup_ui(self):
        """UI 구성"""
        layout = QHBoxLayout(self)
        
        # 왼쪽 패널
        left_panel = self._create_left_panel()
        layout.addWidget(left_panel)
        
        # 오른쪽: 로그 테이블
        self.log_table = QTableWidget()
        self.log_table.setColumnCount(4)
        self.log_table.setRowCount(25)
        self.log_table.setHorizontalHeaderLabels(["시간", "탭1", "탭2", "탭3"])
        
        # 헤더 설정
        header = self.log_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        
        # 스타일 설정
        self.log_table.setStyleSheet("""
            QTableWidget {
                background-color: #1e1e1e;
                color: #d4d4d4;
                gridline-color: #3c3c3c;
                border: 1px solid #3c3c3c;
            }
            QTableWidget::item {
                padding: 5px;
            }
            QHeaderView::section {
                background-color: #2d2d30;
                color: #ffffff;
                padding: 8px;
                border: 1px solid #3c3c3c;
                font-weight: bold;
            }
        """)
        
        # 행 번호 헤더 설정
        self.log_table.verticalHeader().setDefaultSectionSize(30)
        
        layout.addWidget(self.log_table)
    
    def _create_left_panel(self):
        """왼쪽 패널 (캘린더 + 버튼)"""
        left_widget = QWidget()
        left_widget.setFixedWidth(350)
        left_layout = QVBoxLayout(left_widget)
        
        # 캘린더
        self.calendar = QCalendarWidget()
        self.calendar.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.calendar.setFixedWidth(330)
        self.calendar.setGridVisible(True)
        self.calendar.selectionChanged.connect(self.filter_log_by_date)
        left_layout.addWidget(self.calendar)
        
        # 버튼들
        for text in ["자재", "모니터링", "알람"]:
            btn = QPushButton(text)
            btn.setFixedWidth(260)
            btn.setFixedHeight(40)
            left_layout.addWidget(btn)
        
        left_layout.addStretch()
        
        return left_widget
    
    def filter_log_by_date(self):
        """날짜별 로그 필터링"""
        selected_date = self.calendar.selectedDate().toString("yyyy-MM-dd")
        # TODO: DB에서 해당 날짜의 로그 조회 및 테이블 업데이트
        print(f"📅 [{selected_date}] 날짜 선택됨")
