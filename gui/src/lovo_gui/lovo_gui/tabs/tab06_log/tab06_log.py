"""
Log 탭
"""
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QCalendarWidget,
    QSizePolicy, QTableWidget, QTableWidgetItem, QHeaderView
)
from PyQt6.QtCore import Qt

class LogTab(QWidget):
    """Log 탭 - 시스템 로그"""
    
    def __init__(self, comm_manager, parent=None):
        super().__init__(parent)
        self.comm_manager = comm_manager
        self.api_client = comm_manager.api_client
        self.log_table = None
        self.filter_buttons = []
        self.button_handlers = [
            self.on_click_robot,
            self.on_click_orders,
            self.on_click_customer,
            self.on_click_charging_station,
            self.on_click_furniture,
            self.on_click_inventory_tx,
            self.on_click_robot_job,
            self.on_click_robot_state_log, # Reserved for fire incidents
            self.on_click_fire_log,
        ]
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
        # self.log_table.setRowCount(100) # Dynamic row count is better
        self.log_table.setHorizontalHeaderLabels(["시간", "내용", "상세1", "상세2"])
        
        # 헤더 설정
        header = self.log_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        
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
        button_names = [
            "1) Robot Status",
            "2) Orders",
            "3) Customers",
            "4) Charging Stations",
            "5) Furniture & Materials",
            "6) Inventory Tx Log",
            "7) Robot Jobs",
            "8) Robot State Log",
            "9) Fire Log",
        ]
        for text, handler in zip(button_names, self.button_handlers):
            btn = QPushButton(text)
            btn.setFixedWidth(330)
            btn.setFixedHeight(40)
            btn.setStyleSheet("""
                QPushButton {
                    text-align: left;
                    padding-left: 20px;
                    background-color: #333;
                    color: white;
                    border: 1px solid #555;
                    border-radius: 4px;
                }
                QPushButton:hover {
                    background-color: #444;
                }
                QPushButton:pressed {
                    background-color: #222;
                }
            """)
            btn.clicked.connect(handler)
            self.filter_buttons.append(btn)
            left_layout.addWidget(btn)
        
        left_layout.addStretch()
        
        return left_widget
    
    def _update_table(self, headers, data, keys):
        """테이블 갱신 (헤더 리스트, 데이터 리스트 of dicts, 키 리스트)"""
        self.log_table.clear()
        self.log_table.setColumnCount(len(headers))
        self.log_table.setHorizontalHeaderLabels(headers)
        
        self.log_table.setRowCount(len(data))
        
        for row_idx, item in enumerate(data):
            for col_idx, key in enumerate(keys):
                val = item.get(key, "")
                # Format specific values if needed
                if isinstance(val, (dict, list)):
                    val = str(val)
                    
                widget_item = QTableWidgetItem(str(val))
                widget_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.log_table.setItem(row_idx, col_idx, widget_item)
                
        self.log_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

    def filter_log_by_date(self):
        """날짜별 로그 필터링"""
        selected_date = self.calendar.selectedDate().toString("yyyy-MM-dd")
        print(f"📅 [{selected_date}] 날짜 선택됨 - DB 쿼리 시 날짜 필터링 적용 가능")
        # 현재는 버튼 클릭 시 전체 조회로 구현, 추후 API에 날짜 파라미터 추가 가능

    def filter_log_by_date(self):
        """날짜별 로그 필터링"""
        selected_date = self.calendar.selectedDate().toString("yyyy-MM-dd")
        print(f"📅 [{selected_date}] 날짜 선택됨")

    def on_click_robot(self):
        """1) Robot Status"""
        data = self.api_client.get_robots()
        headers = ["ID", "Role", "Kind", "State", "Batt", "X", "Y"]
        keys = ["robot_id", "robot_role", "robot_kind", "action_state", "battery_percent", "pose_x", "pose_y"]
        self._update_table(headers, data, keys)

    def on_click_orders(self):
        """2) Orders"""
        data = self.api_client.get_orders()
        headers = ["ID", "Customer", "Furniture", "Qty", "Status", "Time"]
        keys = ["order_id", "customer_name", "furniture_name", "quantity", "status", "ordered_at"]
        self._update_table(headers, data, keys)

    def on_click_customer(self):
        """3) Customers"""
        data = self.api_client.get_customers()
        headers = ["ID", "Name", "Phone", "Address"]
        keys = ["customer_id", "name", "phone", "address"]
        self._update_table(headers, data, keys)

    def on_click_charging_station(self):
        """4) Charging Stations"""
        data = self.api_client.get_charging_stations()
        headers = ["Name", "X", "Y", "Status"]
        keys = ["name", "x", "y", "status"]
        self._update_table(headers, data, keys)

    def on_click_furniture(self):
        """5) Furniture & Materials"""
        data = self.api_client.get_products()
        headers = ["ID", "Name", "Top", "Leg", "Wheel", "Kit"]
        keys = ["furniture_id", "name", "top_material_id", "leg_material_id", "wheel_material_id", "kit_material_id"]
        self._update_table(headers, data, keys)

    def on_click_inventory_tx(self):
        """6) Inventory Tx Log"""
        data = self.api_client.get_inventory_logs()
        headers = ["Tx ID", "Mat ID", "Qty", "Type", "Time"]
        keys = ["tx_id", "material_id", "qty_delta", "tx_type", "created_at"]
        self._update_table(headers, data, keys)

    def on_click_robot_job(self):
        """7) Robot Jobs"""
        data = self.api_client.get_robot_jobs()
        headers = ["Job ID", "Order", "Robot", "Type", "Status", "Time"]
        keys = ["job_id", "order_id", "robot_id", "job_type", "status", "created_at"]
        self._update_table(headers, data, keys)

    def on_click_robot_state_log(self):
        """8) Robot State Log"""
        data = self.api_client.get_robot_logs()
        headers = ["ID", "Robot", "State", "Bat", "Info", "Time"]
        keys = ["log_id", "robot_id", "state", "battery_level", "location_text", "logged_at"]
        self._update_table(headers, data, keys)

    def on_click_fire_log(self):
        """9) Fire Log"""
        data = self.api_client.get_fire_logs()
        headers = ["Incident ID", "Time", "Severity", "Location", "Image"]
        keys = ["incident_id", "occurred_at", "severity", "location", "image_path"]
        self._update_table(headers, data, keys)
