import sys
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import (QApplication, QWidget, QGridLayout, QLabel, 
                             QPushButton, QMessageBox, QProgressBar)
from PyQt5.QtCore import QTimer
import mysql.connector

class PinkyDashboard(QWidget):
    def __init__(self):
        super().__init__()
        if not rclpy.ok(): rclpy.init()
        self.ros_node = Node('pinky_gui_node')
        
        # [수정] YAML 규격 변경에 맞춰 토픽 경로 수정
        # 관제탑(Domain 59)에서의 리맵 토픽 이름
        self.zone_pubs = {
            "pinky1": self.ros_node.create_publisher(String, '/pinky1/task_zone/arrived', 10),
            "pinky2": self.ros_node.create_publisher(String, '/pinky2/task_zone/arrived', 10),
            "pinky3": self.ros_node.create_publisher(String, '/pinky3/task_zone/arrived', 10)
        }

        self.api_base_url = "http://10.182.25.77:5000/api"
        self.db_config = {'host': '10.182.25.77', 'user': 'lovoDB', 'password': 'LovoDB1234!', 'database': 'factory_system'}
        
        self.init_ui()

        self.sync_timer = QTimer()
        self.sync_timer.timeout.connect(self.update_tick)
        self.sync_timer.start(1000)

    def init_ui(self):
        self.setWindowTitle('Pinky Dashboard (Ready Signal Mode)')
        self.setGeometry(300, 300, 1000, 250)
        layout = QGridLayout()
        headers = ["기기명", "Battery", "Status", "OrderID", "주문 제어", "도착 알림 (Picking)", "도착 알림 (Packing)"]
        for col, h in enumerate(headers): layout.addWidget(QLabel(f"<b>{h}</b>"), 0, col)

        self.robots = {}
        for i, name in enumerate(["pinky1", "pinky2", "pinky3"]):
            row = i + 1
            layout.addWidget(QLabel(f"<b>{name.upper()}</b>"), row, 0)
            bat_bar = QProgressBar()
            layout.addWidget(bat_bar, row, 1)
            lbl_status = QLabel("오프라인"); layout.addWidget(lbl_status, row, 2)
            lbl_oid = QLabel("-"); layout.addWidget(lbl_oid, row, 3)

            btn_order = QPushButton("주문 받기")
            btn_order.clicked.connect(lambda checked, n=name: self.receive_order(n))
            layout.addWidget(btn_order, row, 4)

            # [수정] Picking Ready 버튼 (메시지 대문자화)
            btn_pick_ready = QPushButton("Pick Ready")
            btn_pick_ready.setStyleSheet("background-color: #e3f2fd;")
            btn_pick_ready.clicked.connect(lambda checked, n=name: self.send_ready_signal(n, "PICK_READY"))
            layout.addWidget(btn_pick_ready, row, 5)

            # [수정] Packing Ready 버튼 (메시지 대문자화)
            btn_pack_ready = QPushButton("Pack Ready")
            btn_pack_ready.setStyleSheet("background-color: #f1f8e9;")
            btn_pack_ready.clicked.connect(lambda checked, n=name: self.send_ready_signal(n, "PACK_READY"))
            layout.addWidget(btn_pack_ready, row, 6)

            self.robots[name] = {"bat_bar": bat_bar, "lbl_status": lbl_status, "lbl_oid": lbl_oid}
        self.setLayout(layout)

    def update_tick(self):
        self.sync_with_server()
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def sync_with_server(self):
        try:
            res = requests.get(f"{self.api_base_url}/robots/pinkies", timeout=0.5)
            if res.status_code == 200:
                for r in res.json():
                    rid = r.get('robot_role', '').lower().replace('_', '')
                    if rid in self.robots:
                        self.robots[rid]["bat_bar"].setValue(int(r['battery_percent']))
                        self.robots[rid]["lbl_status"].setText(r['action_state'])
                        self.robots[rid]["lbl_oid"].setText(f"ORD-{r['current_order_id']}" if r['current_order_id'] else "-")
        except: pass

    def send_ready_signal(self, rid, status):
        """로봇이 구역에 도착했음을 알리는 신호(Ready)를 발행합니다."""
        if rid in self.zone_pubs:
            msg = String()
            msg.data = status
            self.zone_pubs[rid].publish(msg)
            print(f"🚩 [READY SENT] {rid} -> {status} (Topic: /task_zone/arrived)")
            QMessageBox.information(self, "도착 알림", f"{rid}가 {status} 상태임을 서버에 알렸습니다.")

    def fetch_next_order(self):
        try:
            conn = mysql.connector.connect(**self.db_config)
            cursor = conn.cursor(dictionary=True)
            cursor.execute("SELECT order_id FROM orders WHERE status = 'RECEIVED' LIMIT 1 FOR UPDATE")
            order = cursor.fetchone()
            if order: 
                cursor.execute("UPDATE orders SET status = 'IN_PROGRESS' WHERE order_id = %s", (order['order_id'],))
                conn.commit(); return {"id": order['order_id']}
            return None
        except: return None
        finally: 
            if 'conn' in locals() and conn.is_connected():
                conn.close()

    def receive_order(self, rid):
        order = self.fetch_next_order()
        if order:
            try:
                conn = mysql.connector.connect(**self.db_config); cur = conn.cursor()
                cur.execute("UPDATE robot_pinky SET current_order_id = %s WHERE robot_role = %s", (order['id'], rid.upper()[:5] + "_" + rid[-1]))
                conn.commit(); conn.close()
            except: pass
            QMessageBox.information(self, "주문 수락", f"{rid} -> ORD-{order['id']}")
        else: QMessageBox.information(self, "알림", "대기 주문 없음")

if __name__ == '__main__':
    app = QApplication(sys.argv); ex = PinkyDashboard(); ex.show(); sys.exit(app.exec_())