"""
Main 탭
"""
from PyQt6.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit,
    QGridLayout, QPushButton, QSizePolicy
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap, QPainter
import cv2
import glob
import os
from datetime import datetime
from lovo_gui.constants import MAIN_SYSTEM_MAP, MAIN_ORDER_LOG, MAIN_ROBOT_GRID, MAIN_CAMERA_VIEW

# 카메라 크롭 설정 (픽셀 단위) - 여기서 조절하세요!
CROP_TOP = 140     # 위쪽 자르기
CROP_BOTTOM = 30  # 아래쪽 자르기
CROP_LEFT = 20      # 왼쪽 자르기
CROP_RIGHT = 40     # 오른쪽 자르기

# 카메라 반전 설정
CAMERA_FLIP_HORIZONTAL = True  # 좌우 반전 (미러링)
CAMERA_FLIP_VERTICAL = True    # 상하 반전

TOPVIEW_USB_PREFIX = "usb-LKZC_USB_Camera"
TOPVIEW_V4L_BY_ID_DIR = "/dev/v4l/by-id"
TOPVIEW_MAX_ZOOM = 5.0
TOPVIEW_MIN_ZOOM = 1.0
TOPVIEW_ZOOM_STEP = 1.1


class ZoomableCameraLabel(QLabel):
    """휠 입력을 외부로 전달하는 카메라 라벨"""
    wheel_zoomed = pyqtSignal(int, float, float)
    resized = pyqtSignal()
    drag_started = pyqtSignal(float, float)
    drag_moved = pyqtSignal(float, float)
    drag_ended = pyqtSignal()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._dragging = False
        self.setMouseTracking(True)

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta != 0:
            pos = event.position()
            self.wheel_zoomed.emit(delta, float(pos.x()), float(pos.y()))
            event.accept()
            return
        super().wheelEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.resized.emit()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = True
            pos = event.position()
            self.drag_started.emit(float(pos.x()), float(pos.y()))
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._dragging:
            pos = event.position()
            self.drag_moved.emit(float(pos.x()), float(pos.y()))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._dragging:
            self._dragging = False
            self.drag_ended.emit()
            event.accept()
            return
        super().mouseReleaseEvent(event)


class LocalCameraController(QThread):
    """로컬 USB 카메라 제어용 스레드"""
    frame_updated = pyqtSignal(QImage)
    camera_status_changed = pyqtSignal(bool, str)

    def __init__(self, camera_source: str):
        super().__init__()
        self.camera_source = camera_source
        self.running = False

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(self.camera_source)
        if not cap.isOpened():
            self.running = False
            self.camera_status_changed.emit(False, f"카메라 오픈 실패: {self.camera_source}")
            return

        self.camera_status_changed.emit(True, f"카메라 연결됨: {self.camera_source}")

        while self.running:
            ret, frame = cap.read()
            if ret:
                try:
                    # 반전 적용
                    if CAMERA_FLIP_HORIZONTAL and CAMERA_FLIP_VERTICAL:
                        frame = cv2.flip(frame, -1)
                    elif CAMERA_FLIP_HORIZONTAL:
                        frame = cv2.flip(frame, 1)
                    elif CAMERA_FLIP_VERTICAL:
                        frame = cv2.flip(frame, 0)

                    # 크롭 적용 (반전 후 기준으로 적용하여 조작 직관성 향상)
                    h, w, _ = frame.shape
                    # 음수 인덱싱 방지를 위한 범위 계산
                    start_y = CROP_TOP
                    end_y = h - CROP_BOTTOM
                    start_x = CROP_LEFT
                    end_x = w - CROP_RIGHT
                    
                    # 유효성 검사 (크롭 영역이 이미지보다 크면 원본 사용)
                    if start_y < end_y and start_x < end_x:
                        frame = frame[start_y:end_y, start_x:end_x]
                    
                    # OpenCV BGR -> RGB
                    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb_image.shape
                    bytes_per_line = ch * w
                    qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                    # QImage 복사본을 전달해야 안전함 (버퍼 문제 방지)
                    self.frame_updated.emit(qt_image.copy())
                except Exception:
                    pass
            else:
                self.running = False
                self.camera_status_changed.emit(False, f"카메라 프레임 읽기 실패: {self.camera_source}")
                break

        cap.release()

    def stop(self):
        self.running = False
        self.wait()


class MainTab(QWidget):
    """Main 탭 - 시스템 맵, 주문 로그, 로봇 상태, 카메라 뷰"""
    topview_camera_state_changed = pyqtSignal(bool)
    
    def __init__(self, robot_settings, comm_manager, parent=None):
        super().__init__(parent)
        self.robot_settings = robot_settings
        self.comm_manager = comm_manager  # 통신 매니저 (API Client 접근용)
        self.camera_title = None
        self.camera_view_label = None
        self.robot_status_labels = {}  # 로봇 상태 라벨들 저장 {robot_id: {'status': label, 'battery': label}}
        
        # 로컬 카메라 컨트롤러
        self.local_camera = None
        self.system_map_cam_label = None
        self._topview_camera_connected = False
        self._topview_zoom_factor = 1.0
        self._latest_topview_pixmap = None
        self._topview_pan_x = 0.0
        self._topview_pan_y = 0.0
        self._topview_drag_last_pos = None
        
        self._setup_ui()
        self._bind_connection_state_store()

        # 시작 시 1회 자동 연결 시도 (USB 탑뷰 카메라만 대상)
        QTimer.singleShot(0, self._auto_connect_topview_camera)
        
        # 데이터 갱신 타이머 (2초 간격)
        self.data_timer = QTimer(self)
        self.data_timer.timeout.connect(self._update_dashboard_data)
        self.data_timer.start(2000)
    
    def cleanup(self):
        """리소스 정리"""
        self.disconnect_topview_camera()

    def _bind_connection_state_store(self):
        """공통 연결 상태 스토어 구독"""
        if not hasattr(self.comm_manager, "state_store"):
            return
        self.comm_manager.state_store.robot_state_changed.connect(self._on_robot_state_changed)

    def _on_robot_state_changed(self, robot_id, state):
        labels = self.robot_status_labels.get(robot_id)
        if not labels:
            return

        ros_connected = state.get("ros_connected")
        ping_connected = state.get("ping_connected")

        status_label = labels.get("status")
        if status_label is None:
            return

        if ros_connected is True:
            self._set_status_cell_text(status_label, "ROS Online", "#1e7e34")
        elif ping_connected is True:
            self._set_status_cell_text(status_label, "Ping Online", "#ef6c00")
        elif ros_connected is False or ping_connected is False:
            self._set_status_cell_text(status_label, "Offline", "#b71c1c")
        else:
            self._set_status_cell_text(status_label, "Unknown", "#616161")

    def _set_status_cell_text(self, label, text, color):
        """통신 상태 셀 텍스트 갱신 (한 칸에 점+문구, 가운데 정렬)"""
        label.setText(f"●  {text}")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setStyleSheet(f"""
            background-color: white;
            color: {color};
            font-size: 12px;
            font-weight: bold;
            padding: 8px;
            border: 1px solid #ccc;
        """)

    def is_topview_camera_connected(self):
        """탑뷰 카메라 연결 상태"""
        return bool(self._topview_camera_connected)

    def _set_topview_camera_connected(self, connected: bool):
        new_state = bool(connected)
        if self._topview_camera_connected != new_state:
            self._topview_camera_connected = new_state
            self.topview_camera_state_changed.emit(new_state)
        else:
            self._topview_camera_connected = new_state

    def _topview_label_message(self, message: str):
        if self.system_map_cam_label:
            self.system_map_cam_label.setPixmap(QPixmap())
            self.system_map_cam_label.setText(message)

    def _is_topview_usb_candidate(self, path: str):
        base = os.path.basename(path)
        return base.startswith(TOPVIEW_USB_PREFIX)

    def _camera_path_priority(self, path: str):
        base = os.path.basename(path)
        if "video-index0" in base:
            return (0, base)
        if "video-index1" in base:
            return (1, base)
        return (2, base)

    def _camera_path_readable(self, path: str):
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            return False
        ok = False
        for _ in range(3):
            ret, _ = cap.read()
            if ret:
                ok = True
                break
        cap.release()
        return ok

    def _find_topview_usb_camera_path(self):
        pattern = os.path.join(TOPVIEW_V4L_BY_ID_DIR, "usb-*")
        candidates = [p for p in glob.glob(pattern) if self._is_topview_usb_candidate(p)]
        candidates = sorted(candidates, key=self._camera_path_priority)

        for path in candidates:
            if self._camera_path_readable(path):
                return path
        return None

    def connect_topview_camera(self):
        """탑뷰 USB 카메라 연결"""
        if self.is_topview_camera_connected():
            return True

        camera_path = self._find_topview_usb_camera_path()
        if not camera_path:
            self._set_topview_camera_connected(False)
            self._topview_label_message("USB 탑뷰카메라 미감지")
            return False

        self.disconnect_topview_camera()
        self._topview_zoom_factor = 1.0
        self._topview_pan_x = 0.0
        self._topview_pan_y = 0.0
        self._topview_drag_last_pos = None
        self._topview_label_message("탑뷰카메라 연결 중...")

        self.local_camera = LocalCameraController(camera_path)
        self.local_camera.frame_updated.connect(self._update_system_map_frame)
        self.local_camera.camera_status_changed.connect(self._on_local_camera_status_changed)
        self.local_camera.start()
        return True

    def _auto_connect_topview_camera(self):
        """앱 시작 시 탑뷰 카메라 자동 연결 1회 시도"""
        self._topview_label_message("탑뷰카메라 자동 연결 시도 중...")
        connected = self.connect_topview_camera()
        if not connected:
            self._topview_label_message("USB 탑뷰카메라 미감지")

    def disconnect_topview_camera(self):
        """탑뷰 카메라 연결 해제"""
        if self.local_camera:
            try:
                self.local_camera.frame_updated.disconnect(self._update_system_map_frame)
            except Exception:
                pass
            try:
                self.local_camera.camera_status_changed.disconnect(self._on_local_camera_status_changed)
            except Exception:
                pass
            self.local_camera.stop()
            self.local_camera = None

        self._set_topview_camera_connected(False)
        self._latest_topview_pixmap = None
        self._topview_zoom_factor = 1.0
        self._topview_pan_x = 0.0
        self._topview_pan_y = 0.0
        self._topview_drag_last_pos = None
        self._topview_label_message("탑뷰카메라 연결 버튼을 눌러 시작하세요")

    def toggle_topview_camera(self):
        """탑뷰 카메라 연결/해제 토글"""
        if self.is_topview_camera_connected():
            self.disconnect_topview_camera()
            return False
        return self.connect_topview_camera()

    def _on_local_camera_status_changed(self, connected: bool, message: str):
        self._set_topview_camera_connected(connected)
        if not connected:
            self._latest_topview_pixmap = None
            self._topview_zoom_factor = 1.0
            self._topview_pan_x = 0.0
            self._topview_pan_y = 0.0
            self._topview_drag_last_pos = None
            self._topview_label_message(message or "탑뷰카메라 연결 실패")

    def _update_dashboard_data(self):
        """대시보드 데이터 갱신 (서버 API 호출)"""
        # 서버 폴링이 비활성화된 경우 바로 리턴
        if hasattr(self.comm_manager, 'server_enabled') and not self.comm_manager.server_enabled:
            return

        # 1. 주문 로그 갱신
        orders = self.comm_manager.api_client.get_orders()
        if orders:
            self._update_order_log(orders)

        # 2. 로봇 상태 갱신
        robots_status = self.comm_manager.api_client.get_robots()
        if robots_status:
            self._update_robot_grid(robots_status)

    def _update_robot_grid(self, robots_data):
        """로봇 상태 그리드 갱신"""
        # robots_data: [{'robot_id': 1, 'action_state': '...', 'battery_percent': 80, ...}]
        
        # map API data by string ID if possible, or just iterate
        # 여기서는 robot_id가 integer라고 가정하고, GUI 설정의 id와 매핑 시도
        # GUI id가 "robot1" 형식이면 "1" 부분과 매칭하거나, 
        # 혹은 단순히 순서대로 매칭하거나 해야 함. 
        # 현재는 DB robot_id와 GUI robot['domain'] 또는 id가 일치한다고 가정하기 어려움.
        # 따라서 DB에 있는 'robot_kind'나 'role'을 보고 추측하거나,
        # 단순하게 battery/status만 업데이트.
        
        # 임시: API 응답의 robot_id를 문자열로 변환하여 GUI id와 매칭 시도
        api_robot_map = {str(r['robot_id']): r for r in robots_data}
        
        for r_id, labels in self.robot_status_labels.items():
            # GUI id가 'robot1' -> '1'로 변환 시도
            key = r_id
            if r_id.startswith('robot'):
                key = r_id.replace('robot', '')
            
            data = api_robot_map.get(key)
            if data:
                # 배터리
                bat = data.get('battery_percent', 0)
                labels['battery'].setText(f"{bat}%")
                
                # 로봇 하는일 텍스트
                work = data.get('action_state', '업무 대기')
                labels['work'].setText(work)
                
                # 연결 표시 (데이터가 들어오면 온라인으로 간주)
                # 하지만 로봇 직접 연결(Ping)과 서버 DB상 상태는 다를 수 있음
                # 여기서는 DB 상태를 우선시하거나, Ping 상태와 병기해야 함.
                # 우선은 배터리와 상태 텍스트만 업데이트.

    def _update_order_log(self, orders):
        """주문 로그 뷰어 업데이트"""
        if not self.order_log_viewer:
            return
            
        # 현재 텍스트 유지 여부 결정 (스크롤 등을 위해)
        # 여기서는 매번 새로 쓰지 않고, 포맷팅해서 다시 보여주는 방식을 택함 (간단히)
        
        log_text = ""
        for order in orders:
            # order dict structure: 
            # {'order_id': 1, 'customer_name': '...', 'furniture_name': '...', 'quantity': 1, 'status': 'PENDING', 'ordered_at': '...'}
            try:
                oid = order.get('order_id', '?')
                cust = order.get('customer_name', 'Unknown')
                item = order.get('furniture_name', 'Item')
                qty = order.get('quantity', 1)
                status = order.get('status', 'UNKNOWN')
                time_str = order.get('ordered_at', '')
                
                # 시간 포맷팅 (datetime object일 수도 있고 string일 수도 있음)
                if isinstance(time_str, str) and 'T' in time_str:
                    time_str = time_str.split('T')[1][:8]  # HH:MM:SS
                
                line = f"[{time_str}] Order #{oid}: {item} x{qty} ({cust}) - {status}"
                log_text += line + "\n"
            except Exception:
                continue
                
        self.order_log_viewer.setText(log_text)
        # 스크롤 최하단?
        # self.order_log_viewer.verticalScrollBar().setValue(self.order_log_viewer.verticalScrollBar().maximum())

    def _setup_ui(self):
        """UI 구성"""
        # 좌상단: 시스템 맵
        self._create_system_map()
        
        # 우상단: 주문 로그
        self._create_order_log()
        
        # 좌하단: 로봇 상태 그리드
        self._create_robot_grid()
        
        # 우하단: 카메라 뷰
        self._create_camera_view()

    def _create_section_title(self, text: str):
        """메인 섹션 공통 타이틀 라벨"""
        title = QLabel(text)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(30)
        title.setStyleSheet("""
            background-color: #3f3f3f;
            color: #f2f2f2;
            font-size: 13px;
            font-weight: bold;
            border: 1px solid #555;
        """)
        return title
    
    def _create_system_map(self):
        """시스템 맵 (USB 카메라)"""
        x, y, w, h = MAIN_SYSTEM_MAP
        system_map = QFrame(self)
        system_map.setGeometry(x, y, w, h)
        system_map.setStyleSheet("QFrame { background-color: #000000; border: none; }")
        
        layout = QVBoxLayout(system_map)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        title = self._create_section_title("탑뷰 영상 (실시간)")
        layout.addWidget(title)
        
        # 카메라 영상 표시용 라벨
        self.system_map_cam_label = ZoomableCameraLabel("탑뷰카메라 연결 버튼을 눌러 시작하세요")
        self.system_map_cam_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.system_map_cam_label.setStyleSheet("color: white;")
        self.system_map_cam_label.wheel_zoomed.connect(self._on_topview_wheel_zoom)
        self.system_map_cam_label.drag_started.connect(self._on_topview_drag_started)
        self.system_map_cam_label.drag_moved.connect(self._on_topview_drag_moved)
        self.system_map_cam_label.drag_ended.connect(self._on_topview_drag_ended)
        self.system_map_cam_label.resized.connect(self._render_topview_frame)
        layout.addWidget(self.system_map_cam_label)

    def _update_system_map_frame(self, image):
        """시스템 맵 카메라 프레임 갱신"""
        if self.system_map_cam_label:
            self._set_topview_camera_connected(True)
            self._latest_topview_pixmap = QPixmap.fromImage(image)
            self._render_topview_frame()

    def _topview_fit_size(self, label_w, label_h):
        if not self._latest_topview_pixmap:
            return 0.0, 0.0
        src_w = self._latest_topview_pixmap.width()
        src_h = self._latest_topview_pixmap.height()
        if src_w <= 0 or src_h <= 0 or label_w <= 0 or label_h <= 0:
            return 0.0, 0.0
        scale = min(label_w / src_w, label_h / src_h)
        return src_w * scale, src_h * scale

    def _clamp_topview_pan(self, label_w, label_h, zoom_w, zoom_h, base_x, base_y):
        if zoom_w <= label_w:
            self._topview_pan_x = 0.0
        else:
            min_pan_x = label_w - zoom_w - base_x
            max_pan_x = -base_x
            self._topview_pan_x = min(max(self._topview_pan_x, min_pan_x), max_pan_x)

        if zoom_h <= label_h:
            self._topview_pan_y = 0.0
        else:
            min_pan_y = label_h - zoom_h - base_y
            max_pan_y = -base_y
            self._topview_pan_y = min(max(self._topview_pan_y, min_pan_y), max_pan_y)

    def _on_topview_wheel_zoom(self, delta, mouse_x, mouse_y):
        """휠 스크롤로 탑뷰 화면 확대/축소"""
        if not self._latest_topview_pixmap:
            return

        label_w = self.system_map_cam_label.width()
        label_h = self.system_map_cam_label.height()
        fit_w, fit_h = self._topview_fit_size(label_w, label_h)
        if fit_w <= 0 or fit_h <= 0:
            return

        old_zoom = self._topview_zoom_factor
        if delta > 0:
            new_zoom = min(TOPVIEW_MAX_ZOOM, old_zoom * TOPVIEW_ZOOM_STEP)
        else:
            new_zoom = max(TOPVIEW_MIN_ZOOM, old_zoom / TOPVIEW_ZOOM_STEP)

        if abs(new_zoom - old_zoom) < 1e-9:
            return

        old_zoom_w = fit_w * old_zoom
        old_zoom_h = fit_h * old_zoom
        old_base_x = (label_w - old_zoom_w) / 2.0
        old_base_y = (label_h - old_zoom_h) / 2.0

        rel_x = None
        rel_y = None
        if old_zoom_w > 0 and old_zoom_h > 0:
            rel_x = (mouse_x - (old_base_x + self._topview_pan_x)) / old_zoom_w
            rel_y = (mouse_y - (old_base_y + self._topview_pan_y)) / old_zoom_h

        self._topview_zoom_factor = new_zoom

        if rel_x is not None and rel_y is not None and 0.0 <= rel_x <= 1.0 and 0.0 <= rel_y <= 1.0:
            new_zoom_w = fit_w * new_zoom
            new_zoom_h = fit_h * new_zoom
            new_base_x = (label_w - new_zoom_w) / 2.0
            new_base_y = (label_h - new_zoom_h) / 2.0
            self._topview_pan_x = mouse_x - new_base_x - (rel_x * new_zoom_w)
            self._topview_pan_y = mouse_y - new_base_y - (rel_y * new_zoom_h)

        zoom_w = fit_w * self._topview_zoom_factor
        zoom_h = fit_h * self._topview_zoom_factor
        base_x = (label_w - zoom_w) / 2.0
        base_y = (label_h - zoom_h) / 2.0
        self._clamp_topview_pan(label_w, label_h, zoom_w, zoom_h, base_x, base_y)
        self._render_topview_frame()

    def _on_topview_drag_started(self, x, y):
        if not self._latest_topview_pixmap or self._topview_zoom_factor <= TOPVIEW_MIN_ZOOM:
            self._topview_drag_last_pos = None
            return
        self._topview_drag_last_pos = (x, y)

    def _on_topview_drag_moved(self, x, y):
        if not self._latest_topview_pixmap or self._topview_drag_last_pos is None:
            return

        last_x, last_y = self._topview_drag_last_pos
        dx = x - last_x
        dy = y - last_y
        self._topview_drag_last_pos = (x, y)

        self._topview_pan_x += dx
        self._topview_pan_y += dy
        self._render_topview_frame()

    def _on_topview_drag_ended(self):
        self._topview_drag_last_pos = None

    def _render_topview_frame(self):
        """현재 줌 비율 기준으로 탑뷰 프레임 렌더링"""
        if not self.system_map_cam_label or not self._latest_topview_pixmap:
            return

        label_w = self.system_map_cam_label.width()
        label_h = self.system_map_cam_label.height()
        if label_w <= 0 or label_h <= 0:
            return

        fit_w, fit_h = self._topview_fit_size(label_w, label_h)
        if fit_w <= 0 or fit_h <= 0:
            return

        fit_pix = self._latest_topview_pixmap.scaled(
            int(fit_w),
            int(fit_h),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )

        zoom_w = max(1, int(fit_pix.width() * self._topview_zoom_factor))
        zoom_h = max(1, int(fit_pix.height() * self._topview_zoom_factor))
        zoom_pix = fit_pix.scaled(
            zoom_w,
            zoom_h,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )

        canvas = QPixmap(label_w, label_h)
        canvas.fill(Qt.GlobalColor.black)

        base_x = (label_w - zoom_pix.width()) / 2.0
        base_y = (label_h - zoom_pix.height()) / 2.0
        self._clamp_topview_pan(label_w, label_h, zoom_pix.width(), zoom_pix.height(), base_x, base_y)

        painter = QPainter(canvas)
        draw_x = int(round(base_x + self._topview_pan_x))
        draw_y = int(round(base_y + self._topview_pan_y))
        painter.drawPixmap(draw_x, draw_y, zoom_pix)
        painter.end()

        self.system_map_cam_label.setPixmap(canvas)

    def _create_order_log(self):
        """주문 로그"""
        x, y, w, h = MAIN_ORDER_LOG
        order_log_frame = QFrame(self)
        order_log_frame.setGeometry(x, y, w, h)
        order_log_frame.setStyleSheet("QFrame { background-color: #f5f5f5; border: none; }")
        
        layout = QVBoxLayout(order_log_frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        title = self._create_section_title("주문 로그 (실시간)")
        layout.addWidget(title)
        
        self.order_log_viewer = QTextEdit()
        self.order_log_viewer.setReadOnly(True)
        self.order_log_viewer.setStyleSheet("""
            background-color: white;
            color: #333;
            font-family: Consolas;
            font-size: 11px;
            border: 1px solid #ccc;
        """)
        layout.addWidget(self.order_log_viewer)
    
    def _create_robot_grid(self):
        """로봇 상태 그리드"""
        x, y, w, h = MAIN_ROBOT_GRID
        grid_container = QWidget(self)
        grid_container.setGeometry(x, y, w, h)
        grid_container.setStyleSheet("background-color: white;")
        
        grid_layout = QGridLayout(grid_container)
        grid_layout.setSpacing(0)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        
        # 헤더
        headers = ["로봇 이름", "통신 연결 상태", "로봇 하는일", "배터리 잔량", "캠 연결"]
        for col, header in enumerate(headers):
            header_label = QLabel(header)
            header_label.setStyleSheet("""
                background-color: #4a4a4a;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 10px;
                border: 1px solid #333;
            """)
            header_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(header_label, 0, col)

        # 통신/업무 칸을 분리해 보이도록 컬럼 비율 설정
        grid_layout.setColumnStretch(0, 1)
        grid_layout.setColumnStretch(1, 1)
        grid_layout.setColumnStretch(2, 8)
        grid_layout.setColumnStretch(3, 1)
        grid_layout.setColumnStretch(4, 1)
        
        # 로봇별 정보
        robots = self.robot_settings.get_robots()
        for row, robot in enumerate(robots, start=1):
            robot_id = robot.get("id")
            robot_name = robot.get("name", f"로봇 {row}")
            
            # 로봇 이름
            name_label = QLabel(robot_name)
            name_label.setStyleSheet("""
                background-color: #f0f0f0;
                color: black;
                font-size: 13px;
                padding: 8px;
                border: 1px solid #ccc;
            """)
            name_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(name_label, row, 0)
            
            # 통신 연결 상태
            status_label = QLabel()
            self._set_status_cell_text(status_label, "Offline", "#b71c1c")
            grid_layout.addWidget(status_label, row, 1)
            
            # 로봇 하는일
            work_label = QLabel("업무 대기")
            work_label.setStyleSheet("""
                background-color: white;
                color: black;
                font-size: 12px;
                padding: 8px;
                border: 1px solid #ccc;
            """)
            work_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(work_label, row, 2)

            # 배터리 잔량
            battery_label = QLabel("-")
            battery_label.setStyleSheet("""
                background-color: white;
                color: black;
                font-size: 13px;
                padding: 8px;
                border: 1px solid #ccc;
            """)
            battery_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(battery_label, row, 3)
            
            # 캠 연결 버튼
            cam_btn = QPushButton("📷 CAM")
            cam_btn.setFixedSize(80, 35)
            cam_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2196F3;
                    color: white;
                    font-size: 12px;
                    font-weight: bold;
                    border-radius: 4px;
                    border: none;
                }
                QPushButton:hover {
                    background-color: #1976D2;
                }
                QPushButton:pressed {
                    background-color: #0D47A1;
                }
            """)
            cam_btn.clicked.connect(lambda checked, r=robot: self.show_camera_view(r))
            
            btn_container = QWidget()
            btn_container.setStyleSheet("background-color: white; border: 1px solid #ccc;")
            btn_layout = QHBoxLayout(btn_container)
            btn_layout.setContentsMargins(0, 0, 0, 0)
            btn_layout.addWidget(cam_btn, alignment=Qt.AlignmentFlag.AlignCenter)
            grid_layout.addWidget(btn_container, row, 4)
            
            # 나중에 업데이트를 위해 저장
            self.robot_status_labels[robot_id] = {
                'status': status_label,
                'battery': battery_label,
                'work': work_label
            }

    def _create_camera_view(self):
        """카메라 뷰"""
        x, y, w, h = MAIN_CAMERA_VIEW
        self.camera_view_frame = QFrame(self)
        self.camera_view_frame.setGeometry(x, y, w, h)
        self.camera_view_frame.setStyleSheet("""
            QFrame {
                background-color: #2a2a2a;
                border: 1px solid #555;
                border-radius: 4px;
            }
        """)
        
        layout = QVBoxLayout(self.camera_view_frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        self.camera_title = self._create_section_title("카메라 뷰")
        layout.addWidget(self.camera_title)
        
        self.camera_view_label = QLabel("캠 버튼을 눌러 카메라를 선택하세요")
        self.camera_view_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.camera_view_label.setMinimumSize(0, 0)
        self.camera_view_label.setStyleSheet(
            "background-color: black; border: 1px solid #444; border-radius: 4px; color: #666;"
        )
        self.camera_view_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.camera_view_label, 1)

        # 현재 연결된 카메라 컨트롤러 참조 (frame_updated 연결/해제 관리)
        self._current_camera_controller = None
        self._camera_frame_slot = None
    
    def show_camera_view(self, robot):
        """카메라 뷰 표시"""
        robot_name = robot.get("name", "로봇")
        self.camera_view_label.setText(f"{robot_name} 카메라 스트리밍 대기 중...")

        # 윈도우 레벨에서 CommunicationTab의 CameraController를 가져와 프레임 업데이트를 연결
        try:
            win = self.window()
            cam_ctrl = None
            if hasattr(win, 'communication_tab'):
                # robot dict에는 id가 있어야 함
                robot_id = robot.get('id')
                cam_ctrl = win.communication_tab.get_camera_controller(robot_id)

            # 이전 컨트롤러와 연결되어 있으면 해제
            if hasattr(self, '_current_camera_controller') and self._current_camera_controller:
                try:
                    if self._camera_frame_slot and self._current_camera_controller:
                        self._current_camera_controller.frame_updated.disconnect(self._camera_frame_slot)
                except Exception:
                    pass
                self._current_camera_controller = None
                self._camera_frame_slot = None

            if cam_ctrl is None:
                self.camera_view_label.setText("카메라가 연결되어 있지 않습니다")
                return

            # 슬롯 생성 및 연결
            def _on_frame(frame):
                try:
                    # OpenCV BGR -> RGB
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb.shape
                    bytes_per_line = ch * w
                    qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                    pix = QPixmap.fromImage(qimg).scaled(
                        self.camera_view_label.width(),
                        self.camera_view_label.height(),
                        Qt.AspectRatioMode.KeepAspectRatio
                    )
                    self.camera_view_label.setPixmap(pix)
                except Exception:
                    pass

            # 연결 저장 및 시그널 연결
            self._camera_frame_slot = _on_frame
            cam_ctrl.frame_updated.connect(self._camera_frame_slot)
            self._current_camera_controller = cam_ctrl

            # 상태 텍스트 제거
            self.camera_view_label.setText("")
        except Exception:
            # 안전하게 실패
            self.camera_view_label.setText("카메라 표시 실패")
