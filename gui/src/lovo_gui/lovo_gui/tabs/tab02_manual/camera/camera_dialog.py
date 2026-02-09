"""
카메라 뷰 위젯
"""
import time
import json
import yaml
import cv2
from pathlib import Path
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
    QSpinBox, QDoubleSpinBox, QCheckBox, QGridLayout, QComboBox, QRadioButton, QButtonGroup
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
import numpy as np

# 알고리즘 모듈 (robot_arm에서 임포트)
from ..robot_arm.algorithm import (
    CoordinateTransformer,
    HandEyeTransformer,
    ArucoMarkerProcessor,
    PickupSequence
)

# ArUco 마커 감지를 위한 라이브러리
ARUCO_AVAILABLE = False
aruco = None

try:
    import cv2.aruco as aruco
    ARUCO_AVAILABLE = True
except ImportError:
    try:
        from cv2 import aruco
        ARUCO_AVAILABLE = True
    except ImportError:
        print(f"⚠️ ArUco 모듈을 찾을 수 없습니다")
        ARUCO_AVAILABLE = False


class CameraWidget(QWidget):
    """카메라 뷰 위젯"""
    
    # Signal
    work_log_signal = pyqtSignal(str)  # 작업 로그 메시지
    
    def __init__(self, robot_name, parent=None):
        super().__init__(parent)
        self.robot_name = robot_name
        self.camera_controller = None
        self.robot_dashboard = None  # 좌표 저장을 위한 로봇 대시보드 참조
        self.is_aligning = False
        self.align_frame = None
        
        # ArUco 감지 결과 저장
        self.aruco_detected = False
        self.aruco_frozen_frame = None  # 정지된 프레임
        self.aruco_target_coords = None  # 감지된 마커의 로봇 좌표 [x, y, z, r, p, yaw]
        self.aruco_marker_id = None
        
        # 픽업 시퀀스 관리
        self.pickup_sequence = PickupSequence()
        
        # 캘리브레이션 데이터 로드
        self.camera_matrix = None
        self.dist_coeffs = None
        self.hand_eye_matrix = None
        self._load_calibration_data()
        
        # 좌표 변환 알고리즘 내부 인스턴스
        self.coord_transformer = CoordinateTransformer()
        self.hand_eye_transformer = None
        if self.hand_eye_matrix is not None and self.camera_matrix is not None:
            self.hand_eye_transformer = HandEyeTransformer(self.hand_eye_matrix, self.camera_matrix)
        
        # ArUco 마커 감지기 초기화
        self.aruco_dict = None
        self.aruco_detector = None
        self.aruco_processor = None
        
        if ARUCO_AVAILABLE and aruco is not None:
            try:
                # 4x4 마커 사용 (작은 마커에 적합)
                self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
                try:
                    params = aruco.DetectorParameters()
                    self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, params)
                except Exception as e1:
                    self.aruco_detector = None  # 구 API 사용
                
                # ArucoMarkerProcessor 인스턴스 생성
                self.aruco_processor = ArucoMarkerProcessor()
            except Exception as e:
                print(f"⚠️ ArUco 초기화 실패: {e}")
                self.aruco_dict = None
                self.aruco_detector = None
        
        # ArUco 감지 파라미터
        self.aruco_params = {
            'adaptiveThreshConstant': 7,
            'minMarkerPerimeterRate': 0.03,
            'maxMarkerPerimeterRate': 4.0,
            'polygonalApproxAccuracyRate': 0.02,
            'minCornerDistanceRate': 0.02,
            'minDistanceToBorder': 3,
            'minMarkerDistanceRate': 0.05,
            'cornerRefinementMethod': 0,
            'cornerRefinementWinSize': 5,
            'cornerRefinementMaxIterations': 30,
            'minOtsuStdDev': 5.0,
            'errorCorrectionRate': 0.6,
            'detectInvertedMarker': False,
            'perspectiveRemovePixelPerCell': 4,
            'perspectiveRemoveIgnoredMarginPerCell': 13,
            'maxMarkerDistanceRate': 0.73,
            'polygonalApproxAccuracyRateScale': 0.1
        }
        
        self._setup_ui()
    
    def _setup_ui(self):
        """UI 구성"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(3, 2, 3, 2)
        layout.setSpacing(2)
        
        vision_group = QGroupBox(f"📷 {self.robot_name} Camera")
        vision_group.setStyleSheet("""
            QGroupBox { 
                font-size: 10px; 
                padding-top: 8px; 
                margin-top: 0px; 
            }
            QGroupBox::title { 
                subcontrol-origin: margin; 
                subcontrol-position: top left; 
                padding: 0px 3px;
                top: -2px;
            }
        """)
        vision_layout = QVBoxLayout()
        vision_layout.setContentsMargins(2, 5, 2, 2)
        vision_layout.setSpacing(2)
        
        # 카메라 뷰
        self.cam_view = QLabel("카메라 대기 중...")
        self.cam_view.setFixedSize(640, 480)
        self.cam_view.setStyleSheet("background-color: black; color: white; border: 1px solid #555; border-radius: 4px;")
        self.cam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vision_layout.addWidget(self.cam_view)
        
        # 컨트롤 버튼 (2줄로 배치, 4개씩)
        btn_container = QWidget()
        btn_container.setMaximumWidth(640)
        btn_layout = QVBoxLayout(btn_container)
        btn_layout.setContentsMargins(0, 5, 0, 0)
        btn_layout.setSpacing(3)
        
        # ============ 버튼 크기 설정 (여기서 수정) ============
        BTN_WIDTH = 100   # 버튼 가로 크기 (축소)
        BTN_HEIGHT = 24   # 버튼 세로 크기 (축소)
        # =====================================================
        
        # 첫 번째 줄: Connect, 캡쳐, Live, Align
        btn_row1 = QHBoxLayout()
        btn_row1.setSpacing(3)
        
        self.btn_connect = QPushButton("🔌 Connect")
        self.btn_connect.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_connect.clicked.connect(self._camera_connect)
        btn_row1.addWidget(self.btn_connect)
        
        self.btn_capture = QPushButton("📸 캡쳐")
        self.btn_capture.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_capture.setEnabled(False)
        self.btn_capture.clicked.connect(self._camera_capture)
        btn_row1.addWidget(self.btn_capture)
        
        self.btn_live = QPushButton("📺 Live")
        self.btn_live.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_live.setEnabled(False)
        self.btn_live.clicked.connect(self._camera_live)
        btn_row1.addWidget(self.btn_live)
        
        self.btn_align = QPushButton("🎯 Align")
        self.btn_align.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_align.setEnabled(False)
        self.btn_align.clicked.connect(self._camera_align)
        btn_row1.addWidget(self.btn_align)
        
        btn_row1.addStretch()
        btn_layout.addLayout(btn_row1)
        
        # 두 번째 줄: Disconnect, 캡쳐+좌표, Pickup, ArUco
        btn_row2 = QHBoxLayout()
        btn_row2.setSpacing(3)
        
        self.btn_disconnect = QPushButton("❌ Disconnect")
        self.btn_disconnect.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.clicked.connect(self._camera_disconnect)
        btn_row2.addWidget(self.btn_disconnect)
        
        self.btn_capture_with_coords = QPushButton("📸 캡쳐+좌표")
        self.btn_capture_with_coords.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_capture_with_coords.setEnabled(False)
        self.btn_capture_with_coords.clicked.connect(self._camera_capture_with_coords)
        btn_row2.addWidget(self.btn_capture_with_coords)
        
        self.btn_pickup = QPushButton("🤖 Pickup")
        self.btn_pickup.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_pickup.setEnabled(False)
        self.btn_pickup.clicked.connect(self._camera_pickup)
        btn_row2.addWidget(self.btn_pickup)
        
        self.btn_aruco = QPushButton("📌 ArUco")
        self.btn_aruco.setFixedSize(BTN_WIDTH, BTN_HEIGHT)
        self.btn_aruco.setEnabled(False)
        self.btn_aruco.clicked.connect(self._test_aruco_step_by_step)
        btn_row2.addWidget(self.btn_aruco)
        
        btn_row2.addStretch()
        btn_layout.addLayout(btn_row2)
        vision_layout.addWidget(btn_container)
        vision_group.setLayout(vision_layout)
        layout.addWidget(vision_group)
        
        # 파라미터 토글 섹션 (라디오 버튼)
        toggle_layout = QHBoxLayout()
        toggle_layout.addStretch()
        
        # 라디오 버튼 그룹
        self.param_button_group = QButtonGroup(self)
        
        self.radio_show_aruco = QRadioButton("🔧 ArUco 파라미터")
        self.radio_show_aruco.setChecked(True)
        self.radio_show_aruco.toggled.connect(self._toggle_parameters)
        self.param_button_group.addButton(self.radio_show_aruco)
        toggle_layout.addWidget(self.radio_show_aruco)
        
        self.radio_show_coord = QRadioButton("📐 좌표 변환 파라미터")
        self.radio_show_coord.toggled.connect(self._toggle_parameters)
        self.param_button_group.addButton(self.radio_show_coord)
        toggle_layout.addWidget(self.radio_show_coord)
        
        toggle_layout.addStretch()
        layout.addLayout(toggle_layout)
        
        # ArUco 파라미터 설정 섹션 (640px 너비 맞춤)
        param_group = QGroupBox("🔧 ArUco 감지 파라미터")
        param_group.setStyleSheet("""
            QGroupBox { 
                font-size: 9px; 
                padding-top: 6px; 
                margin-top: 0px; 
            }
            QGroupBox::title { 
                subcontrol-origin: margin; 
                subcontrol-position: top left; 
                padding: 0px 2px;
                top: -2px;
            }
        """)
        param_container = QWidget()
        param_container.setMaximumWidth(480)
        param_container.setMaximumHeight(80)
        param_layout = QGridLayout(param_container)
        param_layout.setContentsMargins(2, 2, 2, 2)
        param_layout.setSpacing(2)
        
        # 주요 파라미터들 (2x3 그리드로 축소)
        param_layout.addWidget(QLabel("최소 둘레율"), 0, 0)
        self.spin_min_perimeter = QDoubleSpinBox()
        self.spin_min_perimeter.setRange(0.01, 0.5)
        self.spin_min_perimeter.setValue(self.aruco_params['minMarkerPerimeterRate'])
        self.spin_min_perimeter.setSingleStep(0.01)
        self.spin_min_perimeter.setMaximumWidth(80)
        self.spin_min_perimeter.setFixedHeight(20)
        self.spin_min_perimeter.valueChanged.connect(self._update_aruco_param)
        param_layout.addWidget(self.spin_min_perimeter, 0, 1)
        
        param_layout.addWidget(QLabel("최대 둘레율"), 0, 2)
        self.spin_max_perimeter = QDoubleSpinBox()
        self.spin_max_perimeter.setRange(0.5, 10.0)
        self.spin_max_perimeter.setValue(self.aruco_params['maxMarkerPerimeterRate'])
        self.spin_max_perimeter.setSingleStep(0.1)
        self.spin_max_perimeter.setMaximumWidth(80)
        self.spin_max_perimeter.valueChanged.connect(self._update_aruco_param)
        param_layout.addWidget(self.spin_max_perimeter, 0, 3)
        
        param_layout.addWidget(QLabel("다각형 정확도"), 1, 0)
        self.spin_polygon_accu = QDoubleSpinBox()
        self.spin_polygon_accu.setRange(0.01, 0.1)
        self.spin_polygon_accu.setValue(self.aruco_params['polygonalApproxAccuracyRate'])
        self.spin_polygon_accu.setSingleStep(0.001)
        self.spin_polygon_accu.setMaximumWidth(80)
        self.spin_polygon_accu.valueChanged.connect(self._update_aruco_param)
        param_layout.addWidget(self.spin_polygon_accu, 1, 1)
        
        param_layout.addWidget(QLabel("Otsu 표준편차"), 1, 2)
        self.spin_min_otsu = QDoubleSpinBox()
        self.spin_min_otsu.setRange(0.0, 20.0)
        self.spin_min_otsu.setValue(self.aruco_params['minOtsuStdDev'])
        self.spin_min_otsu.setSingleStep(0.5)
        self.spin_min_otsu.setMaximumWidth(80)
        self.spin_min_otsu.valueChanged.connect(self._update_aruco_param)
        param_layout.addWidget(self.spin_min_otsu, 1, 3)
        
        param_layout.addWidget(QLabel("반전 마커"), 2, 0)
        self.check_inverted = QCheckBox()
        self.check_inverted.setChecked(self.aruco_params['detectInvertedMarker'])
        self.check_inverted.toggled.connect(self._update_aruco_param)
        param_layout.addWidget(self.check_inverted, 2, 1)
        
        param_layout.addWidget(QLabel("오류 정정율"), 2, 2)
        self.spin_error_correction = QDoubleSpinBox()
        self.spin_error_correction.setRange(0.0, 1.0)
        self.spin_error_correction.setValue(self.aruco_params['errorCorrectionRate'])
        self.spin_error_correction.setSingleStep(0.05)
        self.spin_error_correction.setMaximumWidth(80)
        self.spin_error_correction.valueChanged.connect(self._update_aruco_param)
        param_layout.addWidget(self.spin_error_correction, 2, 3)
        
        param_group.setLayout(param_layout)
        self.aruco_group = param_group  # 참조 저장
        layout.addWidget(param_group)
        
        # 좌표 변환 파라미터 그룹
        coord_group = QGroupBox("📐 좌표 변환 파라미터")
        coord_group.setStyleSheet("""
            QGroupBox { 
                font-size: 9px; 
                padding-top: 6px; 
                margin-top: 0px; 
            }
            QGroupBox::title { 
                subcontrol-origin: margin; 
                subcontrol-position: top left; 
                padding: 0px 2px;
                top: -2px;
            }
        """)
        coord_container = QWidget()
        coord_container.setMaximumWidth(480)
        coord_container.setMaximumHeight(80)
        coord_layout = QGridLayout(coord_container)
        coord_layout.setContentsMargins(2, 2, 2, 2)
        coord_layout.setSpacing(2)
        
        # 1행: 스케일 X, Y
        coord_layout.addWidget(QLabel("스케일X"), 0, 0)
        self.spin_scale_x = QDoubleSpinBox()
        self.spin_scale_x.setRange(0.01, 5.0)
        self.spin_scale_x.setValue(0.5)
        self.spin_scale_x.setSingleStep(0.05)
        self.spin_scale_x.setMaximumWidth(80)
        self.spin_scale_x.setFixedHeight(20)
        coord_layout.addWidget(self.spin_scale_x, 0, 1)
        
        coord_layout.addWidget(QLabel("스케일Y"), 0, 2)
        self.spin_scale_y = QDoubleSpinBox()
        self.spin_scale_y.setRange(0.01, 5.0)
        self.spin_scale_y.setValue(0.5)
        self.spin_scale_y.setSingleStep(0.05)
        self.spin_scale_y.setMaximumWidth(80)
        self.spin_scale_y.setFixedHeight(20)
        coord_layout.addWidget(self.spin_scale_y, 0, 3)
        
        # 2행: 부호 X, Y
        coord_layout.addWidget(QLabel("X부호"), 1, 0)
        self.combo_sign_x = QComboBox()
        self.combo_sign_x.addItems(["+", "-"])
        self.combo_sign_x.setMaximumWidth(80)
        self.combo_sign_x.setFixedHeight(20)
        coord_layout.addWidget(self.combo_sign_x, 1, 1)
        
        coord_layout.addWidget(QLabel("Y부호"), 1, 2)
        self.combo_sign_y = QComboBox()
        self.combo_sign_y.addItems(["+", "-"])
        self.combo_sign_y.setMaximumWidth(80)
        self.combo_sign_y.setFixedHeight(20)
        coord_layout.addWidget(self.combo_sign_y, 1, 3)
        
        # 3행: H-E 캘리브레이션, Z 오프셋, 스왑
        coord_layout.addWidget(QLabel("H-E"), 2, 0)
        self.check_use_hand_eye = QCheckBox("캘리브레이션")
        self.check_use_hand_eye.setChecked(False)
        coord_layout.addWidget(self.check_use_hand_eye, 2, 1)
        
        coord_layout.addWidget(QLabel("Z오프셋"), 2, 2)
        self.spin_z_offset = QDoubleSpinBox()
        self.spin_z_offset.setRange(-500.0, 500.0)
        self.spin_z_offset.setValue(-100.0)
        self.spin_z_offset.setSingleStep(10.0)
        self.spin_z_offset.setMaximumWidth(80)
        self.spin_z_offset.setFixedHeight(20)
        coord_layout.addWidget(self.spin_z_offset, 2, 3)
        
        coord_layout.addWidget(QLabel("스왑"), 2, 4)
        self.check_swap_xy = QCheckBox("X↔Y")
        coord_layout.addWidget(self.check_swap_xy, 2, 5)
        
        coord_group.setLayout(coord_layout)
        self.coord_group = coord_group  # 참조 저장
        layout.addWidget(coord_group)
        
        # 초기에는 ArUco 파라미터만 표시 (라디오 버튼 기본값과 일치)
        self.coord_group.setVisible(False)
        
        # 파라미터 토글 버튼 (상단에 추가하기 위해 나중에 처리)
        # 여기서는 UI 구조 정의만 하고, 토글 기능은 아래에서 추가
        self.check_show_aruco = None  # 나중에 설정
        
        layout.addStretch()
    
    def set_camera_controller(self, camera_controller):
        """카메라 컨트롤러 설정 및 Signal 연결"""
        self.camera_controller = camera_controller
        
        if camera_controller:
            # QueuedConnection으로 스레드 안전하게 연결
            from PyQt6.QtCore import Qt
            camera_controller.frame_updated.connect(
                self.update_camera_frame, 
                Qt.ConnectionType.QueuedConnection
            )
            camera_controller.connection_changed.connect(
                self.on_camera_connection_changed,
                Qt.ConnectionType.QueuedConnection
            )

    def set_robot_dashboard(self, robot_dashboard):
        """로봇 대시보드 참조 설정 (좌표 저장용)"""
        self.robot_dashboard = robot_dashboard
    
    def _load_calibration_data(self):
        """카메라 및 핸드-아이 캘리브레이션 데이터 로드"""
        try:
            calibration_file = Path.home() / "roscamp-repo-3" / "gui" / "systemData" / "jetcobot_2" / "hand_eye_result.yaml"
            
            if not calibration_file.exists():
                msg = f"ℹ️ 캘리브레이션 파일 없음: {calibration_file}"
                print(msg)
                try:
                    self.work_log_signal.emit(msg)
                except:
                    pass
                return
            
            with open(calibration_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # YAML 파싱 시도
            try:
                data = yaml.safe_load(content)
            except yaml.YAMLError:
                return
            
            if data is None:
                msg = f"ℹ️ 캘리브레이션 파일이 비어있음"
                print(msg)
                try:
                    self.work_log_signal.emit(msg)
                except:
                    pass
                return
            
            # 카메라 내부 파라미터
            if 'K' in data and data['K'] and 'data' in data['K']:
                self.camera_matrix = np.array(data['K']['data']).reshape(3, 3)
                msg = f"📷 카메라 매트릭스 K:\n{self.camera_matrix}"
                print(msg)
                try:
                    self.work_log_signal.emit(msg)
                except:
                    pass
            
            # 왜곡 계수
            if 'D' in data and data['D'] and 'data' in data['D']:
                self.dist_coeffs = np.array(data['D']['data']).reshape(-1)
                msg = f"🔧 왜곡 계수 D: {self.dist_coeffs}"
                print(msg)
                try:
                    self.work_log_signal.emit(msg)
                except:
                    pass
            
            # 핸드-아이 캘리브레이션 행렬 (카메라-로봇 관계)
            if 'T_hand_eye' in data and data['T_hand_eye'] and 'data' in data['T_hand_eye']:
                self.hand_eye_matrix = np.array(data['T_hand_eye']['data']).reshape(4, 4)
                msg = f"🤖 Hand-Eye 변환 행렬 T_hand_eye:\n{self.hand_eye_matrix}"
                print(msg)
                try:
                    self.work_log_signal.emit(msg)
                except:
                    pass
            else:
                msg = f"⚠️ Hand-Eye 변환 행렬이 없습니다!"
                print(msg)
                try:
                    self.work_log_signal.emit(msg)
                except:
                    pass
            
            msg = f"✅ 캘리브레이션 데이터 로드 완료: {calibration_file}"
            print(msg)
            try:
                self.work_log_signal.emit(msg)
            except:
                pass
        except Exception as e:
            print(f"⚠️ 캘리브레이션 데이터 로드 실패: {e}")
    
    def _update_aruco_param(self):
        """ArUco 파라미터 업데이트"""
        self.aruco_params['minMarkerPerimeterRate'] = self.spin_min_perimeter.value()
        self.aruco_params['maxMarkerPerimeterRate'] = self.spin_max_perimeter.value()
        self.aruco_params['polygonalApproxAccuracyRate'] = self.spin_polygon_accu.value()
        self.aruco_params['minOtsuStdDev'] = self.spin_min_otsu.value()
        self.aruco_params['detectInvertedMarker'] = self.check_inverted.isChecked()
        self.aruco_params['errorCorrectionRate'] = self.spin_error_correction.value()
        
        print(f"📋 ArUco 파라미터 업데이트됨")
    
    def _toggle_parameters(self):
        """파라미터 그룹 토글 - 라디오 버튼 사용"""
        # ArUco 파라미터가 선택되면 ArUco 표시, 아니면 좌표 변환 표시
        show_aruco = self.radio_show_aruco.isChecked()
        
        self.aruco_group.setVisible(show_aruco)
        self.coord_group.setVisible(not show_aruco)
    
    def _hand_eye_transform(self, pixel_x, pixel_y, camera_z=500.0):
        """
        Hand-Eye 캘리브레이션을 사용한 좌표 변환
        픽셀 좌표 → 카메라 좌표 → 로봇 좌표
        
        Args:
            pixel_x: 카메라 이미지의 픽셀 X (중심 기준 오프셋)
            pixel_y: 카메라 이미지의 픽셀 Y (중심 기준 오프셋)
            camera_z: 카메라 기울임 때문에 Z 거리 추정 (mm)
        
        Returns:
            [x, y, z] 로봇 좌표계의 오프셋
        """
        
        if self.hand_eye_matrix is None or self.camera_matrix is None:
            log_msg = f"⚠️ Hand-Eye 캘리브레이션 데이터 없음 - 간단한 변환 사용"
            print(log_msg)
            try:
                self.work_log_signal.emit(log_msg)
            except:
                pass
            
            # Fall back to simple scaling
            scale_x = self.spin_scale_x.value()
            scale_y = self.spin_scale_y.value()
            return [pixel_x * scale_x, pixel_y * scale_y, 0]
        
        try:
            # 카메라 내부 파라미터에서 초점거리와 주점 추출
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            
            log_msg = f"📸 카메라 파라미터: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}"
            print(log_msg)
            try:
                self.work_log_signal.emit(log_msg)
            except:
                pass
            
            # 카메라 좌표 계산 (역 카메라 행렬 사용)
            # 픽셀 좌표 → 정규화 이미지 좌표 → 카메라 3D 좌표
            x_normalized = pixel_x / fx
            y_normalized = pixel_y / fy
            
            # 카메라 3D 좌표 (Z = camera_z)
            cam_x = x_normalized * camera_z
            cam_y = y_normalized * camera_z
            cam_z = camera_z
            
            log_msg = f"📷 카메라 좌표: X={cam_x:.1f}, Y={cam_y:.1f}, Z={cam_z:.1f}"
            print(log_msg)
            try:
                self.work_log_signal.emit(log_msg)
            except:
                pass
            
            # Hand-Eye 변환 행렬 적용
            # 카메라 좌표 → 로봇 베이스 좌표
            cam_point = np.array([cam_x, cam_y, cam_z, 1.0])  # 동차 좌표
            
            robot_point = self.hand_eye_matrix @ cam_point  # 4x4 행렬 × 4x1 벡터
            
            robot_x = robot_point[0]
            robot_y = robot_point[1]
            robot_z = robot_point[2]
            
            log_msg = f"🤖 Hand-Eye 변환 후 로봇 좌표: X={robot_x:.1f}, Y={robot_y:.1f}, Z={robot_z:.1f}"
            print(log_msg)
            try:
                self.work_log_signal.emit(log_msg)
            except:
                pass
            
            return [robot_x, robot_y, robot_z]
            
        except Exception as e:
            log_msg = f"❌ Hand-Eye 변환 오류: {e}"
            print(log_msg)
            try:
                self.work_log_signal.emit(log_msg)
            except:
                pass
            
            # Fall back to simple scaling
            scale_x = self.spin_scale_x.value()
            scale_y = self.spin_scale_y.value()
            return [pixel_x * scale_x, pixel_y * scale_y, 0]
    
    
    def _get_aruco_detector(self):
        """현재 파라미터로 ArUco 감지기 생성"""
        if not ARUCO_AVAILABLE:
            return None
        
        try:
            if self.aruco_detector is not None and hasattr(self.aruco_detector, '__class__'):
                # OpenCV 4.7.0+
                params = aruco.DetectorParameters()
                params.adaptiveThreshConstant = self.aruco_params['adaptiveThreshConstant']
                params.minMarkerPerimeterRate = self.aruco_params['minMarkerPerimeterRate']
                params.maxMarkerPerimeterRate = self.aruco_params['maxMarkerPerimeterRate']
                params.polygonalApproxAccuracyRate = self.aruco_params['polygonalApproxAccuracyRate']
                params.minCornerDistanceRate = self.aruco_params['minCornerDistanceRate']
                params.minDistanceToBorder = self.aruco_params['minDistanceToBorder']
                params.minMarkerDistanceRate = self.aruco_params['minMarkerDistanceRate']
                params.cornerRefinementMethod = self.aruco_params['cornerRefinementMethod']
                params.cornerRefinementWinSize = self.aruco_params['cornerRefinementWinSize']
                params.cornerRefinementMaxIterations = self.aruco_params['cornerRefinementMaxIterations']
                params.minOtsuStdDev = self.aruco_params['minOtsuStdDev']
                params.errorCorrectionRate = self.aruco_params['errorCorrectionRate']
                params.detectInvertedMarker = self.aruco_params['detectInvertedMarker']
                
                return aruco.ArucoDetector(self.aruco_dict, params)
            else:
                return None
        except Exception as e:
            print(f"⚠️ ArUco 감지기 생성 실패: {e}")
            return None
    
    def _camera_connect(self):
        """카메라 연결"""
        if self.camera_controller:
            self.camera_controller.start()
    
    def _camera_disconnect(self):
        """카메라 연결 해제"""
        if self.camera_controller:
            self.camera_controller.stop()
    
    def _camera_capture(self):
        """카메라 캡처"""
        if self.camera_controller:
            # 절대 경로 사용 (home 디렉토리 기준)
            home_dir = Path.home()
            captures_dir = home_dir / "lovo_ws" / "captures" / self.robot_name
            
            # 타임스탬프 기반 파일명 생성
            timestamp = int(time.time())
            filename = captures_dir / f"capture_{timestamp}.jpg"
            
            # 캡처 저장 (경로 전달, 폴더 생성은 controller에서 처리)
            result = self.camera_controller.capture(str(filename))
            if result:
                log_msg = f"📸 캡처 저장됨: {filename}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
            else:
                log_msg = f"⚠️ 캡처 저장 실패"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
    
    def _camera_live(self):
        """Live 버튼 - ArUco 감지 상태를 해제하고 라이브 영상으로 복귀"""
        if self.aruco_detected:
            self.aruco_detected = False
            self.aruco_frozen_frame = None
            self.aruco_target_coords = None
            self.aruco_marker_id = None
            log_msg = "📺 라이브 영상 모드로 복귀"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
        else:
            log_msg = "📺 이미 라이브 영상 모드입니다"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
    
    def _camera_align(self):
        """얼라인 실행 - 현재 프레임에 십자가 표시"""
        if self.camera_controller and self.camera_controller.latest_frame is not None:
            self.is_aligning = True
            # 현재 프레임을 얼라인 프레임으로 저장
            self.align_frame = self.camera_controller.latest_frame.copy()
            self._draw_align_crosshair()
            print(f"🎯 {self.robot_name} 얼라인 포인트 설정 완료")
        else:
            print(f"⚠️ {self.robot_name} 카메라가 연결되지 않았습니다")
    
    def _camera_pickup(self):
        """픽업 실행 - 단계별 진행 (Step 0: 이동, Step 1: GRIP, Step 2: Z축 상승)"""
        try:
            # ArUco 감지된 좌표가 있는지 확인
            if not self.aruco_detected or self.aruco_target_coords is None:
                log_msg = "⚠️ 먼저 ArUco 감지 버튼을 눌러 마커를 감지하세요"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                return
            
            # 로봇 컨트롤러 확인
            if not self.robot_dashboard or not hasattr(self.robot_dashboard, 'controller') or not self.robot_dashboard.controller:
                log_msg = "⚠️ 로봇 컨트롤러가 연결되지 않았습니다"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                return
            
            controller = self.robot_dashboard.controller
            
            # 단계별 실행
            if self.pickup_step == 0:
                # Step 0: 마커 위치로 이동
                if hasattr(controller, 'robot_connected') and not controller.robot_connected:
                    log_msg = "⚠️ 로봇 연결 상태가 Offline 입니다 (픽업 명령 전송은 시도합니다)"
                    print(log_msg)
                    self.work_log_signal.emit(log_msg)
                else:
                    log_msg = "✅ 로봇 연결 상태: Online"
                    print(log_msg)
                    self.work_log_signal.emit(log_msg)

                if hasattr(controller, 'send_servo'):
                    controller.send_servo(True)
                    log_msg = "🔌 Servo ON 요청"
                    print(log_msg)
                    self.work_log_signal.emit(log_msg)
                
                target = [float(c) for c in self.aruco_target_coords]
                
                log_msg = f"🤖 PICKUP Step 1/3 - 마커 위치로 이동 (ID: {self.aruco_marker_id})"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                log_msg = f"   Target: X={target[0]:.1f}, Y={target[1]:.1f}, Z={target[2]:.1f}, R={target[3]:.1f}, P={target[4]:.1f}, Yaw={target[5]:.1f}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
                # 현재 좌표 출력
                if hasattr(controller, 'current_coords'):
                    current = controller.current_coords
                    log_msg = f"   Current: X={current[0]:.1f}, Y={current[1]:.1f}, Z={current[2]:.1f}, R={current[3]:.1f}, P={current[4]:.1f}, Yaw={current[5]:.1f}"
                    print(log_msg)
                    self.work_log_signal.emit(log_msg)
                
                # 토픽 이름 확인
                if hasattr(controller, 'pub_target_coords'):
                    log_msg = f"   📡 토픽: {controller.pub_target_coords.topic_name}"
                    print(log_msg)
                    self.work_log_signal.emit(log_msg)
                
                print(f"[DEBUG] publish_coords 호출: {target}")
                controller.publish_coords(target)
                print(f"[DEBUG] publish_coords 완료")
                
                self.pickup_step = 1
                self.btn_pickup.setText("✊ GRIP")
                log_msg = "💡 이동 명령 전송 완료! 다시 버튼을 눌러 GRIP 하세요."
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
            elif self.pickup_step == 1:
                # Step 1: GRIP 실행
                log_msg = "✊ PICKUP Step 2/3 - GRIP 실행"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
                controller.send_gripper(1)  # 1 = GRIP
                
                self.pickup_step = 2
                self.btn_pickup.setText("⬆️ 들기")
                log_msg = "💡 GRIP 명령 전송 완료! 다시 버튼을 눌러 Z축 상승하세요."
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                print(f"[DEBUG] pickup_step 변경: 1 → {self.pickup_step}")
                
            elif self.pickup_step == 2:
                # Step 2: Z축 100mm 상승 (현재 로봇 위치 기준)
                log_msg = f"[Step 3 진입] pickup_step={self.pickup_step}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
                current_coords = list(controller.current_coords)
                current_coords[2] += 100.0  # Z축만 100mm 상승
                
                log_msg = f"⬆️ PICKUP Step 3/3 - Z축 100mm 상승"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                log_msg = f"   현재 Z: {controller.current_coords[2]:.1f}mm → 목표 Z: {current_coords[2]:.1f}mm"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
                log_msg = f"[상승 명령] 전송 좌표: {current_coords}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
                controller.publish_coords(current_coords)
                
                log_msg = "✅ Z축 상승 명령 완료!"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
                self.pickup_step = 0
                self.btn_pickup.setText("🤖 Pickup")
                log_msg = f"✅ PICKUP 완료! 마커 ID: {self.aruco_marker_id}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
                # ArUco 상태 초기화 (영상 재개)
                self.aruco_detected = False
                self.aruco_frozen_frame = None
            
        except Exception as e:
            log_msg = f"❌ PICKUP 오류: {e}"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            import traceback
            traceback.print_exc()

    def _camera_capture_with_coords(self):
        """카메라 캡처 및 현재 좌표 저장"""
        if not self.camera_controller:
            log_msg = "⚠️ 카메라가 연결되지 않았습니다"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            return

        home_dir = Path.home()
        captures_dir = home_dir / "lovo_ws" / "captures" / self.robot_name

        timestamp = int(time.time())
        image_filename = captures_dir / f"capture_{timestamp}.jpg"
        coords_filename = captures_dir / f"coords_{timestamp}.json"

        result = self.camera_controller.capture(str(image_filename))
        if not result:
            log_msg = "❌ 이미지 캡처 실패"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            return

        if self.robot_dashboard and hasattr(self.robot_dashboard, 'pose_target_inputs'):
            try:
                coords = []
                for i in range(6):
                    if self.robot_dashboard.pose_target_inputs[i]:
                        text = self.robot_dashboard.pose_target_inputs[i].text()
                        coords.append(float(text) if text else 0.0)
                    else:
                        coords.append(0.0)

                data = {
                    "timestamp": timestamp,
                    "robot_name": self.robot_name,
                    "image": image_filename.name,
                    "coordinates": {
                        "X": coords[0],
                        "Y": coords[1],
                        "Z": coords[2],
                        "R": coords[3],
                        "P": coords[4],
                        "Y": coords[5]
                    }
                }

                with open(coords_filename, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)

                log_msg = (
                    f"✅ 캡처+좌표 저장 완료: {image_filename.name}, {coords_filename.name}"
                )
                print(log_msg)
                self.work_log_signal.emit(log_msg)
            except Exception as e:
                log_msg = f"❌ 좌표 저장 오류: {e}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
        else:
            log_msg = "⚠️ 로봇 대시보드가 연결되지 않았습니다"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
    
    def _test_aruco_step_by_step(self):
        """단계별 ArUco 테스트 - 1.프레임확인 2.오버레이 3.ArUco"""
        try:
            # Step 1: 프레임 수신 확인
            log_msg = "🔍 Step 1: 프레임 수신 확인..."
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            
            if not self.camera_controller:
                log_msg = "❌ 카메라 컨트롤러가 없습니다"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                return
            
            if not hasattr(self.camera_controller, 'latest_frame'):
                log_msg = "❌ latest_frame 속성이 없습니다"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                return
            
            frame = self.camera_controller.latest_frame
            if frame is None:
                log_msg = "❌ 프레임이 None입니다"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                return
            
            log_msg = f"✅ 프레임 수신 OK - shape: {frame.shape}, dtype: {frame.dtype}"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            
            # Step 2: 프레임 복사 및 오버레이 테스트
            log_msg = "🔍 Step 2: 오버레이 테스트..."
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            
            display_frame = frame.copy()
            h, w = display_frame.shape[:2]
            cx, cy = w // 2, h // 2
            
            # 십자가 그리기
            cv2.line(display_frame, (cx - 50, cy), (cx + 50, cy), (0, 255, 0), 2)
            cv2.line(display_frame, (cx, cy - 50), (cx, cy + 50), (0, 255, 0), 2)
            cv2.circle(display_frame, (cx, cy), 10, (0, 255, 0), 2)
            
            # 텍스트 그리기
            cv2.putText(display_frame, f"Frame: {w}x{h}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Center: ({cx}, {cy})", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            log_msg = f"✅ 오버레이 그리기 완료"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            
            # Step 3: QImage 변환 테스트
            log_msg = "🔍 Step 3: QImage 변환..."
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            
            # BGR -> RGB
            rgb_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            rgb_frame = np.ascontiguousarray(rgb_frame)
            
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            
            # QImage 생성
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
            
            log_msg = f"✅ QImage 생성 완료 - {qt_image.width()}x{qt_image.height()}"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            
            # Step 4: Pixmap 변환 및 표시
            log_msg = "🔍 Step 4: Pixmap 표시..."
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.cam_view.width(), 
                self.cam_view.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            self.cam_view.setPixmap(scaled_pixmap)
            
            log_msg = f"✅ 오버레이 표시 완료!"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            
            # Step 5: ArUco 감지 (선택적)
            if ARUCO_AVAILABLE and self.aruco_dict is not None:
                log_msg = "🔍 Step 5: ArUco 감지 시도..."
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # 구 API 사용 (DetectorParameters_create)
                try:
                    params = aruco.DetectorParameters_create()
                    # 작은 마커를 위한 파라미터 조정
                    params.minMarkerPerimeterRate = 0.01  # 더 작은 마커 허용
                    params.maxMarkerPerimeterRate = 4.0
                    params.polygonalApproxAccuracyRate = 0.05  # 더 관대하게
                    params.minCornerDistanceRate = 0.01
                    params.minOtsuStdDev = 3.0  # 낮추기
                    params.adaptiveThreshWinSizeMin = 3
                    params.adaptiveThreshWinSizeMax = 23
                    params.adaptiveThreshWinSizeStep = 10
                    
                    corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=params)
                    
                    log_msg = f"📊 감지 결과: ids={ids is not None and len(ids) or 0}, rejected={len(rejected) if rejected else 0}"
                    print(log_msg)
                    self.work_log_signal.emit(log_msg)
                    
                    if ids is not None and len(ids) > 0:
                        marker_id = int(ids[0][0])
                        marker_corners = corners[0][0]
                        mcx = int(np.mean(marker_corners[:, 0]))
                        mcy = int(np.mean(marker_corners[:, 1]))
                        
                        log_msg = f"✅ ArUco 마커 감지! ID={marker_id}, 픽셀 좌표=({mcx}, {mcy})"
                        print(log_msg)
                        self.work_log_signal.emit(log_msg)
                        
                        # 마커 ID 저장
                        self.aruco_marker_id = marker_id
                        
                        # Step 6: 픽셀 → 로봇 좌표 변환
                        log_msg = "🔍 Step 6: 상세 좌표 변환..."
                        print(log_msg)
                        self.work_log_signal.emit(log_msg)
                        
                        # 카메라 중심 기준 오프셋 (픽셀)
                        center_x, center_y = w // 2, h // 2
                        offset_px_x = mcx - center_x  # 양수 = 오른쪽
                        offset_px_y = mcy - center_y  # 양수 = 아래쪽
                        
                        log_msg = f"📸 카메라 해상도: {w}x{h}, 중심: ({center_x}, {center_y}), 마커 위치: ({mcx}, {mcy})"
                        print(log_msg)
                        self.work_log_signal.emit(log_msg)
                        
                        log_msg = f"📏 카메라 중심 오프셋 (픽셀): X={offset_px_x}, Y={offset_px_y}"
                        print(log_msg)
                        self.work_log_signal.emit(log_msg)
                        
                        # 픽셀 → mm 변환 (스케일 팩터 - UI에서 조정 가능)
                        scale_x = self.spin_scale_x.value()  # mm/pixel (X축)
                        scale_y = self.spin_scale_y.value()  # mm/pixel (Y축)
                        
                        offset_mm_x = offset_px_x * scale_x
                        offset_mm_y = offset_px_y * scale_y
                        
                        log_msg = f"📐 스케일 팩터: X={scale_x}, Y={scale_y} (mm/pixel)"
                        print(log_msg)
                        self.work_log_signal.emit(log_msg)
                        
                        log_msg = f"📐 변환된 오프셋 (mm): X={offset_mm_x:.1f}, Y={offset_mm_y:.1f}"
                        print(log_msg)
                        self.work_log_signal.emit(log_msg)
                        
                        # Step 7: 로봇 좌표 계산 및 저장
                        if self.robot_dashboard and hasattr(self.robot_dashboard, 'pose_actual_labels'):
                            try:
                                # 현재 로봇 실제 좌표 가져오기
                                current_x = float(self.robot_dashboard.pose_actual_labels[0].text() or 0)
                                current_y = float(self.robot_dashboard.pose_actual_labels[1].text() or 0)
                                current_z = float(self.robot_dashboard.pose_actual_labels[2].text() or 0)
                                current_r = float(self.robot_dashboard.pose_actual_labels[3].text() or 0)
                                current_p = float(self.robot_dashboard.pose_actual_labels[4].text() or 0)
                                current_yaw = float(self.robot_dashboard.pose_actual_labels[5].text() or 0)
                                
                                log_msg = f"🤖 현재 로봇 좌표: X={current_x:.1f}, Y={current_y:.1f}, Z={current_z:.1f}, R={current_r:.1f}, P={current_p:.1f}, Yaw={current_yaw:.1f}"
                                print(log_msg)
                                self.work_log_signal.emit(log_msg)
                                
                                # Hand-Eye 캘리브레이션 사용 여부
                                if self.check_use_hand_eye.isChecked() and self.hand_eye_matrix is not None:
                                    log_msg = f"✅ Hand-Eye 캘리브레이션 적용"
                                    print(log_msg)
                                    self.work_log_signal.emit(log_msg)
                                    
                                    # Hand-Eye 변환 사용
                                    hand_eye_offset = self._hand_eye_transform(offset_px_x, offset_px_y, camera_z=500.0)
                                    target_x = current_x + hand_eye_offset[0]
                                    target_y = current_y + hand_eye_offset[1]
                                    target_z = current_z + hand_eye_offset[2] + self.spin_z_offset.value()
                                    
                                    log_msg = f"🤖 Hand-Eye 오프셋: X={hand_eye_offset[0]:.1f}, Y={hand_eye_offset[1]:.1f}, Z={hand_eye_offset[2]:.1f}"
                                    print(log_msg)
                                    self.work_log_signal.emit(log_msg)
                                else:
                                    log_msg = f"❌ 간단한 스케일 변환 사용 (Hand-Eye 미적용)"
                                    print(log_msg)
                                    self.work_log_signal.emit(log_msg)
                                    
                                    # 부호 적용
                                    sign_x = 1.0 if self.combo_sign_x.currentText() == "+" else -1.0
                                    sign_y = 1.0 if self.combo_sign_y.currentText() == "+" else -1.0
                                    
                                    # 축 스왑 여부 확인
                                    if self.check_swap_xy.isChecked():
                                        # X, Y 축 스왑
                                        target_x = current_x + sign_y * offset_mm_y
                                        target_y = current_y + sign_x * offset_mm_x
                                        log_msg = f"🔄 축 스왑 ON: 카메라 X({offset_mm_x:.1f}) → 로봇 Y, 카메라 Y({offset_mm_y:.1f}) → 로봇 X"
                                    else:
                                        # 직접 매핑
                                        target_x = current_x + sign_x * offset_mm_x
                                        target_y = current_y + sign_y * offset_mm_y
                                        log_msg = f"🔄 축 스왑 OFF: 카메라 X({offset_mm_x:.1f}) → 로봇 X, 카메라 Y({offset_mm_y:.1f}) → 로봇 Y"
                                    
                                    print(log_msg)
                                    self.work_log_signal.emit(log_msg)
                                    
                                    # Z축 오프셋 적용 (카메라와 로봇 높이 차이 보정)
                                    target_z = current_z + self.spin_z_offset.value()
                                
                                # 목표 좌표 저장 (PICKUP에서 사용)
                                self.aruco_target_coords = [target_x, target_y, target_z, current_r, current_p, current_yaw]
                                self.aruco_detected = True
                                self.pickup_step = 0  # 픽업 단계 초기화
                                self.btn_pickup.setText("🤖 Pickup")  # 버튼 텍스트 초기화
                                
                                log_msg = f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                                print(log_msg)
                                self.work_log_signal.emit(log_msg)
                                
                                log_msg = f"🤖 [현재 위치] X={current_x:.1f}, Y={current_y:.1f}, Z={current_z:.1f}, R={current_r:.1f}, P={current_p:.1f}, Yaw={current_yaw:.1f}"
                                print(log_msg)
                                self.work_log_signal.emit(log_msg)
                                
                                log_msg = f"🎯 [목표 위치] X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}, R={current_r:.1f}, P={current_p:.1f}, Yaw={current_yaw:.1f}"
                                print(log_msg)
                                self.work_log_signal.emit(log_msg)
                                
                                log_msg = f"📍 [차이] ΔX={target_x - current_x:.1f}, ΔY={target_y - current_y:.1f}, ΔZ={target_z - current_z:.1f}"
                                print(log_msg)
                                self.work_log_signal.emit(log_msg)
                                
                                log_msg = f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                                print(log_msg)
                                self.work_log_signal.emit(log_msg)
                                
                                log_msg = f"💡 PICKUP 버튼을 눌러 마커 위치로 이동하세요!"
                                print(log_msg)
                                self.work_log_signal.emit(log_msg)
                                    
                            except Exception as e:
                                log_msg = f"❌ 좌표 변환 오류: {e}"
                                print(log_msg)
                                self.work_log_signal.emit(log_msg)
                                import traceback
                                traceback.print_exc()
                        else:
                            log_msg = "⚠️ 로봇 대시보드 미연결"
                            print(log_msg)
                            self.work_log_signal.emit(log_msg)
                        
                        # 마커 표시
                        aruco.drawDetectedMarkers(display_frame, corners, ids)
                        cv2.circle(display_frame, (mcx, mcy), 15, (0, 0, 255), 3)
                        cv2.putText(display_frame, f"Marker {marker_id}", (mcx-30, mcy-20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        
                        # 오프셋 정보 표시
                        cv2.putText(display_frame, f"Offset: ({offset_mm_x:.1f}, {offset_mm_y:.1f})mm", 
                                   (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                        
                        # 목표 좌표 표시
                        if self.aruco_target_coords:
                            cv2.putText(display_frame, f"Target: ({self.aruco_target_coords[0]:.1f}, {self.aruco_target_coords[1]:.1f}, {self.aruco_target_coords[2]:.1f})", 
                                       (10, h-50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        
                        # PICKUP 안내 표시
                        cv2.putText(display_frame, "Press PICKUP to grab!", 
                                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        # 영상 정지를 위해 프레임 저장
                        self.aruco_frozen_frame = display_frame.copy()
                        
                        # 정지된 프레임 표시
                        rgb_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                        rgb_frame = np.ascontiguousarray(rgb_frame)
                        qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
                        pixmap = QPixmap.fromImage(qt_image)
                        scaled_pixmap = pixmap.scaled(
                            self.cam_view.width(), self.cam_view.height(),
                            Qt.AspectRatioMode.KeepAspectRatio,
                            Qt.TransformationMode.SmoothTransformation
                        )
                        self.cam_view.setPixmap(scaled_pixmap)
                        
                        log_msg = f"🔒 영상 정지됨 - PICKUP 버튼으로 진행하세요"
                        print(log_msg)
                        self.work_log_signal.emit(log_msg)
                    else:
                        log_msg = f"⚠️ ArUco 마커를 찾지 못했습니다 (rejected: {len(rejected) if rejected else 0})"
                        print(log_msg)
                        self.work_log_signal.emit(log_msg)
                except Exception as e:
                    log_msg = f"❌ ArUco 감지 오류: {e}"
                    print(log_msg)
                    self.work_log_signal.emit(log_msg)
            else:
                log_msg = f"⚠️ ArUco 사용 불가 (ARUCO_AVAILABLE={ARUCO_AVAILABLE}, dict={self.aruco_dict is not None})"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
        except Exception as e:
            log_msg = f"❌ 테스트 오류: {e}"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            import traceback
            traceback.print_exc()

    def _detect_and_move_aruco(self):
        """ArUco 마커 감지 및 로봇팔 자동 이동"""
        try:
            if not ARUCO_AVAILABLE:
                log_msg = "⚠️ ArUco 감지를 위해 opencv-contrib-python이 필요합니다"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                return
            
            if not self.camera_controller:
                log_msg = "⚠️ 카메라 컨트롤러가 없습니다"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                return
            
            if not hasattr(self.camera_controller, 'latest_frame') or self.camera_controller.latest_frame is None:
                log_msg = "⚠️ 카메라 프레임이 없습니다. 카메라를 먼저 연결하세요."
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                return
            
            frame = self.camera_controller.latest_frame.copy()
            
            # 그레이스케일 변환
            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame
            
            corners = None
            ids = None
            
            # OpenCV 버전에 따른 ArUco 감지 방식 분기
            try:
                if self.aruco_detector is not None:
                    # OpenCV 4.7.0+ (새 API)
                    detector = self._get_aruco_detector()
                    if detector:
                        corners, ids, rejected = detector.detectMarkers(gray)
                    else:
                        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
                else:
                    # 구 버전 OpenCV
                    try:
                        arucoParams = aruco.DetectorParameters()
                        arucoParams.minMarkerPerimeterRate = self.aruco_params['minMarkerPerimeterRate']
                        arucoParams.maxMarkerPerimeterRate = self.aruco_params['maxMarkerPerimeterRate']
                        
                        detector = aruco.ArucoDetector(self.aruco_dict, arucoParams)
                        corners, ids, rejected = detector.detectMarkers(gray)
                    except AttributeError:
                        # 더 이전 버전
                        arucoParams = aruco.DetectorParameters_create()
                        arucoParams.minMarkerPerimeterRate = self.aruco_params['minMarkerPerimeterRate']
                        arucoParams.maxMarkerPerimeterRate = self.aruco_params['maxMarkerPerimeterRate']
                        
                        corners, ids, rejected = aruco.detectMarkers(
                            gray, 
                            self.aruco_dict,
                            parameters=arucoParams
                        )
            except Exception as e:
                log_msg = f"❌ ArUco 감지 중 오류: {e}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                return
            
            if ids is not None and len(ids) > 0:
                # 첫 번째 감지된 마커 사용
                marker_corners = corners[0]
                marker_id = int(ids[0][0])
                
                # 마커의 중심점 계산
                cx = int(np.mean(marker_corners[0][:, 0]))
                cy = int(np.mean(marker_corners[0][:, 1]))
                
                log_msg = f"📌 ArUco 마커 {marker_id} 감지 - 중심: ({cx}, {cy})"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
                # 마커 시각화
                display_frame = frame.copy()
                try:
                    aruco.drawDetectedMarkers(display_frame, corners, ids)
                except Exception as e:
                    print(f"⚠️ 마커 시각화 오류: {e}")
                
                # 마커 중심에 십자가 그리기
                cv2.line(display_frame, (cx - 30, cy), (cx + 30, cy), (0, 255, 0), 3)
                cv2.line(display_frame, (cx, cy - 30), (cx, cy + 30), (0, 255, 0), 3)
                cv2.circle(display_frame, (cx, cy), 10, (0, 255, 0), 2)
                
                cv2.putText(display_frame, f"Marker {marker_id}: ({cx}, {cy})",
                           (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                self.update_camera_frame(display_frame)
                
                # 로봇팔 이동은 선택적 (컨트롤러 있을 때만)
                if self.robot_dashboard and hasattr(self.robot_dashboard, 'controller') and self.robot_dashboard.controller:
                    self._move_robot_to_marker(cx, cy, frame.shape)
                else:
                    log_msg = f"ℹ️ 마커 감지 완료 (로봇 컨트롤러 미연결)"
                    print(log_msg)
                    self.work_log_signal.emit(log_msg)
            else:
                log_msg = "⚠️ ArUco 마커를 감지하지 못했습니다"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
                
        except Exception as e:
            log_msg = f"❌ ArUco 감지 오류: {e}"
            print(log_msg)
            self.work_log_signal.emit(log_msg)
            import traceback
            traceback.print_exc()
    
    def _move_robot_to_marker(self, marker_cx, marker_cy, frame_shape):
        """감지된 마커 중심으로 로봇팔 이동"""
        frame_h, frame_w = frame_shape[:2]
        
        # 카메라 중심을 기준으로 오프셋 계산
        center_x = frame_w // 2
        center_y = frame_h // 2
        
        offset_x = marker_cx - center_x
        offset_y = marker_cy - center_y
        
        log_msg = f"🎯 마커 오프셋 - X: {offset_x}px, Y: {offset_y}px"
        print(log_msg)
        self.work_log_signal.emit(log_msg)
        
        # 현재 목표 좌표 가져오기
        if hasattr(self.robot_dashboard, 'pose_target_inputs') and self.robot_dashboard.pose_target_inputs:
            try:
                current_coords = []
                for i in range(6):
                    if self.robot_dashboard.pose_target_inputs[i]:
                        text = self.robot_dashboard.pose_target_inputs[i].text()
                        current_coords.append(float(text) if text else 0.0)
                    else:
                        current_coords.append(0.0)
                
                if len(current_coords) >= 6:
                    # 캘리브레이션 데이터가 있으면 사용, 없으면 스케일 사용
                    if self.camera_matrix is not None:
                        # 카메라 초점 거리를 이용한 변환
                        fx = self.camera_matrix[0, 0]
                        fy = self.camera_matrix[1, 1]
                        
                        # 마커와 카메라의 거리 추정 (기본값: 300mm)
                        depth = 300.0
                        
                        # 픽셀 오프셋을 mm로 변환
                        real_offset_x = (offset_x * depth) / fx
                        real_offset_y = (offset_y * depth) / fy
                        
                        log_msg = f"📐 캘리브레이션 기반 변환: X={real_offset_x:.2f}mm, Y={real_offset_y:.2f}mm"
                        print(log_msg)
                        self.work_log_signal.emit(log_msg)
                    else:
                        # 캘리브레이션 데이터 없을 때: 스케일 사용
                        scale = 0.1  # 1픽셀 = 0.1mm
                        real_offset_x = offset_x * scale
                        real_offset_y = offset_y * scale
                    
                    adjusted_coords = [
                        current_coords[0] + real_offset_x,  # X
                        current_coords[1] + real_offset_y,  # Y
                        current_coords[2],  # Z (변경 없음)
                        current_coords[3],  # R (변경 없음)
                        current_coords[4],  # P (변경 없음)
                        current_coords[5],  # Y (변경 없음)
                    ]
                    
                    log_msg = f"🤖 로봇팔 이동: X={adjusted_coords[0]:.1f}, Y={adjusted_coords[1]:.1f}"
                    print(log_msg)
                    self.work_log_signal.emit(log_msg)
                    
                    self.robot_dashboard.controller.publish_coords(adjusted_coords)
            except Exception as e:
                log_msg = f"❌ 로봇 이동 오류: {e}"
                print(log_msg)
                self.work_log_signal.emit(log_msg)
    
    def update_camera_frame(self, frame):
        """카메라 프레임 업데이트"""
        try:
            if frame is None:
                return
            
            # ArUco 감지 후 영상 정지 상태면 정지된 프레임 사용
            if self.aruco_detected and self.aruco_frozen_frame is not None:
                display_frame = self.aruco_frozen_frame
            # 얼라인 모드면 십자가가 그려진 프레임 사용
            elif self.is_aligning and self.align_frame is not None:
                display_frame = self.align_frame
            else:
                display_frame = frame
            
            if display_frame is None or display_frame.size == 0:
                return
            
            # 프레임 복사 (메모리 안전성)
            display_frame = np.ascontiguousarray(display_frame)
            
            # OpenCV BGR → RGB 변환
            if len(display_frame.shape) == 3 and display_frame.shape[2] == 3:
                rgb_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            elif len(display_frame.shape) == 2:
                # 그레이스케일인 경우
                rgb_frame = cv2.cvtColor(display_frame, cv2.COLOR_GRAY2RGB)
            else:
                rgb_frame = display_frame
            
            rgb_frame = np.ascontiguousarray(rgb_frame)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            
            # QImage 생성 (데이터 복사)
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
            
            # QLabel 크기에 맞게 스케일링
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.cam_view.width(), 
                self.cam_view.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            
            self.cam_view.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"⚠️ 프레임 업데이트 오류: {e}")
    
    def _draw_align_crosshair(self):
        """얼라인 십자가 그리기"""
        if self.align_frame is None:
            return
        
        h, w = self.align_frame.shape[:2]
        cx, cy = w // 2, h // 2
        
        # 십자가 그리기 (빨간색)
        cv2.line(self.align_frame, (cx - 50, cy), (cx + 50, cy), (0, 0, 255), 3)  # 가로
        cv2.line(self.align_frame, (cx, cy - 50), (cx, cy + 50), (0, 0, 255), 3)  # 세로
        
        # 중앙 원 그리기 (파란색)
        cv2.circle(self.align_frame, (cx, cy), 15, (255, 0, 0), 2)
        
        # 얼라인 좌표 텍스트 표시
        cv2.putText(
            self.align_frame,
            f"Align Point: ({cx}, {cy})",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        # 업데이트된 프레임 표시
        self.update_camera_frame(self.align_frame)
    
    def on_camera_connection_changed(self, connected):
        """카메라 연결 상태 변경"""
        if connected:
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.btn_capture.setEnabled(True)
            self.btn_live.setEnabled(True)
            self.btn_align.setEnabled(True)
            self.btn_pickup.setEnabled(True)
            self.btn_capture_with_coords.setEnabled(True)
            self.btn_aruco.setEnabled(True)
            self.cam_view.setText("")
        else:
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
            self.btn_capture.setEnabled(False)
            self.btn_live.setEnabled(False)
            self.btn_align.setEnabled(False)
            self.btn_pickup.setEnabled(False)
            self.btn_capture_with_coords.setEnabled(False)
            self.btn_aruco.setEnabled(False)
            self.cam_view.clear()
            self.cam_view.setText("카메라 연결 끊김")
