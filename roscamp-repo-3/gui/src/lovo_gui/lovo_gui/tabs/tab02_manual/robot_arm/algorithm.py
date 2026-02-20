"""
ArUco 마커 기반 로봇팔 픽업 알고리즘 모듈
- 픽셀 좌표 → 로봇 좌표 변환
- Hand-Eye 캘리브레이션 변환
"""

import numpy as np


class CoordinateTransformer:
    """좌표 변환 클래스"""
    
    @staticmethod
    def pixel_to_mm(offset_px_x, offset_px_y, scale_x=0.5, scale_y=0.5):
        """
        픽셀 오프셋을 mm 오프셋으로 변환
        
        Args:
            offset_px_x (float): 카메라 중심 기준 픽셀 X 오프셋
            offset_px_y (float): 카메라 중심 기준 픽셀 Y 오프셋
            scale_x (float): X축 스케일 팩터 (mm/pixel)
            scale_y (float): Y축 스케일 팩터 (mm/pixel)
        
        Returns:
            tuple: (offset_mm_x, offset_mm_y)
        """
        offset_mm_x = offset_px_x * scale_x
        offset_mm_y = offset_px_y * scale_y
        return offset_mm_x, offset_mm_y
    
    @staticmethod
    def apply_sign_and_swap(offset_mm_x, offset_mm_y, sign_x="+", sign_y="+", swap_xy=False):
        """
        부호와 축 스왑을 적용한 좌표 변환
        
        Args:
            offset_mm_x (float): X축 오프셋 (mm)
            offset_mm_y (float): Y축 오프셋 (mm)
            sign_x (str): X축 부호 ("+", "-")
            sign_y (str): Y축 부호 ("+", "-")
            swap_xy (bool): X, Y 축을 스왑할지 여부
        
        Returns:
            tuple: (final_x, final_y) - 부호 및 스왑 적용된 좌표
        """
        # 부호 적용
        sign_x_val = 1.0 if sign_x == "+" else -1.0
        sign_y_val = 1.0 if sign_y == "+" else -1.0
        
        if swap_xy:
            # X, Y 축 스왑
            final_x = sign_y_val * offset_mm_y
            final_y = sign_x_val * offset_mm_x
        else:
            # 직접 매핑
            final_x = sign_x_val * offset_mm_x
            final_y = sign_y_val * offset_mm_y
        
        return final_x, final_y
    
    @staticmethod
    def calculate_target_coords(current_coords, offset_x, offset_y, offset_z=0.0):
        """
        현재 로봇 좌표에 오프셋을 더해 목표 좌표 계산
        
        Args:
            current_coords (list): 현재 로봇 좌표 [x, y, z, r, p, yaw]
            offset_x (float): X축 오프셋
            offset_y (float): Y축 오프셋
            offset_z (float): Z축 오프셋
        
        Returns:
            list: 목표 좌표 [x, y, z, r, p, yaw]
        """
        target = list(current_coords)
        target[0] += offset_x
        target[1] += offset_y
        target[2] += offset_z
        # R, P, Yaw는 변경 없음
        return target


class HandEyeTransformer:
    """Hand-Eye 캘리브레이션 기반 좌표 변환 클래스"""
    
    def __init__(self, hand_eye_matrix, camera_matrix):
        """
        Hand-Eye 변환기 초기화
        
        Args:
            hand_eye_matrix (np.ndarray): 4x4 Hand-Eye 변환 행렬
            camera_matrix (np.ndarray): 3x3 카메라 내부 파라미터 행렬
        """
        self.hand_eye_matrix = hand_eye_matrix
        self.camera_matrix = camera_matrix
    
    def transform(self, pixel_x, pixel_y, camera_z=500.0):
        """
        픽셀 좌표를 Hand-Eye 캘리브레이션을 사용하여 로봇 좌표로 변환
        
        변환 과정:
        1. 픽셀 좌표 → 정규화 이미지 좌표
        2. 카메라 3D 좌표 (깊이 기반)
        3. Hand-Eye 행렬을 사용하여 로봇 베이스 좌표로 변환
        
        Args:
            pixel_x (float): 카메라 중심 기준 픽셀 X 오프셋
            pixel_y (float): 카메라 중심 기준 픽셀 Y 오프셋
            camera_z (float): 카메라-마커 간 예상 깊이 (mm, 기본값: 500)
        
        Returns:
            list: [x, y, z] 로봇 베이스 좌표계의 오프셋
        
        Raises:
            ValueError: 필수 캘리브레이션 데이터가 없을 때
        """
        if self.hand_eye_matrix is None or self.camera_matrix is None:
            raise ValueError("Hand-Eye 캘리브레이션 데이터가 없습니다")
        
        # 카메라 내부 파라미터 추출
        fx = self.camera_matrix[0, 0]  # X축 초점거리
        fy = self.camera_matrix[1, 1]  # Y축 초점거리
        
        # 픽셀 좌표 → 정규화 이미지 좌표
        # 카메라 중심 기준 오프셋이므로 중심 좌표는 이미 0,0
        x_normalized = pixel_x / fx
        y_normalized = pixel_y / fy
        
        # 카메라 3D 좌표 (Z = camera_z로 설정된 깊이)
        cam_x = x_normalized * camera_z
        cam_y = y_normalized * camera_z
        cam_z = camera_z
        
        # 동차 좌표 (4x1 벡터)
        cam_point = np.array([cam_x, cam_y, cam_z, 1.0])
        
        # Hand-Eye 변환 행렬 적용
        # 4x4 행렬 × 4x1 벡터
        robot_point = self.hand_eye_matrix @ cam_point
        
        # 로봇 베이스 좌표 추출
        robot_x = robot_point[0]
        robot_y = robot_point[1]
        robot_z = robot_point[2]
        
        return [robot_x, robot_y, robot_z]


class ArucoMarkerProcessor:
    """ArUco 마커 감지 및 처리 클래스"""
    
    @staticmethod
    def calculate_pixel_offset(marker_cx, marker_cy, frame_width, frame_height):
        """
        마커 중심 픽셀 좌표를 카메라 중심 기준 오프셋으로 변환
        
        Args:
            marker_cx (int): 마커 중심의 픽셀 X 좌표
            marker_cy (int): 마커 중심의 픽셀 Y 좌표
            frame_width (int): 프레임 너비
            frame_height (int): 프레임 높이
        
        Returns:
            tuple: (offset_px_x, offset_px_y) 카메라 중심 기준 오프셋
        """
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        offset_px_x = marker_cx - center_x  # 양수 = 오른쪽
        offset_px_y = marker_cy - center_y  # 양수 = 아래쪽
        
        return offset_px_x, offset_px_y
    
    @staticmethod
    def detect_aruco_marker(frame, aruco_dict, aruco_detector=None, detector_params=None):
        """
        프레임에서 ArUco 마커 감지
        
        Args:
            frame (np.ndarray): BGR 형식의 카메라 프레임
            aruco_dict: ArUco 마커 사전
            aruco_detector: ArUco 감지기 (새 API 사용 시)
            detector_params: 감지 파라미터
        
        Returns:
            dict: 감지 결과 {
                'detected': bool,
                'marker_id': int,
                'marker_corners': np.ndarray,
                'marker_cx': int,
                'marker_cy': int,
                'corners': list,
                'ids': np.ndarray
            }
        
        Note:
            이 함수는 OpenCV aruco 모듈의 구 API와 새 API를 모두 지원합니다.
        """
        import cv2
        try:
            import cv2.aruco as aruco
            ARUCO_AVAILABLE = True
        except ImportError:
            ARUCO_AVAILABLE = False
        
        result = {
            'detected': False,
            'marker_id': None,
            'marker_corners': None,
            'marker_cx': None,
            'marker_cy': None,
            'corners': None,
            'ids': None
        }
        
        if not ARUCO_AVAILABLE or aruco_dict is None:
            return result
        
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 새 API 사용
            if aruco_detector is not None:
                corners, ids, rejected = aruco_detector.detectMarkers(gray)
            else:
                # 구 API 사용 (DetectorParameters_create)
                try:
                    if detector_params is None:
                        detector_params = aruco.DetectorParameters_create()
                    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)
                except:
                    # 더 이상 지원되지 않는 API
                    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)
            
            if ids is not None and len(ids) > 0:
                marker_id = int(ids[0][0])
                marker_corners = corners[0][0]
                
                # 마커 중심 계산 (4개 코너의 평균)
                mcx = int(np.mean(marker_corners[:, 0]))
                mcy = int(np.mean(marker_corners[:, 1]))
                
                result['detected'] = True
                result['marker_id'] = marker_id
                result['marker_corners'] = marker_corners
                result['marker_cx'] = mcx
                result['marker_cy'] = mcy
                result['corners'] = corners
                result['ids'] = ids
        
        except Exception as e:
            print(f"❌ ArUco 감지 오류: {e}")
        
        return result


class PickupSequence:
    """픽업 시퀀스 관리 클래스"""
    
    STEP_MOVE = 0      # 단계 0: 마커 위치로 이동
    STEP_GRIP = 1      # 단계 1: GRIP 실행
    STEP_LIFT = 2      # 단계 2: Z축 상승
    STEP_DONE = 3      # 단계 3: 완료
    
    def __init__(self):
        """픽업 시퀀스 초기화"""
        self.current_step = self.STEP_MOVE
        self.target_coords = None
        self.marker_id = None
    
    def get_next_action(self):
        """현재 단계의 동작 반환"""
        if self.current_step == self.STEP_MOVE:
            return "MOVE_TO_MARKER"
        elif self.current_step == self.STEP_GRIP:
            return "GRIP"
        elif self.current_step == self.STEP_LIFT:
            return "LIFT_Z"
        else:
            return "DONE"
    
    def advance_step(self):
        """다음 단계로 진행"""
        if self.current_step < self.STEP_DONE:
            self.current_step += 1
        return self.current_step
    
    def reset(self):
        """시퀀스 초기화"""
        self.current_step = self.STEP_MOVE
        self.target_coords = None
        self.marker_id = None
