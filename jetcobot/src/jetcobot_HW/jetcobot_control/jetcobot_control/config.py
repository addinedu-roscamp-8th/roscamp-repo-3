"""
카메라 스트리밍 설정 파일
이 파일을 수정하여 카메라 및 네트워크 설정을 변경할 수 있습니다.
"""

# ==========================================
# 현재 로봇 ID 설정 (각 로봇에서 이 부분만 수정)
# ==========================================
MY_ROBOT_ID = 1  # 1~5 사이의 로봇 번호

# ==========================================
# 로봇별 상세 정보 (포트, 카메라 번호 등)
# ==========================================
ROBOT_DATA = {
    1: {"name": "Jetcobot_01", "port": 9510, "cam_idx": 0},
    2: {"name": "Jetcobot_02", "port": 9520, "cam_idx": 0},
    3: {"name": "pinky_01", "port": 9530, "cam_idx": 0},
    4: {"name": "pinky_02", "port": 9540, "cam_idx": 0},
    5: {"name": "pinky_03", "port": 9550, "cam_idx": 0},
}

# 현재 로봇의 설정 자동 적용
if MY_ROBOT_ID not in ROBOT_DATA:
    raise ValueError(f"잘못된 로봇 ID: {MY_ROBOT_ID}. 1~5 사이의 값을 입력하세요.")

_current_robot = ROBOT_DATA[MY_ROBOT_ID]
ROBOT_NAME = _current_robot["name"]
CAMERA_DEVICE = _current_robot["cam_idx"]
PORT = _current_robot["port"]

# ==========================================
# 공통 설정 (모든 로봇에서 동일)
# ==========================================

# 카메라 해상도 설정
FRAME_WIDTH = 640       # 프레임 너비 (픽셀)
FRAME_HEIGHT = 480      # 프레임 높이 (픽셀)
FPS = 10                # 초당 프레임 수

# 이미지 압축 설정
QUALITY = 50            # JPEG 압축 품질 (0~100, 낮을수록 빠르지만 화질 저하)

# 네트워크 설정
# PC_IP = "192.168.1.5"   리눅스 PC의 IP 주소 (hostname -I로 확인)
PC_IP = "192.168.0.13"  # 리눅스 PC의 IP 주소 (hostname -I로 확인)



# UDP 패킷 최대 크기
MAX_UDP_SIZE = 65507    # UDP 패킷 최대 크기 (바이트)

# 카메라 재연결 설정
RECONNECT_DELAY = 5.0   # 카메라 재연결 시도 간격 (초)

# 이미지 전송 모드
# True이면 검은색(모든 픽값 0) 이미지를 전송합니다. False이면 컬러로 전송합니다.
# 주의: 수신측이 컬러와 그레이/흑백 단일 채널을 모두 처리할 수 있어야 합니다.
GRAYSCALE = True

# 서비스 응답으로 이미지 바이트를 직접 반환할지 여부
# True로 설정하면 ROS2 서비스 호출 시 JPEG 바이트를 base64로 인코딩하여
# 응답 문자열에 담아 반환합니다. (네트워크/메시지 크기 제한을 확인하세요)
SERVICE_RETURN_IMAGE = False
