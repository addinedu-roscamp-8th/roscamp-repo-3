"""
상수 및 스타일 정의
"""

# 윈도우 크기
WINDOW_WIDTH = 1920
WINDOW_HEIGHT = 1080

# 사이드바
SIDEBAR_WIDTH = 150
SIDEBAR_BUTTON_HEIGHT = 80

# 탭 크기
TAB_HEIGHT = 72
TAB_WIDTH = 200

# Main 탭 레이아웃 (절대 좌표)
MAIN_SYSTEM_MAP = (0, 0, 1080, 620)
MAIN_ORDER_LOG = (1080, 0, 680, 460)
MAIN_ROBOT_GRID = (0, 620, 1080, 360)
MAIN_CAMERA_VIEW = (1080, 460, 680, 520)

# 카메라 크기
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# 통신 탭 테이블
COMM_TABLE_WIDTH = 700
COMM_TABLE_COL_WIDTHS = {
    'status': 100,
    'domain': 100,
    'connect': 100
}

# 색상
COLOR_GREEN = "#28a745"
COLOR_RED = "#dc3545"
COLOR_YELLOW = "#ffc107"
COLOR_GRAY = "#6c757d"
COLOR_BLUE = "#2196F3"
COLOR_DARK_BG = "#2d2d2d"
COLOR_ORANGE = "#ff9800"

# 버튼 스타일
STYLE_BUTTON_GREEN = f"""
    QPushButton {{
        background-color: {COLOR_GREEN};
        color: white;
        font-size: 16px;
        font-weight: bold;
        border-radius: 5px;
    }}
    QPushButton:hover {{
        background-color: #218838;
    }}
    QPushButton:pressed {{
        background-color: #1e7e34;
    }}
"""

STYLE_BUTTON_RED = f"""
    QPushButton {{
        background-color: {COLOR_RED};
        color: white;
        font-size: 16px;
        font-weight: bold;
        border-radius: 5px;
    }}
    QPushButton:hover {{
        background-color: #c82333;
    }}
    QPushButton:pressed {{
        background-color: #bd2130;
    }}
"""

STYLE_BUTTON_YELLOW = f"""
    QPushButton {{
        background-color: {COLOR_YELLOW};
        color: #333;
        font-size: 16px;
        font-weight: bold;
        border-radius: 5px;
    }}
    QPushButton:hover {{
        background-color: #e0a800;
    }}
    QPushButton:pressed {{
        background-color: #d39e00;
    }}
"""

STYLE_BUTTON_ORANGE = f"""
    QPushButton {{
        background-color: {COLOR_ORANGE};
        color: white;
        font-size: 16px;
        font-weight: bold;
        border-radius: 5px;
    }}
    QPushButton:hover {{
        background-color: #fb8c00;
    }}
    QPushButton:pressed {{
        background-color: #f57c00;
    }}
"""

STYLE_BUTTON_BLUE = f"""
    QPushButton {{
        background-color: {COLOR_BLUE};
        color: white;
        font-size: 16px;
        font-weight: bold;
        border-radius: 5px;
    }}
    QPushButton:hover {{
        background-color: #1976D2;
    }}
    QPushButton:pressed {{
        background-color: #0D47A1;
    }}
"""

STYLE_BUTTON_GRAY = f"""
    QPushButton {{
        background-color: {COLOR_GRAY};
        color: white;
        font-size: 16px;
        font-weight: bold;
        border-radius: 5px;
    }}
    QPushButton:hover {{
        background-color: #5a6268;
    }}
    QPushButton:pressed {{
        background-color: #545b62;
    }}
"""

# 로그 뷰어 스타일
STYLE_LOG_VIEWER = """
    background-color: #1e1e1e;
    color: #00ff00;
    font-family: Consolas;
    font-size: 11px;
    border: 1px solid #555;
"""

# 테이블 스타일
STYLE_TABLE = """
    QTableWidget {
        background-color: white;
        gridline-color: #d0d0d0;
        color: black;
    }
    QHeaderView::section {
        background-color: #4a90e2;
        color: white;
        padding: 5px;
        border: 1px solid #357abd;
        font-weight: bold;
    }
    QTableWidget::item {
        padding: 5px;
    }
"""

# ===== 로봇팔 상태 머신 (FSM) 정의 =====
# 상태 코드
ROBOT_STATE_INIT = 0
ROBOT_STATE_IDLE = 1
ROBOT_STATE_BUSY = 2
ROBOT_STATE_SUCCESS = 3
ROBOT_STATE_ERROR = 4
ROBOT_STATE_FIRE = 5

# 상태 이름 매핑
ROBOT_STATE_NAMES = {
    0: "INIT",
    1: "IDLE",
    2: "BUSY",
    3: "SUCCESS",
    4: "ERROR",
    5: "FIRE"
}

# 상태 설명
ROBOT_STATE_DESCRIPTIONS = {
    0: "초기화 및 부팅",
    1: "명령 대기 중",
    2: "작업 수행 중",
    3: "완료 보고 중",
    4: "일반 시스템 오류",
    5: "화재 긴급 상태"
}

# 상태별 색상
ROBOT_STATE_COLORS = {
    0: "#9E9E9E",  # INIT - 회색
    1: "#4CAF50",  # IDLE - 녹색
    2: "#2196F3",  # BUSY - 파란색
    3: "#8BC34A",  # SUCCESS - 연두색
    4: "#F44336",  # ERROR - 빨간색
    5: "#FF5722"   # FIRE - 주황빨강
}

# 상태별 이모지
ROBOT_STATE_EMOJIS = {
    0: "🔄",  # INIT
    1: "✅",  # IDLE
    2: "⚙️",  # BUSY
    3: "🎉",  # SUCCESS
    4: "❌",  # ERROR
    5: "🔥"   # FIRE
}

# ==================== AMR (운송로봇) 상태 코드 ====================
# AMR 상태 이름
AMR_STATE_NAMES = {
    1: "WAITING",
    2: "TRANSPORTING",
    3: "SHIPPING",
    4: "PACKING_WAIT",
    5: "RETURNING",
    6: "CHARGING",
    7: "ERROR"
}

# AMR 상태 설명
AMR_STATE_DESCRIPTIONS = {
    1: "운송대기중",
    2: "운송중",
    3: "출고중",
    4: "패킹대기중",
    5: "복귀중",
    6: "충전중",
    7: "에러"
}

# AMR 상태별 색상
AMR_STATE_COLORS = {
    1: "#4CAF50",  # 운송대기중 - 녹색
    2: "#2196F3",  # 운송중 - 파란색
    3: "#2196F3",  # 출고중 - 파란색
    4: "#2196F3",  # 패킹대기중 - 파란색
    5: "#2196F3",  # 복귀중 - 파란색
    6: "#FFC107",  # 충전중 - 노란색
    7: "#F44336"   # 에러 - 빨간색
}

# AMR 상태별 이모지
AMR_STATE_EMOJIS = {
    1: "✅",  # 운송대기중
    2: "🚚",  # 운송중
    3: "📦",  # 출고중
    4: "📦",  # 패킹대기중
    5: "🔙",  # 복귀중
    6: "🔋",  # 충전중
    7: "⚠️"   # 에러
}
