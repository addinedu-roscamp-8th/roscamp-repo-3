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
TAB_HEIGHT = 80
TAB_WIDTH = 200

# Main 탭 레이아웃 (절대 좌표)
MAIN_SYSTEM_MAP = (0, 0, 1300, 650)
MAIN_ORDER_LOG = (1300, 0, 460, 650)
MAIN_ROBOT_GRID = (0, 650, 1300, 300)
MAIN_CAMERA_VIEW = (1300, 650, 460, 320)

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
