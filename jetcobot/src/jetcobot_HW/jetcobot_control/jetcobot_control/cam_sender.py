import cv2
import socket
import numpy as np
import sys
import threading
from . import config
import logging
import os
from datetime import datetime

# Initialize logging
log_dir = "logs"
os.makedirs(log_dir, exist_ok=True)

# Create a logger
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# File handler
file_handler = logging.FileHandler(os.path.join(log_dir, "cam_sender.log"))
file_handler.setLevel(logging.INFO)
file_formatter = logging.Formatter("%(asctime)s [%(levelname)s]: %(message)s")
file_handler.setFormatter(file_formatter)
logger.addHandler(file_handler)

# Console handler
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_formatter = logging.Formatter("%(asctime)s [%(levelname)s]: %(message)s")
console_handler.setFormatter(console_formatter)
logger.addHandler(console_handler)

# Directory to save captured images
image_dir = "captured_images"
os.makedirs(image_dir, exist_ok=True)

# Optional ROS2 service support: creates a GetSnapshot service '/capture_image'
# that captures a single frame and sends it over UDP to the configured PC.
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    ROS2_AVAILABLE = True
except Exception:
    ROS2_AVAILABLE = False

# 설정값은 config.py에서 관리됩니다

# 전역 변수
cap = None
sock = None

def handle_commands():
    """Handle incoming commands from the GUI."""
    command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command_sock.bind(("0.0.0.0", config.PORT + 1))  # Use a different port for commands
    print(f"📡 명령 수신 대기 중 -> 포트: {config.PORT + 1}")

    while True:
        try:
            command, addr = command_sock.recvfrom(1024)
            command = command.decode("utf-8")
            print(f"📩 명령 수신: {command}")

            if command == "START":
                if not cap.isOpened():
                    reconnect_camera()
            elif command == "STOP":
                stop_camera()
        except Exception as e:
            print(f"⚠️ 명령 처리 중 오류: {e}")


def reconnect_camera():
    """Attempt to reconnect the camera."""
    global cap
    print("🔄 카메라 재연결 시도 중...")
    cap.release()
    cap = cv2.VideoCapture(config.CAMERA_DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, config.FPS)

    if cap.isOpened():
        print("✅ 카메라 재연결 성공.")
    else:
        print("❌ 카메라 재연결 실패. 다시 시도 중...")
        threading.Timer(config.RECONNECT_DELAY, reconnect_camera).start()


def stop_camera():
    """Stop the camera streaming."""
    global cap
    print("🛑 카메라 스트리밍 중단.")
    cap.release()


def attempt_reconnect():
    """Attempt to reconnect the camera until successful."""
    global cap
    reconnect_attempts = 0
    while not cap.isOpened():
        reconnect_attempts += 1
        logging.warning(f"Camera disconnected. Attempting to reconnect... (Attempt {reconnect_attempts})")
        reconnect_camera()
        if cap.isOpened():
            logging.info("Camera reconnected successfully.")
            break
        else:
            logging.error("Reconnection attempt failed. Retrying...")
        # Delay before next attempt
        threading.Event().wait(config.RECONNECT_DELAY)


def main():
    """Main function for ROS2 node entry point."""
    global cap, sock
    
    # UDP 소켓 생성
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 카메라 설정 (최신 OS용 V4L2 백엔드 사용)
    cap = cv2.VideoCapture(config.CAMERA_DEVICE, cv2.CAP_V4L2)

    # 해상도 및 프레임 속도 최적화
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, config.FPS)

    if not cap.isOpened():
        print("❌ 에러: 카메라를 열 수 없습니다.")
        print("팁: ls /dev/video* 명령어로 카메라 번호를 확인하거나 케이블을 점검하세요.")
        sys.exit()

    print(f"🤖 로봇: {config.ROBOT_NAME} (ID: {config.MY_ROBOT_ID})")
    print(f"🚀 UDP 스트리밍 시작 -> PC 주소: {config.PC_IP}:{config.PORT}")
    print(f"📹 카메라: /dev/video{config.CAMERA_DEVICE}")
    print("중단하려면 Ctrl+C를 누르세요.")

    # 명령 처리 스레드 시작
    threading.Thread(target=handle_commands, daemon=True).start()

    # ROS2 service: capture once and send via UDP (if rclpy available)
    service_node = None
    ros2_ok = ROS2_AVAILABLE
    if ros2_ok:
        try:
            from lovo_interfaces.srv import GetSnapshot
        except Exception:
            print("⚠️ 서비스 타입 lovo_interfaces.srv.GetSnapshot을 찾을 수 없습니다. 인터페이스 패키지를 빌드하고 'source install/setup.bash'를 실행하세요.")
            ros2_ok = False

    if ros2_ok:
        class CaptureServiceNode(Node):
            def __init__(self):
                super().__init__('cam_sender_service')
                self.srv = self.create_service(GetSnapshot, '/capture_image', self.capture_callback)

            def capture_callback(self, request, response):
                global cap, sock
                try:
                    ret, frame = cap.read()
                    if not ret:
                        response.success = False
                        logging.error("Failed to capture frame from camera.")
                        return response

                    if getattr(config, 'GRAYSCALE', False):
                        frame_to_send = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    else:
                        frame_to_send = frame

                    _, buffer = cv2.imencode('.jpg', frame_to_send, [cv2.IMWRITE_JPEG_QUALITY, config.QUALITY])
                    data = buffer.tobytes()

                    # Save the captured image locally
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S%f")
                    image_path = os.path.join(image_dir, f"capture_{timestamp}.jpg")
                    with open(image_path, "wb") as f:
                        f.write(data)
                    logging.info(f"Image saved: {image_path}")

                    # Send via UDP if it fits
                    if len(data) < config.MAX_UDP_SIZE:
                        sock.sendto(data, (config.PC_IP, config.PORT))
                        logging.info(f"Image sent to {config.PC_IP}:{config.PORT}")
                    else:
                        logging.warning(f"Frame too large to send ({len(data)} bytes)")

                    # Fill response image
                    try:
                        response.image = CompressedImage()
                        response.image.format = 'jpeg'
                        response.image.data = data
                        response.success = True
                        logging.info("Service response sent successfully.")
                    except Exception as e:
                        response.success = False
                        logging.error(f"Failed to set response image: {e}")
                except Exception as e:
                    response.success = False
                    logging.error(f"capture_callback error: {e}")
                return response

        try:
            rclpy.init()
            service_node = CaptureServiceNode()
            threading.Thread(target=rclpy.spin, args=(service_node,), daemon=True).start()
            print('🔧 ROS2 서비스 시작됨 -> 서비스명: /capture_image')
        except Exception as e:
            print(f'⚠️ ROS2 서비스 초기화 실패: {e}')

    try:
        while True:
            # 프레임 읽기
            ret, frame = cap.read()
            if not ret:
                logging.error("Failed to read frame from camera. Attempting to reconnect...")
                attempt_reconnect()
                continue  # Retry frame reading after reconnection

            # GRAYSCALE 모드일 때는 실제 그레이스케일(Luminance)로 전송
            if getattr(config, "GRAYSCALE", False):
                frame_to_send = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                frame_to_send = frame

            # 데이터 압축 (JPEG)
            _, buffer = cv2.imencode('.jpg', frame_to_send, [cv2.IMWRITE_JPEG_QUALITY, config.QUALITY])

            # 데이터 전송
            data = buffer.tobytes()
            if len(data) < config.MAX_UDP_SIZE:
                sock.sendto(data, (config.PC_IP, config.PORT))
            else:
                logging.warning(f"Frame too large to send ({len(data)} bytes)")

    except KeyboardInterrupt:
        print("\n👋 사용자에 의해 스트리밍이 중단되었습니다.")
    finally:
        # 리소스 해제
        cap.release()
        sock.close()
        if 'ros2_ok' in locals() and ros2_ok and service_node is not None:
            try:
                service_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        print("🔌 카메라 및 소켓 연결 종료.")
        logging.info("Camera and socket connections closed.")


if __name__ == '__main__':
    main()