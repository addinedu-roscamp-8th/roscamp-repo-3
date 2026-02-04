import cv2
import socket
import numpy as np
import sys
import threading
from . import config

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

    try:
        while True:
            # 프레임 읽기
            ret, frame = cap.read()
            if not ret:
                print("❌ 프레임을 읽지 못했습니다.")
                break

            # 데이터 압축 (JPEG)
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, config.QUALITY])
            
            # 데이터 전송
            data = buffer.tobytes()
            if len(data) < config.MAX_UDP_SIZE:
                sock.sendto(data, (config.PC_IP, config.PORT))
            else:
                print(f"⚠️ 경고: 프레임이 너무 큼 ({len(data)} bytes)")

    except KeyboardInterrupt:
        print("\n👋 사용자에 의해 스트리밍이 중단되었습니다.")
    finally:
        # 리소스 해제
        cap.release()
        sock.close()
        print("🔌 카메라 및 소켓 연결 종료.")


if __name__ == '__main__':
    main()