import cv2
import socket
import numpy as np
import time


SERVER_IP = "192.168.0.184" 
UDP_PORT = 9541   

def main():
    # Create test image
    cap = cv2.VideoCapture(0) # Use webcam if available, else static
    if not cap.isOpened():
        print("Webcam not found, using static image simulation.")
        img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"Simulating Robot 2: Sending to {SERVER_IP}:{UDP_PORT}")
    
    try:
        while True:
            if cap.isOpened():
                ret, frame = cap.read()
                if not ret: break
            else:
                # Static image with moving text
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, f"Simulated Robot 2 Feed: {time.time():.2f}", 
                            (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Encode as JPEG
            _, img_encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            data = img_encoded.tobytes()
            
            # Send (UDP packet size limit is ~65KB, typical JPEG is fine)
            if len(data) > 65507:
                print("Frame too large!")
            else:
                sock.sendto(data, (SERVER_IP, UDP_PORT))
            
            time.sleep(0.05) # 20 FPS
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        cap.release()
        sock.close()

if __name__ == "__main__":
    main()
