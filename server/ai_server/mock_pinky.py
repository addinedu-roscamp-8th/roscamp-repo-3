import socket
import json

PORT = 9542 # Pinky control/observation port

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    print(f"Mock Pinky: Listening for observations on port {PORT}...")
    
    try:
        while True:
            data, addr = sock.recvfrom(65507)
            try:
                msg = json.loads(data.decode('utf-8'))
                print(f"\n[RCV from {addr}]")
                print(json.dumps(msg, indent=2))
            except Exception as e:
                print(f"Error decoding: {e}")
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
