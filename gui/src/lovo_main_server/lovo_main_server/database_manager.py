import mysql.connector

# 1. DB 연결 설정 함수
def get_connection():
    return mysql.connector.connect(
        host="localhost",
        user="root",
        password="1",
        database="lovo_db",
        use_pure=True
    )

# 2. 데이터를 집어넣는 함수
def log_work_status(work_id, status, message):
    try:
        conn = get_connection()
        cursor = conn.cursor()
        
        # 데이터 삽입 명령
        sql = "INSERT INTO work_log (work_id, status, message) VALUES (%s, %s, %s)"
        val = (work_id, status, message)
        
        cursor.execute(sql, val)
        conn.commit()  # 실제로 저장 확정
        
        print(f"✅ DB에 성공적으로 저장됨: {status} - {message}")
        
        cursor.close()
        conn.close()
    except Exception as e:
        print(f"❌ 오류 발생: {e}")

# 3. 테스트 실행 (이 파일을 직접 실행할 때만 작동)
if __name__ == "__main__":
    # 첫 번째 테스트
    log_work_status(1, "Success", "첫 번째 테스트 기록입니다.")
    # 서버 시작 로그 테스트
    log_work_status(0, "Running", "Lovo 메인 서버가 가동되었습니다.")