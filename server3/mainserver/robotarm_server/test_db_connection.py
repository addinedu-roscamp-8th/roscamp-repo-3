
import mysql.connector
from mysql.connector import Error

def try_connect(user, password):
    print(f"Trying user='{user}', password='{password}'...")
    try:
        conn = mysql.connector.connect(
            host='127.0.0.1',
            user=user,
            password=password,
            database='factory_system'
        )
        if conn.is_connected():
            print("SUCCESS!")
            conn.close()
            return True
    except Error as e:
        print(f"FAILED: {e}")
    return False

if __name__ == "__main__":
    # 1. Try lovoDB again
    try_connect('lovoDB', 'LovoDB1234!')
    
    # 2. Try root with no password
    try_connect('root', '')
    
    # 3. Try root with password '1234'
    try_connect('root', '1234')

    # 4. Try root with password 'root'
    try_connect('root', 'root')
