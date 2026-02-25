import mysql.connector
from mysql.connector import Error, pooling

class DBManager:
    """LOVO Factory System Database Manager"""
    _instance = None
    _pool = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DBManager, cls).__new__(cls)
            cls._instance._init_pool()
        return cls._instance

    def _init_pool(self):
        """Initialize Connection Pool for better performance in multi-robot environments"""
        self.db_config = {
            'host': 'localhost',
            'user': 'lovoDB',
            'password': 'LovoDB1234!',
            'database': 'factory_system'
        }
        try:
            # Create a connection pool to handle multiple connections efficiently
            self._pool = pooling.MySQLConnectionPool(
                pool_name="lovo_pool",
                pool_size=10,
                **self.db_config
            )
            print("💾 DB Connection Pool initialized.")
        except Error as e:
            print(f"❌ Error while initializing DB Pool: {e}")

    def get_connection(self):
        """Get a connection from the pool"""
        try:
            if self._pool:
                return self._pool.get_connection()
            else:
                return mysql.connector.connect(**self.db_config)
        except Error as e:
            print(f"❌ Error getting DB connection: {e}")
            return None

    def execute_query(self, query, params=None, commit=True):
        """Execute a query (INSERT, UPDATE, DELETE)"""
        conn = self.get_connection()
        if not conn: return False
        
        try:
            cursor = conn.cursor()
            cursor.execute(query, params or ())
            if commit:
                conn.commit()
            cursor.close()
            return True
        except Error as e:
            print(f"❌ Query execution error: {e}")
            if commit: conn.rollback()
            return False
        finally:
            if conn.is_connected():
                conn.close()

    def fetch_all(self, query, params=None, dictionary=True):
        """Fetch all results for a query (SELECT)"""
        conn = self.get_connection()
        if not conn: return []
        
        try:
            cursor = conn.cursor(dictionary=dictionary)
            cursor.execute(query, params or ())
            result = cursor.fetchall()
            cursor.close()
            return result
        except Error as e:
            print(f"❌ Fetch all error: {e}")
            return []
        finally:
            if conn.is_connected():
                conn.close()

    def fetch_one(self, query, params=None, dictionary=True):
        """Fetch a single result for a query (SELECT ... LIMIT 1)"""
        conn = self.get_connection()
        if not conn: return None
        
        try:
            cursor = conn.cursor(dictionary=dictionary)
            cursor.execute(query, params or ())
            result = cursor.fetchone()
            cursor.close()
            return result
        except Error as e:
            print(f"❌ Fetch one error: {e}")
            return None
        finally:
            if conn.is_connected():
                conn.close()

# Create a global instance for easy import
db = DBManager()
