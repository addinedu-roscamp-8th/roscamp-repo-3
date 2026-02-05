import getpass
import os
import subprocess
import sys

def main():
    print("=== Factory System Database Setup ===")
    print("The agent could not access the database automatically.")
    print("This script will help you set up the 'factory_system' database from 'sf.sql'.")
    print("")

    # 1. Get credentials
    db_user = input("Enter MySQL User (default: root): ").strip() or "root"
    db_pass = getpass.getpass(f"Enter password for '{db_user}': ")

    # 2. Check connection and create DB
    print(f"\n[INFO] Connecting to MySQL as '{db_user}'...")
    
    # We use mysql command line tool for simplicity and robustness with .sql files
    # Construct command to create DB
    create_db_cmd = [
        "mysql", 
        f"-u{db_user}", 
        f"-p{db_pass}" if db_pass else "",
        "-e", "CREATE DATABASE IF NOT EXISTS factory_system DEFAULT CHARACTER SET utf8mb4;"
    ]
    # Remove empty empty strings if no password
    create_db_cmd = [x for x in create_db_cmd if x]

    try:
        # We manually handle the password argument to avoid it being shown in process list if possible,
        # but for this local setup, passing -pPASSWORD is the simplest way for subprocess if we don't want to pipe stdin strictly.
        # Actually safer to use env var or config file, but let's try direct execution first or piping.
        # To avoid -p on command line, we can use MYSQL_PWD env var (warned as insecure but fine for local setup script).
        
        env = os.environ.copy()
        if db_pass:
            env["MYSQL_PWD"] = db_pass
        
        # Base command without password arg
        base_cmd = ["mysql", f"-u{db_user}"]

        # Create DB
        subprocess.run(base_cmd + ["-e", "CREATE DATABASE IF NOT EXISTS factory_system DEFAULT CHARACTER SET utf8mb4;"], env=env, check=True)
        print("[SUCCESS] Database 'factory_system' created (or already exists).")

        # 3. Read and execute sf.sql
        sql_file = "sf.sql"
        if not os.path.exists(sql_file):
            print(f"[ERROR] '{sql_file}' not found in current directory.")
            sys.exit(1)

        print(f"[INFO] Importing '{sql_file}' into 'factory_system'...")
        
        # Use shell redirection style via python file opening
        with open(sql_file, 'r') as f:
            # We enforce using factory_system
            subprocess.run(base_cmd + ["factory_system"], env=env, stdin=f, check=True)
        
        print("\n[SUCCESS] Database import complete!")
        print("You can now verify with: mysql -u root -p -D factory_system -e 'SHOW TABLES;'")

    except subprocess.CalledProcessError as e:
        print(f"\n[ERROR] Command failed with exit code {e.returncode}.")
        print("Please check your password and MySQL installation.")
        sys.exit(e.returncode)
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
