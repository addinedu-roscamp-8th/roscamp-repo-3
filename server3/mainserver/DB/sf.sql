-- =========================================================
-- factory_full_schema.sql (MySQL 8+, InnoDB, utf8mb4)
-- 테이블 8개 + (분리된) 자재 마스터 + 재고변동기록 포함
-- 1) robot
-- 2) orders
-- 3) customer
-- 4) charging_station
-- 5) furniture (가구 + 자재구성 통합)
-- 6) inventory_tx (자재 재고변동기록)
-- 7) robot_job
-- 8) robot_state_log
-- + material (자재 마스터)
-- =========================================================

CREATE DATABASE IF NOT EXISTS factory_system DEFAULT CHARACTER SET utf8mb4 DEFAULT COLLATE utf8mb4_0900_ai_ci;

USE factory_system;

-- ---------------------------------------------------------
-- 3) 고객
-- ---------------------------------------------------------
CREATE TABLE customer (
    customer_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    name VARCHAR(100) NULL,
    phone VARCHAR(30) NOT NULL,
    address VARCHAR(255) NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    UNIQUE KEY uq_customer_phone (phone)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 4) 충전소
-- ---------------------------------------------------------
CREATE TABLE charging_station (
    charger_id INT AUTO_INCREMENT PRIMARY KEY,
    name VARCHAR(60) NOT NULL,
    x DOUBLE NULL,
    y DOUBLE NULL,
    status ENUM(
        'AVAILABLE',
        'OCCUPIED',
        'ERROR'
    ) NOT NULL DEFAULT 'AVAILABLE',
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    UNIQUE KEY uq_charger_name (name)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 자재 마스터 (분리)
-- ---------------------------------------------------------
CREATE TABLE material (
    material_id INT AUTO_INCREMENT PRIMARY KEY,
    name VARCHAR(80) NOT NULL, -- 예: 침대상판1, 다리2, 작업킷, 의자바퀴
    material_type ENUM('TOP', 'LEG', 'WHEEL', 'KIT') NOT NULL,
    qty_on_hand INT NOT NULL DEFAULT 0, -- 현재 재고(캐시)
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    UNIQUE KEY uq_material_name (name),
    KEY idx_material_type (material_type)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 5) 가구 (가구 + 자재구성 통합)
-- furniture_id는 1~14 고정(침대4/책상4/의자8)
-- 의자에서 바퀴가 없는 경우: wheel_material_id NULL, wheel_qty_per_unit=0
-- ---------------------------------------------------------
CREATE TABLE furniture (
    furniture_id INT PRIMARY KEY, -- 1~14 고정 값 사용(자동증가 아님)
    category ENUM('BED', 'DESK', 'CHAIR') NOT NULL,
    name VARCHAR(60) NOT NULL,
    top_material_id INT NOT NULL,
    top_qty_per_unit INT NOT NULL DEFAULT 1,
    leg_material_id INT NOT NULL,
    leg_qty_per_unit INT NOT NULL DEFAULT 4,
    wheel_material_id INT NULL,
    wheel_qty_per_unit INT NOT NULL DEFAULT 0,
    kit_material_id INT NOT NULL,
    kit_qty_per_unit INT NOT NULL DEFAULT 1,
    active BOOLEAN NOT NULL DEFAULT TRUE,
    CONSTRAINT fk_furn_top FOREIGN KEY (top_material_id) REFERENCES material (material_id) ON DELETE RESTRICT ON UPDATE CASCADE,
    CONSTRAINT fk_furn_leg FOREIGN KEY (leg_material_id) REFERENCES material (material_id) ON DELETE RESTRICT ON UPDATE CASCADE,
    CONSTRAINT fk_furn_whl FOREIGN KEY (wheel_material_id) REFERENCES material (material_id) ON DELETE RESTRICT ON UPDATE CASCADE,
    CONSTRAINT fk_furn_kit FOREIGN KEY (kit_material_id) REFERENCES material (material_id) ON DELETE RESTRICT ON UPDATE CASCADE,
    UNIQUE KEY uq_furniture_name (name),
    KEY idx_furniture_category (category)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 2) 주문 (고객 전화번호 컬럼 제거 버전)
-- ---------------------------------------------------------
CREATE TABLE orders (
    order_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    customer_id BIGINT NOT NULL,
    furniture_id INT NOT NULL,
    quantity INT NOT NULL DEFAULT 1,
    status ENUM(
        'RECEIVED',
        'MAKING',
        'IN_PROGRESS',
        'DONE',
        'CANCELLED'
    ) NOT NULL DEFAULT 'RECEIVED',
    ordered_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    started_at TIMESTAMP NULL,
    finished_at TIMESTAMP NULL,
    CONSTRAINT fk_orders_customer FOREIGN KEY (customer_id) REFERENCES customer (customer_id) ON DELETE RESTRICT ON UPDATE CASCADE,
    CONSTRAINT fk_orders_furniture FOREIGN KEY (furniture_id) REFERENCES furniture (furniture_id) ON DELETE RESTRICT ON UPDATE CASCADE,
    KEY idx_orders_customer_time (customer_id, ordered_at),
    KEY idx_orders_status_time (status, ordered_at)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 1) 로봇 (현재 상태 테이블)
-- 로봇팔 4대, 운송 핑키 2대, 정찰 핑키 1대는 row 7개로 관리
-- ---------------------------------------------------------
CREATE TABLE robot (
    robot_id INT AUTO_INCREMENT PRIMARY KEY,
    robot_kind ENUM('ARM', 'PINKY') NOT NULL,
    robot_role ENUM(
        'ARM_1',
        'ARM_2',
        'ARM_3',
        'ARM_4',
        'PINKY_TRANS_1',
        'PINKY_TRANS_2',
        'PINKY_PATROL'
    ) NOT NULL,
    pose_x DOUBLE NULL,
    pose_y DOUBLE NULL,
    pose_yaw DOUBLE NULL,
    current_order_id BIGINT NULL,
    action_state ENUM(
        'IDLE',
        'TRANSPORTING',
        'PICKING',
        'ASSEMBLING',
        'PATROLLING',
        'CHARGING',
        'ERROR',
        'OFFLINE'
    ) NOT NULL DEFAULT 'OFFLINE',
    battery_percent DOUBLE NULL,
    charger_id INT NULL,
    is_charging BOOLEAN NOT NULL DEFAULT FALSE,
    last_seen_at TIMESTAMP NULL,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    CONSTRAINT fk_robot_current_order FOREIGN KEY (current_order_id) REFERENCES orders (order_id) ON DELETE SET NULL ON UPDATE CASCADE,
    CONSTRAINT fk_robot_charger FOREIGN KEY (charger_id) REFERENCES charging_station (charger_id) ON DELETE SET NULL ON UPDATE CASCADE,
    UNIQUE KEY uq_robot_role (robot_role),
    KEY idx_robot_state (action_state),
    KEY idx_robot_kind (robot_kind)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 7) 로봇 작업 테이블 robot_job
-- 주문을 공장 실행 단계로 쪼개서 로봇에 할당/추적
-- ---------------------------------------------------------
CREATE TABLE robot_job (
    job_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    order_id BIGINT NOT NULL,
    robot_id INT NULL,
    job_type ENUM(
        'TRANSPORT',
        'PICK',
        'PLACE',
        'ASSEMBLE',
        'PATROL',
        'CHARGE'
    ) NOT NULL,
    status ENUM(
        'QUEUED',
        'RUNNING',
        'DONE',
        'FAILED',
        'CANCELLED'
    ) NOT NULL DEFAULT 'QUEUED',
    from_charger_id INT NULL,
    to_charger_id INT NULL,
    note VARCHAR(255) NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    started_at TIMESTAMP NULL,
    finished_at TIMESTAMP NULL,
    CONSTRAINT fk_robot_job_order FOREIGN KEY (order_id) REFERENCES orders (order_id) ON DELETE CASCADE ON UPDATE CASCADE,
    CONSTRAINT fk_robot_job_robot FOREIGN KEY (robot_id) REFERENCES robot (robot_id) ON DELETE SET NULL ON UPDATE CASCADE,
    CONSTRAINT fk_robot_job_from_charger FOREIGN KEY (from_charger_id) REFERENCES charging_station (charger_id) ON DELETE SET NULL ON UPDATE CASCADE,
    CONSTRAINT fk_robot_job_to_charger FOREIGN KEY (to_charger_id) REFERENCES charging_station (charger_id) ON DELETE SET NULL ON UPDATE CASCADE,
    KEY idx_job_order (order_id),
    KEY idx_job_robot_status (robot_id, status),
    KEY idx_job_status_time (status, created_at)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 8) 로봇 상태 로그 robot_state_log
-- 로봇의 좌표/배터리/상태 변화를 시계열로 저장
-- ---------------------------------------------------------
CREATE TABLE robot_state_log (
    log_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    robot_id INT NOT NULL,
    pose_x DOUBLE NULL,
    pose_y DOUBLE NULL,
    pose_yaw DOUBLE NULL,
    battery_percent DOUBLE NULL,
    action_state ENUM(
        'IDLE',
        'TRANSPORTING',
        'PICKING',
        'ASSEMBLING',
        'PATROLLING',
        'CHARGING',
        'ERROR',
        'OFFLINE'
    ) NOT NULL,
    current_order_id BIGINT NULL,
    ts TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_rsl_robot FOREIGN KEY (robot_id) REFERENCES robot (robot_id) ON DELETE CASCADE ON UPDATE CASCADE,
    CONSTRAINT fk_rsl_order FOREIGN KEY (current_order_id) REFERENCES orders (order_id) ON DELETE SET NULL ON UPDATE CASCADE,
    KEY idx_rsl_robot_ts (robot_id, ts),
    KEY idx_rsl_state_ts (action_state, ts)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 6) 자재 재고변동기록 inventory_tx
-- OUT은 qty_delta를 음수로 저장하는 것을 권장
-- ---------------------------------------------------------
CREATE TABLE inventory_tx (
    tx_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    material_id INT NOT NULL,
    order_id BIGINT NULL,
    tx_type ENUM('IN', 'OUT', 'ADJUST') NOT NULL,
    qty_delta INT NOT NULL,
    reason VARCHAR(255) NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_invtx_material FOREIGN KEY (material_id) REFERENCES material (material_id) ON DELETE RESTRICT ON UPDATE CASCADE,
    CONSTRAINT fk_invtx_order FOREIGN KEY (order_id) REFERENCES orders (order_id) ON DELETE SET NULL ON UPDATE CASCADE,
    KEY idx_invtx_material_time (material_id, created_at),
    KEY idx_invtx_order (order_id)
) ENGINE = InnoDB;