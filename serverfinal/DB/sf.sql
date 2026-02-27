-- =========================================================
-- factory_full_schema.sql (MySQL 8+, InnoDB, utf8mb4)
-- 총 테이블 10개
-- 1) customer (고객)
-- 2) charging_station (충전소)
-- 3) material (자재 마스터)
-- 4) furniture (가구 + 자재구성 통합)
-- 5) orders (주문)
-- 6) robot (로봇 상태)
-- 7) robot_job (로봇 작업)
-- 8) robot_state_log (로봇 상태 로그)
-- 9) inventory_tx (자재 재고변동기록)
-- 10) fire_incidents (화재 사건 로그)
-- =========================================================

CREATE DATABASE IF NOT EXISTS factory_system DEFAULT CHARACTER SET utf8mb4 DEFAULT COLLATE utf8mb4_0900_ai_ci;

SET FOREIGN_KEY_CHECKS = 0;

-- ---------------------------------------------------------
-- 1) 고객
-- ---------------------------------------------------------
DROP TABLE IF EXISTS customer;
CREATE TABLE customer (
    customer_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    name VARCHAR(100) NULL,
    phone VARCHAR(30) NOT NULL,
    address VARCHAR(255) NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    UNIQUE KEY uq_customer_phone (phone)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 2) 충전소
-- ---------------------------------------------------------
DROP TABLE IF EXISTS charging_station;
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
-- 3) 자재 마스터 (분리)
-- ---------------------------------------------------------
DROP TABLE IF EXISTS material;
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
-- 4) 가구 (가구 + 자재구성 통합)
-- furniture_id는 1~14 고정(침대4/책상4/의자8)
-- 의자에서 바퀴가 없는 경우: wheel_material_id NULL, wheel_qty_per_unit=0
-- ---------------------------------------------------------
DROP TABLE IF EXISTS furniture;
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
-- 5) 주문 
-- ---------------------------------------------------------
DROP TABLE IF EXISTS orders;
CREATE TABLE orders (
    order_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    customer_id BIGINT NOT NULL,
    furniture_id INT NOT NULL,
    quantity INT NOT NULL DEFAULT 1,
    status ENUM(
        'RECEIVED',
        'IN_PROGRESS',
        'DONE'
    ) NOT NULL DEFAULT 'RECEIVED',
    picking_command DOUBLE NULL,
    packing_command DOUBLE NULL,
    ordered_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    started_at TIMESTAMP NULL,
    finished_at TIMESTAMP NULL,
    CONSTRAINT fk_orders_customer FOREIGN KEY (customer_id) REFERENCES customer (customer_id) ON DELETE RESTRICT ON UPDATE CASCADE,
    CONSTRAINT fk_orders_furniture FOREIGN KEY (furniture_id) REFERENCES furniture (furniture_id) ON DELETE RESTRICT ON UPDATE CASCADE,
    KEY idx_orders_customer_time (customer_id, ordered_at),
    KEY idx_orders_status_time (status, ordered_at)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 6) 로봇팔 (Robot Arm)
-- ---------------------------------------------------------
DROP TABLE IF EXISTS robot_arm;
CREATE TABLE robot_arm (
    arm_id INT AUTO_INCREMENT PRIMARY KEY,
    robot_role ENUM('ARM_1', 'ARM_2') NOT NULL,
    pose_x DOUBLE NULL,
    pose_y DOUBLE NULL,
    pose_z DOUBLE NULL,
    pose_roll DOUBLE NULL,
    pose_pitch DOUBLE NULL,
    pose_yaw DOUBLE NULL,
    current_order_id BIGINT NULL,
    action_state INT NOT NULL DEFAULT 0,
    last_seen_at TIMESTAMP NULL,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    CONSTRAINT fk_arm_current_order FOREIGN KEY (current_order_id) REFERENCES orders (order_id) ON DELETE SET NULL ON UPDATE CASCADE,
    UNIQUE KEY uq_arm_role (robot_role),
    KEY idx_arm_state (action_state)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 6-1) 핑키 (Robot Pinky)
-- ---------------------------------------------------------
DROP TABLE IF EXISTS robot_pinky;
CREATE TABLE robot_pinky (
    pinky_id INT AUTO_INCREMENT PRIMARY KEY,
    robot_role ENUM('PINKY_1', 'PINKY_2', 'PINKY_3') NOT NULL,
    battery_percent DOUBLE NULL,
    charger_id INT NULL,
    is_charging BOOLEAN NOT NULL DEFAULT FALSE,
    current_order_id BIGINT NULL,
    action_state INT NOT NULL DEFAULT 0,
    last_seen_at TIMESTAMP NULL,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    CONSTRAINT fk_pinky_current_order FOREIGN KEY (current_order_id) REFERENCES orders (order_id) ON DELETE SET NULL ON UPDATE CASCADE,
    CONSTRAINT fk_pinky_charger FOREIGN KEY (charger_id) REFERENCES charging_station (charger_id) ON DELETE SET NULL ON UPDATE CASCADE,
    UNIQUE KEY uq_pinky_role (robot_role),
    KEY idx_pinky_state (action_state)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 7) 로봇 작업 테이블 robot_job
-- 주문을 공장 실행 단계로 쪼개서 로봇에 할당/추적
-- ---------------------------------------------------------
DROP TABLE IF EXISTS robot_job;
CREATE TABLE robot_job (
    job_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    order_id BIGINT NOT NULL,
    arm_id INT NULL,
    pinky_id INT NULL,
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
    CONSTRAINT fk_robot_job_arm FOREIGN KEY (arm_id) REFERENCES robot_arm (arm_id) ON DELETE SET NULL ON UPDATE CASCADE,
    CONSTRAINT fk_robot_job_pinky FOREIGN KEY (pinky_id) REFERENCES robot_pinky (pinky_id) ON DELETE SET NULL ON UPDATE CASCADE,
    CONSTRAINT fk_robot_job_from_charger FOREIGN KEY (from_charger_id) REFERENCES charging_station (charger_id) ON DELETE SET NULL ON UPDATE CASCADE,
    CONSTRAINT fk_robot_job_to_charger FOREIGN KEY (to_charger_id) REFERENCES charging_station (charger_id) ON DELETE SET NULL ON UPDATE CASCADE,
    KEY idx_job_order (order_id),
    KEY idx_job_status_time (status, created_at)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 8) 로봇 상태 로그 robot_state_log
-- 로봇의 좌표/배터리/상태 변화를 시계열로 저장
-- ---------------------------------------------------------
DROP TABLE IF EXISTS robot_state_log;
CREATE TABLE robot_state_log (
    log_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    arm_id INT NULL,
    pinky_id INT NULL,
    action_state INT NOT NULL,
    ts TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_rsl_arm FOREIGN KEY (arm_id) REFERENCES robot_arm (arm_id) ON DELETE CASCADE ON UPDATE CASCADE,
    CONSTRAINT fk_rsl_pinky FOREIGN KEY (pinky_id) REFERENCES robot_pinky (pinky_id) ON DELETE CASCADE ON UPDATE CASCADE,
    KEY idx_rsl_arm_ts (arm_id, ts),
    KEY idx_rsl_pinky_ts (pinky_id, ts)
) ENGINE = InnoDB;

-- ---------------------------------------------------------
-- 9) 자재 재고변동기록 inventory_tx
-- OUT은 qty_delta를 음수로 저장하는 것을 권장
-- ---------------------------------------------------------
DROP TABLE IF EXISTS inventory_tx;
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

-- ---------------------------------------------------------
-- 10) 화재 사건 로그 fire_incidents
-- ---------------------------------------------------------
DROP TABLE IF EXISTS fire_incidents;
CREATE TABLE fire_incidents (
    incident_id BIGINT AUTO_INCREMENT PRIMARY KEY,
    occurred_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    location VARCHAR(255) NULL,
    description TEXT NULL,
    image_path VARCHAR(512) NULL,
    is_handled BOOLEAN NOT NULL DEFAULT FALSE,
    handled_at TIMESTAMP NULL,
    KEY idx_fire_time (occurred_at),
    KEY idx_fire_handled (is_handled)
) ENGINE = InnoDB;