-- =========================================================
-- Comprehensive Seed Data for Factory System
-- Covers: robot, robot_job, robot_state_log, charging_station, fire_incidents
-- (Customer, Material, Furniture, ORDERS are in basestructure.sql)
-- =========================================================

USE factory_system;

-- 1. Setup Variables (Needs to re-select because context is fresh per file)
SET @admin_id = (SELECT customer_id FROM customer WHERE name='todaylunch' LIMIT 1);
SET @user_id = (SELECT customer_id FROM customer WHERE name='isperfect' LIMIT 1);

-- 2. Charging Station (충전소)
-- 이미 존재하면 상태(status)와 좌표(x, y)를 업데이트합니다.
INSERT INTO charging_station (name, x, y, status) VALUES 
('Charger-1', 10.0, 10.0, 'AVAILABLE'),
('Charger-2', 90.0, 10.0, 'AVAILABLE'),
('Charger-3', 10.0, 90.0, 'OCCUPIED')
ON DUPLICATE KEY UPDATE
    x = VALUES(x),
    y = VALUES(y),
    status = VALUES(status);

-- Capture IDs for robot jobs (Orders are now in basestructure)
SET @order_done = (SELECT order_id FROM orders WHERE status='DONE' LIMIT 1);
SET @order_progress = (SELECT order_id FROM orders WHERE status='IN_PROGRESS' LIMIT 1);

-- 3. Robot (로봇)
INSERT INTO robot (robot_kind, robot_role, pose_x, pose_y, action_state, battery_percent, current_order_id) VALUES 
('ARM', 'ARM_1', 20.0, 20.0, 'PICKING', 85.5, @order_progress),
('ARM', 'ARM_2', 30.0, 20.0, 'IDLE', 92.0, NULL),
('PINKY', 'PINKY_1', 15.0, 15.0, 'TRANSPORTING', 75.0, @order_progress),
('PINKY', 'PINKY_2', 50.0, 50.0, 'IDLE', 100.0, NULL),
('PINKY', 'PINKY_3', 90.0, 90.0, 'CHARGING', 40.0, NULL)
ON DUPLICATE KEY UPDATE
    pose_x = VALUES(pose_x),
    pose_y = VALUES(pose_y),
    action_state = VALUES(action_state),
    battery_percent = VALUES(battery_percent),
    current_order_id = VALUES(current_order_id);

SET @robot_arm1 = (SELECT robot_id FROM robot WHERE robot_role='ARM_1');
SET @robot_pinky1 = (SELECT robot_id FROM robot WHERE robot_role='PINKY_1');

-- 4. Robot Job (로봇 작업)
INSERT INTO robot_job (order_id, robot_id, job_type, status, created_at, started_at) VALUES 
(@order_done, @robot_arm1, 'PICK', 'DONE', DATE_SUB(NOW(), INTERVAL 25 HOUR), DATE_SUB(NOW(), INTERVAL 24 HOUR)),
(@order_progress, @robot_arm1, 'PICK', 'RUNNING', NOW(), NOW()),
(@order_progress, @robot_pinky1, 'TRANSPORT', 'QUEUED', NOW(), NULL)
ON DUPLICATE KEY UPDATE
    status = VALUES(status),
    started_at = VALUES(started_at);

-- 5. Robot State Log (로봇 상태 로그)
INSERT INTO robot_state_log (robot_id, pose_x, pose_y, battery_percent, action_state, ts) VALUES 
(@robot_pinky1, 10.0, 10.0, 76.0, 'IDLE', DATE_SUB(NOW(), INTERVAL 10 MINUTE)),
(@robot_pinky1, 12.0, 12.0, 75.8, 'TRANSPORTING', DATE_SUB(NOW(), INTERVAL 5 MINUTE)),
(@robot_pinky1, 15.0, 15.0, 75.0, 'TRANSPORTING', NOW());

-- 6. Fire Incidents (화재 사건 로그)
INSERT INTO fire_incidents (location, severity, description, is_handled) VALUES 
('Warehouse Zone A', 'LOW', 'Small smoke detected near charging station', TRUE),
('Assembly Line 2', 'MEDIUM', 'Overheating detected on conveyor belt', FALSE)
ON DUPLICATE KEY UPDATE
    severity = VALUES(severity),
    description = VALUES(description),
    is_handled = VALUES(is_handled);
