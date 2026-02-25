-- =========================================================
-- Base Structure Data for Factory System
-- Covers fixed metadata: 
-- 1) Customer (Fixed Users)
-- 2) Material (Inventory Master)
-- 3) Furniture (Product Configuration)
-- 4) ORDERS (Fixed Test Orders)
-- =========================================================

USE factory_system;

-- =========================================================
-- 1. Fixed Customer Data (고정 고객)
-- =========================================================
INSERT INTO customer (name, phone, address) VALUES 
('todaylunch', '010-1234-5678', 'Main Control Room'),
('isperfect', '010-9876-5432', 'Seoul, Korea')
ON DUPLICATE KEY UPDATE
    phone = VALUES(phone),
    address = VALUES(address);

-- Variable setting
SET @todaylunch = (SELECT customer_id FROM customer WHERE name='todaylunch' LIMIT 1);
SET @isperfect = (SELECT customer_id FROM customer WHERE name='isperfect' LIMIT 1);


-- =========================================================
-- 2. Material (자재)
-- =========================================================
INSERT INTO material (name, material_type, qty_on_hand) VALUES 
('Wood Top', 'TOP', 100),
('Plywood Top', 'TOP', 100),
('Leg Style A', 'LEG', 400),
('Leg Style B', 'LEG', 400),
('Rubber Wheel', 'WHEEL', 200),
('Assembly Kit', 'KIT', 1000)
ON DUPLICATE KEY UPDATE
    qty_on_hand = VALUES(qty_on_hand);


-- =========================================================
-- 3. Configure Furniture Products (가구 구성)
-- IDs 1~14 Fixed
-- =========================================================
INSERT INTO
    furniture (
        furniture_id,
        category,
        name,
        top_material_id,
        top_qty_per_unit,
        leg_material_id,
        leg_qty_per_unit,
        wheel_material_id,
        wheel_qty_per_unit,
        kit_material_id
    )
VALUES 
    -- BED (1~4): Top + 4 Legs + Kit
    (1, 'BED', 'Bed (Wood/LegA)', 1, 1, 3, 4, NULL, 0, 6),
    (2, 'BED', 'Bed (Wood/LegB)', 1, 1, 4, 4, NULL, 0, 6),
    (3, 'BED', 'Bed (Ply/LegA)', 2, 1, 3, 4, NULL, 0, 6),
    (4, 'BED', 'Bed (Ply/LegB)', 2, 1, 4, 4, NULL, 0, 6),
    
    -- DESK (5~8): Top + 4 Legs + Kit
    (5, 'DESK', 'Desk (Wood/LegA)', 1, 1, 3, 4, NULL, 0, 6),
    (6, 'DESK', 'Desk (Wood/LegB)', 1, 1, 4, 4, NULL, 0, 6),
    (7, 'DESK', 'Desk (Ply/LegA)', 2, 1, 3, 4, NULL, 0, 6),
    (8, 'DESK', 'Desk (Ply/LegB)', 2, 1, 4, 4, NULL, 0, 6),
    
    -- CHAIR (9~14): Top + 4 Legs + (Optional 4 Wheels) + Kit
    (9, 'CHAIR', 'Chair (Wood/Fixed)', 1, 1, 3, 4, NULL, 0, 6),
    (10, 'CHAIR', 'Chair (Ply/Fixed)', 2, 1, 4, 4, NULL, 0, 6),
    (11, 'CHAIR', 'Office Chair (Wood/A)', 1, 1, 3, 4, 5, 4, 6),
    (12, 'CHAIR', 'Office Chair (Wood/B)', 1, 1, 4, 4, 5, 4, 6),
    (13, 'CHAIR', 'Office Chair (Ply/A)', 2, 1, 3, 4, 5, 4, 6),
    (14, 'CHAIR', 'Office Chair (Ply/B)', 2, 1, 4, 4, 5, 4, 6)
ON DUPLICATE KEY UPDATE
    name = VALUES(name),
    top_material_id = VALUES(top_material_id),
    leg_material_id = VALUES(leg_material_id),
    wheel_material_id = VALUES(wheel_material_id),
    kit_material_id = VALUES(kit_material_id);

-- =========================================================
-- 4. Orders (주문) - 5 Orders with mixed status for simulation
-- =========================================================
-- Order 1: RECEIVED
INSERT INTO orders (customer_id, furniture_id, quantity, status, picking_command, packing_command, ordered_at) 
VALUES (@isperfect, 1, 1, 'RECEIVED', NULL, NULL, DATE_SUB(NOW(), INTERVAL 2 DAY))
ON DUPLICATE KEY UPDATE status=VALUES(status);

-- Order 2: RECEIVED
INSERT INTO orders (customer_id, furniture_id, quantity, status, picking_command, packing_command, ordered_at) 
VALUES (@isperfect, 9, 2, 'RECEIVED', NULL, NULL, NOW())
ON DUPLICATE KEY UPDATE status=VALUES(status);

-- Order 3: RECEIVED
INSERT INTO orders (customer_id, furniture_id, quantity, status, picking_command, packing_command, ordered_at) 
VALUES (@todaylunch, 5, 1, 'RECEIVED', NULL, NULL, NOW())
ON DUPLICATE KEY UPDATE status=VALUES(status);

-- Order 4: RECEIVED
INSERT INTO orders (customer_id, furniture_id, quantity, status, picking_command, packing_command, ordered_at) 
VALUES (@isperfect, 11, 1, 'RECEIVED', NULL, NULL, DATE_SUB(NOW(), INTERVAL 1 DAY))
ON DUPLICATE KEY UPDATE status=VALUES(status);

-- Order 5: RECEIVED
INSERT INTO orders (customer_id, furniture_id, quantity, status, picking_command, packing_command, ordered_at) 
VALUES (@todaylunch, 5, 2, 'RECEIVED', NULL, NULL, DATE_SUB(NOW(), INTERVAL 1 HOUR))
ON DUPLICATE KEY UPDATE status=VALUES(status);
