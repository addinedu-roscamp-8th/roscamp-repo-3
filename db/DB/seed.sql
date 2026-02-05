-- =========================================================
-- seed.sql
-- 초기 데이터 적재 (자재, 가구, 로봇, 충전소)
-- =========================================================

USE factory_system;

-- 1. Charging Stations (충전소)
INSERT INTO
    charging_station (name, x, y, status)
VALUES (
        'Charger-1',
        10.0,
        10.0,
        'AVAILABLE'
    ),
    (
        'Charger-2',
        90.0,
        10.0,
        'AVAILABLE'
    ),
    (
        'Charger-3',
        10.0,
        90.0,
        'AVAILABLE'
    ),
    (
        'Charger-4',
        90.0,
        90.0,
        'AVAILABLE'
    );

-- 2. Setup Materials (자재)
-- Material Types: 'TOP', 'LEG', 'WHEEL', 'KIT'
INSERT INTO
    material (
        name,
        material_type,
        qty_on_hand
    )
VALUES ('Wood Top', 'TOP', 50),
    ('Plywood Top', 'TOP', 50),
    ('Leg Style A', 'LEG', 200),
    ('Leg Style B', 'LEG', 200),
    ('Rubber Wheel', 'WHEEL', 100),
    ('Assembly Kit', 'KIT', 500);

-- 3. Configure Furniture Products (가구 구성)
-- IDs 1~14 Fixed
-- Assume Materials IDs:
-- 1: Wood Top, 2: Plywood Top
-- 3: Leg A, 4: Leg B
-- 5: Wheel
-- 6: Kit

-- BED (1~4): Top + 4 Legs + Kit
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
VALUES (
        1,
        'BED',
        'Bed (Wood/LegA)',
        1,
        1,
        3,
        4,
        NULL,
        0,
        6
    ),
    (
        2,
        'BED',
        'Bed (Wood/LegB)',
        1,
        1,
        4,
        4,
        NULL,
        0,
        6
    ),
    (
        3,
        'BED',
        'Bed (Ply/LegA)',
        2,
        1,
        3,
        4,
        NULL,
        0,
        6
    ),
    (
        4,
        'BED',
        'Bed (Ply/LegB)',
        2,
        1,
        4,
        4,
        NULL,
        0,
        6
    );

-- DESK (5~8): Top + 4 Legs + Kit
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
VALUES (
        5,
        'DESK',
        'Desk (Wood/LegA)',
        1,
        1,
        3,
        4,
        NULL,
        0,
        6
    ),
    (
        6,
        'DESK',
        'Desk (Wood/LegB)',
        1,
        1,
        4,
        4,
        NULL,
        0,
        6
    ),
    (
        7,
        'DESK',
        'Desk (Ply/LegA)',
        2,
        1,
        3,
        4,
        NULL,
        0,
        6
    ),
    (
        8,
        'DESK',
        'Desk (Ply/LegB)',
        2,
        1,
        4,
        4,
        NULL,
        0,
        6
    );

-- CHAIR (9~14): Top + 4 Legs + (Optional 4 Wheels) + Kit
-- 9-10: No Wheels
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
VALUES (
        9,
        'CHAIR',
        'Chair (Wood/Fixed)',
        1,
        1,
        3,
        4,
        NULL,
        0,
        6
    ),
    (
        10,
        'CHAIR',
        'Chair (Ply/Fixed)',
        2,
        1,
        4,
        4,
        NULL,
        0,
        6
    );

-- 11-14: With Wheels
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
VALUES (
        11,
        'CHAIR',
        'Office Chair (Wood/A)',
        1,
        1,
        3,
        4,
        5,
        4,
        6
    ),
    (
        12,
        'CHAIR',
        'Office Chair (Wood/B)',
        1,
        1,
        4,
        4,
        5,
        4,
        6
    ),
    (
        13,
        'CHAIR',
        'Office Chair (Ply/A)',
        2,
        1,
        3,
        4,
        5,
        4,
        6
    ),
    (
        14,
        'CHAIR',
        'Office Chair (Ply/B)',
        2,
        1,
        4,
        4,
        5,
        4,
        6
    );

-- 4. Initial Robot Fleet (로봇)
INSERT INTO
    robot (
        robot_kind,
        robot_role,
        pose_x,
        pose_y,
        action_state,
        battery_percent,
        charger_id
    )
VALUES (
        'ARM',
        'ARM_1',
        20,
        20,
        'IDLE',
        100,
        NULL
    ),
    (
        'ARM',
        'ARM_2',
        20,
        80,
        'IDLE',
        100,
        NULL
    ),
    (
        'ARM',
        'ARM_3',
        80,
        20,
        'IDLE',
        100,
        NULL
    ),
    (
        'ARM',
        'ARM_4',
        80,
        80,
        'IDLE',
        100,
        NULL
    ),
    (
        'PINKY',
        'PINKY_TRANS_1',
        10,
        10,
        'CHARGING',
        95,
        1
    ),
    (
        'PINKY',
        'PINKY_TRANS_2',
        90,
        10,
        'CHARGING',
        98,
        2
    ),
    (
        'PINKY',
        'PINKY_PATROL',
        50,
        50,
        'PATROLLING',
        80,
        NULL
    );