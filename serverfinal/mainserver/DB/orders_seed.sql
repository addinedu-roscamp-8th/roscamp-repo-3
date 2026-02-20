-- =========================================================
-- Order Seed Data (Simulating Web Orders)
-- =========================================================

USE factory_system;

-- 0. Cleanup Previous Test Data (For repeated testing)
-- Delete orders for WebUser
DELETE FROM orders WHERE customer_id IN (SELECT customer_id FROM customer WHERE name='WebUser');
-- Delete WebUser
DELETE FROM customer WHERE name='WebUser';


-- 1. Create a Customer
INSERT INTO customer (name, phone, address) VALUES ('WebUser', '010-0000-0000', 'Seoul');

SET @cust_id = LAST_INSERT_ID();

-- 2. Create Orders
-- Order 1: Bed (id=1) x 1
INSERT INTO orders (customer_id, furniture_id, quantity, status) 
VALUES (@cust_id, 1, 1, 'RECEIVED');

-- Order 2: Chair (id=9) x 2
INSERT INTO orders (customer_id, furniture_id, quantity, status) 
VALUES (@cust_id, 9, 2, 'RECEIVED');

-- Order 3: Desk (id=5) x 2
INSERT INTO orders (customer_id, furniture_id, quantity, status) 
VALUES (@cust_id, 5, 2, 'RECEIVED');
