import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Twist

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Parameters
        self.declare_parameter('speed', 0.1)        # Linear velocity (m/s)
        self.declare_parameter('kp', 0.5)          # Proportional gain
        self.declare_parameter('kd', 0.1)          # Derivative gain
        self.declare_parameter('threshold', 500)   # IR threshold (Black/White boundary)
        
        # Subscriptions & Publishers
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            'ir_sensor/range',
            self.ir_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # State variables
        self.last_error = 0.0
        self.last_valid_error = 0.0
        
        self.get_logger().info('Line Follower Node has been started')

    def ir_callback(self, msg):
        if len(msg.data) < 3:
            return

        # IR Sensor indices: Left (0), Center (1), Right (2)
        # Based on PinkySensorADC: push(adc[2]), push(adc[1]), push(adc[0])
        # So data[0] is adc[2], data[1] is adc[1], data[2] is adc[0]
        # We need to confirm robot's physical sensor mapping.
        # Assuming typical 3-ch arrangement:
        left = msg.data[0]
        center = msg.data[1]
        right = msg.data[2]
        
        threshold = self.get_parameter('threshold').value
        
        # Binary state (1 if on line/black, 0 if off line/white)
        l_state = 1 if left > threshold else 0
        c_state = 1 if center > threshold else 0
        r_state = 1 if right > threshold else 0
        
        # Calculate error
        if l_state == 0 and c_state == 1 and r_state == 0:
            error = 0.0
        elif l_state == 1 and c_state == 1 and r_state == 0:
            error = 0.5
        elif l_state == 1 and c_state == 0 and r_state == 0:
            error = 1.0
        elif l_state == 0 and c_state == 1 and r_state == 1:
            error = -0.5
        elif l_state == 0 and c_state == 0 and r_state == 1:
            error = -1.0
        elif l_state == 0 and c_state == 0 and r_state == 0:
            error = self.last_valid_error * 1.5 # Lost the line, overshoot
        else:
            error = 0.0

        if l_state != 0 or c_state != 0 or r_state != 0:
            self.last_valid_error = error

        # PD Control
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        
        steering = (kp * error) + (kd * (error - self.last_error))
        self.last_error = error
        
        twist = Twist()
        if l_state == 0 and c_state == 0 and r_state == 0:
            # Search mode
            twist.linear.x = 0.0
            twist.angular.z = 0.3 if self.last_valid_error > 0 else -0.3
        else:
            twist.linear.x = self.get_parameter('speed').value
            twist.angular.z = float(steering)
            
        self.publisher.publish(twist)
        self.get_logger().debug(f'L:{l_state} C:{c_state} R:{r_state} | Error:{error:.2f} | Steering:{twist.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
