#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
import json
import paho.mqtt.client as mqtt

from custom_interfaces.srv import UpdateStatus
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32

from .dynamixel_driver import DynamixelDriver
from pinkylib import IR  # ir.py의 IR 클래스 임포트

# --- ROS 토픽 및 프레임 설정 ---
TWIST_SUB_TOPIC_NAME = "cmd_vel"
ODOM_PUB_TOPIC_NAME = "odom"
JOINT_PUB_TOPIC_NAME = "joint_states"
ODOM_FRAME_ID = "odom"
ODOM_CHILD_FRAME_ID = "base_footprint"

# --- 하드웨어 제원 및 포트 설정 ---
SERIAL_PORT_NAME = "/dev/ttyAMA4"
BAUDRATE = 1000000
DYNAMIXEL_IDS = [1, 2] 
JOINT_NAME_WHEEL_L = "left_wheel_joint"
JOINT_NAME_WHEEL_R = "right_wheel_joint"

WHEEL_RAD = 0.028
PULSE_PER_ROT = 4096 
WHEEL_BASE = 0.0961
RPM2RAD = 2 * math.pi / 60
CIRCUMFERENCE = 2 * math.pi * WHEEL_RAD

# --- 배터리 및 MQTT 설정 ---
BATTERY_VOLTAGE_TOPIC = "battery/voltage"
LOW_BATTERY_THRESHOLD = 6.8
MQTT_BROKER_IP = "192.168.4.7"
MQTT_TOPIC = "robot/status"

# --- 라인 트레이싱 PD 제어 파라미터 ---
# 센서 특성: 300(이탈) ~ 3900(중앙)
VAL_MIN = 300       
VAL_MAX = 3900      
STOP_LINE_VAL = 10000  # 세 센서 모두 이 값 이상이면 가로 정지선 감지
BASE_RPM = 27.0      # 라인 트레이싱 시 기본 전진 속도
KP = 0.006           # 비례 이득 (에러 값의 범위가 크므로 미세 조정)
KD = 0.010           # 미분 이득 (진동 억제)

class Pinky(Node):
    def __init__(self):
        super().__init__('pinky_bringup')
        self.is_initialized = False

        # 1. IR 센서 객체 생성
        try:
            self.ir_sensor = IR()
            self.get_logger().info('IR Sensor (Analog) initialized.')
        except Exception as e:
            self.get_logger().error(f'IR Sensor Init Failed: {e}')
            return

        # 2. MQTT 클라이언트 설정
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.connect(MQTT_BROKER_IP, 1883)
            self.mqtt_client.loop_start()
            self.get_logger().info('MQTT Broker connected.')
        except Exception as e:
            self.get_logger().error(f'MQTT Connection Failed: {e}')

        self.virtual_battery = 100.0
        self.robot_id = 'Pinky_d9ec'

        # 3. Dynamixel 드라이버 초기화
        self.driver = DynamixelDriver(SERIAL_PORT_NAME, BAUDRATE, DYNAMIXEL_IDS)
        if not self.driver.begin() or not self.driver.initialize_motors():
            self.get_logger().error("Hardware Initialization Failed!")
            return

        # 4. 초기 상태 및 변수 설정
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_analog_error = 0.0
        self.current_moving_state = "IDLE"
        self.last_time = self.get_clock().now()
        
        _, _, self.last_encoder_l, self.last_encoder_r = self.driver.get_feedback()

        # 5. 통신 및 타이머 설정
        self.odom_pub = self.create_publisher(Odometry, ODOM_PUB_TOPIC_NAME, 10)
        self.joint_pub = self.create_publisher(JointState, JOINT_PUB_TOPIC_NAME, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.db_client = self.create_client(UpdateStatus, 'update_status')

        self.battery_timer = self.create_timer(1.0, self.virtual_battery_timer_callback)
        self.timer = self.create_timer(1.0 / 30.0, self.update_and_publish) # 30Hz 제어 루프

        self.get_logger().info('Pinky Line-Tracer Bringup Started.')
        self.is_initialized = True

    def get_line_error(self):
        """아날로그 값을 읽어 PD 제어용 에러 반환"""
        vals = self.ir_sensor.read_ir()
        if vals is None or len(vals) < 3:
            return None

        l_val, c_val, r_val = vals

        # 가로 정지선 체크: 세 센서 모두 검은색(높은 값) 감지 시
        if l_val > STOP_LINE_VAL and c_val > STOP_LINE_VAL and r_val > STOP_LINE_VAL:
            return "STOP"

        # 라인 이탈 체크: 모든 센서가 흰색 바닥(최저 값) 감지 시
        if max(vals) < (VAL_MIN + 200):
            return "LOST"

        # 오차 계산: (오른쪽 센서 - 왼쪽 센서)
        # 오른쪽 센서값이 높으면 양수(+) 에러 -> 좌회전 필요
        return float(r_val - l_val)

    def update_and_publish(self):
        if not self.is_initialized:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        # --- [라인 트레이싱 PD 제어 핵심 로직] ---
        error = self.get_line_error()

        if error == "STOP":
            self.driver.set_double_rpm(0.0, 0.0)
            if self.current_moving_state != "IDLE":
                self.send_db_update("IDLE")
                self.current_moving_state = "IDLE"
                self.get_logger().info("STOP LINE DETECTED: Stopping Robot.")
        
        elif error == "LOST":
            self.driver.set_double_rpm(0.0, 0.0)
            self.get_logger().warn("LINE LOST: Emergency Stop.")

        elif error is not None:
            # PD 제어량 계산
            derivative = (error - self.prev_analog_error) / dt
            pd_output = (KP * error) + (KD * derivative)

            # 조향을 위한 좌우 RPM 배분
            rpm_l = BASE_RPM + pd_output
            rpm_r = -(BASE_RPM - pd_output) # 모터 방향 반전 반영

            # 모터 명령 전송
            if self.driver.set_double_rpm(rpm_l, rpm_r):
                self.prev_analog_error = error
                if self.current_moving_state != "MOVING":
                    self.send_db_update("MOVING")
                    self.current_moving_state = "MOVING"

        # --- [오도메트리 및 상태 정보 업데이트] ---
        feedback = self.driver.get_feedback()
        if feedback[0] is not None:
            rpm_l_fb, rpm_r_fb, encoder_l, encoder_r = feedback
            
            # 인코더 변화량 계산
            delta_l = encoder_l - self.last_encoder_l
            delta_r = -(encoder_r - self.last_encoder_r) # 오른쪽 모터 역방향 처리
            
            self.last_encoder_l = encoder_l
            self.last_encoder_r = encoder_r

            # 이동 거리 계산
            dist_l = (delta_l / PULSE_PER_ROT) * CIRCUMFERENCE
            dist_r = (delta_r / PULSE_PER_ROT) * CIRCUMFERENCE
            delta_distance = (dist_r + dist_l) / 2.0
            delta_theta = (dist_r - dist_l) / WHEEL_BASE
            
            # 위치(Pose) 적분
            self.theta += delta_theta
            self.x += delta_distance * math.cos(self.theta)
            self.y += delta_distance * math.sin(self.theta)
            
            v_x = delta_distance / dt
            vth = delta_theta / dt

            # ROS 메시지 발행
            self._publish_tf(current_time)
            self._publish_odometry(current_time, v_x, vth)
            self._publish_joint_states(current_time, rpm_l_fb, rpm_r_fb)

        self.last_time = current_time

    def virtual_battery_timer_callback(self):
        """시나리오: 1초에 1%씩 배터리 감소 및 MQTT 전송"""
        if self.virtual_battery > 0:
            self.virtual_battery -= 1.0
        
        payload = {
            "robot_id": self.robot_id,
            "battery": round(self.virtual_battery, 1),
            "status": self.current_moving_state,
            "timestamp": time.time()
        }
        try:
            self.mqtt_client.publish(MQTT_TOPIC, json.dumps(payload))
        except:
            pass

    def send_db_update(self, state):
        """DB 서비스로 로봇 상태 전송"""
        if not self.db_client.wait_for_service(timeout_sec=0.1):
            return
        req = UpdateStatus.Request()
        req.robot_id = self.robot_id
        req.status = state
        self.db_client.call_async(req)

    def _publish_tf(self, current_time):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = ODOM_FRAME_ID
        t.child_frame_id = ODOM_CHILD_FRAME_ID
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

    def _publish_odometry(self, current_time, v_x, vth):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = ODOM_FRAME_ID
        odom_msg.child_frame_id = ODOM_CHILD_FRAME_ID
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y = self.x, self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = q
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z = v_x, vth
        self.odom_pub.publish(odom_msg)

    def _publish_joint_states(self, current_time, rpm_l, rpm_r):
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = [JOINT_NAME_WHEEL_L, JOINT_NAME_WHEEL_R]
        pos_l = (self.last_encoder_l / PULSE_PER_ROT) * (2 * math.pi)
        pos_r = (self.last_encoder_r / PULSE_PER_ROT) * (2 * math.pi)
        joint_msg.position = [pos_l, pos_r]
        joint_msg.velocity = [rpm_l * RPM2RAD, rpm_r * RPM2RAD]
        self.joint_pub.publish(joint_msg)

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.driver.set_double_rpm(0, 0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Pinky()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()