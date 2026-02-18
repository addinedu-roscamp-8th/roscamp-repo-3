#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
import math
import networkx as nx
import time
import numpy as np
import sys
from collections import deque

# IR 센서 모듈 임포트 (하드웨어 없는 경우 더미 클래스 사용)
try:
    from ir import IR
except ImportError:
    class IR:
        def __init__(self): pass
        def read_ir(self): return [3900, 3900, 3900] # 항상 라인 위라고 가정

class PinkyStateMachine(Node):
    def __init__(self, robot_id):
        super().__init__(f'pinky_machine_{robot_id}')
        self.robot_id = robot_id
        
        # --- State Machine & Battery ---
        self.state = 6       # 초기 상태: 6 (충전중)
        self.battery = 50.0  # 초기 배터리 (테스트용 50%)
        self.current_job = None # 할당받은 픽업 WP
        
        # --- Nav2 & ROS ---
        self.navigator = BasicNavigator()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Traffic & Comms ---
        self.traffic_pub = self.create_publisher(String, '/traffic/request', 10)
        self.traffic_sub = self.create_subscription(String, '/traffic/response', self.traffic_callback, 10)
        self.latest_traffic_status = None 
        self.server_cmd = None # DISPATCH, PACKING_DONE, MOVE_UP 등

        # --- Battery Timer (1.0s) ---
        self.create_timer(1.0, self.battery_callback)

        # --- IR Sensor & Line Tracing ---
        try:
            self.ir_sensor = IR()
            self.get_logger().info("IR Sensor Initialized.")
        except Exception:
            self.ir_sensor = None
            self.get_logger().warn("IR Sensor Init Failed.")
            
        self.VAL_MIN, self.VAL_MAX, self.STOP_LINE_VAL = 300, 3900, 3500
        self.BASE_SPEED_MPS = 0.08 
        self.KP = 0.003
        self.prev_error = 0.0

        # --- Map & Graph ---
        self.waypoints = {
            'WP1': (0.435, -0.36), 'WP2': (0.435, 0.01), 'WP3': (0.435, 0.38),
            'WP4': (0.88, -0.36), 'WP5': (0.88, 0.01), 'WP6': (0.88, 0.38),
            'WP7': (1.325, -0.36), 'WP8': (1.325, 0.01), 'WP9': (1.325, 0.38),
            'WP10': (0.205, -0.36), 'WP11': (-0.015, -0.36), 'WP12': (-0.205, -0.36)
        }
        self.graph = nx.Graph()
        self.graph.add_edge('WP1', 'WP2'); self.graph.add_edge('WP2', 'WP3')
        self.graph.add_edge('WP4', 'WP5'); self.graph.add_edge('WP5', 'WP6')
        self.graph.add_edge('WP7', 'WP8'); self.graph.add_edge('WP8', 'WP9')
        self.graph.add_edge('WP1', 'WP4'); self.graph.add_edge('WP4', 'WP7')
        self.graph.add_edge('WP2', 'WP5'); self.graph.add_edge('WP5', 'WP8')
        self.graph.add_edge('WP3', 'WP6'); self.graph.add_edge('WP6', 'WP9')
        self.graph.add_edge('WP12', 'WP11'); self.graph.add_edge('WP11', 'WP10'); self.graph.add_edge('WP10', 'WP1')

    # =========================================================
    # [Core] Battery & Comms Logic
    # =========================================================
    def battery_callback(self):
        """1초마다 호출: 배터리 증감 로직"""
        if self.state == 6: # 충전중
            self.battery += 1.0
            if self.battery > 100.0: self.battery = 100.0
        else: # 그 외 모든 상태 (운송, 대기, 이동 등)
            self.battery -= 1.0
            if self.battery < 0.0: self.battery = 0.0

    def traffic_callback(self, msg):
        try:
            parts = msg.data.split('|')
            if parts[0] == self.robot_id:
                cmd_type = parts[1]
                if cmd_type in ["APPROVED", "WAIT"]:
                    self.latest_traffic_status = cmd_type
                else:
                    # DISPATCH, PACKING_DONE, CHARGING_ASSIGN, MOVE_UP 등
                    self.server_cmd = parts 
        except IndexError:
            pass

    def send_request(self, msg_str):
        msg = String()
        msg.data = msg_str
        self.traffic_pub.publish(msg)

    # =========================================================
    # [State Machine] Main Loop
    # =========================================================
    def run_state_machine(self):
        self.navigator.waitUntilNav2Active()
        self.initialize_pose()
        
        self.get_logger().info(f"🤖 State Machine Start! Initial State: {self.state}")
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # --- State 1: 운송 대기중 ---
            if self.state == 1:
                # 조건: 배터리 80% 이상 & 관제 서버에서 작업 할당
                if self.battery < 80.0:
                    print("⚠️ 배터리 부족 (80% 미만). 충전 필요.")
                    self.state = 6 # 즉시 충전 모드로 (현 위치가 충전소라고 가정)
                    continue

                if self.current_job is None:
                    # 작업 요청 (2초에 한번)
                    if int(time.time()) % 2 == 0:
                        print(f"[{self.robot_id}] State 1: 작업 요청 중... (Batt: {self.battery}%)")
                        self.send_request(f"REQ_TASK|{self.robot_id}")
                    
                    # 서버 응답 확인
                    if self.server_cmd and self.server_cmd[1] == "DISPATCH":
                        self.current_job = self.server_cmd[2] # WP4, WP6 등
                        self.server_cmd = None
                        print(f"✅ 작업 할당됨! 픽업지: {self.current_job}")
                        self.state = 2
            
            # --- State 2: 운송중 (Nav2) ---
            elif self.state == 2:
                print(f"[{self.robot_id}] State 2: 운송 시작 (Pickup: {self.current_job} -> Drop: WP3)")
                
                # 1. 현재 위치 파악
                cands = self.get_candidate_wps(self.get_current_xy())
                curr_node = cands[0] if cands else "Start_Point"
                
                # 2. 픽업지로 이동
                print(f">>> 🚚 픽업지({self.current_job})로 이동 중...")
                self.navigate_segment(curr_node, self.current_job, 90.0, "픽업 이동")
                print(">>> 📦 물건 적재 (2초)")
                time.sleep(2.0)
                
                # 3. 하차지(WP3)로 이동
                print(f">>> 🚚 하차지(WP3)로 이동 중...")
                self.navigate_segment(self.current_job, 'WP3', 180.0, "하차지 이동")
                
                print(f"[{self.robot_id}] WP3 도착. State 3(출고)로 전환.")
                self.state = 3
                
            # --- State 3: 출고중 (Line Trace) ---
            elif self.state == 3:
                print(f"[{self.robot_id}] State 3: 라인 트레이싱 시작 (포장 구역 이동)")
                # 첫 번째 정지선까지 이동
                self.follow_line_until_stop(stop_count_target=1)
                print(f"🛑 포장 구역 도착. State 4(패킹대기)로 전환.")
                self.state = 4
                
            # --- State 4: 패킹 대기중 ---
            elif self.state == 4:
                print(f"[{self.robot_id}] State 4: 패킹 작업 대기 요청...")
                self.send_request(f"PACKING_START|{self.robot_id}")
                
                # 완료 신호 대기
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.2)
                    self.battery -= (0.2 / 60) # 대기 중 소모
                    if self.server_cmd and self.server_cmd[1] == "PACKING_DONE":
                        self.server_cmd = None
                        print("✅ 패킹 완료 신호 수신! State 5(복귀)로 전환.")
                        self.state = 5
                        break
                    time.sleep(0.5)

            # --- State 5: 복귀중 (충전소 이동) ---
            elif self.state == 5:
                print(f"[{self.robot_id}] State 5: 충전소 슬롯 요청 중...")
                
                assigned_slot = None
                while rclpy.ok():
                    self.send_request(f"CHARGING_REQ|{self.robot_id}")
                    rclpy.spin_once(self, timeout_sec=0.2)
                    if self.server_cmd and self.server_cmd[1] == "CHARGING_ASSIGN":
                        assigned_slot = int(self.server_cmd[2])
                        self.server_cmd = None
                        break
                    time.sleep(1.0)
                
                print(f">>> {assigned_slot}번 충전소로 이동")
                # 라인트레이싱으로 슬롯까지 이동
                self.follow_line_until_stop(stop_count_target=assigned_slot)
                
                print(f"🏁 충전소 도착. State 6(충전)로 전환.")
                self.state = 6

            # --- State 6: 충전중 (+ 자동정렬) ---
            elif self.state == 6:
                # 메인 루프가 빨리 돌면서 배터리는 timer에 의해 참
                # 여기서는 "MOVE_UP" 명령 감시 및 배터리 만충 체크 수행
                
                if self.battery >= 80.0:
                    print(f"🔋 배터리 충전 완료 ({self.battery:.1f}%). State 1(대기)로 전환.")
                    self.current_job = None # 초기화
                    self.state = 1
                    time.sleep(1.0)
                    continue
                
                # Move Up 명령 확인
                if self.server_cmd and self.server_cmd[1] == "MOVE_UP":
                    print(f"[{self.robot_id}] 📢 앞자리 비움 -> 1칸 전진!")
                    self.server_cmd = None
                    self.follow_line_until_stop(stop_count_target=1)
                    print("✅ 전진 완료. 계속 충전.")
                
                time.sleep(0.5) # CPU 대기

    # =========================================================
    # [Helper] Navigation & Logic
    # =========================================================
    def navigate_segment(self, start_node, target_node, yaw_deg, desc):
        start_pos = self.waypoints[start_node]
        target_pos = self.waypoints[target_node]
        
        route_wps = self.get_smart_route(start_pos, target_pos)
        
        # 중복 제거 (시작점, 도착점과 너무 가까운 경유지 제외)
        filtered_wps = []
        for wp in route_wps:
            c = self.waypoints[wp]
            d_start = math.sqrt((c[0]-start_pos[0])**2 + (c[1]-start_pos[1])**2)
            d_target = math.sqrt((c[0]-target_pos[0])**2 + (c[1]-target_pos[1])**2)
            if d_start > 0.05 and d_target > 0.05:
                filtered_wps.append(wp)

        full_route = filtered_wps + [target_node]
        
        curr_node_name = start_node
        curr_pos = start_pos

        for next_wp_name in full_route:
            next_pos = self.waypoints[next_wp_name]
            
            # 관제 요청 (Blocking)
            self.request_access(curr_node_name, next_wp_name)
            
            # 이동 (회전 + 직진)
            dx, dy = next_pos[0] - curr_pos[0], next_pos[1] - curr_pos[1]
            dist = math.sqrt(dx**2 + dy**2)
            
            # 각도 계산
            if next_wp_name == target_node and dist < 0.1:
                req_yaw = math.radians(yaw_deg)
            else:
                req_yaw = math.atan2(dy, dx)

            self.rotate_precision(req_yaw)
            
            pose = self.create_pose(next_pos[0], next_pos[1], req_yaw)
            self.navigator.goToPose(pose)
            
            while not self.navigator.isTaskComplete(): 
                pass # 배터리 타이머는 백그라운드에서 동작함

            if self.navigator.getResult() == TaskResult.SUCCEEDED:
                # 반납
                self.release_access(curr_node_name, next_wp_name)
                curr_node_name = next_wp_name
                curr_pos = next_pos
            else:
                print("❌ 이동 실패")
                return
            
    def request_access(self, curr, next_n):
        req_str = f"REQUEST|{self.robot_id}|{curr}|{next_n}"
        print(f"🚦 진입 요청: {curr}->{next_n}")
        while rclpy.ok():
            self.send_request(req_str)
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_traffic_status == "APPROVED":
                self.latest_traffic_status = None
                print(f"🚀 승인됨!")
                return
            time.sleep(1.0)

    def release_access(self, prev, curr):
        self.send_request(f"RELEASE|{self.robot_id}|{prev}|{curr}")

    # =========================================================
    # [Helper] Line Tracing
    # =========================================================
    def follow_line_until_stop(self, stop_count_target=1):
        detected = 0
        cmd = Twist()
        on_line = False
        rate = self.create_rate(30)
        
        while rclpy.ok():
            err = self.get_line_error()
            
            if err == "STOP":
                if not on_line:
                    detected += 1
                    on_line = True
                    print(f"🛑 정지선 감지 ({detected}/{stop_count_target})")
                    if detected >= stop_count_target:
                        self.cmd_pub.publish(Twist())
                        break
            elif err == "LOST":
                self.cmd_pub.publish(Twist()) # 정지
                on_line = False
            else:
                on_line = False
                # PD Control
                angular = err * self.KP
                angular = max(min(angular, 1.0), -1.0)
                cmd.linear.x = self.BASE_SPEED_MPS
                cmd.angular.z = angular
                self.cmd_pub.publish(cmd)
            
            rate.sleep()
        time.sleep(0.5)

    def get_line_error(self):
        if not self.ir_sensor: return 0.0
        vals = self.ir_sensor.read_ir()
        if not vals: return None
        l, c, r = vals
        if l > 3500 and c > 3500 and r > 3500: return "STOP"
        if max(vals) < 500: return "LOST"
        return float(r - l)

    # =========================================================
    # [Helper] Utils & Graph
    # =========================================================
    def initialize_pose(self):
        # 로봇 ID별 초기 위치 (충전소)
        sx, sy = 0.205, -0.36
        if self.robot_id == "robot2": sx, sy = -0.015, -0.36
        elif self.robot_id == "robot3": sx, sy = -0.205, -0.36
        
        pose = self.create_pose(sx, sy, 0.0)
        self.navigator.setInitialPose(pose)
        
        self.get_logger().info("⏳ TF 대기 중...")
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                self.get_logger().info("✅ TF 연결 완료!")
                return
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(1.0)
                rclpy.spin_once(self, timeout_sec=0.1)

    def get_current_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except:
            return (0.0, 0.0)

    def create_pose(self, x, y, yaw_rad):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        return pose

    def rotate_precision(self, target_yaw_rad):
        self.navigator.clearAllCostmaps() 
        cmd = Twist()
        while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                q = trans.transform.rotation
                _, _, current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            except:
                time.sleep(0.1); continue

            error = target_yaw_rad - current_yaw
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < 0.017: # 1도
                self.cmd_pub.publish(Twist()) 
                break

            angular_vel = 2.0 * error
            angular_vel = max(min(angular_vel, 0.85), -0.85)
            if abs(angular_vel) < 0.2: angular_vel = 0.2 * np.sign(angular_vel)

            cmd.angular.z = angular_vel
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
        time.sleep(0.5)

    # --- Graph Search Helpers ---
    def get_candidate_wps(self, pos, tolerance=0.1):
        distances = []
        for name, coords in self.waypoints.items():
            dist = math.sqrt((pos[0]-coords[0])**2 + (pos[1]-coords[1])**2)
            distances.append((dist, name))
        min_dist = min(distances, key=lambda x: x[0])[0]
        return [name for dist, name in distances if dist <= (min_dist + tolerance)]

    def count_turns(self, path, start_pos, end_pos):
        raw_path = [start_pos] + [self.waypoints[wp] for wp in path] + [end_pos]
        clean_path = []
        if raw_path:
            clean_path.append(raw_path[0])
            for i in range(1, len(raw_path)):
                dist = math.sqrt((raw_path[i][0]-clean_path[-1][0])**2 + (raw_path[i][1]-clean_path[-1][1])**2)
                if dist > 0.01: clean_path.append(raw_path[i])
        if len(clean_path) < 3: return 0
        turns = 0
        for i in range(len(clean_path) - 2):
            ang1 = math.atan2(clean_path[i+1][1]-clean_path[i][1], clean_path[i+1][0]-clean_path[i][0])
            ang2 = math.atan2(clean_path[i+2][1]-clean_path[i+1][1], clean_path[i+2][0]-clean_path[i+1][0])
            if abs((ang2 - ang1 + math.pi) % (2 * math.pi) - math.pi) > 0.17: turns += 1
        return turns

    def get_first_straight_distance(self, path, start_pos, end_pos):
        raw_path = [start_pos] + [self.waypoints[wp] for wp in path] + [end_pos]
        clean_path = []
        if raw_path:
            clean_path.append(raw_path[0])
            for i in range(1, len(raw_path)):
                if math.sqrt((raw_path[i][0]-clean_path[-1][0])**2 + (raw_path[i][1]-clean_path[-1][1])**2) > 0.01:
                    clean_path.append(raw_path[i])
        if len(clean_path) < 2: return 0.0
        first_dist = math.sqrt((clean_path[1][0]-clean_path[0][0])**2 + (clean_path[1][1]-clean_path[0][1])**2)
        base_angle = math.atan2(clean_path[1][1]-clean_path[0][1], clean_path[1][0]-clean_path[0][0])
        for i in range(1, len(clean_path) - 1):
            dx, dy = clean_path[i+1][0]-clean_path[i][0], clean_path[i+1][1]-clean_path[i][1]
            if abs((math.atan2(dy, dx) - base_angle + math.pi) % (2 * math.pi) - math.pi) < 0.17:
                first_dist += math.sqrt(dx**2 + dy**2)
            else: break
        return first_dist

    def get_axis_preference(self, path, start_pos, end_pos):
        raw_path = [start_pos] + [self.waypoints[wp] for wp in path] + [end_pos]
        if len(raw_path) < 2: return 0
        dx = abs(raw_path[1][0] - raw_path[0][0])
        dy = abs(raw_path[1][1] - raw_path[0][1])
        return 0 if dx >= dy else 1

    def get_smart_route(self, start_pos, end_pos):
        start_cands = self.get_candidate_wps(start_pos)
        end_cands = self.get_candidate_wps(end_pos)
        routes = []
        for s in start_cands:
            for e in end_cands:
                if s == e: routes.append([s]); continue
                try: routes.extend(list(nx.all_shortest_paths(self.graph, s, e)))
                except: continue
        if not routes: return []
        return min(routes, key=lambda p: (
            len(p),
            self.count_turns(p, start_pos, end_pos),
            -self.get_first_straight_distance(p, start_pos, end_pos),
            self.get_axis_preference(p, start_pos, end_pos)
        ))

def main():
    rclpy.init()
    if len(sys.argv) > 1: my_id = sys.argv[1]
    else: my_id = "robot1"
    
    node = PinkyStateMachine(my_id)
    node.run_state_machine() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()