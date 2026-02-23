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

try:
    from pinkylib import IR
except ImportError as e:
    print(f"🚨 IR 센서 모듈 임포트 실패: {e}")
    sys.exit(1)

class PinkyStateMachine(Node):
    def __init__(self, robot_id):
        super().__init__(f'pinky_machine_{robot_id}')
        self.robot_id = robot_id
        
        self.state = 6       
        self.battery = 80.0  
        self.current_job = None 
        self.current_slot_num = 0 
        
        self.stop_spike_count = 0 
        
        self.navigator = BasicNavigator()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.traffic_pub = self.create_publisher(String, '/traffic/request', 10)
        self.traffic_sub = self.create_subscription(String, '/traffic/response', self.traffic_callback, 10)
        
        self.server_cmd = None 

        self.create_timer(3.0, self.battery_callback)

        try:
            self.ir_sensor = IR()
            self.get_logger().info("✅ IR Sensor Initialized.")
        except Exception:
            self.ir_sensor = None
            
        self.STOP_LINE_VAL = 3900  
        self.BASE_SPEED_MPS = 0.06  
        self.KP = 0.042

        self.waypoints = {
            'WP1': (0.435, -0.36), 'WP2': (0.435, 0.01), 'WP3': (0.435, 0.38),
            'WP4': (0.88, -0.36), 'WP5': (0.88, 0.01), 'WP6': (0.88, 0.38),
            'WP7': (1.325, -0.36), 'WP8': (1.325, 0.01), 'WP9': (1.325, 0.38),
            'WP10': (0.205, -0.36), 'WP11': (-0.015, -0.36), 'WP12': (-0.235, -0.36)
        }
        self.graph = nx.Graph()
        self.graph.add_edge('WP1', 'WP2'); self.graph.add_edge('WP2', 'WP3')
        self.graph.add_edge('WP4', 'WP5'); self.graph.add_edge('WP5', 'WP6')
        self.graph.add_edge('WP7', 'WP8'); self.graph.add_edge('WP8', 'WP9')
        self.graph.add_edge('WP1', 'WP4'); self.graph.add_edge('WP4', 'WP7')
        self.graph.add_edge('WP2', 'WP5'); self.graph.add_edge('WP5', 'WP8')
        self.graph.add_edge('WP3', 'WP6'); self.graph.add_edge('WP6', 'WP9')
        self.graph.add_edge('WP12', 'WP11'); self.graph.add_edge('WP11', 'WP10'); self.graph.add_edge('WP10', 'WP1')

    def battery_callback(self):
        if self.state == 6:
            self.battery += 5.0
            if self.battery > 100.0: self.battery = 100.0
        else:
            self.battery -= 2.0
            if self.battery < 0.0: self.battery = 0.0

    def traffic_callback(self, msg):
        try:
            parts = msg.data.split('|')
            if parts[0] == self.robot_id:
                self.server_cmd = parts 
        except IndexError:
            pass

    def reliable_request(self, req_str, expected_ack_types, timeout_sec=1.5):
        if isinstance(expected_ack_types, str):
            expected_ack_types = [expected_ack_types]
        msg = String()
        msg.data = req_str
        while rclpy.ok():
            self.traffic_pub.publish(msg)
            steps = int(timeout_sec / 0.1)
            for _ in range(steps):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.server_cmd and self.server_cmd[1] in expected_ack_types:
                    res = self.server_cmd
                    self.server_cmd = None
                    return res

    def report_state(self, state_num):
        self.state = state_num
        self.reliable_request(f"REPORT_STATE|{self.robot_id}|{state_num}", "REPORT_ACK")
        print(f"✅ State {state_num} 전환 보고 완료")

    def run_state_machine(self):
        self.navigator.waitUntilNav2Active()
        self.initialize_pose()
        
        self.report_state(6)
        if self.robot_id == "robot1": self.current_slot_num = 3 
        if self.robot_id == "robot2": self.current_slot_num = 2
        if self.robot_id == "robot3": self.current_slot_num = 1 
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.state == 1:
                # [해결 3] 히스테리시스 적용: 60.0% 밑으로 떨어져야만 다시 충전하러 감 (핑퐁 방지)
                if self.battery < 60.0:
                    print("⚠️ 배터리 부족. 충전 모드 복귀.")
                    self.report_state(6)
                    continue
                
                if self.current_slot_num != 3:
                    self.report_state(6) 
                    continue
                
                if self.current_job is None:
                    res = self.reliable_request(f"REQ_TASK|{self.robot_id}", "DISPATCH")
                    self.current_job = res[2]
                    print(f"✅ 작업 할당됨! 픽업지: {self.current_job}")
                    # 여기서 current_slot_num을 0으로 덮어쓰지 않고 살려둡니다! (물리적 이탈 판정을 위해)
                    self.report_state(2)
            
            elif self.state == 2:
                print(f"[{self.robot_id}] State 2: 운송 시작")
                cands = self.get_candidate_wps(self.get_current_xy())
                curr_node = cands[0] if cands else "Start_Point"
                
                pickup_pos = self.waypoints[self.current_job]
                arm_target_pos = self.waypoints['WP5'] 
                dx = arm_target_pos[0] - pickup_pos[0]
                dy = arm_target_pos[1] - pickup_pos[1]
                target_yaw_deg = math.degrees(math.atan2(dy, dx))
                
                # is_leaving_slot=True 옵션을 켜서, 첫 웨이포인트(안전 구역) 도달 시 슬롯을 반납하게 함
                self.navigate_segment(curr_node, self.current_job, target_yaw_deg, is_leaving_slot=True)
                print("픽업존 도착")
                time.sleep(3.0)
                
                self.navigate_segment(self.current_job, 'WP3', 180.0) # 하차지로 이동할 때는 False
                self.report_state(3)
                
            elif self.state == 3:
                print(f"[{self.robot_id}] 패킹 구역 확인...")
                while rclpy.ok():
                    res = self.reliable_request(f"CHECK_PACKING|{self.robot_id}", ["PACKING_AREA_FREE", "PACKING_AREA_BUSY"])
                    if res[1] == "PACKING_AREA_FREE": break
                    time.sleep(1.0)
                
                print(f"[{self.robot_id}] 라인 주행 시작")
                self.follow_line_until_stop(stop_count_target=1)
                self.report_state(4)
                
            elif self.state == 4:
                self.reliable_request(f"PACKING_START|{self.robot_id}", "PACKING_DONE", timeout_sec=2.0)
                self.report_state(5)

            elif self.state == 5:
                res = self.reliable_request(f"CHARGING_REQ|{self.robot_id}", "CHARGING_ASSIGN")
                assigned_slot = int(res[2])
                print(f">>> {assigned_slot}번 충전소로 이동")
                
                self.current_slot_num = assigned_slot
                self.follow_line_until_stop(stop_count_target=assigned_slot)
                self.report_state(6)

            elif self.state == 6:
                # 충전 완료 목표치는 80.0 유지
                if self.battery >= 80.0 and self.current_slot_num == 3:
                    self.report_state(1)
                    time.sleep(1.0)
                    continue
                
                if self.current_slot_num < 3:
                    res = self.reliable_request(f"CHECK_ADVANCE|{self.robot_id}|{self.current_slot_num}", ["ADVANCE_OK", "STAY"], timeout_sec=1.5)
                    if res[1] == "ADVANCE_OK":
                        print(f"[{self.robot_id}] 📢 전진 승인 획득!")
                        self.follow_line_until_stop(stop_count_target=1)
                        self.current_slot_num = int(res[2])
                        print(f"✅ 전진 완료. (현재: {self.current_slot_num}번)")
                    else:
                        time.sleep(1.0) 
                else:
                    time.sleep(1.0)

    # =========================================================
    # Navigation & Traffic
    # =========================================================
    # [수정] is_leaving_slot 매개변수 추가 (기본값 False)
    def navigate_segment(self, start_node, target_node, yaw_deg, is_leaving_slot=False):
        start_pos = self.waypoints[start_node]
        target_pos = self.waypoints[target_node]
        route_wps = self.get_smart_route(start_pos, target_pos)
        filtered_wps = []
        for wp in route_wps:
            c = self.waypoints[wp]
            d_start = math.sqrt((c[0]-start_pos[0])**2 + (c[1]-start_pos[1])**2)
            d_target = math.sqrt((c[0]-target_pos[0])**2 + (c[1]-target_pos[1])**2)
            if d_start > 0.05 and d_target > 0.05: filtered_wps.append(wp)
        full_route = filtered_wps + [target_node]
        curr_node_name = start_node
        curr_pos = start_pos
        
        for i, next_wp_name in enumerate(full_route):
            next_pos = self.waypoints[next_wp_name]
            self.request_access(curr_node_name, next_wp_name)
            dx, dy = next_pos[0] - curr_pos[0], next_pos[1] - curr_pos[1]
            dist = math.sqrt(dx**2 + dy**2)
            if next_wp_name == target_node and dist < 0.1: req_yaw = math.radians(yaw_deg)
            else: req_yaw = math.atan2(dy, dx)
            self.rotate_precision(req_yaw)
            pose = self.create_pose(next_pos[0], next_pos[1], req_yaw)
            self.navigator.goToPose(pose)
            
            while not self.navigator.isTaskComplete(): 
                rclpy.spin_once(self, timeout_sec=0.01)
                
            if self.navigator.getResult() == TaskResult.SUCCEEDED:
                self.release_access(curr_node_name, next_wp_name)
                
                # [해결 2] 첫 번째 웨이포인트(충전소 바깥의 안전지대)에 도착했을 때만 서버에 반납 통보!
                if is_leaving_slot and i == 0 and self.current_slot_num != 0:
                    print(f"[{self.robot_id}] 첫 안전 웨이포인트({next_wp_name}) 도달! 슬롯 {self.current_slot_num}번 확실히 반납.")
                    self.reliable_request(f"RELEASE_SLOT|{self.robot_id}|{self.current_slot_num}", "RELEASE_SLOT_ACK")
                    self.current_slot_num = 0 # 이제 진짜로 내 슬롯 번호를 0으로 초기화
                    
                curr_node_name = next_wp_name
                curr_pos = next_pos
            else: return
            
    def request_access(self, curr, next_n):
        while rclpy.ok():
            res = self.reliable_request(f"REQUEST|{self.robot_id}|{curr}|{next_n}", ["APPROVED", "WAIT"], timeout_sec=1.0)
            if res[1] == "APPROVED": return
            time.sleep(0.5)

    def release_access(self, prev, curr):
        self.reliable_request(f"RELEASE|{self.robot_id}|{prev}|{curr}", "RELEASE_ACK")

    # =========================================================
    # 스마트 라인 트레이싱
    # =========================================================
    def get_line_error(self):
        if not self.ir_sensor: return "LOST"
        vals = self.ir_sensor.read_ir()
        if not vals: return "LOST"
        
        l, c, r = vals
        
        if l > self.STOP_LINE_VAL and c > self.STOP_LINE_VAL and r > self.STOP_LINE_VAL: 
            self.stop_spike_count += 1
            if self.stop_spike_count >= 5:
                return "STOP"
            else:
                return 0.0 
        else:
            self.stop_spike_count = 0 

        if max(vals) < 800: 
            self.stop_spike_count = 0
            return "LOST"
            
        return float(l - r)

    def follow_line_until_stop(self, stop_count_target=1):
        detected = 0
        cmd = Twist()
        
        on_stop_line = True 
        has_cleared_initial_stop = False 
        white_line_count = 0 
        
        print(f"🏁 라인 주행 시작 (목표 정지선: {stop_count_target}개)")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.03)
            err = self.get_line_error()
            
            if err == "STOP":
                white_line_count = 0 
                if has_cleared_initial_stop:
                    if not on_stop_line:
                        detected += 1
                        on_stop_line = True
                        print(f"🛑 정지선 감지! ({detected}/{stop_count_target})")
                        if detected >= stop_count_target:
                            self.cmd_pub.publish(Twist())
                            break
                
                cmd.linear.x = self.BASE_SPEED_MPS
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                
            elif err == "LOST":
                self.cmd_pub.publish(Twist()) 
            
            else:
                white_line_count += 1
                if white_line_count > 20: 
                    on_stop_line = False 
                    has_cleared_initial_stop = True 
                    
                abs_err = abs(err)
                
                angular = (err * self.KP)
                angular = max(min(angular, 1.0), -1.0)
                
                speed_drop = (abs_err / 2500.0) * (self.BASE_SPEED_MPS * 1.0)
                cmd.linear.x = max(self.BASE_SPEED_MPS - speed_drop, 0.01)
                cmd.angular.z = angular
                    
                self.cmd_pub.publish(cmd)
                
        time.sleep(0.5)

    # =========================================================
    # Utils
    # =========================================================
    def initialize_pose(self):
        sx, sy = 0.205, -0.36
        if self.robot_id == "robot2": sx, sy = -0.015, -0.36
        elif self.robot_id == "robot3": sx, sy = -0.235, -0.36
        pose = self.create_pose(sx, sy, 0.0)
        self.navigator.setInitialPose(pose)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                return
            except:
                time.sleep(0.5)

    def get_current_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except: return (0.0, 0.0)

    def create_pose(self, x, y, yaw_rad):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        return pose

    def rotate_precision(self, target_yaw_rad):
        self.navigator.clearAllCostmaps() 
        cmd = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                q = t.transform.rotation
                _, _, cyaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            except: 
                time.sleep(0.05)
                continue
                
            err = target_yaw_rad - cyaw
            err = math.atan2(math.sin(err), math.cos(err))
            if abs(err) < 0.017: self.cmd_pub.publish(Twist()); break
            vel = max(min(2.0 * err, 0.85), -0.85)
            if abs(vel) < 0.2: vel = 0.2 * np.sign(vel)
            cmd.angular.z = vel
            self.cmd_pub.publish(cmd)
        time.sleep(0.5)

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
    my_id = sys.argv[1] if len(sys.argv) > 1 else "robot1"
    node = PinkyStateMachine(my_id)
    node.run_state_machine() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()