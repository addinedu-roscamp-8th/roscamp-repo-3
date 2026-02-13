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

class PrecisionWaypointSystem(Node):
    def __init__(self, robot_id):
        # 노드 이름을 로봇 ID에 맞춰 유니크하게 생성
        super().__init__(f'precision_navigator_{robot_id}')
        self.robot_id = robot_id
        
        self.navigator = BasicNavigator()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------------------------------------
        # [NEW] 교통 관제 통신 설정
        # ---------------------------------------------------------
        self.traffic_pub = self.create_publisher(String, '/traffic/request', 10)
        self.traffic_sub = self.create_subscription(String, '/traffic/response', self.traffic_callback, 10)
        self.latest_traffic_status = None 

        # 1. 맵 웨이포인트
        self.waypoints = {
            'WP1': (0.435, -0.36), 'WP2': (0.435, 0.01), 'WP3': (0.435, 0.38),
            'WP4': (0.88, -0.36), 'WP5': (0.88, 0.01), 'WP6': (0.88, 0.38),
            'WP7': (1.325, -0.36), 'WP8': (1.325, 0.01), 'WP9': (1.325, 0.38)
        }
        
        # 2. 그래프 연결
        self.graph = nx.Graph()
        self.graph.add_edge('WP1', 'WP2'); self.graph.add_edge('WP2', 'WP3')
        self.graph.add_edge('WP4', 'WP5'); self.graph.add_edge('WP5', 'WP6')
        self.graph.add_edge('WP7', 'WP8'); self.graph.add_edge('WP8', 'WP9')
        self.graph.add_edge('WP1', 'WP4'); self.graph.add_edge('WP4', 'WP7')
        self.graph.add_edge('WP2', 'WP5'); self.graph.add_edge('WP5', 'WP8')
        self.graph.add_edge('WP3', 'WP6'); self.graph.add_edge('WP6', 'WP9')

    # ------------------------------------------------------------------
    # [NEW] 교통 신호 처리 함수들
    # ------------------------------------------------------------------
    def traffic_callback(self, msg):
        # msg format: "robot_id|STATUS|wp1|wp2"
        try:
            parts = msg.data.split('|')
            if parts[0] == self.robot_id:
                self.latest_traffic_status = parts[1] # APPROVED or WAIT
        except IndexError:
            pass

    def request_access(self, current_node, next_node):
        """교통 관제소에 진입 요청을 보내고 승인될 때까지 대기"""
        req_msg = String()
        # REQUEST|robot_id|current_node|next_node
        req_msg.data = f"REQUEST|{self.robot_id}|{current_node}|{next_node}"
        
        print(f"🚦 [{self.robot_id}] 진입 요청: {current_node} -> {next_node} ...")
        
        while rclpy.ok():
            # 1. 요청 전송
            self.traffic_pub.publish(req_msg)
            
            # 2. 응답 대기 (Spin)
            rclpy.spin_once(self, timeout_sec=0.2)
            
            if self.latest_traffic_status == "APPROVED":
                print(f"🚀 [{self.robot_id}] 승인됨! ({current_node}->{next_node})")
                self.latest_traffic_status = None 
                return True
            
            elif self.latest_traffic_status == "WAIT":
                print(f"zzz [{self.robot_id}] 대기 중... (구간 또는 노드 점유됨)")
                time.sleep(1.0) # 1초 뒤 재요청
            
            # 응답 없으면 계속 재요청

    def release_access(self, prev_node, current_node):
        """지나온 구간 반납"""
        rel_msg = String()
        # RELEASE|robot_id|prev_node|current_node
        # 여기서 prev_node는 방금 떠나온 곳, current_node는 지금 도착한 곳
        rel_msg.data = f"RELEASE|{self.robot_id}|{prev_node}|{current_node}"
        self.traffic_pub.publish(rel_msg)
        # print(f"🔓 [{self.robot_id}] 구간 해제 요청 보냄")

    # ------------------------------------------------------------------
    # 기존 Helper 함수들 (TF, 회전, 경로계획)
    # ------------------------------------------------------------------
    def wait_for_tf(self):
        self.get_logger().info(f"[{self.robot_id}] ⏳ TF 연결 대기 중...")
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                self.get_logger().info(f"[{self.robot_id}] ✅ TF 연결 완료!")
                return
            except (LookupException, ConnectivityException, ExtrapolationException):
                time.sleep(1.0)
                rclpy.spin_once(self, timeout_sec=0.1)

    def get_current_yaw(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            q = trans.transform.rotation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return yaw
        except Exception:
            return None

    def rotate_precision(self, target_yaw_rad):
        # self.get_logger().info(f"🔄 정밀 회전 시작...")
        self.navigator.clearAllCostmaps() 
        cmd = Twist()
        while rclpy.ok():
            current_yaw = self.get_current_yaw()
            if current_yaw is None:
                time.sleep(0.1); rclpy.spin_once(self, timeout_sec=0.1); continue

            error = target_yaw_rad - current_yaw
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < 0.017: # 1도 이내
                self.cmd_pub.publish(Twist()) 
                break

            kp = 2.0
            angular_vel = kp * error
            max_vel, min_vel = 0.85, 0.2
            
            if abs(angular_vel) > max_vel: angular_vel = max_vel * np.sign(angular_vel)
            elif abs(angular_vel) < min_vel: angular_vel = min_vel * np.sign(angular_vel)

            cmd.angular.z = angular_vel
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
        time.sleep(0.5)

    def get_candidate_wps(self, pos, tolerance=0.05):
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
                dist = math.sqrt((raw_path[i][0] - clean_path[-1][0])**2 + (raw_path[i][1] - clean_path[-1][1])**2)
                if dist > 0.01: clean_path.append(raw_path[i])
        if len(clean_path) < 3: return 0
        turns = 0
        for i in range(len(clean_path) - 2):
            ang1 = math.atan2(clean_path[i+1][1] - clean_path[i][1], clean_path[i+1][0] - clean_path[i][0])
            ang2 = math.atan2(clean_path[i+2][1] - clean_path[i+1][1], clean_path[i+2][0] - clean_path[i+1][0])
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

    def create_pose(self, x, y, yaw_rad):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        return pose

    # ------------------------------------------------------------------
    # [핵심] 주행 실행 (교통 관제 적용)
    # ------------------------------------------------------------------
    def navigate_segment(self, start_pos, target_pos, target_yaw_deg, description="이동"):
        print(f"\n--- [{self.robot_id}] {description} ---")
        route_wps = self.get_smart_route(start_pos, target_pos)
        
        filtered_wps = []
        for wp in route_wps:
            c = self.waypoints[wp]
            # 시작점/도착점과 너무 가까운(5센치 이내) 웨이포인트 제거
            if math.sqrt((c[0]-start_pos[0])**2 + (c[1]-start_pos[1])**2) > 0.05 and \
               math.sqrt((c[0]-target_pos[0])**2 + (c[1]-target_pos[1])**2) > 0.05:
                filtered_wps.append(wp)
                
        coords_list = [self.waypoints[wp] for wp in filtered_wps]
        coords_list.append(target_pos)
        wp_names = filtered_wps + ["최종목적지"]

        # 현재 위치 추적용 (초기값: 시작점)
        current_pose = start_pos
        
        # [중요] 현재 내가 있는 노드 이름 찾기 (가장 가까운 WP)
        # 출발할 때 이미 어떤 노드에 있다고 가정 (없으면 Start로)
        cands = self.get_candidate_wps(start_pos)
        current_node_name = cands[0] if cands else "Start_Point"

        for i in range(len(coords_list)):
            next_target = coords_list[i]
            target_name = wp_names[i]
            
            # --- 1. 다음 노드 결정 (목적지가 WP가 아니면 이름 생성) ---
            next_node_name = target_name
            if target_name == "최종목적지":
                 # 최종 목적지 좌표와 가장 가까운 WP 이름을 찾아서 락을 걸어야 함
                 # 만약 맵 밖의 좌표라면 임의의 유니크 이름 사용 필요
                 final_cands = self.get_candidate_wps(next_target)
                 next_node_name = final_cands[0] if final_cands else "Goal_Point"

            # --- 2. 교통 관제 승인 요청 (Blocking) ---
            # current_node_name -> next_node_name 이동 허가
            self.request_access(current_node_name, next_node_name)

            # --- 3. 정밀 회전 및 주행 ---
            dx, dy = next_target[0] - current_pose[0], next_target[1] - current_pose[1]
            dist = math.sqrt(dx**2 + dy**2)
            
            if i == len(coords_list) - 1 and dist < 0.05: required_yaw = math.radians(target_yaw_deg)
            else: required_yaw = math.atan2(dy, dx)

            self.rotate_precision(required_yaw)
            
            if i == len(coords_list) - 1: final_yaw = math.radians(target_yaw_deg)
            else: final_yaw = required_yaw
            
            pose = self.create_pose(next_target[0], next_target[1], final_yaw)
            # print(f"🚀 [Nav2] {target_name}로 이동 시작")
            self.navigator.goToPose(pose)
            while not self.navigator.isTaskComplete(): pass 
            
            if self.navigator.getResult() == TaskResult.SUCCEEDED:
                print(f"✅ {target_name} 도착")
                
                # --- 4. 구간 반납 (이전 노드 + 통로 해제) ---
                self.release_access(current_node_name, next_node_name)
                
                # 상태 업데이트
                current_pose = next_target
                current_node_name = next_node_name
            else:
                print("❌ 이동 실패")
                return target_pos
        return target_pos

    def run_full_mission(self, pickup_x, pickup_y, pickup_yaw):
        self.navigator.waitUntilNav2Active()
        
        # --- 0. 초기 위치 설정 (Nav2) ---
        # 주의: 로봇별로 시작 위치가 다를 수 있으므로 
        # 실제로는 외부에서 초기 위치를 주거나, 2D Pose Estimate로 잡는 것을 권장
        # 여기서는 하드코딩된 값을 사용하므로 겹치지 않게 주의
        
        # start_x, start_y = 0.265, -0.36 -> IR센서를 벽 라인에 맞췄을때 로봇 좌표
        
        # 로봇 ID별로 시작 위치를 다르게 주려면 아래처럼 분기 가능
        if self.robot_id == "robot1": start_x, start_y = 0.205, -0.36
        elif self.robot_id == "robot2": start_x, start_y = -0.015, -0.36
        elif self.robot_id == "robot3": start_x, start_y = -0.205, -0.36
            
        init_pose = self.create_pose(start_x, start_y, 0.0)
        self.navigator.setInitialPose(init_pose)
        
        self.wait_for_tf()
        
        # --- 미션 시작 ---
        curr = (start_x, start_y)
        
        # 1단계
        curr = self.navigate_segment(curr, (pickup_x, pickup_y), pickup_yaw, "1단계: 픽업 이동")
        print(f"[{self.robot_id}] >>> 🤖 작업 중...")
        time.sleep(2.0)
        
        # 2단계
        drop_pos = self.waypoints['WP3']
        self.navigate_segment(curr, drop_pos, 180.0, "2단계: 하차 이동")
        print(f"[{self.robot_id}] 🎉 미션 완료!")

def main():
    rclpy.init()
    
    # 터미널 인자로 로봇 ID 받기 (기본값: robot1)
    if len(sys.argv) > 1:
        my_robot_id = sys.argv[1]
    else:
        my_robot_id = "robot1"

    print(f"🤖 로봇 클라이언트 시작! ID: {my_robot_id}")
    node = PrecisionWaypointSystem(my_robot_id)
    
    # 로봇별 미션 할당 (예시)
    if my_robot_id == "robot1":
        node.run_full_mission(1.325, 0.01, 90.0) # WP8
    elif my_robot_id == "robot2":
        node.run_full_mission(0.88, -0.36, 90.0)  # WP4
    elif my_robot_id == "robot3":
        node.run_full_mission(0.88, 0.38, 90.0) # WP6
    else:
        # 기본 미션
        node.run_full_mission(1.325, 0.01, 90.0)

    rclpy.shutdown()

if __name__ == '__main__':
    main()