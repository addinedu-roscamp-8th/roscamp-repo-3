# 핑키 관제 서버 (Pinky Control Server) 🚀

이 프로젝트는 3대의 핑키 로봇(Domain 50, 51, 52)의 데이터를 관제 PC(Domain 59)로 통합하고, 터미널 대시보드를 통해 실시간으로 상태를 모니터링하는 시스템입니다.
지금 실제로 받아오는 값은 배터리 상태, 그리고 map기준 핑키의 좌표입니다. state라고 적혀있는건 저희가 앞으로 발행햐야할값입니다 
---

## 🏗️ 1. 전체 시스템 구조
- **로봇 본체**: 로컬 지도를 기반으로 자기 위치를 계산하고 데이터 발행.
- **관제 PC (브릿지 & 모니터)**: 로봇 도메인과 소통하며 데이터를 59번 도메인으로 통합하고, 실시간 대시보드 화면을 출력.

---

## 🚀 2. 실행 순서 (매우 중요!)

### **[Step 1] 관제 시스템 가동 (PC 측)**
터미널에서 아래 명령 하나로 모든 브릿지와 모니터링 대시보드를 실행합니다.
하나의 런치파일을 실행시키면 3개의 yaml파일이 작동됩니다
```bash
ros2 launch /home/addinedu/Desktop/roscamp-repo-3/server/pinky_control_server/domain_bridge.launch.py
```
> [!NOTE] 
> 이 명령어는 환경 변수(`ROS_DOMAIN_ID=59`) 설정, 도메인 브릿지 실행, 상태 모니터 실행을 모두 자동으로 처리합니다.

### **[Step 2] 로봇 가동 (로봇 본체 측)**
각 로봇에 SSH 접속하여 아래 명령어를 **순서대로** 실행하세요.

1. **하드웨어 가동:**
   ```bash
   ros2 launch pinky_bringup bringup_robot.launch.xml
   ```
2. **위치 추정 가동:** (설정은 `nav2_params.yaml` 참조)
   ```bash
   ros2 launch pinky_navigation localization_launch.xml map:=/home/pinky/pinky_pro/lovo_clean_map.yaml
   ```

### **[Step 3] 초기 위치 설정 (필수!)**
로봇이 지도 상에서 어디 있는지 알려줘야 위치 데이터가 발행됩니다.

1. 관제 PC에서 `RViz2`를 실행합니다.
2. 상단 **[2D Pose Estimate]** 버튼을 클릭합니다.
3. 지도 위에서 로봇의 현재 실제 위치를 클릭하고 방향에 맞춰 드래그하세요.
4. 좌표로 알려줘도 ok

---

## 📊 3. 상태 확인 및 모니터링

### **실시간 대시보드 종료**
- 실행 중인 터미널에서 `Ctrl + C`를 누르면 대시보드와 모든 브릿지 프로세스가 안전하게 한 번에 종료됩니다.

### **개별 데이터 확인**
특정 로봇의 데이터를 수동으로 확인하고 싶을 때 사용하세요.
```bash
export ROS_DOMAIN_ID=59
ros2 topic echo /pinky_2/pose
```


