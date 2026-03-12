<br />
<p align="center">
  <a href="https://github.com/addinedu-roscamp-8th/roscamp-repo-3">
    <img src="https://github.com/addinedu-roscamp-8th/roscamp-repo-3/blob/12de46abfdf4ef5c27808b2bca33f56fa3812dcd/assets/LOVO%20logo.png" alt="Logo" width="1240px">
  </a>

  <h3 align="center">LOVO (Logistics Of Value and Order)</h3>
  <p align="center">
    <a href="https://www.canva.com/design/DAHCTh33jQ4/XuixnsayHb_guG8WM1gKiA/edit?utm_content=DAHCTh33jQ4&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton">Presentation</a>
  </p>
</p>
<hr>

> DIY 가구 자재 물류 자동화 서비스

---

## 프로젝트 개요

로봇 자율 작업 수행 및 중앙 관제 기반 통합 제어를 기반으로 제품 주문부터 출하까지 물류 공정의 전 과정을 실시간으로 모니터링 제어 및 최적화 하는 관리 시스템

- **프로젝트명**: LOVO (Logistics Of Value and Order)
- **주제**: DIY 가구 자재 물류 자동화 서비스
- **핵심 기술**: Linux, ROS2, Python, C++, Nav2, MoveIt2, YOLO, OpenCV, MySQL, FastAPI, React, PyQt

## 팀 구성 및 역할
|        | Name | Job |
|--------|------|-----|
| Leader | 강태용 |  Project Manage, Integrated Manipulator Control, GUI |   
| Worker | 김동영 |  Mobile Robot, Main Server |   
| Worker | 서태현 |  Manipulator, Motion Planning |    
| Worker | 박현서 |  Web, DB Manage, server Integration | 
| Worker | 이예지 |  AI Server | 

---

## 기술 스택 (Tech Stack)
| Category | Technology |
|----------|------------|
| Development Environment	| Linux(Ubuntu 24.04 LTS) |
| Middle Ware	| ROS2(Jazzy) |
| Language | Python, C++ |
| Framework |	Nav2, YOLO, OpenCV, PyQt, React, MySQL |
| Configuration Management | Github, Jira, Confluence, Slack |


---

## System Architecture
![Image](https://github.com/addinedu-roscamp-8th/roscamp-repo-3/blob/1f7d82c786ce4e3175324b4f4a0e90a2ba804f55/assets/sw%20Architecture.png)

<br >

## Data Structure
![Image](https://github.com/user-attachments/assets/9c227d2b-735e-447f-b443-378c764ac177)

<br >

## Interface Specification
![Image](https://github.com/user-attachments/assets/5980b5cf-c713-46f2-b932-a4a1e79b41f0)
![Image](https://github.com/user-attachments/assets/1ad8dcbe-bb60-4604-b8e8-d1728766a4c6)
![Image](https://github.com/user-attachments/assets/77c768c6-cb1d-4fd6-8569-78f131175441)
![Image](https://github.com/user-attachments/assets/86d3a779-1ed6-48c7-a054-6efef0a5ca3b)

<br >

## Sequence Diagram
![Image](https://github.com/user-attachments/assets/93c2a772-3cf0-4064-bf61-013953c38a1c)
![Image](https://github.com/user-attachments/assets/d79edd55-8f75-4c25-a874-2bc3c725babd)
![Image](https://github.com/user-attachments/assets/5cad0199-bd49-4cfc-ba54-cded051bed52)
![Image](https://github.com/user-attachments/assets/dd78f1ed-0975-4409-b1b6-00f302927d9a)
![Image](https://github.com/user-attachments/assets/eb78f408-9201-4f79-a414-2cef01e72076)

<br >

## Map
![Image](https://github.com/user-attachments/assets/70f208c6-8202-43f4-b70c-6ff377562111)

<br >

## 3D Modeling
![Image](https://github.com/user-attachments/assets/3137c0dc-7103-48db-94c8-5b9c5eff2d0a)

<br >

## Implements
### Scenario
![Image](https://github.com/user-attachments/assets/34e3c718-48e7-46f7-8f9f-0aa2894e73d4)
![Image](https://github.com/user-attachments/assets/36888708-8558-4cf7-ab7f-1323dec34a75)

## Visual Perception
### Fall Detection <a href="https://youtu.be/RGaHGk8g8CM?si=JeyQEyDHIPCZi-QU">Video Demo</a>
![Image](docs/global_camera_0_fall_detection.png)

<br >


### ArUco Marker Detection
![Image](docs/global_camera_1_aruco_detection.png)
![Image](docs/global_camera_2_aruco_result.png)
![Image](docs/global_camera_3_aruco_distribution_result.png)
<br >

## Mobile Robots
### Autonomous Driving Architecture
![Image](docs/mobile_robot_0_architecture.png) 
### Aruco Marker Odometry with Sensor Fusion
![Image](docs/mobile_robot_1_aruco_marker_odom.png)
![Image](docs/mobile_robot_2_aruco_marker_ekf.png)
![Image](docs/mobile_robot_3_aruco_marker_odom_result.png)

### Mobile Robot Control
![Image](docs/mobile_robot_4_precise_control.png)
![Image](docs/mobile_robot_4_result.png)

![Image](docs/mobile_robot_5_obstacle.png)
![Image](docs/mobile_robot_5_safety_control.png)

<br >

## Multi-Robot Control System
### Multi-Robot Coordination
![Image](docs/multiriobot_0.png)
![Image](docs/multiriobot_1.png)
![Image](docs/multiriobot_2.png)
![Image](docs/multiriobot_3.png)
<br >

## Manipulator
### TCP vs. UDP <a href="https://youtube.com/shorts/jgOKWCgC6g4?si=p_9TIRYr5YMuHrkH">Video Demo</a>
![Image](https://github.com/user-attachments/assets/e2ac3703-e394-4f92-ad52-86c038c69b65)
![Image](https://github.com/user-attachments/assets/1190fe8f-aede-45a5-b1e8-a516d5a99e50)

### 물품 탐색 및 분류 후 Pick & Place <a href="https://youtube.com/shorts/nGim0zZXDp8?si=m3Fh_BKGykL91u_g">Video Demo</a>
![Image](https://github.com/user-attachments/assets/a4ed023e-5298-4649-aafe-ac842888e21a)


<br >


## GUI
### Client GUI
![Image](docs/client_gui_0.png)

<br >

### Admin GUI
![Image](docs/admin_gui_0.png)
![Image](docs/admin_gui_1.png)
<br >

## AI 기반 요양 케어 자동화 시스템 <a href="https://youtu.be/ZZ3zroNBlqo?si=c7IFToJwzir5f4Py">Video Demo</a>

<br >

## Project Schedule
Project Period: 2025.06.23~2025.08.22
![Image](https://github.com/user-attachments/assets/64a657e1-f6d0-4e71-a621-926d4976c5ed)

<br >
