## 탑뷰 ai server 테스트

### 실행 명령어
- ENABLE_MAP_BASE_COORDS=1 python3 main.py

### 프로세스
- startup_event에서 서버 시작 시 스레드들을 띄움
- 거기서 ENABLE_MAP_BASE_COORDS 값을 확인해서 map_base_coords_task() 스레드를 추가로 시작함
- map_base_coords_task() 안에서 run_map_base_coords(...)를 실행함