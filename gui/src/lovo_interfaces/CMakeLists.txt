cmake_minimum_required(VERSION 3.8)
project(lovo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) # 메시지 생성을 위해 필수

# 2. 메시지 파일 등록
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LovoStatus.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 3. 패키지 정보 내보내기
ament_package()