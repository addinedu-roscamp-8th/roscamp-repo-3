cmake_minimum_required(VERSION 3.8)
project(jetcobot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(lovo_interfaces REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python executables using setup.py scripts
install(PROGRAMS
  scripts/jetcobot_pc_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jetcobot_pc_node.py
)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
