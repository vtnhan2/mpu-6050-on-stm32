cmake_minimum_required(VERSION 3.8)
project(imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executable for C++ node
add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install executables
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/imu_python_node.py
  scripts/demo_imu.py
  scripts/validate_imu_message.py
  scripts/test_imu_compliance.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY
  imu
  DESTINATION lib/python3.8/site-packages/
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Install documentation
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu
    test/test_imu.cpp
  )
  target_link_libraries(test_imu
    imu_node
  )
endif()

ament_package()
