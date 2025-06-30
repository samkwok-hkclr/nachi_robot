cmake_minimum_required(VERSION 3.8)
project(nachi_robot)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(std_srvs REQUIRED)

include_directories(
  include 
)

add_executable(nachi_robot_node 
  src/nachi_robot_node.cpp
  src/nachi_driver.cpp
  src/nachi_fake_driver.cpp
)
target_include_directories(nachi_robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)
ament_target_dependencies(nachi_robot_node 
  rclcpp 
  rclcpp_action
  std_msgs
  sensor_msgs
  control_msgs
  std_srvs
)
target_compile_features(nachi_robot_node PUBLIC c_std_99 cxx_std_14)  # Require C99 and C++14

ament_export_libraries(${PROJECT_NAME})

target_link_libraries(nachi_robot_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libOpennrif.so
)

install(TARGETS 
  nachi_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install nachi libraries
install(
  PROGRAMS
  lib/libOpennrif.so
  license/license.dat
  license/UserApp_protocol_verification.xsd
  DESTINATION lib
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
