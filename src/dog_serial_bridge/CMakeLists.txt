cmake_minimum_required(VERSION 3.8)
project(dog_serial_bridge)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands for IDE tooling" FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dog_interfaces REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/serial_bridge_node.cpp
  src/nav_telemetry_serial_node.cpp
  src/serial_protocol.cpp
  src/system_serial_connection.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  dog_interfaces
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME}_nav_telemetry_node src/nav_telemetry_main.cpp)
target_link_libraries(${PROJECT_NAME}_nav_telemetry_node ${PROJECT_NAME}_lib)

install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_nav_telemetry_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_ROS_LOG_DIR "${CMAKE_CURRENT_BINARY_DIR}/ros_log")
  file(MAKE_DIRECTORY "${TEST_ROS_LOG_DIR}")

  ament_add_gtest(test_serial_protocol test/test_serial_protocol.cpp)
  if(TARGET test_serial_protocol)
    target_link_libraries(test_serial_protocol ${PROJECT_NAME}_lib)
  endif()

  ament_add_gtest(test_serial_bridge_node test/test_serial_bridge_node.cpp)
  if(TARGET test_serial_bridge_node)
    target_link_libraries(test_serial_bridge_node ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_serial_bridge_node
      rclcpp
      rclcpp_action
      std_msgs
      geometry_msgs
      dog_interfaces
    )
    set_tests_properties(test_serial_bridge_node
      PROPERTIES ENVIRONMENT "ROS_LOG_DIR=${TEST_ROS_LOG_DIR}")
  endif()

  ament_add_gtest(test_nav_telemetry_serial_node test/test_nav_telemetry_serial_node.cpp)
  if(TARGET test_nav_telemetry_serial_node)
    target_link_libraries(test_nav_telemetry_serial_node ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_nav_telemetry_serial_node
      rclcpp
      geometry_msgs
    )
    set_tests_properties(test_nav_telemetry_serial_node
      PROPERTIES ENVIRONMENT "ROS_LOG_DIR=${TEST_ROS_LOG_DIR}")
  endif()
endif()

ament_package()
