cmake_minimum_required(VERSION 3.8)
project(dog_lifecycle)

# colcon uses per-package build directories, so export compile commands here.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands for IDE tooling" FORCE)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dog_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/lifecycle_node.cpp
  src/yaml_state_store.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  dog_interfaces
)
target_link_libraries(${PROJECT_NAME}_lib yaml-cpp)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lifecycle_node test/test_lifecycle_node.cpp)
  if(TARGET test_lifecycle_node)
    target_link_libraries(test_lifecycle_node ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_lifecycle_node rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs dog_interfaces)
  endif()

  ament_add_gtest(test_yaml_state_store test/test_yaml_state_store.cpp)
  if(TARGET test_yaml_state_store)
    target_link_libraries(test_yaml_state_store ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_yaml_state_store rclcpp)
  endif()
endif()

ament_package()