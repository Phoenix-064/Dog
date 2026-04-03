cmake_minimum_required(VERSION 3.8)
project(dog_perception)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(dog_interfaces REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/perception_node.cpp
  src/target_3d_solver_factory.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
  message_filters
  geometry_msgs
  tf2
  tf2_ros
  ament_index_cpp
  dog_interfaces
)
target_link_libraries(${PROJECT_NAME}_lib yaml-cpp Boost::boost)

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

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(tf2_msgs REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_node test/test_perception_node.cpp)
  if(TARGET test_perception_node)
    target_link_libraries(test_perception_node ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_perception_node rclcpp tf2_msgs)
  endif()
endif()

ament_package()