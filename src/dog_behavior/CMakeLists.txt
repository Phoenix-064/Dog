cmake_minimum_required(VERSION 3.8)
project(dog_behavior)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dog_interfaces REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/behavior_node.cpp
  src/behavior_tree.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  ament_index_cpp
  behaviortree_cpp_v3
  nav_msgs
  geometry_msgs
  std_msgs
  dog_interfaces
)

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

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_behavior_tree test/test_behavior_tree.cpp)
  if(TARGET test_behavior_tree)
    target_compile_definitions(test_behavior_tree PRIVATE
      DOG_BEHAVIOR_TEST_BT_XML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/execute_trigger_tree.xml"
    )
    target_link_libraries(test_behavior_tree ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_behavior_tree
      rclcpp
      behaviortree_cpp_v3
    )
  endif()
  ament_add_gtest(test_behavior_node test/test_behavior_node.cpp)
  if(TARGET test_behavior_node)
    target_link_libraries(test_behavior_node ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_behavior_node
      rclcpp
      rclcpp_action
      geometry_msgs
      nav_msgs
      std_msgs
      dog_interfaces
    )
  endif()
endif()

ament_package()