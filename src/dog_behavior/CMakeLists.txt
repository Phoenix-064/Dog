cmake_minimum_required(VERSION 3.8)
project(dog_behavior)

# colcon uses per-package build directories, so export compile commands here
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands for IDE tooling" FORCE)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dog_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/behavior_node.cpp
  src/behavior_tree.cpp
  src/behavior_tree_node.cpp
  src/bt_nodes/check_system_mode.cpp
  src/bt_nodes/wait_for_pose_condition.cpp
  src/bt_nodes/select_waypoint_action.cpp
  src/bt_nodes/execute_behavior_action.cpp
  src/bt_nodes/navigate_to_pose_action.cpp
  src/navigation_executor_node.cpp
  src/common/payload_utils.cpp
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
  nav2_msgs
  geometry_msgs
  std_msgs
  dog_interfaces
)
target_link_libraries(${PROJECT_NAME}_lib yaml-cpp)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME}_bt_node src/behavior_tree_main.cpp)
target_link_libraries(${PROJECT_NAME}_bt_node ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME}_navigation_executor_node src/navigation_executor_main.cpp)
target_link_libraries(${PROJECT_NAME}_navigation_executor_node ${PROJECT_NAME}_lib)

install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_bt_node
  ${PROJECT_NAME}_navigation_executor_node
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
      nav2_msgs
      std_msgs
      dog_interfaces
    )
  endif()

  ament_add_gtest(test_navigation_executor_node test/test_navigation_executor_node.cpp)
  if(TARGET test_navigation_executor_node)
    target_link_libraries(test_navigation_executor_node ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_navigation_executor_node
      rclcpp
      rclcpp_action
      nav2_msgs
      std_msgs
    )
  endif()

  ament_add_gtest(test_payload_utils test/test_payload_utils.cpp)
  if(TARGET test_payload_utils)
    target_link_libraries(test_payload_utils ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_payload_utils
      geometry_msgs
    )
  endif()

  ament_add_gtest(test_behavior_tree_node test/test_behavior_tree_node.cpp)
  if(TARGET test_behavior_tree_node)
    target_compile_definitions(test_behavior_tree_node PRIVATE
      DOG_BEHAVIOR_TEST_BT_MAIN_XML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/behavior_tree_test.xml"
    )
    target_link_libraries(test_behavior_tree_node ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_behavior_tree_node
      rclcpp
      nav_msgs
      std_msgs
      geometry_msgs
    )
  endif()

  ament_add_gtest(test_check_system_mode test/test_check_system_mode.cpp)
  if(TARGET test_check_system_mode)
    target_link_libraries(test_check_system_mode ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_check_system_mode
      behaviortree_cpp_v3
    )
  endif()

  ament_add_gtest(test_wait_for_pose_condition test/test_wait_for_pose_condition.cpp)
  if(TARGET test_wait_for_pose_condition)
    target_link_libraries(test_wait_for_pose_condition ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_wait_for_pose_condition
      behaviortree_cpp_v3
    )
  endif()

  ament_add_gtest(test_select_waypoint_action test/test_select_waypoint_action.cpp)
  if(TARGET test_select_waypoint_action)
    target_link_libraries(test_select_waypoint_action ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_select_waypoint_action
      behaviortree_cpp_v3
      geometry_msgs
    )
  endif()

  ament_add_gtest(test_execute_behavior_action test/test_execute_behavior_action.cpp)
  if(TARGET test_execute_behavior_action)
    target_link_libraries(test_execute_behavior_action ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_execute_behavior_action
      rclcpp
      rclcpp_action
      behaviortree_cpp_v3
      dog_interfaces
      geometry_msgs
    )
  endif()

  ament_add_gtest(test_navigate_to_pose_action test/test_navigate_to_pose_action.cpp)
  if(TARGET test_navigate_to_pose_action)
    target_link_libraries(test_navigate_to_pose_action ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_navigate_to_pose_action
      rclcpp
      rclcpp_action
      behaviortree_cpp_v3
      nav2_msgs
      geometry_msgs
      std_msgs
    )
  endif()
endif()

ament_package()