cmake_minimum_required(VERSION 3.8)
project(dog_perception)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(DOG_PERCEPTION_PINNED_OPENCV_VERSION "4.5.4")
set(DOG_PERCEPTION_PINNED_OPENCV_CMAKE_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
set(OpenCV_DIR "${DOG_PERCEPTION_PINNED_OPENCV_CMAKE_DIR}" CACHE PATH "Pinned OpenCV CMake dir" FORCE)
find_package(OpenCV ${DOG_PERCEPTION_PINNED_OPENCV_VERSION} EXACT REQUIRED
  COMPONENTS core imgproc imgcodecs videoio dnn calib3d
  PATHS ${DOG_PERCEPTION_PINNED_OPENCV_CMAKE_DIR}
  NO_DEFAULT_PATH)

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

file(GLOB DOG_PERCEPTION_DETECTOR_SOURCES CONFIGURE_DEPENDS
  src/detectors/*.cpp
)

add_library(${PROJECT_NAME}_lib SHARED
  src/perception_node.cpp
  src/camera_publisher_node.cpp
  src/digit_recognizer_factory.cpp
  ${DOG_PERCEPTION_DETECTOR_SOURCES}
  src/target_3d_solver_factory.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_lib PRIVATE ${OpenCV_INCLUDE_DIRS})
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
target_link_libraries(
  ${PROJECT_NAME}_lib
  yaml-cpp
  Boost::boost
  ${OpenCV_LIBS}
)
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE DOG_WORKSPACE_ROOT="${CMAKE_SOURCE_DIR}")

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

add_executable(${PROJECT_NAME}_camera_node src/camera_main.cpp)
target_link_libraries(${PROJECT_NAME}_camera_node ${PROJECT_NAME}_lib)

install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_camera_node
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

  ament_add_gtest(test_digit_recognizer_factory test/test_digit_recognizer_factory.cpp)
  if(TARGET test_digit_recognizer_factory)
    target_link_libraries(test_digit_recognizer_factory ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_digit_recognizer_factory rclcpp)
  endif()

  ament_add_gtest(test_camera_publisher_node test/test_camera_publisher_node.cpp)
  if(TARGET test_camera_publisher_node)
    target_link_libraries(test_camera_publisher_node ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_camera_publisher_node rclcpp sensor_msgs)
  endif()

  ament_add_gtest(test_box_detector test/test_box_detector.cpp)
  if(TARGET test_box_detector)
    target_link_libraries(test_box_detector ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_box_detector rclcpp sensor_msgs)
  endif()
endif()

ament_package()