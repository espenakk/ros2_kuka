cmake_minimum_required(VERSION 3.8)
project(kuka_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(ball_trajectory_prediction_node src/ball_trajectory_prediction_node.cpp src/kalman_filter.cpp)
ament_target_dependencies(ball_trajectory_prediction_node rclcpp geometry_msgs visualization_msgs Eigen3)

add_executable(test_ball_publisher_node src/test_ball_publisher_node.cpp)
ament_target_dependencies(test_ball_publisher_node rclcpp geometry_msgs)

target_include_directories(ball_trajectory_prediction_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(ball_trajectory_prediction_node PUBLIC c_std_99 cxx_std_17)

target_include_directories(test_ball_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_ball_publisher_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS 
  ball_trajectory_prediction_node
  test_ball_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()