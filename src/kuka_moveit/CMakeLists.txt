cmake_minimum_required(VERSION 3.8)
project(kuka_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
add_executable(trajectory_planning_node
  src/trajectory_planning_node.cpp
  src/TrajectoryPlanningNode.cpp
)

ament_target_dependencies(trajectory_planning_node
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  rclcpp
)


add_executable(point_to_legal_pose_node
  src/point_to_legal_pose_node.cpp
  src/PointToLegalPoseNode.cpp
)

ament_target_dependencies(point_to_legal_pose_node
  moveit_ros_planning_interface
  rclcpp
  Eigen3
  geometry_msgs
  moveit_core
)


add_executable(plan_execution_node
  src/plan_execution_node.cpp
  src/PlanExecutionNode.cpp
)

ament_target_dependencies(plan_execution_node
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  rclcpp
)
   

install(TARGETS
  trajectory_planning_node
  point_to_legal_pose_node
  plan_execution_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	launch
  config
	DESTINATION share/${PROJECT_NAME}
)

target_include_directories(trajectory_planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(trajectory_planning_node PUBLIC c_std_99 cxx_std_17)

target_include_directories(trajectory_planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(trajectory_planning_node PUBLIC c_std_99 cxx_std_17)

target_include_directories(point_to_legal_pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(point_to_legal_pose_node PUBLIC c_std_99 cxx_std_17)

target_include_directories(point_to_legal_pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(point_to_legal_pose_node PUBLIC c_std_99 cxx_std_17)

target_include_directories(plan_execution_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(plan_execution_node PUBLIC c_std_99 cxx_std_17)

target_include_directories(plan_execution_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(plan_execution_node PUBLIC c_std_99 cxx_std_17)

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
