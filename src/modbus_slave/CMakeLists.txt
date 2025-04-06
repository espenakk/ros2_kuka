cmake_minimum_required(VERSION 3.8)
project(modbus_slave)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_subdirectory(lib/modbus_tcp_cpp)

add_executable(modbus_slave src/main.cpp src/Slave.cpp)
target_include_directories(modbus_slave PRIVATE lib/modbus_tcp_cpp/include
	include)

find_package(rclcpp REQUIRED)
ament_target_dependencies(modbus_slave rclcpp)
target_link_libraries(modbus_slave modbus_tcp_cpp)

install(TARGETS
	modbus_slave
	DESTINATION lib/${PROJECT_NAME})

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
