cmake_minimum_required(VERSION 3.28)
project(kuka_rsi_io)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(asio CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(pugixml CONFIG REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(kuka_rsi_io
  src/spinlock.cpp
  src/udpserver.cpp
  src/kukarsiinterface.cpp
  src/kukarsixmlserializer.cpp
  include/rsi/spinlock.h
  include/rsi/udpserver.h
  include/rsi/kukarsiinterface.h
  include/rsi/kukarsixmlserializer.h
)

target_link_libraries(kuka_rsi_io
  PUBLIC
    spdlog::spdlog_header_only
    Eigen3::Eigen
    asio::asio
    pugixml::pugixml
)

ament_target_dependencies(kuka_rsi_io PUBLIC
  rclcpp
)

target_include_directories(kuka_rsi_io
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

# Executable: demo
add_executable(demo demo/main.cpp)
target_link_libraries(demo kuka_rsi_io)

# Executable: demo_node (ROS2)
add_executable(demo_node demo/demo_node.cpp)
ament_target_dependencies(demo_node rclcpp sensor_msgs Eigen3)
target_link_libraries(demo_node kuka_rsi_io)
target_include_directories(demo_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${ASIO_INCLUDE_DIR}
  PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(demo_node PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS kuka_rsi_io
  EXPORT kuka_rsi_ioTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS demo RUNTIME DESTINATION bin)
install(TARGETS demo_node DESTINATION lib/${PROJECT_NAME})

ament_package()
