cmake_minimum_required(VERSION 3.22)
project(ris)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_compile_options(-Wall -Wextra -Wpedantic -fPIC)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ris_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(fmt CONFIG REQUIRED)

set(RIS_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include/)

add_subdirectory(src)

ament_package()
